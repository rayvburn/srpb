#pragma once

#include "srpb_evaluation/metric_gaussian.h"

#include <social_nav_utils/personal_space_intrusion.h>

namespace srpb {
namespace evaluation {

/**
 * @brief Computes people personal space intrusion score
 *
 * People have their constrained area attached to the body projection. Once robot moves around people it may come
 * too closer or further. In each time step a value of a person area (modelled by an Asymmetric Gaussian) is computed.
 * This function returns scores: min Gaussian, max Gaussian and normalized to execution time. Maximum of Gaussian,
 * which is 1.0, shows that robot was located in the same position as person over the whole experiment. On the other
 * hand, normalized value of 0.0 means that robot never moved close to any person (according to the given variances).
 *
 * @param robot_data
 * @param people_data
 * @param var_h variance to the heading direction of the person (Gaussian)
 * @param var_r variance to the rear (Gaussian)
 * @param var_s variance to the side (Gaussian)
 * @param personal_space_threshold Gaussian values bigger than that will be considered as violation of personal space
 * @param max_method set to true (default) to use max element from Gaussians to normalize metrics;
 * false means averaging over all Gaussian occurrences in a current time step
 *
 * @return std::tuple<double, double, double, unsigned int> tuple with scores: min, max and normalized to execution
 * time and number of personal space violations (according to @ref personal_space_threshold)
 *
 * The closest to our method is the approach presented by Truong and Ngo in
 * “To Approach Humans?”: A Unified Framework for Approaching Pose Prediction and Socially Aware Robot Navigation
 * They called it `Social Individual Index`. Their method lacks normalization in terms of path duration.
 */
class PersonalSpaceIntrusion: public MetricGaussian {
public:
  PersonalSpaceIntrusion(
    const std::vector<std::pair<double, logger::RobotData>>& robot_data,
    const std::vector<std::pair<double, people_msgs_utils::Person>>& people_data,
    double var_h,
    double var_r,
    double var_s,
    double personal_space_threshold,
    bool max_method = true
  ):
    MetricGaussian(robot_data, people_data),
    var_front_(var_h),
    var_rear_(var_r),
    var_side_(var_s),
    personal_space_threshold_(personal_space_threshold),
    max_method_(max_method)
  {
    if (people_data.empty()) {
      return;
    }
    compute();
  }

  /// Returns total percentage throughout the scenario
  virtual double getValue() const override {
    return intrusion_total_ * 100.0;
  }

  /// Returns minimum percentage throughout the scenario
  virtual double getValueMin() const override {
    return intrusion_min_ * 100.0;
  }

  /// Returns maximum percentage throughout the scenario
  virtual double getValueMax() const override {
    return intrusion_max_ * 100.0;
  }

  /// Returns percentage of violations throughout the scenario (regarding threshold)
  virtual double getViolations() const override {
    return violations_percentage_ * 100.0;
  }

  void printResults() const override {
    printf(
      "Personal space intrusion = %.4f [%%] (min = %.4f [%%], max = %.4f [%%], violations %.4f [%%])\n",
      intrusion_total_ * 100.0,
      intrusion_min_ * 100.0,
      intrusion_max_ * 100.0,
      violations_percentage_ * 100.0
    );
  }

protected:
  // parameters
  double var_front_;
  double var_rear_;
  double var_side_;
  double personal_space_threshold_;
  bool max_method_;

  // results
  double intrusion_min_;
  double intrusion_max_;
  double intrusion_total_;
  double violations_percentage_;

  void compute() override {
    // store durations and Gaussians of the robot in terms of nearby people
    std::vector<std::pair<double, std::vector<double>>> timed_gaussians;
    /// prepare container for gaussian values in this `for` iteration
    std::pair<double, std::vector<double>> timed_gaussian;

    rewinder_.setHandlerNextTimestamp(
      [&]() {
        // prepare container for upcoming calculations related to human personal spaces
        timed_gaussian = std::make_pair(rewinder_.getTimestampNext() - rewinder_.getTimestampCurr(), std::vector<double>());
      }
    );

    // on event 'iterating through next person in the timestamp' - compute Gaussian of Personal Zone at robot position
    rewinder_.setHandlerNextPersonTimestamp(
      [&]() {
        // compute gaussian at position of robot
        social_nav_utils::PersonalSpaceIntrusion psi(
          rewinder_.getPersonCurr().getPositionX(),
          rewinder_.getPersonCurr().getPositionY(),
          rewinder_.getPersonCurr().getOrientationYaw(),
          rewinder_.getPersonCurr().getCovariancePoseXX(),
          rewinder_.getPersonCurr().getCovariancePoseXY(),
          rewinder_.getPersonCurr().getCovariancePoseYX(),
          rewinder_.getPersonCurr().getCovariancePoseYY(),
          var_front_,
          var_rear_,
          var_side_,
          rewinder_.getRobotCurr().getPositionX(),
          rewinder_.getRobotCurr().getPositionY(),
          true
        );
        psi.normalize();

        // store result for later aggregation
        timed_gaussian.second.push_back(psi.getScale());
      }
    );

    // on event 'iterated through all people in the timestamp'
    rewinder_.setHandlerAllPeopleTimestamp(
      [&]() {
        if (!timed_gaussian.second.empty()) {
          timed_gaussians.push_back(timed_gaussian);
        }
      }
    );
    rewinder_.perform();

    std::tie(
      intrusion_min_,
      intrusion_max_,
      intrusion_total_,
      violations_percentage_
    ) = MetricGaussian::calculateGaussianStatistics(timed_gaussians, personal_space_threshold_, max_method_);
  }
};

} // namespace evaluation
} // namespace srpb
