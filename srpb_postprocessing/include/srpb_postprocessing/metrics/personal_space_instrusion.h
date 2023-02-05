#pragma once

#include "postprocessing/metric_gaussian.h"

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
 * @param sigma_h variance to the heading direction of the person (Gaussian)
 * @param sigma_r variance to the rear (Gaussian)
 * @param sigma_s variance to the side (Gaussian)
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
    const std::vector<std::pair<double, RobotData>>& robot_data,
    const std::vector<std::pair<double, people_msgs_utils::Person>>& people_data,
    double sigma_h,
    double sigma_r,
    double sigma_s,
    double personal_space_threshold,
    bool max_method = true
  ):
    MetricGaussian(robot_data, people_data),
    sigma_h_(sigma_h),
    sigma_r_(sigma_r),
    sigma_s_(sigma_s),
    personal_space_threshold_(personal_space_threshold),
    max_method_(max_method)
  {
    compute();
  }

  void printResults() const override {
    printf(
      "Personal space intrusion = %.4f [%%] (min = %.4f [%%], max = %.4f [%%], violations %3u)\n",
      intrusion_total_,
      intrusion_min_,
      intrusion_max_,
      violations_num_
    );
  }

protected:
  // parameters
  double sigma_h_;
  double sigma_r_;
  double sigma_s_;
  double personal_space_threshold_;
  bool max_method_;

  // results
  double intrusion_min_;
  double intrusion_max_;
  double intrusion_total_;
  unsigned int violations_num_;

  void compute() override {
    // store durations and Gaussians of the robot in terms of nearby people
    std::vector<std::pair<double, std::vector<double>>> timed_gaussians;
    /// prepare container for gaussian values in this `for` iteration
    std::pair<double, std::vector<double>> timed_gaussian;

    rewinder_.setHandlerNextTimestamp(
      [&]() {
        if (rewinder_.getTimestampCurr() == rewinder_.getTimestampLast()) {
          return;
        }
        timed_gaussian = std::make_pair(rewinder_.getTimestampNext() - rewinder_.getTimestampCurr(), std::vector<double>());
      }
    );

    rewinder_.setHandlerNextPersonTimestamp(
      [&]() {
        // processing
        double gaussian = calculateGaussian(
          rewinder_.getRobotCurr().getPositionX(),
          rewinder_.getRobotCurr().getPositionY(),
          rewinder_.getPersonCurr().getPositionX(),
          rewinder_.getPersonCurr().getPositionY(),
          rewinder_.getPersonCurr().getOrientationYaw(),
          sigma_h_,
          sigma_r_,
          sigma_s_
        );
        // count in the person's tracking reliability - we want to avoid penalizing robot that investigates `old` tracks
        double cost = gaussian * rewinder_.getPersonCurr().getReliability();
        timed_gaussian.second.push_back(cost);
      }
    );

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
      violations_num_
    ) = MetricGaussian::calculateGaussianStatistics(timed_gaussians, personal_space_threshold_, max_method_);
  }
};
