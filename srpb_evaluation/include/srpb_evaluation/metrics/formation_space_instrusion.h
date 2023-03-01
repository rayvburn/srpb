#pragma once

#include "srpb_evaluation/metric_gaussian.h"

#include <social_nav_utils/formation_space_intrusion.h>

namespace srpb {
namespace evaluation {

/// Similar to personal space intrusion but related to the group space
class FormationSpaceIntrusion: public MetricGaussian {
public:
  FormationSpaceIntrusion(
    const std::vector<std::pair<double, logger::RobotData>>& robot_data,
    const std::vector<std::pair<double, people_msgs_utils::Person>>& people_data,
    const std::vector<std::pair<double, people_msgs_utils::Group>>& groups_data,
    double group_space_threshold,
    bool max_method = true
  ):
    MetricGaussian(robot_data, people_data, groups_data),
    group_space_threshold_(group_space_threshold),
    max_method_(max_method)
  {
    if (people_data.empty() || groups_data.empty()) {
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
      "Formation space intrusion = %.4f [%%] (min = %.4f [%%], max = %.4f [%%], violations %.4f [%%])\n",
      intrusion_total_ * 100.0,
      intrusion_min_ * 100.0,
      intrusion_max_ * 100.0,
      violations_percentage_ * 100.0
    );
  }

protected:
  // parameters
  double group_space_threshold_;
  bool max_method_;

  // results
  double intrusion_min_;
  double intrusion_max_;
  double intrusion_total_;
  double violations_percentage_;

  void compute() override {
    // store durations and Gaussians of the robot in terms of nearby groups
    std::vector<std::pair<double, std::vector<double>>> timed_gaussians;
    /// prepare container for gaussian values in this `for` iteration
    std::pair<double, std::vector<double>> timed_gaussian;

    rewinder_.setHandlerNextTimestamp(
      [&]() {
        // prepare container for upcoming calculations related to human personal spaces
        timed_gaussian = std::make_pair(rewinder_.getTimestampNext() - rewinder_.getTimestampCurr(), std::vector<double>());
      }
    );

    rewinder_.setHandlerNextGroupTimestamp(
      [&]() {
        // computations for the group
        // take half of the span and apply 2 sigma rule
        // (mean is the center of the O-space, 2 times stddev corresponds to its span)
        double variance_ospace_x = std::pow((rewinder_.getGroupCurr().getSpanX() / 2.0) / 2.0, 2);
        double variance_ospace_y = std::pow((rewinder_.getGroupCurr().getSpanY() / 2.0) / 2.0, 2);

        social_nav_utils::FormationSpaceIntrusion fsi(
          rewinder_.getGroupCurr().getPositionX(),
          rewinder_.getGroupCurr().getPositionY(),
          rewinder_.getGroupCurr().getOrientationYaw(),
          variance_ospace_x,
          variance_ospace_y,
          rewinder_.getGroupCurr().getCovariancePoseXX(),
          rewinder_.getGroupCurr().getCovariancePoseXY(),
          rewinder_.getGroupCurr().getCovariancePoseYY(),
          rewinder_.getRobotCurr().getPositionX(),
          rewinder_.getRobotCurr().getPositionY()
        );
        fsi.normalize();

        // Gaussian cost of the robot being located in the current pose; cost related to the investigated group of people
        timed_gaussian.second.push_back(fsi.getScale());
      }
    );

    rewinder_.setHandlerAllGroupsTimestamp(
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
    ) = MetricGaussian::calculateGaussianStatistics(timed_gaussians, group_space_threshold_, max_method_);
  }
};

} // namespace evaluation
} // namespace srpb
