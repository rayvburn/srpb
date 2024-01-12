#pragma once

#include "srpb_evaluation/metric_gaussian.h"

#include <social_nav_utils/passing_speed_comfort.h>

namespace srpb {
namespace evaluation {

/**
 * @brief Computes the score of the robot's distance and speed characteristics during the motion around the humans.
 *
 * This metric implements the findings from "The effect of robot speed on comfortable passing distances"
 * by Neggers et al. (2022). A continuous comfort model was recreated based on the data published in this
 * paper. Calculations are performed in a fully deterministic manner.
 * Refer to the social_nav_utils::PassingSpeedComfort implementation for a detailed description of the underlying
 * bivariate model that fits the published data best.
 *
 * @note The class inherits from the @ref MetricGaussian class but in fact @ref PassingSpeedDiscomfort is
 * not a Gaussian metric. It just uses min/max and violations calculation methods from the base class.
 *
 * @param robot_data
 * @param people_data
 * @param distance_min minimum distance between the center of the robot and a human at which (when robot speed
 * is equal to @ref speed_max) the normalized discomfort is the highest (1.0); the human is treated as a point here
 * @param speed_max the maximum speed of the robot; used for a discomfort normalization (see @ref distance_min too)
 * @param discomfort_threshold discomfort values above this level will be treated as violations
 * @param max_method whether to use the maximum discomfort in a given timestamp as an indicator (true); the average
 * is used when set to false
 */
class PassingSpeedDiscomfort: public MetricGaussian {
public:
  PassingSpeedDiscomfort(
    const std::vector<std::pair<double, logger::RobotData>>& robot_data,
    const std::vector<std::pair<double, people_msgs_utils::Person>>& people_data,
    double distance_min = 0.275,
    double speed_max = 0.55,
    double discomfort_threshold = 0.4,
    bool max_method = true
  ):
    MetricGaussian(robot_data, people_data),
    distance_min_(distance_min),
    robot_speed_max_(speed_max),
    discomfort_threshold_(discomfort_threshold),
    max_method_(max_method),
    discomfort_min_(0.0),
    discomfort_max_(0.0),
    discomfort_total_(0.0),
    violations_percentage_(0.0)
  {
    if (people_data.empty()) {
      return;
    }
    compute();
  }

  /// Returns total percentage throughout the scenario
  virtual double getValue() const override {
    return discomfort_total_ * 100.0;
  }

  /// Returns minimum percentage throughout the scenario
  virtual double getValueMin() const override {
    return discomfort_min_ * 100.0;
  }

  /// Returns maximum percentage throughout the scenario
  virtual double getValueMax() const override {
    return discomfort_max_ * 100.0;
  }

  /// Returns percentage of violations throughout the scenario (regarding threshold)
  virtual double getViolations() const override {
    return violations_percentage_ * 100.0;
  }

  void printResults() const override {
    printf(
      "Passing speed discomfort = %.4f [%%] (min = %.4f [%%], max = %.4f [%%], violations %.4f [%%])\n",
      discomfort_total_ * 100.0,
      discomfort_min_ * 100.0,
      discomfort_max_ * 100.0,
      violations_percentage_ * 100.0
    );
  }

protected:
  // parameters
  double distance_min_;
  double robot_speed_max_;
  double discomfort_threshold_;
  bool max_method_;

  // results
  double discomfort_min_;
  double discomfort_max_;
  double discomfort_total_;
  double violations_percentage_;

  void compute() override {
    // store durations and discomforts of the humans in terms of robot motions
    std::vector<std::pair<double, std::vector<double>>> timed_discomforts;
    /// prepare a container for discomfort values in this `for` iteration
    std::pair<double, std::vector<double>> timed_discomfort;

    rewinder_.setHandlerNextTimestamp(
      [&]() {
        // prepare a container for upcoming calculations related to passing speed discomforts
        timed_discomfort = std::make_pair(
          rewinder_.getTimestampNext() - rewinder_.getTimestampCurr(), std::vector<double>()
        );
      }
    );

    // on event 'iterating through next person in the timestamp' - compute a human discomfort given the robot position
    // and a speed
    rewinder_.setHandlerNextPersonTimestamp(
      [&]() {
        // Euclidean distance between centers
        double distance = std::sqrt(
          std::pow(rewinder_.getRobotCurr().getPositionX() - rewinder_.getPersonCurr().getPositionX(), 2.0) +
          std::pow(rewinder_.getRobotCurr().getPositionY() - rewinder_.getPersonCurr().getPositionY(), 2.0)
        );
        double robot_speed = std::hypot(
          rewinder_.getRobotCurr().getVelocityX(), rewinder_.getRobotCurr().getVelocityY()
        );

        // compute discomfort given the distance between the centers and robot speed
        social_nav_utils::PassingSpeedComfort psd(
          distance,
          robot_speed,
          distance_min_,
          robot_speed_max_
        );

        // store result for later aggregation
        timed_discomfort.second.push_back(psd.getDiscomfortNormalized());
      }
    );

    // on event 'iterated through all people in the timestamp'
    rewinder_.setHandlerAllPeopleTimestamp(
      [&]() {
        if (!timed_discomfort.second.empty()) {
          timed_discomforts.push_back(timed_discomfort);
        }
      }
    );
    rewinder_.perform();

    std::tie(
      discomfort_min_,
      discomfort_max_,
      discomfort_total_,
      violations_percentage_
    ) = MetricGaussian::calculateGaussianStatistics(timed_discomforts, discomfort_threshold_, max_method_);
  }
};

} // namespace evaluation
} // namespace srpb
