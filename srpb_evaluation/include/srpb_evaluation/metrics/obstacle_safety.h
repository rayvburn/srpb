#pragma once

#include "srpb_evaluation/metric_statistics.h"

#include <numeric>

namespace srpb {
namespace evaluation {

class ObstacleSafety: public MetricStatistics {
public:
  ObstacleSafety(
    const std::vector<std::pair<double, logger::RobotData>>& robot_data,
    double safety_distance
  ):
    MetricStatistics(robot_data),
    safety_distance_(safety_distance)
  {
    compute();
  }

  /// Returns the time-corrected mean distance to the closest obstacle in subsequent time steps of the scenario
  virtual double getValue() const override {
    return obstacle_distance_total_;
  }

  /// Returns minimum value obtained throughout the scenario
  virtual double getValueMin() const override {
    return obstacle_distance_min_;
  }

  /// Returns maximum value obtained throughout the scenario
  virtual double getValueMax() const override {
    return obstacle_distance_max_;
  }

  /**
   * Returns the percentage of violations obtained throughout the scenario (considering the threshold value)
   * The value according to the metric originally implemented in MRPB 1.0
   * See https://github.com/NKU-MobFly-Robotics/local-planning-benchmark for details
   */
  virtual double getViolations() const override {
    return violations_percentage_ * 100.0;
  }

  virtual void printResults() const override {
    printf(
      "Obstacle safety = %.4f [m] (min = %.4f [m], max = %.4f [m], violations %.4f [%%])\n",
      obstacle_distance_total_,
      obstacle_distance_min_,
      obstacle_distance_max_,
      violations_percentage_ * 100.0
    );
  }

protected:
  double safety_distance_;

  double obstacle_distance_min_;
  double obstacle_distance_max_;
  double obstacle_distance_total_;
  /// "Obstacle safety" in MRPB 1.0
  double violations_percentage_;

  void compute() override {
    // container to compute, i.a., min and max values of distances to obstacles
    std::vector<std::pair<double, std::vector<double>>> timed_obs_distances;

    rewinder_.setHandlerNextTimestamp(
      [&]() {
        timed_obs_distances.push_back({
          // step duration
          rewinder_.getTimestampNext() - rewinder_.getTimestampCurr(),
          // single value container (there is only 1 closest obstacle)
          std::vector<double>{rewinder_.getRobotCurr().getDistToObstacle()}
        });
      }
    );
    rewinder_.perform();

    // NOTE1: a violation is regarded when distance is less than the threshold
    // NOTE2: max method does not matter here as only 1 observation in each step is available
    std::tie(
      obstacle_distance_min_,
      obstacle_distance_max_,
      obstacle_distance_total_,
      violations_percentage_
    ) = MetricStatistics::calculateStatistics(timed_obs_distances, safety_distance_, false, true);
  }
};

} // namespace evaluation
} // namespace srpb
