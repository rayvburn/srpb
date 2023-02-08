#pragma once

#include "srpb_evaluation/metric.h"

namespace srpb {
namespace evaluation {

/// @details Originally implemented in MRPB 1.0 (https://github.com/NKU-MobFly-Robotics/local-planning-benchmark)
class ObstacleSafety: public Metric {
public:
  ObstacleSafety(
    const std::vector<std::pair<double, logger::RobotData>>& robot_data,
    double safety_distance
  ):
    Metric(robot_data),
    safety_distance_(safety_distance)
  {
    compute();
  }

  void printResults() const override {
    printf("Obstacle safety = %.4f [%%]\n", obstacle_safety_ * 1e2);
  }

protected:
  double safety_distance_;
  double obstacle_safety_;

  void compute() override {
    // timestamps to sum up time
    const double TS_NOT_SET = std::numeric_limits<double>::min();
    double ts_start = TS_NOT_SET;
    double ts_end = TS_NOT_SET;

    double ts_sum = 0.0;
    auto calc_next = [&]() {
      // compute how long robot traveled near obstacles located within safety distance
      if (rewinder_.getRobotCurr().getDistToObstacle() < safety_distance_) {
        if (ts_start == TS_NOT_SET) {
          ts_start = rewinder_.getTimestampCurr();
        } else {
          // save timestamp
          ts_end = rewinder_.getTimestampCurr();
        }
      } else {
        if (ts_end != TS_NOT_SET) {
          // robot started to maintain safety distance again
          ts_sum += (ts_end - ts_start);

          // reset timestamps for incoming counts
          ts_start = TS_NOT_SET;
          ts_end = TS_NOT_SET;
        }
      }
    };
    // NOTE: external lambda must be explicitly passed to capture
    auto calc_last = [&, calc_next]() {
      calc_next();
      // sum up the last period (if robot finished course not maintaining the safety distance)
      if (ts_start != TS_NOT_SET && ts_end != TS_NOT_SET) {
        ts_sum += ts_end - ts_start;
      }
    };
    rewinder_.setHandlerNextTimestamp(calc_next);
    rewinder_.setHandlerLastTimestamp(calc_last);
    rewinder_.perform();

    // save result
    obstacle_safety_ = ts_sum / rewinder_.getDuration();
  }
};

} // namespace evaluation
} // namespace srpb
