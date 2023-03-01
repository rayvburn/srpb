#pragma once

#include "srpb_evaluation/metric.h"

namespace srpb {
namespace evaluation {

class BackwardMovements: public Metric {
public:
  /**
   * @brief Constructor
   *
   * @param vel_x_threshold backwards movement threshold (must be orthogonal to oscillation metric)
   */
  BackwardMovements(const std::vector<std::pair<double, logger::RobotData>>& robot_data, double vel_x_threshold)
  : Metric(robot_data),
    vel_x_threshold_(vel_x_threshold)
  {
    compute();
  }

  /// Returns percentage of backward motions
  virtual double getValue() const override {
    return bwd_ * 100.0;
  }

  void printResults() const override {
    printf("Backward movements = %.4f [%%]\n", bwd_ * 100.0);
  }

protected:
  double vel_x_threshold_;
  /// Percentage
  double bwd_;

  void compute() override {
    double bwd_time = 0.0;
    // make sure that we store negative value
    double threshold = -std::abs(vel_x_threshold_);

    rewinder_.setHandlerNextTimestamp(
      [&]() {
        if (rewinder_.getRobotCurr().getVelocityX() < threshold) {
          double dt = rewinder_.getTimestampNext() - rewinder_.getTimestampCurr();
          bwd_time += dt;
        }
      }
    );
    rewinder_.perform();

    // save result
    double total_duration = rewinder_.getTimestampLast() - rewinder_.getTimestampFirst();
    bwd_ = bwd_time / total_duration;
  }
};

} // namespace evaluation
} // namespace srpb
