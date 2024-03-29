#pragma once

#include "srpb_evaluation/metric.h"

namespace srpb {
namespace evaluation {

class VelocitySmoothness: public Metric {
public:
  VelocitySmoothness(const std::vector<std::pair<double, logger::RobotData>>& robot_data): Metric(robot_data) {
    compute();
  }

  /// Returns value in m/s^2
  virtual double getValue() const override {
    return velocity_smoothness_;
  }

  void printResults() const override {
    printf("Velocity smoothness = %.4f [m/s^2]\n", velocity_smoothness_);
  }

protected:
  double velocity_smoothness_;

  void compute() override {
    double mean = 0.0;
    rewinder_.setHandlerNextTimestamp(
      [&]() {
        double dt = rewinder_.getTimestampNext() - rewinder_.getTimestampCurr();
        double acc_x = std::abs(rewinder_.getRobotNext().getVelocityX() - rewinder_.getRobotCurr().getVelocityX()) / dt;
        double acc_y = std::abs(rewinder_.getRobotNext().getVelocityY() - rewinder_.getRobotCurr().getVelocityY()) / dt;
        mean += std::hypot(acc_x, acc_y);
      }
    );
    rewinder_.perform();

    mean /= (rewinder_.getTimestampsNum() - 1);

    // save results
    velocity_smoothness_ = mean;
  }
};

} // namespace evaluation
} // namespace srpb
