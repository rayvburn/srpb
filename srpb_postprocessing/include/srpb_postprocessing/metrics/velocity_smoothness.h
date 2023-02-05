#pragma once

#include "srpb_postprocessing/metric.h"

namespace srpb {
namespace postprocessing {

class VelocitySmoothness: public Metric {
public:
  VelocitySmoothness(const std::vector<std::pair<double, logger::RobotData>>& robot_data): Metric(robot_data) {
    compute();
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
        double acc = std::abs(rewinder_.getRobotNext().getVelocityX() - rewinder_.getRobotCurr().getVelocityX()) / dt;
        mean += acc;
      }
    );
    rewinder_.perform();

    mean /= (rewinder_.getTimestampsNum() - 1);

    // save results
    velocity_smoothness_ = mean;
  }
};

} // namespace postprocessing
} // namespace srpb
