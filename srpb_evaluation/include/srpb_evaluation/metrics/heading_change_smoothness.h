#pragma once

#include "srpb_evaluation/metric.h"

namespace srpb {
namespace evaluation {

class HeadingChangeSmoothness: public Metric {
public:
  HeadingChangeSmoothness(const std::vector<std::pair<double, logger::RobotData>>& robot_data): Metric(robot_data) {
    compute();
  }

  /// Returns value in rad/s^2
  virtual double getValue() const override {
    return hsm_;
  }

  void printResults() const override {
    printf("Heading Change Smoothness = %.4f [rad/s^2]\n", hsm_);
  }

protected:
  double hsm_;

  void compute() override {
    double hsm = 0.0;
    rewinder_.setHandlerNextTimestamp(
      [&]() {
        double dt = rewinder_.getTimestampNext() - rewinder_.getTimestampCurr();
        double dtheta = rewinder_.getRobotNext().getVelocityTheta() - rewinder_.getRobotCurr().getVelocityTheta();
        hsm += (std::abs(dtheta) / dt);
      }
    );
    rewinder_.perform();

    // mean
    hsm /= (rewinder_.getTimestampsNum() - 1);

    // save result
    hsm_ = hsm;
  }
};

} // namespace evaluation
} // namespace srpb
