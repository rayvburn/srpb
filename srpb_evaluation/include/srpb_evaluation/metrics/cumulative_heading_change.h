#pragma once

#include "srpb_evaluation/metric.h"

namespace srpb {
namespace evaluation {

class CumulativeHeadingChange: public Metric {
public:
  CumulativeHeadingChange(const std::vector<std::pair<double, logger::RobotData>>& robot_data): Metric(robot_data) {
    compute();
  }

  /// Returns value in rad/s^2
  virtual double getValue() const override {
    return chc_;
  }

  void printResults() const override {
    printf("Cumulative Heading Change = %.4f [rad/s^2]\n", chc_);
  }

protected:
  double chc_;

  void compute() override {
    double chc = 0.0;
    rewinder_.setHandlerNextTimestamp(
      [&]() {
        double dt = rewinder_.getTimestampNext() - rewinder_.getTimestampCurr();
        double dtheta = rewinder_.getRobotNext().getVelocityTheta() - rewinder_.getRobotCurr().getVelocityTheta();
        chc += (std::abs(dtheta) / dt);
      }
    );
    rewinder_.perform();

    // save result
    chc_ = chc;
  }
};

} // namespace evaluation
} // namespace srpb
