#pragma once

#include "srpb_evaluation/metric.h"

namespace srpb {
namespace evaluation {

class CumulativeHeadingChange: public Metric {
public:
  CumulativeHeadingChange(const std::vector<std::pair<double, logger::RobotData>>& robot_data): Metric(robot_data) {
    compute();
  }

  /// Returns value in rad
  virtual double getValue() const override {
    return chc_;
  }

  void printResults() const override {
    printf("Cumulative Heading Change = %.4f [rad]\n", chc_);
  }

protected:
  double chc_;

  void compute() override {
    double chc = 0.0;
    rewinder_.setHandlerNextTimestamp(
      [&]() {
        double dtheta = rewinder_.getRobotNext().getOrientationYaw() - rewinder_.getRobotCurr().getOrientationYaw();
        chc += std::abs(dtheta);
      }
    );
    rewinder_.perform();

    // save result
    chc_ = chc;
  }
};

} // namespace evaluation
} // namespace srpb
