#pragma once

#include "srpb_postprocessing/metric.h"

namespace srpb {
namespace postprocessing {

class CumulativeHeadingChange: public Metric {
public:
  CumulativeHeadingChange(const std::vector<std::pair<double, logger::RobotData>>& robot_data): Metric(robot_data) {
    compute();
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
        double dt = rewinder_.getTimestampNext() - rewinder_.getTimestampCurr();
        chc += (std::abs(rewinder_.getRobotNext().getOrientationYaw() - rewinder_.getRobotCurr().getOrientationYaw()) / dt);
      }
    );
    rewinder_.perform();

    // save result
    chc_ = chc;
  }
};

} // namespace postprocessing
} // namespace srpb
