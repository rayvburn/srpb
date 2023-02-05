#pragma once

#include "srpb_postprocessing/metric.h"

namespace srpb {
namespace postprocessing {

class BackwardMovements: public Metric {
public:
  BackwardMovements(const std::vector<std::pair<double, logger::RobotData>>& robot_data): Metric(robot_data) {
    compute();
  }

  void printResults() const override {
    printf("Backward movements = %.4f [m]\n", bwd_);
  }

protected:
  double bwd_;

  void compute() override {
    double bwd = 0.0;
    rewinder_.setHandlerNextTimestamp(
      [&]() {
        if (rewinder_.getRobotCurr().getVelocityX() < 0.0) {
          bwd += std::abs(rewinder_.getRobotCurr().getVelocityX());
        }
      }
    );
    rewinder_.perform();

    // save result
    bwd_ = bwd;
  }
};

} // namespace postprocessing
} // namespace srpb
