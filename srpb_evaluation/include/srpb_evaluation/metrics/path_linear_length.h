#pragma once

#include "srpb_evaluation/metric.h"

namespace srpb {
namespace evaluation {

class PathLinearLength: public Metric {
public:
  PathLinearLength(const std::vector<std::pair<double, logger::RobotData>>& robot_data): Metric(robot_data) {
    compute();
  }

  void printResults() const override {
    printf("Path linear length = %.4f [m]\n", path_length_linear_);
  }

protected:
  double path_length_linear_;

  void compute() override {
    double path_length = 0.0;
    rewinder_.setHandlerNextTimestamp(
      [&]() {
        double dx = std::abs(rewinder_.getRobotNext().getPositionX() - rewinder_.getRobotCurr().getPositionX());
        double dy = std::abs(rewinder_.getRobotNext().getPositionY() - rewinder_.getRobotCurr().getPositionY());
        path_length += std::hypot(dx, dy);
      }
    );
    rewinder_.perform();

    // save results
    path_length_linear_ = path_length;
  }
};

} // namespace evaluation
} // namespace srpb
