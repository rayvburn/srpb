#pragma once

#include "srpb_postprocessing/metric.h"

namespace srpb {
namespace postprocessing {

class MotionEfficiency: public Metric {
public:
  MotionEfficiency(const std::vector<std::pair<double, logger::RobotData>>& robot_data): Metric(robot_data) {
    compute();
  }

  void printResults() const override {
    printf("Motion efficiency = %.4f [secs]\n", motion_efficiency_);
  }

protected:
  double motion_efficiency_;

  void compute() override {
    // save result
    motion_efficiency_ = rewinder_.getDuration();
  }
};

} // namespace postprocessing
} // namespace srpb
