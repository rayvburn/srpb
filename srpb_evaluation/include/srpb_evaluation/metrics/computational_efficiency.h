#pragma once

#include "srpb_evaluation/metric.h"

namespace srpb {
namespace evaluation {

class ComputationalEfficiency: public Metric {
public:
  ComputationalEfficiency(const std::vector<std::pair<double, logger::RobotData>>& robot_data): Metric(robot_data) {
    compute();
  }

  /// Returns efficiency in milliseconds
  virtual double getValue() const override {
    return computational_efficiency_ * 1e3;
  }

  void printResults() const override {
    printf("Computational efficiency = %.4f [msecs]\n", computational_efficiency_ * 1e3);
  }

protected:
  double computational_efficiency_;

  void compute() override {
    double mean = 0.0;
    auto fun_calc = [&](){
      mean += rewinder_.getRobotCurr().getLocalPlanningTime();
    };
    rewinder_.setHandlerNextTimestamp(fun_calc);
    rewinder_.setHandlerLastTimestamp(fun_calc);
    rewinder_.perform();
    // save result
    computational_efficiency_ = mean / rewinder_.getTimestampsNum();
  }
};

} // namespace evaluation
} // namespace srpb
