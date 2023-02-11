#pragma once

#include "srpb_evaluation/metric.h"

namespace srpb {
namespace evaluation {

class ComputationalTimeRepeatability: public Metric {
public:
  ComputationalTimeRepeatability(const std::vector<std::pair<double, logger::RobotData>>& robot_data)
    : Metric(robot_data)
  {
    compute();
  }

  void printResults() const override {
    printf("Computational time repeatability = ±%.4f [msecs]\n", time_repeatability_ * 1e3);
  }

protected:
  double time_repeatability_;

  void compute() override {
    std::vector<double> computation_times;
    double mean = 0.0;

    auto fun_calc = [&](){
      computation_times.push_back(rewinder_.getRobotCurr().getLocalPlanningTime());
      mean += rewinder_.getRobotCurr().getLocalPlanningTime();
    };
    rewinder_.setHandlerNextTimestamp(fun_calc);
    rewinder_.setHandlerLastTimestamp(fun_calc);
    rewinder_.perform();

    // in fact this is computational efficiency
    mean /= static_cast<double>(computation_times.size());

    // mean is computed, we're ready to find the metric value
    double sum = 0.0;
    for (const auto& cn: computation_times) {
      sum += std::pow(cn - mean, 2);
    }

    // save result
    time_repeatability_ = std::sqrt((1.0 / computation_times.size()) * sum);
  }
};

} // namespace evaluation
} // namespace srpb
