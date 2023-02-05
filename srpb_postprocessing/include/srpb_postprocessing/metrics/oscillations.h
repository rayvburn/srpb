#pragma once

#include "srpb_postprocessing/metric.h"

namespace srpb {
namespace postprocessing {

class Oscillations: public Metric {
public:
  Oscillations(
    const std::vector<std::pair<double, logger::RobotData>>& robot_data,
    double osc_lin_threshold,
    double osc_ang_threshold
  ):
    Metric(robot_data),
    osc_lin_threshold_(osc_lin_threshold),
    osc_ang_threshold_(osc_ang_threshold)
  {
    compute();
  }

  void printResults() const override {
    printf("Oscillations = %.4f [rad]\n", osc_);
  }

protected:
  double osc_lin_threshold_;
  double osc_ang_threshold_;
  double osc_;

  void compute() override {
    double osc = 0.0;
    rewinder_.setHandlerNextTimestamp(
      [&]() {
        bool robot_stopped = rewinder_.getRobotCurr().getVelocityX() < osc_lin_threshold_ && rewinder_.getRobotCurr().getVelocityX() >= 0.0;
        bool robot_oscillates = std::abs(rewinder_.getRobotCurr().getVelocityTheta()) < osc_ang_threshold_;
        if (robot_stopped && robot_oscillates) {
          double dt = rewinder_.getTimestampNext() - rewinder_.getTimestampCurr();
          osc += (std::abs(rewinder_.getRobotCurr().getVelocityTheta()) / dt);
        }
      }
    );
    rewinder_.perform();

    // save result
    osc_ = osc;
  }
};

} // namespace postprocessing
} // namespace srpb
