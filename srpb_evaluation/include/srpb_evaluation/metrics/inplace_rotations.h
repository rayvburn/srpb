#pragma once

#include "srpb_evaluation/metric.h"

namespace srpb {
namespace evaluation {

class InplaceRotations: public Metric {
public:
  InplaceRotations(
    const std::vector<std::pair<double, logger::RobotData>>& robot_data,
    double osc_lin_threshold
  ):
    Metric(robot_data),
    osc_lin_threshold_(osc_lin_threshold)
  {
    compute();
  }

  void printResults() const override {
    printf("In-place rotations = %.4f [rad]\n", in_place_rotations_);
  }

protected:
  double osc_lin_threshold_;
  double in_place_rotations_;

  void compute() override {
    double inplace_rot = 0.0;
    rewinder_.setHandlerNextTimestamp(
      [&]() {
        bool robot_stopped = rewinder_.getRobotCurr().getVelocityX() < osc_lin_threshold_ && rewinder_.getRobotCurr().getVelocityX() >= 0.0;
        bool robot_rotates = std::abs(rewinder_.getRobotCurr().getVelocityTheta()) >= 0.0;
        if (robot_stopped && robot_rotates) {
          double dt = rewinder_.getTimestampNext() - rewinder_.getTimestampCurr();
          inplace_rot += (std::abs(rewinder_.getRobotCurr().getVelocityTheta()) / dt);
        }
      }
    );
    rewinder_.perform();

    // save result
    in_place_rotations_ = inplace_rot;
  }
};

} // namespace evaluation
} // namespace srpb
