#pragma once

#include "srpb_evaluation/metric.h"

namespace srpb {
namespace evaluation {

class InplaceRotations: public Metric {
public:
  /// Tolerance for velocity components to consider them as negligible
  static constexpr double VEL_STOPPED_TOLERANCE = 5e-03;

  /**
   * @brief Constructor
   *
   * @param vel_ang_threshold in-place rotation threshold (must be orthogonal to oscillation metric)
   */
  InplaceRotations(
    const std::vector<std::pair<double, logger::RobotData>>& robot_data,
    double vel_ang_threshold
  ):
    Metric(robot_data),
    vel_ang_threshold_(vel_ang_threshold)
  {
    compute();
  }

  /// Returns percentage of in-place rotations throughout the scenario
  virtual double getValue() const override {
    return in_place_rotations_ * 100.0;
  }

  void printResults() const override {
    printf("In-place rotations = %.4f [%%]\n", in_place_rotations_ * 100.0);
  }

protected:
  double vel_ang_threshold_;
  double in_place_rotations_;

  void compute() override {
    double inplace_rot_time = 0.0;

    rewinder_.setHandlerNextTimestamp(
      [&]() {
        bool vel_x_matches = std::abs(rewinder_.getRobotCurr().getVelocityX()) < VEL_STOPPED_TOLERANCE;
        bool vel_y_matches = std::abs(rewinder_.getRobotCurr().getVelocityY()) < VEL_STOPPED_TOLERANCE;
        bool vel_th_matches = std::abs(rewinder_.getRobotCurr().getVelocityTheta()) >= vel_ang_threshold_;

        if (!vel_x_matches || !vel_y_matches || !vel_th_matches) {
          return;
        }

        double dt = rewinder_.getTimestampNext() - rewinder_.getTimestampCurr();
        inplace_rot_time += dt;
      }
    );
    rewinder_.perform();

    // save result
    double total_duration = rewinder_.getTimestampLast() - rewinder_.getTimestampFirst();
    in_place_rotations_ = inplace_rot_time / total_duration;
  }
};

} // namespace evaluation
} // namespace srpb
