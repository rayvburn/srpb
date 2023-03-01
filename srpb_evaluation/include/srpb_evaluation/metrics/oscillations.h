#pragma once

#include "srpb_evaluation/metric.h"

namespace srpb {
namespace evaluation {

class Oscillations: public Metric {
public:
  Oscillations(
    const std::vector<std::pair<double, logger::RobotData>>& robot_data,
    double vel_lin_threshold,
    double vel_x_threshold,
    double vel_y_threshold,
    double vel_ang_threshold
  ):
    Metric(robot_data),
    vel_lin_threshold_(vel_lin_threshold),
    vel_x_threshold_(vel_x_threshold),
    vel_y_threshold_(vel_y_threshold),
    vel_ang_threshold_(vel_ang_threshold)
  {
    compute();
  }

  /// Returns percentage of oscillations throughout the scenario
  virtual double getValue() const override {
    return osc_ * 100.0;
  }

  void printResults() const override {
    printf("Oscillations = %.4f [%%]\n", osc_ * 100.0);
  }

protected:
  double vel_lin_threshold_;
  double vel_x_threshold_;
  double vel_y_threshold_;
  double vel_ang_threshold_;
  /// Percentage
  double osc_;

  void compute() override {
    double osc_time = 0.0;

    rewinder_.setHandlerNextTimestamp(
      [&]() {
        // evaluate oscillation conditions
        bool vx_matches = std::abs(rewinder_.getRobotCurr().getVelocityX()) < vel_x_threshold_;
        bool vy_matches = std::abs(rewinder_.getRobotCurr().getVelocityX()) < vel_y_threshold_;
        bool vth_matches = std::abs(rewinder_.getRobotCurr().getVelocityTheta()) < vel_ang_threshold_;

        double vel_lin = std::hypot(rewinder_.getRobotCurr().getVelocityX(), rewinder_.getRobotCurr().getVelocityY());
        bool vel_lin_matches = vel_lin < vel_lin_threshold_;

        if (!vx_matches || !vy_matches || !vth_matches || !vel_lin_matches) {
          return;
        }

        double dt = rewinder_.getTimestampNext() - rewinder_.getTimestampCurr();
        osc_time += dt;
      }
    );
    rewinder_.perform();

    // save result
    double total_duration = rewinder_.getTimestampLast() - rewinder_.getTimestampFirst();
    osc_ = osc_time / total_duration;
  }
};

} // namespace evaluation
} // namespace srpb
