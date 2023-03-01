#pragma once

#include "srpb_evaluation/metric.h"

#include <angles/angles.h>

namespace srpb {
namespace evaluation {

// Do not over-constrain tolerances if planners were not aware during the execution that their tolerances were enough
class GoalReached: public Metric {
public:
  GoalReached(
    const std::vector<std::pair<double, logger::RobotData>>& robot_data,
    double tolerance_xy = 0.1,
    double tolerance_yaw = 0.2
  ): Metric(robot_data),
    tolerance_xy_(tolerance_xy),
    tolerance_yaw_(tolerance_yaw),
    tolerance_xy_violated_(false),
    tolerance_yaw_violated_(false)
  {
    compute();
  }

  /// Returns boolean value
  virtual double getValue() const override {
    return static_cast<double>(!tolerance_xy_violated_ && !tolerance_yaw_violated_);
  }

  void printResults() const override {
    printf(
      "Goal reached = %d [bool] (tolerance violations: position %d, orientation %d)\n",
      static_cast<int>(!tolerance_xy_violated_ && !tolerance_yaw_violated_),
      static_cast<int>(tolerance_xy_violated_),
      static_cast<int>(tolerance_yaw_violated_)
    );
  }

protected:
  double tolerance_xy_;
  double tolerance_yaw_;
  bool tolerance_xy_violated_;
  bool tolerance_yaw_violated_;

  void compute() override {
    rewinder_.setHandlerLastTimestamp(
      [&]() {
        double dist_to_goal_xy = std::hypot(
          rewinder_.getRobotCurr().getPositionX() - rewinder_.getRobotCurr().getGoalPositionX(),
          rewinder_.getRobotCurr().getPositionY() - rewinder_.getRobotCurr().getGoalPositionY()
        );

        double dist_to_goal_yaw = std::abs(
          angles::shortest_angular_distance(
            rewinder_.getRobotCurr().getOrientationYaw(),
            rewinder_.getRobotCurr().getGoalOrientationYaw()
          )
        );

        // save results
        tolerance_xy_violated_ = dist_to_goal_xy > tolerance_xy_;
        tolerance_yaw_violated_ = std::abs(dist_to_goal_yaw) > std::abs(tolerance_yaw_);
      }
    );
    rewinder_.perform();
  }
};

} // namespace evaluation
} // namespace srpb
