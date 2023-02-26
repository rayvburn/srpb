#pragma once

#include "srpb_evaluation/metric_gaussian.h"

#include <social_nav_utils/heading_direction_disturbance.h>

namespace srpb {
namespace evaluation {

/// Related to velocity and direction of the robot movement towards person
class HeadingDirectionDisturbance: public MetricGaussian {
public:
  HeadingDirectionDisturbance(
    const std::vector<std::pair<double, logger::RobotData>>& robot_data,
    const std::vector<std::pair<double, people_msgs_utils::Person>>& people_data,
    double disturbance_threshold,
    double person_occupancy_radius = social_nav_utils::HeadingDirectionDisturbance::OCCUPANCY_MODEL_RADIUS_DEFAULT,
    double person_fov = social_nav_utils::HeadingDirectionDisturbance::FOV_DEFAULT,
    double robot_circumradius = social_nav_utils::HeadingDirectionDisturbance::CIRCUMRADIUS_DEFAULT,
    double robot_max_speed = social_nav_utils::HeadingDirectionDisturbance::MAX_SPEED_DEFAULT,
    bool max_method = true
  ):
    MetricGaussian(robot_data, people_data),
    disturbance_threshold_(disturbance_threshold),
    person_occupancy_radius_(person_occupancy_radius),
    person_fov_(person_fov),
    robot_circumradius_(robot_circumradius),
    robot_max_speed_(robot_max_speed),
    max_method_(max_method)
  {
    if (people_data.empty()) {
      return;
    }
    compute();
  }

  void printResults() const override {
    printf(
    "Heading direction disturbance = %.4f [%%] (min = %.4f [%%], max = %.4f [%%], violations %.4f [%%])\n",
      disturbance_total_ * 100.0,
      disturbance_min_ * 100.0,
      disturbance_max_ * 100.0,
      violations_percentage_ * 100.0
    );
  }

protected:
  // parameters
  double disturbance_threshold_;
  double person_occupancy_radius_;
  double person_fov_;
  double robot_circumradius_;
  double robot_max_speed_;
  bool max_method_;

  // results
  double disturbance_min_;
  double disturbance_max_;
  double disturbance_total_;
  double violations_percentage_;

  void compute() override {
    // store durations and disturbance indices of the robot in terms of nearby people
    std::vector<std::pair<double, std::vector<double>>> timed_disturbances;

    /// prepare container for gaussian values in this `for` iteration
    std::pair<double, std::vector<double>> timed_disturbance;

    rewinder_.setHandlerNextTimestamp(
      [&]() {
        // prepare container for upcoming calculations related to human personal spaces
        timed_disturbance = std::make_pair(rewinder_.getTimestampNext() - rewinder_.getTimestampCurr(), std::vector<double>());
      }
    );

    rewinder_.setHandlerNextPersonTimestamp(
      [&]() {
        social_nav_utils::HeadingDirectionDisturbance heading(
          rewinder_.getPersonCurr().getPositionX(),
          rewinder_.getPersonCurr().getPositionY(),
          rewinder_.getPersonCurr().getOrientationYaw(),
          rewinder_.getPersonCurr().getCovariancePoseXX(),
          rewinder_.getPersonCurr().getCovariancePoseXY(),
          rewinder_.getPersonCurr().getCovariancePoseYY(),
          rewinder_.getRobotCurr().getPositionX(),
          rewinder_.getRobotCurr().getPositionY(),
          rewinder_.getRobotCurr().getOrientationYaw(),
          rewinder_.getRobotCurr().getVelocityX(),
          rewinder_.getRobotCurr().getVelocityY(),
          person_occupancy_radius_,
          person_fov_
        );
        // normalize cost
        heading.normalize(robot_circumradius_, robot_max_speed_);

        timed_disturbance.second.push_back(heading.getScale());
      }
    );

    // on event 'iterated through all people in the timestamp'
    rewinder_.setHandlerAllPeopleTimestamp(
      [&]() {
        if (!timed_disturbance.second.empty()) {
          timed_disturbances.push_back(timed_disturbance);
        }
      }
    );
    rewinder_.perform();

    std::tie(
      disturbance_min_,
      disturbance_max_,
      disturbance_total_,
      violations_percentage_
    ) = MetricGaussian::calculateGaussianStatistics(timed_disturbances, disturbance_threshold_, max_method_);
  }
};

} // namespace evaluation
} // namespace srpb
