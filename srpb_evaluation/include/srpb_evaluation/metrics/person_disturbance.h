#pragma once

#include "srpb_evaluation/metric_gaussian.h"

#include <angles/angles.h>
#include <srpb_logger/robot_data.h>
#include <people_msgs_utils/person.h>

namespace srpb {
namespace evaluation {

/// Related to velocity and direction of the robot movement towards person
class PersonDisturbance: public MetricGaussian {
public:
  PersonDisturbance(
    const std::vector<std::pair<double, logger::RobotData>>& robot_data,
    const std::vector<std::pair<double, people_msgs_utils::Person>>& people_data,
    double disturbance_threshold,
    double person_fov,
    bool max_method = true
  ):
    MetricGaussian(robot_data, people_data),
    disturbance_threshold_(disturbance_threshold),
    person_fov_(person_fov),
    max_method_(max_method)
  {
    compute();
  }

  /**
   * @brief Calculates value of person disturbance induced by robot motion (its direction in particular) in vicinity of people
   *
   * Disturbance is modeled by a Gaussian function. Its values are computed by arguments given in domain of angles.
   * For further details check `dirCross` concept (location of the intersection point of i and j direction rays
   * in relation to the i centre) in `hubero_local_planner`. Here, `i` is the person and `j` is the robot.
   *
   * @param x_robot
   * @param y_robot
   * @param yaw_robot
   * @param x_person
   * @param y_person
   * @param yaw_person
   * @param fov_person total angular field of view of the person
   * @return Normalized (0-1) disturbance score
   */
  double calculateDirectionDisturbance(
    double x_robot,
    double y_robot,
    double yaw_robot,
    double x_person,
    double y_person,
    double yaw_person,
    double fov_person
  ) {
    double dist_vector[2] = {0.0};
    dist_vector[0] = x_robot - x_person;
    dist_vector[1] = y_robot - y_person;

    // length of the vector
    double dist_vector_length = std::sqrt(
      std::pow(dist_vector[0], 2)
      + std::pow(dist_vector[1], 2)
    );

    // direction of vector connecting robot and person (defines where the robot is located in relation to a person [ego agent])
    double dist_vector_angle = std::atan2(dist_vector[1], dist_vector[0]);

    // relative location vector angle (defines side where the robot is located in relation to a person)
    double rel_loc_angle = angles::normalize_angle(dist_vector_angle - yaw_person);

    // old notation: alpha-beta can be mapped to: i -> robot, j -> person
    double gamma = angles::normalize_angle(rel_loc_angle - yaw_robot);

    // calculate threshold angle values, normalize angles
    /// indicates that j moves in the same direction as i
    double gamma_eq = angles::normalize_angle(dist_vector_angle - 2 * yaw_person);
    /// indicates that j moves in a direction opposite to i
    double gamma_cc = angles::normalize_angle(M_PI - 2 * yaw_person);
    /// indicates that a ray created from a centre point and a heading of j crosses the centre point of i
    double gamma_opp = angles::normalize_angle(gamma_eq - M_PI);

    /*
     * Find range of angles that indicate <opposite, crossing in front, etc> motion direction of the robot towards person
     * e.g. opposite direction adjoins with `cross behind` and `outwards` ranges.
     * Range between direction regions can be used as a variance to model gaussian cost
     */
    // decode relative location (right/left side)
    std::string relative_location_side = "unknown";
    if (rel_loc_angle < 0.0) {
      relative_location_side = "right";
    } else if (rel_loc_angle >= 0.0) {
      relative_location_side = "left";
    }

    // not all angles are required in this method, some values are computed for future use
    double gamma_cf_start = 0.0;
    double gamma_cf_finish = 0.0;
    double gamma_cb_start = 0.0;
    double gamma_cb_finish = 0.0;
    double gamma_out_start = 0.0;
    double gamma_out_finish = 0.0;

    if (relative_location_side == "right") {
      gamma_cf_start = gamma_cc;
      gamma_cf_finish = gamma_eq;
      gamma_cb_start = gamma_opp;
      gamma_cb_finish = gamma_cc;
      gamma_out_start = gamma_eq;
      gamma_out_finish = gamma_opp;
    } else if (relative_location_side == "left") {
      gamma_cf_start = gamma_eq;
      gamma_cf_finish = gamma_cc;
      gamma_cb_start = gamma_cc;
      gamma_cb_finish = gamma_opp;
      gamma_out_start = gamma_opp;
      gamma_out_finish = gamma_eq;
    } else {
      throw std::runtime_error("Unknown value of relative location");
    }

    double gamma_cf_range = std::abs(angles::shortest_angular_distance(gamma_cf_start, gamma_cf_finish));
    double gamma_cb_range = std::abs(angles::shortest_angular_distance(gamma_cb_start, gamma_cb_finish));
    double gamma_out_range = std::abs(angles::shortest_angular_distance(gamma_out_start, gamma_out_finish));

    // determine, how wide the region of, cross_center direction angles, will be (assuming circular model of the person)
    const double PERSON_MODEL_RADIUS = 0.4;
    // we must keep arcsin argument below 1.0, otherwise NAN will be returned instead of a very big angle
    double dist_gamma_range = std::max(PERSON_MODEL_RADIUS, dist_vector_length);
    double gamma_cc_range = 2.0 * std::asin(PERSON_MODEL_RADIUS / dist_gamma_range);
    // Variance is computed according 68–95–99.7 rule https://en.wikipedia.org/wiki/68%E2%80%9395%E2%80%9399.7_rule
    double gamma_cc_stddev = (gamma_cc_range / 2.0) / 3.0;
    double gamma_cc_variance = std::pow(gamma_cc_stddev, 2);

    /*
     * Note that we assume that gaussian cost of disturbance exists only within bounds of following direction angles:
     * - crossing-center (i.e. opposite and moving towards the person center)
     * - crossing-in-front
     */
    // 1D Gaussian function, note that angle domain wraps at 3.14 so we must check for maximum of gaussians
    // located at gamma_X and shifted 2 * pi to the left and right; gamma angle should already be normalized here
    double gaussian_dir_cc = social_nav_utils::calculateGaussianAngle(gamma, gamma_cc, gamma_cc_variance);

    // 3 sigma rule - let the cost spread only over the CF region
    double gamma_cf_stddev = (gamma_cf_range / 2.0) / 3.0;
    double gamma_cf_variance = std::pow(gamma_cf_stddev, 2);
    // mean - center of the cross front region
    double gamma_cf_center = angles::normalize_angle(gamma_cf_start + gamma_cf_range / 2.0);
    double gaussian_dir_cf = social_nav_utils::calculateGaussianAngle(gamma, gamma_cf_center, gamma_cf_variance);

    double gaussian_dir_result = std::max(gaussian_dir_cc, gaussian_dir_cf);

    // check whether the robot is located within person's FOV (only then affects human's behaviour);
    // again, 3 sigma rule is used here -> 3 sigma rule applied to the half of the FOV
    double fov_stddev = (fov_person / 2.0) / 3.0;
    double variance_fov = std::pow(fov_stddev, 2);
    // starting from the left side, half of the `fov_person` is located in 0.0 and rel_loc is 0.0
    // when obstacle is in front of the object
    double gaussian_fov = social_nav_utils::calculateGaussian(rel_loc_angle, 0.0, variance_fov);

    return gaussian_dir_result * gaussian_fov;
  }

  void printResults() const override {
    printf(
    "Person disturbance = %.4f [%%] (min = %.4f [%%], max = %.4f [%%], violations %3u)\n",
      disturbance_total_,
      disturbance_min_,
      disturbance_max_,
      violations_num_
    );
  }

protected:
  // parameters
  double disturbance_threshold_;
  double person_fov_;
  bool max_method_;

  // results
  double disturbance_min_;
  double disturbance_max_;
  double disturbance_total_;
  unsigned int violations_num_;

  void compute() override {
    // store durations and disturbance indices of the robot in terms of nearby people
    std::vector<std::pair<double, std::vector<double>>> timed_disturbances;

    /// prepare container for gaussian values in this `for` iteration
    std::pair<double, std::vector<double>> timed_disturbance;

    rewinder_.setHandlerNextTimestamp(
      [&]() {
        if (rewinder_.getTimestampCurr() == rewinder_.getTimestampLast()) {
          return;
        }
        timed_disturbance = std::make_pair(rewinder_.getTimestampNext() - rewinder_.getTimestampCurr(), std::vector<double>());
      }
    );

    rewinder_.setHandlerNextPersonTimestamp(
      [&]() {
        // processing
        double disturbance = calculateDirectionDisturbance(
          rewinder_.getRobotCurr().getPositionX(),
          rewinder_.getRobotCurr().getPositionY(),
          rewinder_.getRobotCurr().getOrientationYaw(),
          rewinder_.getPersonCurr().getPositionX(),
          rewinder_.getPersonCurr().getPositionY(),
          rewinder_.getPersonCurr().getOrientationYaw(),
          person_fov_
        );

        // check if robot faces person but only rotates or is moving fast
        double speed_factor = std::sqrt(
          std::pow(rewinder_.getRobotCurr().getVelocityX(), 2)
          + std::pow(rewinder_.getRobotCurr().getVelocityY(), 2)
        );

        // check how far the robot is from the person
        double eucl_dist = std::sqrt(
          std::pow(rewinder_.getRobotCurr().getPositionX() - rewinder_.getPersonCurr().getPositionX(), 2)
          + std::pow(rewinder_.getRobotCurr().getPositionY() - rewinder_.getPersonCurr().getPositionY(), 2)
        );
        const double DIST_FACTOR_EXP = -0.8; // exponent provides approx 0.5 @ 1 m between centers of robot and person
        double dist_factor = std::exp(DIST_FACTOR_EXP * eucl_dist);

        // count in factors above
        disturbance *= (speed_factor * dist_factor);

        // count in the person's tracking reliability - we want to avoid penalizing robot that investigates `old` tracks
        double cost = disturbance * rewinder_.getPersonCurr().getReliability();
        timed_disturbance.second.push_back(cost);
      }
    );

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
      violations_num_
    ) = MetricGaussian::calculateGaussianStatistics(timed_disturbances, disturbance_threshold_, max_method_);
  }
};

} // namespace evaluation
} // namespace srpb
