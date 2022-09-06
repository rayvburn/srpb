#include <cstdio>
#include <cstdlib>
#include <vector>
#include <cmath>

#include <exception>
#include <fstream>
#include <numeric>
#include <string_view>
#include <tuple>

#include <move_base/robot_logger.h>
#include <move_base/people_logger.h>

#include <angles/angles.h>

double computeComputationalEfficiency(const std::vector<std::pair<double, RobotData>>& data_log) {
  size_t N = data_log.size();
  double mean = 0.0;

  for (const auto& data: data_log) {
    mean += data.second.getLocalPlanningTime();
  }

  mean /= N;
  return mean;
}

double computeMotionEfficiency(const std::vector<std::pair<double, RobotData>>& data_log) {
  double timestamp_start = data_log.front().first;
  double timestamp_end = data_log.back().first;
  return timestamp_end - timestamp_start;
}

double computeVelocitySmoothness(const std::vector<std::pair<double, RobotData>>& data_log) {
  size_t N = data_log.size();
  double mean = 0.0;

  for (auto it = data_log.cbegin() + 1; it < data_log.cend(); it++) {
    auto prev = std::prev(it);
    double dt = it->first - prev->first;
    double acc = std::abs(it->second.getVelocityX() - prev->second.getVelocityX()) / dt;
    mean += acc;
  }

  mean /= (N - 1);
  return mean;
}

double computeSafety(const std::vector<std::pair<double, RobotData>>& data_log, const double& safety_distance) {
  // timestamp difference
  double time_diff = data_log.back().first - data_log.front().first;

  // timestamps to sum up time
  double TS_NOT_SET = std::numeric_limits<double>::min();
  double ts_start = TS_NOT_SET;
  double ts_end = TS_NOT_SET;

  double sum = 0.0;

  for (const auto& data: data_log) {
    // compute how long robot traveled near obstacles located within safety distance
    if (data.second.getDistToObstacle() < safety_distance) {
      if (ts_start == TS_NOT_SET) {
        ts_start = data.first;
      } else {
        // save timestamp
        ts_end = data.first;
      }
    } else {
      if (ts_end != TS_NOT_SET) {
        // robot started to maintain safety distance again
        sum += (ts_end - ts_start);

        // reset timestamps for incoming counts
        ts_start = TS_NOT_SET;
        ts_end = TS_NOT_SET;
      }
    }
  }

  // sum up the last period (if robot finished course not maintaining the safety distance)
  if (ts_start != TS_NOT_SET && ts_end != TS_NOT_SET) {
    sum += ts_end - ts_start;
  }

  // compute percentage
  return sum / time_diff;
}

double computePathLinearLength(const std::vector<std::pair<double, RobotData>>& data_log) {
  double path_length = 0.0;

  for (auto it = data_log.cbegin() + 1; it < data_log.cend(); it++) {
    auto prev = std::prev(it);
    double dx = std::abs(it->second.getPositionX() - prev->second.getPositionX());
    double dy = std::abs(it->second.getPositionY() - prev->second.getPositionY());
    path_length += std::sqrt(dx * dx + dy * dy);
  }

  return path_length;
}

double computePathRotationalLength(const std::vector<std::pair<double, RobotData>>& data_log) {
  double path_length = 0.0;

  for (auto it = data_log.cbegin() + 1; it < data_log.cend(); it++) {
    auto prev = std::prev(it);
    path_length += std::abs(it->second.getOrientationYaw() - prev->second.getOrientationYaw());
  }

  return path_length;
}

double computeCumulativeHeadingChanges(const std::vector<std::pair<double, RobotData>>& data_log) {
  double chc = 0.0;

  for (auto it = data_log.cbegin() + 1; it < data_log.cend(); it++) {
    auto prev = std::prev(it);
    double dt = it->first - prev->first;
    // pose data used to ignore motion execution errors (velocity-based calculations)
    chc += (std::abs(it->second.getOrientationYaw() - prev->second.getOrientationYaw()) / dt);
  }

  return chc;
}

double computeBackwardMovements(const std::vector<std::pair<double, RobotData>>& data_log) {
  double bwd = 0.0;

  for (const auto& data: data_log) {
    if (data.second.getVelocityX() < 0.0) {
      bwd += std::abs(data.second.getVelocityX());
    }
  }

  return bwd;
}

double computeOscillations(const std::vector<std::pair<double, RobotData>>& data_log, double osc_lin_threshold, double osc_ang_threshold) {
  double N = static_cast<double>(data_log.size());
  double osc = 0.0;

  for (auto it = data_log.cbegin() + 1; it < data_log.cend(); it++) {
    auto prev = std::prev(it);
    bool robot_stopped = it->second.getVelocityX() < osc_lin_threshold && it->second.getVelocityX() >= 0.0;
    bool robot_oscillates = std::abs(it->second.getVelocityTheta()) < osc_ang_threshold;
    if (robot_stopped && robot_oscillates) {
      double dt = it->first - prev->first;
      osc += (std::abs(it->second.getVelocityTheta()) / dt);
    }
  }

  return osc;
}

double computeInPlaceRotations(const std::vector<std::pair<double, RobotData>>& data_log, double osc_lin_threshold) {
  double N = static_cast<double>(data_log.size());
  double inplace_rot = 0.0;

  for (auto it = data_log.cbegin() + 1; it < data_log.cend(); it++) {
    auto prev = std::prev(it);
    bool robot_stopped = it->second.getVelocityX() < osc_lin_threshold && it->second.getVelocityX() >= 0.0;
    bool robot_rotates = std::abs(it->second.getVelocityTheta()) >= 0.0;
    if (robot_stopped && robot_rotates) {
      double dt = it->first - prev->first;
      inplace_rot += (std::abs(it->second.getVelocityTheta()) / dt);
    }
  }

  return inplace_rot;
}

/// Computes value of 1D Gaussian probability density function
/// @url https://en.wikipedia.org/wiki/Gaussian_function
double calculateGaussian(double x, double mean, double variance, bool normalize = false) {
  double scale = 1.0;
  // with normalization, maximum possible value will be 1.0; otherwise, it depends on the value of variance
  if (!normalize) {
    scale = 1.0 / (std::sqrt(variance) * std::sqrt(2 * M_PI));
  }
  return scale * std::exp(-std::pow(x - mean, 2) / (2.0 * variance));
}

/// Computes value of 1D Gaussian PDF but includes wrapped regions of bell curve (shifted -2pi and +2pi)
double calculateGaussianAngle(double x, double mean, double variance, bool normalize = false) {
  double gaussian1 = calculateGaussian(x, mean             , variance, normalize);
  double gaussian2 = calculateGaussian(x, mean - 2.0 * M_PI, variance, normalize);
  double gaussian3 = calculateGaussian(x, mean + 2.0 * M_PI, variance, normalize);
  return std::max(std::max(gaussian1, gaussian2), gaussian3);
}

/// Reference: Algorithm A.1 from Kirby, 2010 PhD thesis "Social Robot Navigation" (p. 166)
/// @url https://www.ri.cmu.edu/pub_files/2010/5/rk_thesis.pdf
double calculateGaussian(
  double x,
  double y,
  double x_center,
  double y_center,
  double yaw,
  double sigma_h,
  double sigma_r,
  double sigma_s
) {
  double alpha = std::atan2(y - y_center, x - x_center) - yaw + M_PI_2;
  alpha = angles::normalize_angle(alpha);
  double sigma = (alpha <= 0.0 ? sigma_r : sigma_h);

  // save values used multiple times in computations;
  // squared cosine/sine of theta (yaw angle)
  double cos_yaw_sq = std::pow(std::cos(yaw), 2);
  double sin_yaw_sq = std::pow(std::sin(yaw), 2);
  double sin_2yaw = std::sin(2.0 * yaw);
  double sigma_sq = std::pow(sigma, 2);
  double sigma_s_sq = std::pow(sigma_s, 2);

  double a = cos_yaw_sq / (2.0 * sigma_sq) + sin_yaw_sq / (2.0 * sigma_s_sq);
  double b = sin_2yaw   / (4.0 * sigma_sq) - sin_2yaw   / (4.0 * sigma_s_sq);
  double c = sin_yaw_sq / (2.0 * sigma_sq) + cos_yaw_sq / (2.0 * sigma_s_sq);

  double exp_arg_a = a * (std::pow(x - x_center, 2));
  double exp_arg_b = 2.0 * b * (x - x_center) * (y - y_center);
  double exp_arg_c = c * std::pow(y - y_center, 2);
  return std::exp(-(exp_arg_a + exp_arg_b + exp_arg_c));
}

/// Calculates radius of the F-formation based on positions of group members and group's center of gravity
double calculateRadiusFformation(
  const people_msgs_utils::Group& group,
  const std::vector<people_msgs_utils::Person>& people_group
) {
  double radius = 0.0;
  for (const auto& person: people_group) {
    double dist_from_cog = std::sqrt(
      std::pow(person.getPositionX() - group.getCenterOfGravity().x, 2)
      + std::pow(person.getPositionY() - group.getCenterOfGravity().y, 2)
    );
    // select the maximum value as radius
    if (radius < dist_from_cog) {
      radius = dist_from_cog;
    }
  }
  return radius;
}

/**
 * @brief Computes min, max, normalized metrics related to gaussian statistics and counts number of space violations
 *
 * @param timed_gaussians vector of pairs with, first, timestamp, and second, values of gaussians
 * @param space_violation_threshold Gaussian values bigger than that will be considered as violation of personal space
 * @param max_method set to true (default) to use max element from Gaussians to normalize metrics;
 * false means averaging over all Gaussian occurrences in a current time step
 *
 * @return std::tuple<double, double, double, unsigned int> tuple with scores: min, max and normalized to execution
 * time and number of personal space violations (according to @ref personal_space_threshold)
 */
std::tuple<double, double, double, unsigned int> calculateGaussianStatistics(
  std::vector<std::pair<double, std::vector<double>>> timed_gaussians,
  double space_violation_threshold,
  bool max_method = true
) {
  // find actual duration
  double duration = 0.0;
  for (const auto& tg: timed_gaussians) {
    duration += tg.first;
  }

  // find number of personal space / f-formation space violations
  unsigned int space_violations = 0;

  // find gaussians and recompute according to recognized people (max/sum)
  double metrics = 0.0;
  double min_elem = std::numeric_limits<double>::max();
  double max_elem = std::numeric_limits<double>::min();

  // rollout gaussians and compute score (metrics)
  for (const auto& tg: timed_gaussians) { // tg in fact might be const but ac
    double dt = tg.first;
    if (tg.second.empty()) {
      std::cout << "Gaussian(s) is empty for at least 1 sample. Metrics will be 0.0" << std::endl;
      return std::make_tuple(0.0, 0.0, 0.0, 0);
    }

    space_violations += std::count_if(
      tg.second.cbegin(),
      tg.second.cend(),
      [&](double g) {
        return g > space_violation_threshold;
      }
    );

    // overall min and max computation
    double local_min_elem = *std::min_element(tg.second.cbegin(), tg.second.cend());
    if (local_min_elem < min_elem) {
      min_elem = local_min_elem;
    }

    double local_max_elem = *std::max_element(tg.second.cbegin(), tg.second.cend());
    if (local_max_elem > max_elem) {
      max_elem = local_max_elem;
    }

    // check for selected method of normalization
    double metrics_elem = 0.0;
    if (max_method) {
      // max method used here for normalization
      metrics_elem = local_max_elem;
    } else {
      // average used for normalization
      metrics_elem = std::accumulate(tg.second.cbegin(), tg.second.cend(), 0.0) / static_cast<double>(tg.second.size());
    }
    metrics += (local_max_elem * (dt / duration));
  }

  return std::make_tuple(min_elem, max_elem, metrics, space_violations);
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
  double gaussian_dir_cc = calculateGaussianAngle(gamma, gamma_cc, gamma_cc_variance, true);

  // 3 sigma rule - let the cost spread only over the CF region
  double gamma_cf_stddev = (gamma_cf_range / 2.0) / 3.0;
  double gamma_cf_variance = std::pow(gamma_cf_stddev, 2);
  // mean - center of the cross front region
  double gamma_cf_center = angles::normalize_angle(gamma_cf_start + gamma_cf_range / 2.0);
  double gaussian_dir_cf = calculateGaussianAngle(gamma, gamma_cf_center, gamma_cf_variance, true);

  double gaussian_dir_result = std::max(gaussian_dir_cc, gaussian_dir_cf);

  // check whether the robot is located within person's FOV (only then affects human's behaviour);
  // again, 3 sigma rule is used here -> 3 sigma rule applied to the half of the FOV
  double fov_stddev = (fov_person / 2.0) / 3.0;
  double variance_fov = std::pow(fov_stddev, 2);
  // starting from the left side, half of the `fov_person` is located in 0.0 and rel_loc is 0.0
  // when obstacle is in front of the object
  double gaussian_fov = calculateGaussian(rel_loc_angle, 0.0, variance_fov);

  return gaussian_dir_result * gaussian_fov;
}

/**
 * @brief Computes people personal space intrusion score
 *
 * People have their constrained area attached to the body projection. Once robot moves around people it may come
 * too closer or further. In each time step a value of a person area (modelled by an Asymmetric Gaussian) is computed.
 * This function returns scores: min Gaussian, max Gaussian and normalized to execution time. Maximum of Gaussian,
 * which is 1.0, shows that robot was located in the same position as person over the whole experiment. On the other
 * hand, normalized value of 0.0 means that robot never moved close to any person (according to the given variances).
 *
 * @param robot_data
 * @param people_data
 * @param sigma_h variance to the heading direction of the person (Gaussian)
 * @param sigma_r variance to the rear (Gaussian)
 * @param sigma_s variance to the side (Gaussian)
 * @param personal_space_threshold Gaussian values bigger than that will be considered as violation of personal space
 * @param max_method set to true (default) to use max element from Gaussians to normalize metrics;
 * false means averaging over all Gaussian occurrences in a current time step
 *
 * @return std::tuple<double, double, double, unsigned int> tuple with scores: min, max and normalized to execution
 * time and number of personal space violations (according to @ref personal_space_threshold)
 *
 * The closest to our method is the approach presented by Truong and Ngo in
 * “To Approach Humans?”: A Unified Framework for Approaching Pose Prediction and Socially Aware Robot Navigation
 * They called it `Social Individual Index`. Their method lacks normalization in terms of path duration.
 */
std::tuple<double, double, double, unsigned int> computePersonalSpaceIntrusion(
  const std::vector<std::pair<double, RobotData>>& robot_data,
  const std::vector<std::pair<double, people_msgs_utils::Person>>& people_data,
  double sigma_h,
  double sigma_r,
  double sigma_s,
  double personal_space_threshold,
  bool max_method = true
) {
  if (robot_data.size() < 2) {
    std::cout << "Robot data size is too small, at least 2 samples are required due to time step difference calculations" << std::endl;
    return std::make_tuple(0.0, 0.0, 0.0, 0);
  }

  // store durations and Gaussians of the robot in terms of nearby people
  std::vector<std::pair<double, std::vector<double>>> timed_gaussians;

  /// match people detections to logged data of the robot
  // iterator to investigate people data
  std::vector<std::pair<double, people_msgs_utils::Person>>::const_iterator it_ppl(people_data.begin());

  // iterator for robot data (e.g. poses)
  // pose of robot in each time step must be investigated against pose of each person
  for (
    std::vector<std::pair<double, RobotData>>::const_iterator it_robot(robot_data.begin());
    it_robot != robot_data.end();
    it_robot++
  ) {
    /// prepare container for gaussian values in this `for` iteration
    std::pair<double, std::vector<double>> timed_gaussian;
    // check if this is the last element of the robot data
    if (std::next(it_robot) == robot_data.end()) {
      // heuristic to compute last time stamp difference (prediction by extrapolation)
      double last_ts_diff = it_robot->first - std::prev(it_robot)->first;
      timed_gaussian = std::make_pair(last_ts_diff, std::vector<double>());
    } else {
      double ts_diff = std::next(it_robot)->first - it_robot->first;
      timed_gaussian = std::make_pair(ts_diff, std::vector<double>());
    }

    // iterate over all people recognized in a given time step
    for (/*initialized above*/; it_ppl != people_data.end(); it_ppl++) {
      // terminal conditions
      bool all_people_checked_timestep = it_robot->first != it_ppl->first;
      bool last_sample_being_checked = std::next(it_robot) == robot_data.end() && std::next(it_ppl) == people_data.end();

      // processing
      double gaussian = calculateGaussian(
        it_robot->second.getPositionX(),
        it_robot->second.getPositionY(),
        it_ppl->second.getPositionX(),
        it_ppl->second.getPositionY(),
        it_ppl->second.getOrientationYaw(),
        sigma_h,
        sigma_r,
        sigma_s
      );
      timed_gaussian.second.push_back(gaussian);

      if (all_people_checked_timestep || last_sample_being_checked) {
        timed_gaussians.push_back(timed_gaussian);
        break;
      }
    } // iterating over people log entries
  } // iteration over robot log entries

  return calculateGaussianStatistics(timed_gaussians, personal_space_threshold, max_method);
}

/// Similar to personal space intrusion but related to the group space
std::tuple<double, double, double, unsigned int> computeGroupsSpaceIntrusion(
  const std::vector<std::pair<double, RobotData>>& robot_data,
  const std::vector<std::pair<double, people_msgs_utils::Person>>& people_data,
  const std::vector<std::pair<double, people_msgs_utils::Group>>& groups_data,
  double group_space_threshold,
  bool max_method = true
) {
  if (robot_data.size() < 2) {
    std::cout << "Robot data size is too small, at least 2 samples are required due to time step difference calculations" << std::endl;
    return std::make_tuple(0.0, 0.0, 0.0, 0);
  }

  // store durations and Gaussians of the robot in terms of nearby groups
  std::vector<std::pair<double, std::vector<double>>> timed_gaussians;

  /// match people detections to logged data of the robot
  // iterator to investigate people data
  std::vector<std::pair<double, people_msgs_utils::Person>>::const_iterator it_ppl(people_data.begin());
  // iterator to investigate groups data
  std::vector<std::pair<double, people_msgs_utils::Group>>::const_iterator it_grp(groups_data.begin());

  // iterator for robot data (e.g. poses)
  // pose of robot in each time step must be investigated against pose of each person
  for (
    std::vector<std::pair<double, RobotData>>::const_iterator it_robot(robot_data.begin());
    it_robot != robot_data.end();
    it_robot++
  ) {
    /// prepare container for gaussian values in this `for` iteration
    std::pair<double, std::vector<double>> timed_gaussian;
    // check if this is the last element of the robot data
    if (std::next(it_robot) == robot_data.end()) {
      // heuristic to compute last time stamp difference (prediction by extrapolation)
      double last_ts_diff = it_robot->first - std::prev(it_robot)->first;
      timed_gaussian = std::make_pair(last_ts_diff, std::vector<double>());
    } else {
      double ts_diff = std::next(it_robot)->first - it_robot->first;
      timed_gaussian = std::make_pair(ts_diff, std::vector<double>());
    }

    // iterate over all groups recognized in a given time step
    for (/*initialized above*/; it_grp != groups_data.end(); it_grp++) {
      bool all_groups_checked_timestep = it_robot->first != it_grp->first;
      if (all_groups_checked_timestep) {
        // extend gaussians only if the gaussian for a group is properly defined, i.e., group is not empty
        if (!timed_gaussian.second.empty()) {
          timed_gaussians.push_back(timed_gaussian);
        }
        break;
      }

      // collect people data into container that store group members
      std::vector<people_msgs_utils::Person> people_this_group;

      // save people iterator to restore it in case of 1 person is assigned to multiple groups (must iterate over all people recognized at a specific time step)
      auto it_ppl_curr_timestamp = it_ppl;

      // iterate over all people recognized in a given time step
      for (/*initialized above*/; it_ppl != people_data.end(); it_ppl++) {
        // break if timestamp does not match group's one
        bool all_people_checked_timestep = it_grp->first != it_ppl->first;
        bool people_it_lagging_behind = it_ppl->first < it_grp->first;
        bool checking_last_group = std::next(it_grp) == groups_data.end();
        bool checking_last_person = std::next(it_ppl) == people_data.end();
        bool last_sample_being_checked = checking_last_group && checking_last_person;

        if (people_it_lagging_behind && !last_sample_being_checked) {
          // if people timestamp hasn't progressed as the group's one, let's skip people entries w/o groups
          while (it_ppl->first < it_grp->first) {
            it_ppl++;
          }
        } else if (all_people_checked_timestep || last_sample_being_checked) {
          break;
        }

        // add person data to the container of group members
        auto group_members = it_grp->second.getTrackIDs();
        if (std::find(group_members.begin(), group_members.end(), it_ppl->second.getID()) != group_members.end()) {
          people_this_group.push_back(it_ppl->second);
        }

      } // iterating over people log entries

      // terminal conditions, based on them, action is taken at the end of the iteration
      bool checking_last_robot = std::next(it_robot) == robot_data.end();
      bool checking_last_group = std::next(it_grp) == groups_data.end();
      bool checking_last_person = std::next(it_ppl) == people_data.end();
      bool last_sample_being_checked = checking_last_robot && checking_last_group && checking_last_person;

      /// computations for the group
      double fformation_radius = calculateRadiusFformation(it_grp->second, people_this_group);

      /*
       * Compute `cost` of the robot being located in the current position - how it affects group's ease.
       * NOTE that we do not possess the orientation of the group (yaw is set to 0)
       * so the circular model is assumed (sigmas are equal)
       *
       * Conversion from radius to variance was mentioned, e.g., by Truong in `To Approach Humans? (..)` article (2017)
       */
      double gaussian = 0.0;
      // perception can report 1 person in a group - do not investigate such situations further
      if (people_this_group.size() > 1) {
        gaussian = calculateGaussian(
          it_robot->second.getPositionX(),
          it_robot->second.getPositionY(),
          it_grp->second.getCenterOfGravity().x,
          it_grp->second.getCenterOfGravity().y,
          0.0,
          fformation_radius / 2.0,
          fformation_radius / 2.0,
          fformation_radius / 2.0
        );
      }

      // Gaussian cost of the robot being located in the current pose; cost related to the investigated group of people
      timed_gaussian.second.push_back(gaussian);

      if (all_groups_checked_timestep || last_sample_being_checked || checking_last_group) {
        // extend gaussians only if the gaussian for a group is properly defined, i.e., group is not empty
        if (!timed_gaussian.second.empty()) {
          timed_gaussians.push_back(timed_gaussian);
        }
        break;
      }

      // safely finished group computations, let's restore people iterator for this time step -
      // only if needed for the next group
      bool last_group_in_dataset = std::next(it_grp) == groups_data.end();
      bool next_group_has_same_timestamp = std::next(it_grp)->first == it_grp->first;
      if (!last_group_in_dataset && next_group_has_same_timestamp) {
        it_ppl = it_ppl_curr_timestamp;
      }

    } // iterating over groups log entries

  } // iteration over robot log entries
  return calculateGaussianStatistics(timed_gaussians, group_space_threshold, max_method);
}

/// Related to velocity and direction of the robot movement towards person
std::tuple<double, double, double, unsigned int> computePersonDisturbance(
  const std::vector<std::pair<double, RobotData>>& robot_data,
  const std::vector<std::pair<double, people_msgs_utils::Person>>& people_data,
  double disturbance_threshold,
  double person_fov,
  bool max_method = true
) {
  if (robot_data.size() < 2) {
    std::cout << "Robot data size is too small, at least 2 samples are required due to time step difference calculations" << std::endl;
    // return 0.0;
    return std::make_tuple(0.0, 0.0, 0.0, 0);
  }

  // store durations and disturbance indices of the robot in terms of nearby people
  std::vector<std::pair<double, std::vector<double>>> timed_disturbances;

  /// match people detections to logged data of the robot
  // iterator to investigate people data
  std::vector<std::pair<double, people_msgs_utils::Person>>::const_iterator it_ppl(people_data.begin());

  // iterator for robot data (e.g. poses)
  // pose of robot in each time step must be investigated against pose of each person
  for (
    std::vector<std::pair<double, RobotData>>::const_iterator it_robot(robot_data.begin());
    it_robot != robot_data.end();
    it_robot++
  ) {
    /// prepare container for gaussian values in this `for` iteration
    std::pair<double, std::vector<double>> timed_disturbance;
    // check if this is the last element of the robot data
    if (std::next(it_robot) == robot_data.end()) {
      // heuristic to compute last time stamp difference (prediction by extrapolation)
      double last_ts_diff = it_robot->first - std::prev(it_robot)->first;
      timed_disturbance = std::make_pair(last_ts_diff, std::vector<double>());
    } else {
      double ts_diff = std::next(it_robot)->first - it_robot->first;
      timed_disturbance = std::make_pair(ts_diff, std::vector<double>());
    }

    // iterate over all people recognized in a given time step
    for (/*initialized above*/; it_ppl != people_data.end(); it_ppl++) {
      // terminal conditions
      bool all_people_checked_timestep = it_robot->first != it_ppl->first;
      bool last_sample_being_checked = std::next(it_robot) == robot_data.end() && std::next(it_ppl) == people_data.end();

      // processing
      double disturbance = calculateDirectionDisturbance(
        it_robot->second.getPositionX(),
        it_robot->second.getPositionY(),
        it_robot->second.getOrientationYaw(),
        it_ppl->second.getPositionX(),
        it_ppl->second.getPositionY(),
        it_ppl->second.getOrientationYaw(),
        person_fov
      );

      // check if robot faces person but only rotates or is moving fast
      double speed_factor = std::sqrt(
        std::pow(it_robot->second.getVelocityX(), 2)
        + std::pow(it_robot->second.getVelocityY(), 2)
      );

      // check how far the robot is from the person
      double eucl_dist = std::sqrt(
        std::pow(it_robot->second.getPositionX() - it_ppl->second.getPositionX(), 2)
        + std::pow(it_robot->second.getPositionY() - it_ppl->second.getPositionY(), 2)
      );
      const double DIST_FACTOR_EXP = -0.8; // exponent provides approx 0.5 @ 1 m between centers of robot and person
      double dist_factor = std::exp(DIST_FACTOR_EXP * eucl_dist);

      // count in factors above
      disturbance *= (speed_factor * dist_factor);

      timed_disturbance.second.push_back(disturbance);

      if (all_people_checked_timestep || last_sample_being_checked) {
        timed_disturbances.push_back(timed_disturbance);
        break;
      }
    } // iterating over people log entries
  } // iteration over robot log entries

  return calculateGaussianStatistics(timed_disturbances, disturbance_threshold, max_method);
}

// string to pair (first = timestamp, second = logged state)
template <typename T>
std::vector<std::pair<double, T>> parseFile(const std::string& filepath, std::function<T(const std::string&)> from_string_fun) {
  std::ifstream file(filepath);
  if (file.bad()) {
      throw std::runtime_error("Could not open the log file");
  }

  std::vector<std::pair<double, T>> data_log;
  std::string line;
  // parse lines 1 by 1
  while (std::getline(file, line) && !file.bad()) {
      if (line.empty()) {
        std::cout << "File contains an empty line, it may be damaged!" << std::endl;
        continue;
      }

      // extract timestamp
      auto ts_start = line.find_first_not_of(' ');
      if (ts_start == std::string::npos) {
        continue;
      }
      std::string timestamp_str = line.substr(ts_start);
      line = line.substr(ts_start);
      auto ts_end = line.find_first_of(' ');
      if (ts_end == std::string::npos) {
        continue;
      }
      timestamp_str = timestamp_str.substr(0, ts_end);

      // extract logged state
      data_log.push_back(
        std::make_pair(
          std::stod(timestamp_str),
          from_string_fun(line.substr(ts_end + 1))
        )
      );
  }

  file.close();
  return data_log;
}

int main(int argc, char* argv[]) {
  if (argc != 5) {
    printf(
      "Please input\r\n"
      "\t* the path to the log file of the robot\r\n"
      "\t* the path to the log file of the people\r\n"
      "\t* the path to the log file of the people groups\r\n"
      "\t* and value of the safety distance [m].\r\n"
    );
    return 1;
  }

  auto file_robot = std::string(argv[1]);
  auto file_people = std::string(argv[2]);
  auto file_groups = std::string(argv[3]);
  auto safety_distance = std::atof(argv[4]);

  // oscillation threshold values
  double osc_vel_lin_x_threshold = 0.05;
  double osc_vel_ang_z_threshold = 0.15;

  // personal space Gaussian model parameters
  // values from Kirby, 2010, Fig. A.1
  double sigma_h = 2.00;
  double sigma_r = 1.00;
  double sigma_s = 1.33;
  // threshold of Gaussian value to detect space violations
  double personal_space_threshold = 0.55;
  double group_space_threshold = 0.55;
  // estimated field of view of people
  double person_fov = angles::from_degrees(190.0);
  // threshold of Gaussian value to detect significant disturbance caused by robot location or motion direction
  double disturbance_threshold = 0.20;

  auto timed_robot_data = parseFile<RobotData>(file_robot, &RobotLogger::robotFromString);
  auto timed_people_data = parseFile<people_msgs_utils::Person>(file_people, &PeopleLogger::personFromString);
  auto timed_groups_data = parseFile<people_msgs_utils::Group>(file_groups, &PeopleLogger::groupFromString);

  double safety = computeSafety(timed_robot_data, safety_distance);
  printf("Safety = %.2f[%%]\n", safety * 1e2);

  double motion_efficiency = computeMotionEfficiency(timed_robot_data);
  printf("Motion efficiency = %.2f[secs]\n", motion_efficiency);

  double computational_efficiency = computeComputationalEfficiency(timed_robot_data);
  printf("Computational efficiency = %.2f[msecs]\n", computational_efficiency * 1e3);

  double velocity_smoothness = computeVelocitySmoothness(timed_robot_data);
  printf("Velocity smoothness = %.3f[m/s^2]\n", velocity_smoothness);

  double path_length_linear = computePathLinearLength(timed_robot_data);
  printf("Path linear length = %.3f[m]\n", path_length_linear);

  double path_length_rotational = computePathRotationalLength(timed_robot_data);
  printf("Path rotational length = %.3f[rad]\n", path_length_rotational);

  double chc = computeCumulativeHeadingChanges(timed_robot_data);
  printf("Cumulative Heading Changes = %.3f[rad]\n", chc);

  double bwd_movements = computeBackwardMovements(timed_robot_data);
  printf("Backward movements = %.3f[m]\n", bwd_movements);

  double oscillations = computeOscillations(timed_robot_data, osc_vel_lin_x_threshold, osc_vel_ang_z_threshold);
  printf("Oscillations = %.3f[rad]\n", oscillations);

  double in_place_rotations = computeInPlaceRotations(timed_robot_data, osc_vel_lin_x_threshold);
  printf("In-place rotations = %.3f[rad]\n", in_place_rotations);

  double personal_space_intrusion_min = 0.0;
  double personal_space_intrusion_max = 0.0;
  double personal_space_intrusion = 0.0;
  unsigned int personal_space_violations = 0;
  std::tie(
    personal_space_intrusion_min,
    personal_space_intrusion_max,
    personal_space_intrusion,
    personal_space_violations
  ) = computePersonalSpaceIntrusion(
    timed_robot_data,
    timed_people_data,
    sigma_h,
    sigma_r,
    sigma_s,
    personal_space_threshold
  );
  printf(
    "Personal space intrusion = %.3f[%%] (min %.3f[%%], max %.3f[%%], violations %3u)\n",
    personal_space_intrusion,
    personal_space_intrusion_min,
    personal_space_intrusion_max,
    personal_space_violations
  );

  double group_space_intrusion_min = 0.0;
  double group_space_intrusion_max = 0.0;
  double group_space_intrusion = 0.0;
  unsigned int group_space_violations = 0;
  std::tie(
    group_space_intrusion_min,
    group_space_intrusion_max,
    group_space_intrusion,
    group_space_violations
  ) = computeGroupsSpaceIntrusion(
    timed_robot_data,
    timed_people_data,
    timed_groups_data,
    group_space_threshold
  );
  printf(
    "Group space intrusion = %.3f[%%] (min %.3f[%%], max %.3f[%%], violations %3u)\n",
    group_space_intrusion,
    group_space_intrusion_min,
    group_space_intrusion_max,
    group_space_violations
  );

  // e.g. approaching person in a straight line, causing person to stop or avoid collision
  double person_disturbance_min = 0.0;
  double person_disturbance_max = 0.0;
  double person_disturbance = 0.0;
  unsigned int person_disturbance_violations = 0;
  std::tie(
    person_disturbance_min,
    person_disturbance_max,
    person_disturbance,
    person_disturbance_violations
  ) = computePersonDisturbance(
    timed_robot_data,
    timed_people_data,
    disturbance_threshold,
    person_fov
  );
  printf(
    "Person disturbance = %.3f[%%] (min %.3f[%%], max %.3f[%%], violations %3u)\n",
    person_disturbance,
    person_disturbance_min,
    person_disturbance_max,
    person_disturbance_violations
  );

  return 0;
}