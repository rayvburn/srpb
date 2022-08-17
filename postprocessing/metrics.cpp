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
 * @param max_method set to true (default) to use max element from Gaussians to normalize metrics;
 * false means averaging over all Gaussian occurrences in a current time step
 *
 * @return std::tuple<double, double, double> tuple with scores: min, max and normalized to execution time
 */
std::tuple<double, double, double> computePersonalSpaceIntrusion(
  const std::vector<std::pair<double, RobotData>>& robot_data,
  const std::vector<std::pair<double, people_msgs_utils::Person>>& people_data,
  double sigma_h,
  double sigma_r,
  double sigma_s,
  bool max_method = true
) {
  if (robot_data.size() < 2) {
    std::cout << "Robot data size is too small, at least 2 samples are required due to time step difference calculations" << std::endl;
    return std::make_tuple(0.0, 0.0, 0.0);
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

  // find actual duration
  double duration = 0.0;
  for (const auto& tg: timed_gaussians) {
    duration += tg.first;
  }

  // find gaussians and recompute according to recognized people (max/sum)
  double metrics = 0.0;
  double min_elem = std::numeric_limits<double>::max();
  double max_elem = std::numeric_limits<double>::min();

  // rollout gaussians and compute score (metrics)
  for (const auto& tg: timed_gaussians) { // tg in fact might be const but ac
    double dt = tg.first;
    if (tg.second.empty()) {
      std::cout << "Gaussian(s) is empty for at least 1 sample. Personal space intrusion metrics will be 0.0" << std::endl;
      return std::make_tuple(0.0, 0.0, 0.0);
    }
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
  return std::make_tuple(min_elem, max_elem, metrics);
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
  std::tie(
    personal_space_intrusion_min,
    personal_space_intrusion_max,
    personal_space_intrusion
  ) = computePersonalSpaceIntrusion(
    timed_robot_data,
    timed_people_data,
    sigma_h,
    sigma_r,
    sigma_s
  );
  printf(
    "Personal space intrusion = %.3f[%%] (min %.3f[%%], max %.3f[%%])\n",
    personal_space_intrusion,
    personal_space_intrusion_min,
    personal_space_intrusion_max
  );

  return 0;
}