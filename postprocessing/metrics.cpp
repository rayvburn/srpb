#include <cstdio>
#include <cstdlib>
#include <vector>
#include <cmath>

#include <exception>
#include <fstream>
#include <string_view>

#include <move_base/robot_logger.h>
#include <move_base/people_logger.h>

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
  return 0;
}