#pragma once

#include "srpb_evaluation/metric.h"

namespace srpb {
namespace evaluation {

class MetricStatistics: public Metric {
public:
	MetricStatistics(
      const std::vector<std::pair<double, logger::RobotData>>& robot_data
  ): Metric(robot_data) {}

  MetricStatistics(
      const std::vector<std::pair<double, logger::RobotData>>& robot_data,
      const std::vector<std::pair<double, people_msgs_utils::Person>>& people_data
  ): Metric(robot_data, people_data) {}

  MetricStatistics(
      const std::vector<std::pair<double, logger::RobotData>>& robot_data,
      const std::vector<std::pair<double, people_msgs_utils::Person>>& people_data,
      const std::vector<std::pair<double, people_msgs_utils::Group>>& groups_data
  ): Metric(robot_data, people_data, groups_data) {}

  /// Returns minimum value obtained throughout the scenario
  virtual double getValueMin() const = 0;

  /// Returns maximum value obtained throughout the scenario
  virtual double getValueMax() const = 0;

  /// Returns the percentage of violations obtained throughout the scenario (considering the threshold value)
  virtual double getViolations() const = 0;

  /// Prints results
  virtual void printResults() const = 0;

  /**
   * @brief Computes min, max, normalized metrics and counts the number of violations (above the certain threshold)
   *
   * @param timed_values vector of pairs with, first, a timestamp, and second, a container with values
   * @param violation_threshold values bigger than that will be considered as violations
   * @param violation_above_threshold selects whether the violation is counted when the value is bigger than
   * the threshold (true, default) or when the violation is less than the threshold (false).
   * This is specific to a certain metric (whether computes "cost" or "reward").
   * @param max_method set to true (default) so the max element is used to normalize metrics;
   * false means averaging over all values/occurrences in a current time step
   *
   * @return std::tuple<double, double, double, double> tuple with scores:
   * 1) min value,
   * 2) max value,
   * 3) a metric value normalized according to the duration/execution time,
   * 4) timing-corrected percentage of the threshold value violations
   */
  static std::tuple<double, double, double, double> calculateStatistics(
    std::vector<std::pair<double, std::vector<double>>> timed_values,
    double violation_threshold,
    bool violation_above_threshold = true,
    bool max_method = true
  );

protected:
  virtual void compute() = 0;
};

} // namespace evaluation
} // namespace srpb
