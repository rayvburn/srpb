#pragma once

#include "srpb_evaluation/metric.h"

// helper functions for calculation of Gaussian distribution value
#include <social_nav_utils/gaussians.h>

namespace srpb {
namespace evaluation {

class MetricGaussian: public Metric {
public:
	MetricGaussian(
      const std::vector<std::pair<double, logger::RobotData>>& robot_data
  ): Metric(robot_data) {}

  MetricGaussian(
      const std::vector<std::pair<double, logger::RobotData>>& robot_data,
      const std::vector<std::pair<double, people_msgs_utils::Person>>& people_data
  ): Metric(robot_data, people_data) {}

  MetricGaussian(
      const std::vector<std::pair<double, logger::RobotData>>& robot_data,
      const std::vector<std::pair<double, people_msgs_utils::Person>>& people_data,
      const std::vector<std::pair<double, people_msgs_utils::Group>>& groups_data
  ): Metric(robot_data, people_data, groups_data) {}

  /// Returns minimum value of Gaussian throughout the scenario
  virtual double getValueMin() const = 0;

  /// Returns maximum value of Gaussian throughout the scenario
  virtual double getValueMax() const = 0;

  /// Returns percentage of violations of Gaussian throughout the scenario (considering the threshold value)
  virtual double getViolations() const = 0;

  /// Prints results
  virtual void printResults() const = 0;

  /**
   * @brief Computes min, max, normalized metrics related to gaussian statistics and counts number of space violations
   *
   * @param timed_gaussians vector of pairs with, first, timestamp, and second, values of gaussians
   * @param violation_threshold Gaussian values bigger than that will be considered as violation of personal space
   * @param max_method set to true (default) to use max element from Gaussians to normalize metrics;
   * false means averaging over all Gaussian occurrences in a current time step
   *
   * @return std::tuple<double, double, double, double> tuple with scores: min, max and normalized to execution
   * time and percentage of violation of, e.g., personal space violations (according to given threshold and gaussians)
   */
  static std::tuple<double, double, double, double> calculateGaussianStatistics(
    std::vector<std::pair<double, std::vector<double>>> timed_gaussians,
    double violation_threshold,
    bool max_method = true
  );

protected:
  virtual void compute() = 0;
};

} // namespace evaluation
} // namespace srpb
