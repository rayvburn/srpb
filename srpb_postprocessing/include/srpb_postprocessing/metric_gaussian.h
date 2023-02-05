#pragma once

#include "srpb_postprocessing/metric.h"

namespace srpb {
namespace postprocessing {

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

  virtual void printResults() const = 0;

  /// Computes value of 1D Gaussian probability density function
  /// @url https://en.wikipedia.org/wiki/Gaussian_function
  static double calculateGaussian(double x, double mean, double variance, bool normalize = false);

  /// Computes value of 1D Gaussian PDF but includes wrapped regions of bell curve (shifted -2pi and +2pi)
  /// Note that with @ref normalize set to true, it no longer computes probability density function
  /// as its integral won't sum up to 1
  static double calculateGaussianAngle(double x, double mean, double variance, bool normalize = false);

  /// Reference: Algorithm A.1 from Kirby, 2010 PhD thesis "Social Robot Navigation" (p. 166)
  /// @url https://www.ri.cmu.edu/pub_files/2010/5/rk_thesis.pdf
  static double calculateGaussian(
    double x,
    double y,
    double x_center,
    double y_center,
    double yaw,
    double sigma_h,
    double sigma_r,
    double sigma_s
  );

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
  static std::tuple<double, double, double, unsigned int> calculateGaussianStatistics(
    std::vector<std::pair<double, std::vector<double>>> timed_gaussians,
    double space_violation_threshold,
    bool max_method = true
  );

protected:
  virtual void compute() = 0;
};

} // namespace postprocessing
} // namespace srpb
