#include "srpb_postprocessing/metric_gaussian.h"

#include <algorithm>
#include <limits>
#include <numeric>

#include <angles/angles.h>

double MetricGaussian::calculateGaussian(double x, double mean, double variance, bool normalize) {
  double scale = 1.0;
  // with normalization, maximum possible value will be 1.0; otherwise, it depends on the value of variance
  if (!normalize) {
    scale = 1.0 / (std::sqrt(variance) * std::sqrt(2 * M_PI));
  }
  return scale * std::exp(-std::pow(x - mean, 2) / (2.0 * variance));
}

double MetricGaussian::calculateGaussianAngle(double x, double mean, double variance, bool normalize) {
  double gaussian1 = calculateGaussian(x, mean             , variance, normalize);
  double gaussian2 = calculateGaussian(x, mean - 2.0 * M_PI, variance, normalize);
  double gaussian3 = calculateGaussian(x, mean + 2.0 * M_PI, variance, normalize);
  return std::max(std::max(gaussian1, gaussian2), gaussian3);
}

double MetricGaussian::calculateGaussian(
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

std::tuple<double, double, double, unsigned int> MetricGaussian::calculateGaussianStatistics(
  std::vector<std::pair<double, std::vector<double>>> timed_gaussians,
  double space_violation_threshold,
  bool max_method
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
