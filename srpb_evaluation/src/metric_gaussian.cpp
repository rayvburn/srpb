#include "srpb_evaluation/metric_gaussian.h"

#include <algorithm>
#include <limits>
#include <numeric>

#include <angles/angles.h>

namespace srpb {
namespace evaluation {

std::tuple<double, double, double, double> MetricGaussian::calculateGaussianStatistics(
  std::vector<std::pair<double, std::vector<double>>> timed_gaussians,
  double violation_threshold,
  bool max_method
) {
  // find actual duration
  double duration = 0.0;
  for (const auto& tg: timed_gaussians) {
    duration += tg.first;
  }

  // find number of personal space / f-formation space violations
  unsigned int gaussian_violations = 0;
  unsigned int gaussians_total = 0;

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

    // count timestamps when violations ocurred
    gaussian_violations += std::count_if(
      tg.second.cbegin(),
      tg.second.cend(),
      [&](double g) {
        // count total numbers to find percentage
        gaussians_total++;
        return g > violation_threshold;
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

    // Gaussians must be referenced to timestamps when, e.g., people were actually detected
    metrics += (metrics_elem * (dt / duration));
  }

  // find percentage of time when personal space / f-formation space etc. thresholds were violated
  double gaussian_violations_percentage = gaussian_violations / static_cast<double>(gaussians_total);

  return std::make_tuple(min_elem, max_elem, metrics, gaussian_violations_percentage);
}

} // namespace evaluation
} // namespace srpb
