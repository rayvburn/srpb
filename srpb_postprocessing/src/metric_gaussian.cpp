#include "srpb_postprocessing/metric_gaussian.h"

#include <algorithm>
#include <limits>
#include <numeric>

#include <angles/angles.h>

namespace srpb {
namespace postprocessing {

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

} // namespace postprocessing
} // namespace srpb
