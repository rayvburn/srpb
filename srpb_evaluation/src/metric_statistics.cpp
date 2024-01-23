#include "srpb_evaluation/metric_statistics.h"

#include <algorithm>
#include <limits>
#include <numeric>

#include <angles/angles.h>

namespace srpb {
namespace evaluation {

std::tuple<double, double, double, double> MetricStatistics::calculateStatistics(
  std::vector<std::pair<double, std::vector<double>>> timed_values,
  double violation_threshold,
  bool violation_above_threshold,
  bool max_method
) {
  // find the actual duration
  double duration = 0.0;
  for (const auto& tg: timed_values) {
    duration += tg.first;
  }

  // find the number of violations of, e.g., personal space / f-formation's O-space
  unsigned int threshold_violations = 0;
  unsigned int values_total = 0;

  // find the values and recompute according to, e.g., recognized people/groups (max/sum)
  double metrics = 0.0;
  double min_elem = std::numeric_limits<double>::max();
  double max_elem = std::numeric_limits<double>::min();

  // rollout values to compute the score (metrics)
  for (const auto& tvalue: timed_values) {
    double dt = tvalue.first;
    if (tvalue.second.empty()) {
      std::cout << "The value container is empty for at least 1 sample. The metrics will be 0.0" << std::endl;
      return std::make_tuple(0.0, 0.0, 0.0, 0);
    }

    // count timestamps when violations ocurred
    threshold_violations += std::count_if(
      tvalue.second.cbegin(),
      tvalue.second.cend(),
      [&](double g) {
        // count total numbers to find percentage
        values_total++;
        return violation_above_threshold ? (g > violation_threshold) : (g < violation_threshold);
      }
    );

    // overall min and max computation
    double local_min_elem = *std::min_element(tvalue.second.cbegin(), tvalue.second.cend());
    if (local_min_elem < min_elem) {
      min_elem = local_min_elem;
    }

    double local_max_elem = *std::max_element(tvalue.second.cbegin(), tvalue.second.cend());
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
      metrics_elem = std::accumulate(
        tvalue.second.cbegin(),
        tvalue.second.cend(),
        0.0
      ) / static_cast<double>(tvalue.second.size());
    }

    // values must be referenced to timestamps when, e.g., people were actually detected
    metrics += (metrics_elem * (dt / duration));
  }

  // find the percentage of time when thresholds of, e.g., personal space / f-formation's O-space etc. were violated
  double threshold_violations_percentage = threshold_violations / static_cast<double>(values_total);

  return std::make_tuple(min_elem, max_elem, metrics, threshold_violations_percentage);
}

} // namespace evaluation
} // namespace srpb
