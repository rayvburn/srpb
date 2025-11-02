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

  // find the values and recompute according to, e.g., recognized people/groups (max/sum)
  double metrics = 0.0;
  double min_elem = std::numeric_limits<double>::max();
  double max_elem = std::numeric_limits<double>::min();
  // find the timing-corrected percentage of threshold violations of, e.g., personal space
  double violations_duration = 0.0;

  // rollout values to compute the score (metrics)
  for (const auto& tvalue: timed_values) {
    double dt = tvalue.first;
    if (tvalue.second.empty()) {
      std::cout
        << "\x1B[33m"
        << "The value container is empty for at least 1 sample. The metrics will be 0.0"
        << "\x1B[0m"
        << std::endl;
      return std::make_tuple(0.0, 0.0, 0.0, 0);
    }

    /*
     * Count the duration when violations ocurred, i.e., it does not matter whether there are ten people whose
     * personal spaces the robot intruded, but rather in it happened in a single time step, it will be regarded
     * in a sum of "violating" time steps (durations)
     */
    size_t threshold_violations_num = std::count_if(
      tvalue.second.cbegin(),
      tvalue.second.cend(),
      [=](double val) {
        return violation_above_threshold ? (val > violation_threshold) : (val < violation_threshold);
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

    // values must be referenced to a duration of time steps when, e.g., people were actually detected
    metrics += (metrics_elem * (dt / duration));

    // sum up the percentage of time when the threshold value of, e.g., personal space was violated
    if (threshold_violations_num > 0) {
      violations_duration += (dt / duration);
    }
  }
  return std::make_tuple(min_elem, max_elem, metrics, violations_duration);
}

} // namespace evaluation
} // namespace srpb
