#pragma once

#include "benchmark_logger.h"
#include "global_planner_data.h"

namespace srpb {
namespace logger {

/**
 * @brief A logger class storing the information about the global planner
 *
 * The class for global planner data logging was separated from the logger of the general robot data (gathered with
 * a frequency of the local planning) as it is usually gathered with a very different frequency.
 *
 * All in all, this class allows avoiding the interpolation during an offline data analysis.
 */
class GlobalPlannerLogger: public BenchmarkLogger {
public:
    /// For odometry, e.g., Z, Roll, Pitch (based on diff_drive controller)
    static constexpr auto COVARIANCE_UNKNOWN = 1000000.0;

    GlobalPlannerLogger() = default;

    void init(ros::NodeHandle& nh);

    void start();

    /// Performs writes to files that this class handles, most recent planner data is used
    void update(double timestamp, const GlobalPlannerData& planner);

    void finish();

    /// Converts a given global planing data into string description
    static std::string plannerToString(const GlobalPlannerData& planner);

    /// Converts a given @ref str string description into a global planing data
    static std::pair<bool, GlobalPlannerData> plannerFromString(const std::string& str);

protected:
    std::fstream log_file_;
};

} // namespace logger
} // namespace srpb
