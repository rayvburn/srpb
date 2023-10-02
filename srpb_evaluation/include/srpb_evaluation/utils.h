#pragma once

#include <exception>
#include <fstream>
#include <functional>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include <people_msgs_utils/utils.h>

#include "srpb_evaluation/metrics/backward_movements.h"
#include "srpb_evaluation/metrics/computational_efficiency.h"
#include "srpb_evaluation/metrics/computational_time_repeatability.h"
#include "srpb_evaluation/metrics/cumulative_heading_change.h"
#include "srpb_evaluation/metrics/formation_space_instrusion.h"
#include "srpb_evaluation/metrics/goal_reached.h"
#include "srpb_evaluation/metrics/heading_direction_disturbance.h"
#include "srpb_evaluation/metrics/heading_change_smoothness.h"
#include "srpb_evaluation/metrics/inplace_rotations.h"
#include "srpb_evaluation/metrics/motion_efficiency.h"
#include "srpb_evaluation/metrics/obstacle_safety.h"
#include "srpb_evaluation/metrics/oscillations.h"
#include "srpb_evaluation/metrics/passing_speed_discomfort.h"
#include "srpb_evaluation/metrics/path_linear_length.h"
#include "srpb_evaluation/metrics/personal_space_instrusion.h"
#include "srpb_evaluation/metrics/velocity_smoothness.h"

namespace srpb {
namespace evaluation {

/**
 * @brief Does similar to what @ref people_msgs_utils::fillGroupsWithMembers does, but considers timestamps of data
 *
 * @return Modified @ref timed_groups vector
 */
std::vector<std::pair<double, people_msgs_utils::Group>> fillGroupsWithMembers(
    const std::vector<std::pair<double, people_msgs_utils::Group>>& timed_groups,
    const std::vector<std::pair<double, people_msgs_utils::Person>>& timed_people
);

/**
 * @brief Creates a CSV file with results
 */
void createResultsFile(
    const std::string& filename,
    const size_t& samples_robot,
    const size_t& samples_people,
    const size_t& samples_groups,
    const GoalReached& goal_reached,
    const ObstacleSafety& obstacle_safety,
    const MotionEfficiency& motion_efficiency,
    const ComputationalEfficiency& computational_efficiency,
    const ComputationalTimeRepeatability& computational_time_repeatability,
    const VelocitySmoothness& velocity_smoothness,
    const HeadingChangeSmoothness& heading_change_smoothness,
    const PathLinearLength& path_linear_length,
    const CumulativeHeadingChange& cumulative_heading_change,
    const Oscillations& oscillations,
    const BackwardMovements& backward_movements,
    const InplaceRotations& inplace_rotations,
    const PersonalSpaceIntrusion& personal_space_intrusion,
    const FormationSpaceIntrusion& formation_space_intrusion,
    const HeadingDirectionDisturbance& heading_direction_disturbance,
    const PassingSpeedDiscomfort& passing_speed_discomfort
);

/**
 * @defgroup templates Template functions
 * @{
 */
// string to pair (first = timestamp, second = logged state)
template <typename T>
static std::vector<std::pair<double, T>> parseFile(
    const std::string& filepath,
    std::function<std::pair<bool, T>(const std::string&)> from_string_fun
) {
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
        auto entity = from_string_fun(line.substr(ts_end + 1));
        // check if conversion from string was successful
        if (!entity.first) {
            continue;
        }

        data_log.push_back(
            std::make_pair(
                std::stod(timestamp_str),
                entity.second
            )
        );
    }

    file.close();
    return data_log;
}
/// @}

} // namespace evaluation
} // namespace srpb
