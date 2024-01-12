#include "srpb_evaluation/utils.h"

#include <fstream>
#include <sstream>

namespace srpb {
namespace evaluation {

std::vector<std::pair<double, people_msgs_utils::Group>> fillGroupsWithMembers(
    const std::vector<std::pair<double, people_msgs_utils::Group>>& timed_groups,
    const std::vector<std::pair<double, people_msgs_utils::Person>>& timed_people
) {
    // make sure that we have data to process
    if (timed_groups.empty() || timed_people.empty()) {
        return timed_groups;
    }

    // resultant set
    std::vector<std::pair<double, people_msgs_utils::Group>> outputt;

    /*
     * We will delegate some operations to @ref people_msgs_utils::fillGroupsWithMembers
     * but have to find groups/people sets valid for a given timestamp
     */
    std::vector<people_msgs_utils::Group> groups_this_ts;
    // storage for people detected in the current timestamp
    std::vector<people_msgs_utils::Person> people_this_ts;
    // timestamp of the currently investigated group
    double timestamp_group = timed_groups.cbegin()->first;
    // switched to true when all people from the given timestamp were already collected
    bool all_people_from_timestamp = false;
    // alows not to check the full list of person entries on each group's timestamp update
    auto person_it = timed_people.cbegin();
    /*
     * Flow:
     * 1) Iterate through people with the same timestamp as group's
     * 2) When all people for a given timestamp checked - switch to the next group and revert people iterator
     *    so it sweeps through people for a timestamp of the group (multiple groups allowed for a single timestamp)
     */
    for (
        auto group_it = timed_groups.cbegin();
        group_it != timed_groups.cend();
        group_it++
    ) {
        // groups from subsequent timestamp will be investigated - update people iterator first
        if (group_it->first != timestamp_group) {
            // steps for groups from the current timestamp finished - associate members to groups
            auto groups_filled = people_msgs_utils::fillGroupsWithMembers(groups_this_ts, people_this_ts);
            for (const auto& groupf: groups_filled) {
                outputt.push_back({timestamp_group, groupf});
            }
            // reset collected data
            people_this_ts.clear();
            groups_this_ts.clear();
            // update time stamp for the next evaluations
            timestamp_group = group_it->first;
            // reset flag too
            all_people_from_timestamp = false;
        }

        // if true, it's time to collect rest of the groups for the current timestamp
        if (all_people_from_timestamp) {
            groups_this_ts.push_back(group_it->second);
            continue;
        }

        /*
         * Outside loop protects from entering this loop after all people sharing timestamps with the group
         * were collected
         */
        for (
            ;
            person_it != timed_people.cend();
            person_it++
        ) {
            /*
             * Handling case when groups were not detected for some time (whereas people were so person iterator kinda
             * lags and must skip some steps)
             */
            while (person_it->first < group_it->first && person_it != timed_people.cend()) {
                person_it++;
            }

            if (person_it->first != group_it->first) {
                throw std::runtime_error("Timestamps between people and groups seem not to be synchronized");
            }

            // collect
            people_this_ts.push_back(person_it->second);

            // check if the next person is from the same timestamp as the group
            if (
                std::next(person_it) == timed_people.cend()
                || std::next(person_it)->first != group_it->first
            ) {
                all_people_from_timestamp = true;
                // prepare for next iteration
                person_it++;
                // let groups collect
                break;
            }
        }
        /*
         * Collect first group in a given timestamp
         * Flow forces that all people from the given timestamp were already collected
         */
        groups_this_ts.push_back(group_it->second);

        // handle 'on exist' from loop
        if (std::next(group_it) == timed_groups.cend() && !groups_this_ts.empty()) {
            // steps for groups from the current timestamp finished - associate members to groups
            auto groups_filled = people_msgs_utils::fillGroupsWithMembers(groups_this_ts, people_this_ts);
            for (const auto& groupf: groups_filled) {
                outputt.push_back({timestamp_group, groupf});
            }
        }
    }
    return outputt;
}

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
) {
    std::fstream file_results;
    file_results.open(filename, std::ios::out);
    if (!file_results.is_open()) {
        printf("Results CSV file cannot be created\r\n");
        return;
    }

    std::stringstream ss;
    ss.setf(std::ios::fixed);
    ss << "s_rbt        ," << std::setw(9) << std::setprecision(4) << samples_robot << std::endl;
    ss << "s_ppl        ," << std::setw(9) << std::setprecision(4) << samples_people << std::endl;
    ss << "s_grp        ," << std::setw(9) << std::setprecision(4) << samples_groups << std::endl;
    ss << "m_goal       ," << std::setw(9) << std::setprecision(4) << goal_reached.getValue() << std::endl;
    ss << "m_obs        ," << std::setw(9) << std::setprecision(4) << obstacle_safety.getValue() << std::endl;
    ss << "m_mef        ," << std::setw(9) << std::setprecision(4) << motion_efficiency.getValue() << std::endl;
    ss << "m_cef        ," << std::setw(9) << std::setprecision(4) << computational_efficiency.getValue() << std::endl;
    ss << "m_cre        ," << std::setw(9) << std::setprecision(4) << computational_time_repeatability.getValue() << std::endl;
    ss << "m_vsm        ," << std::setw(9) << std::setprecision(4) << velocity_smoothness.getValue() << std::endl;
    ss << "m_hsm        ," << std::setw(9) << std::setprecision(4) << heading_change_smoothness.getValue() << std::endl;
    ss << "m_path       ," << std::setw(9) << std::setprecision(4) << path_linear_length.getValue() << std::endl;
    ss << "m_chc        ," << std::setw(9) << std::setprecision(4) << cumulative_heading_change.getValue() << std::endl;
    ss << "m_osc        ," << std::setw(9) << std::setprecision(4) << oscillations.getValue() << std::endl;
    ss << "m_bwd        ," << std::setw(9) << std::setprecision(4) << backward_movements.getValue() << std::endl;
    ss << "m_inp        ," << std::setw(9) << std::setprecision(4) << inplace_rotations.getValue() << std::endl;
    ss << "m_psi        ," << std::setw(9) << std::setprecision(4) << personal_space_intrusion.getValue() << std::endl;
    ss << "m_psi_min    ," << std::setw(9) << std::setprecision(4) << personal_space_intrusion.getValueMin() << std::endl;
    ss << "m_psi_max    ," << std::setw(9) << std::setprecision(4) << personal_space_intrusion.getValueMax() << std::endl;
    ss << "m_psi_viol   ," << std::setw(9) << std::setprecision(4) << personal_space_intrusion.getViolations() << std::endl;
    ss << "m_fsi        ," << std::setw(9) << std::setprecision(4) << formation_space_intrusion.getValue() << std::endl;
    ss << "m_fsi_min    ," << std::setw(9) << std::setprecision(4) << formation_space_intrusion.getValueMin() << std::endl;
    ss << "m_fsi_max    ," << std::setw(9) << std::setprecision(4) << formation_space_intrusion.getValueMax() << std::endl;
    ss << "m_fsi_viol   ," << std::setw(9) << std::setprecision(4) << formation_space_intrusion.getViolations() << std::endl;
    ss << "m_dir        ," << std::setw(9) << std::setprecision(4) << heading_direction_disturbance.getValue() << std::endl;
    ss << "m_dir_min    ," << std::setw(9) << std::setprecision(4) << heading_direction_disturbance.getValueMin() << std::endl;
    ss << "m_dir_max    ," << std::setw(9) << std::setprecision(4) << heading_direction_disturbance.getValueMax() << std::endl;
    ss << "m_dir_viol   ," << std::setw(9) << std::setprecision(4) << heading_direction_disturbance.getViolations() << std::endl;
    ss << "m_psd        ," << std::setw(9) << std::setprecision(4) << passing_speed_discomfort.getValue() << std::endl;
    ss << "m_psd_min    ," << std::setw(9) << std::setprecision(4) << passing_speed_discomfort.getValueMin() << std::endl;
    ss << "m_psd_max    ," << std::setw(9) << std::setprecision(4) << passing_speed_discomfort.getValueMax() << std::endl;
    ss << "m_psd_viol   ," << std::setw(9) << std::setprecision(4) << passing_speed_discomfort.getViolations() << std::endl;

    file_results << ss.str();
    printf("Results CSV file saved at `%s`\r\n", filename.c_str());
}

} // namespace evaluation
} // namespace srpb
