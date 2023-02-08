#include "srpb_evaluation/utils.h"

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
    }
    return outputt;
}

} // namespace evaluation
} // namespace srpb
