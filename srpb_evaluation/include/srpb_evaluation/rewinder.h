#pragma once

#include <functional>
#include <tuple>
#include <memory>
#include <vector>

#include <srpb_logger/robot_data.h>
#include <people_msgs_utils/person.h>
#include <people_msgs_utils/group.h>

namespace srpb {
namespace evaluation {

/**
 * Given robot, people and formation data, processes all information for each entity for each timestamp
 *
 * This class requires data to be given in a chronological order, so first elements of pairs may repeat, but generally
 * should be sorted in an incremental order.
 */
class Rewinder {
public:
    Rewinder(
        const std::vector<std::pair<double, logger::RobotData>>& robot_data
    );

    Rewinder(
        const std::vector<std::pair<double, logger::RobotData>>& robot_data,
        const std::vector<std::pair<double, people_msgs_utils::Person>>& people_data
    );

    Rewinder(
        const std::vector<std::pair<double, logger::RobotData>>& robot_data,
        const std::vector<std::pair<double, people_msgs_utils::Person>>& people_data,
        const std::vector<std::pair<double, people_msgs_utils::Group>>& groups_data
    );

    /**
     * Called at the start of any new timestamp (robot's timestamp is a reference)
     */
    void setHandlerNextTimestamp(std::function<void(void)> fun);
    /**
     * Called at the start of the last timestamp (robot's timestamp is a reference)
     * This will usually be the same handler as given in @ref setHandlerNextTimestamp
     */
    void setHandlerLastTimestamp(std::function<void(void)> fun);

    void setHandlerAllGroupsTimestamp(std::function<void(void)> fun);

    void setHandlerAllPeopleTimestamp(std::function<void(void)> fun);

    void setHandlerNextPersonTimestamp(std::function<void(void)> fun);

    void setHandlerNextGroupTimestamp(std::function<void(void)> fun);

    /// Starts iterating over data samples given in ctor calling event handlers provided by setter methods
    bool perform();

    /**
     * @defgroup Getters getters
     *
     * A more precise approach would return shared_ptr, but this, in turn, requires checking
     * of a pointer validity in the metric implementation which exposes code size
     *
     * @{
     */
    double getTimestampCurr() const;
    double getTimestampPrev() const;
    double getTimestampNext() const;

    double getTimestampFirst() const;
    double getTimestampLast() const;

    inline double getDuration() const {
        return getTimestampLast() - getTimestampFirst();
    }

    inline size_t getTimestampsNum() const {
        return robot_data_.size();
    }

    logger::RobotData getRobotCurr() const;
    logger::RobotData getRobotPrev() const;
    logger::RobotData getRobotNext() const;

    people_msgs_utils::Person getPersonCurr() const;
    people_msgs_utils::Person getPersonPrev() const;
    people_msgs_utils::Person getPersonNext() const;

    people_msgs_utils::Group getGroupCurr() const;
    people_msgs_utils::Group getGroupPrev() const;
    people_msgs_utils::Group getGroupNext() const;
    /// @}

protected:
    template <typename T>
    bool isIteratorInRange(T it, T begin, T end) const {
        return it >= begin && it < end;
    }

    std::runtime_error createRuntimeError(const std::string& position, const std::string& data_set) const;

    const std::vector<std::pair<double, logger::RobotData>>& robot_data_;
    const std::vector<std::pair<double, people_msgs_utils::Person>>& people_data_;
    const std::vector<std::pair<double, people_msgs_utils::Group>>& groups_data_;

    /**
     * @defgroup flags data availability flags
     * Really did not wanted to take shared_ptrs to ctor, hence 2 flags indicating which data is available (besides robot's)
     * Flags are needed since vectors refer to temporary objects, destroyed after ctor execution. Therefore, reported size
     * of a data vector can be totally off
     * @{
     */
    const bool data_people_empty_;
    const bool data_groups_empty_;
    /// @}

    /// Iterator to investigate robot data
    std::vector<std::pair<double, logger::RobotData>>::const_iterator it_robot_;
    /// Iterator to investigate people data
    std::vector<std::pair<double, people_msgs_utils::Person>>::const_iterator it_ppl_;
    /// Iterator to investigate groups data
    std::vector<std::pair<double, people_msgs_utils::Group>>::const_iterator it_grp_;

    std::function<void(void)> handler_next_timestamp_;
    std::function<void(void)> handler_last_timestamp_;
    // called after last group in a given timestamp was checked
    std::function<void(void)> handler_all_groups_timestamp_;
    // called after last person in a given timestamp was checked
    std::function<void(void)> handler_all_people_timestamp_;
    std::function<void(void)> handler_next_person_timestamp_;
    // called after checking all possible people observed in a given timestamp (handler_next_person_timestamp_)
    std::function<void(void)> handler_next_group_timestamp_;
};

} // namespace evaluation
} // namespace srpb
