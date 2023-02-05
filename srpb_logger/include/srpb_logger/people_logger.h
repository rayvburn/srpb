#pragma once

#include "benchmark_logger.h"

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <people_msgs/People.h>
#include <people_msgs_utils/person.h>
#include <people_msgs_utils/group.h>

#include <mutex>

namespace srpb {
namespace logger {

/**
 * @brief Subscribes to people_msgs and saves meaningful data into log files
 *
 * People data are stored in 1 file and group data are stored in another one
 */
class PeopleLogger: public BenchmarkLogger {
public:
    PeopleLogger() = default;

    void init(ros::NodeHandle& nh);

    void start();

    /// Performs writes to files that this class handles, most recent people data is used
    void update(double timestamp);

    void finish();

    /// Converts given @ref person instance into string description
    static std::string personToString(const people_msgs_utils::Person& person);

    /// Converts given @ref str string description into instance of @ref people_msgs_utils::Person
    static people_msgs_utils::Person personFromString(const std::string& str);

    /// Converts given @ref group instance into string description
    static std::string groupToString(const people_msgs_utils::Group& group);

    /**
     * Converts a given @ref str string description into instance of @ref people_msgs_utils::Group
     *
     * People are logged in separation to groups. Group created from string does not contain data of members since
     * these are already logged on the 'People' side of logger. Therefore, loading people and groups can be understood
     * as joining 2 tables, where the primary key is the person ID. After collecting all people and groups,
     * see @ref people_msgs_utils::fillGroupsWithMembers
     */
    static people_msgs_utils::Group groupFromString(const std::string& str);

protected:
    void peopleCB(const people_msgs::PeopleConstPtr& msg);

    ros::Subscriber people_sub_;
    std::mutex cb_mutex_;

    std::vector<people_msgs_utils::Person> people_;
    std::vector<people_msgs_utils::Group> groups_;

    FILE* log_file_people_;
    std::string log_filename_people_;
    FILE* log_file_groups_;
    std::string log_filename_groups_;
};

} // namespace logger
} // namespace srpb
