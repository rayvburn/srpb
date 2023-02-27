#pragma once

#include "benchmark_logger.h"
#include "robot_data.h"

#include <nav_msgs/Odometry.h>

// helper functions
#include <people_msgs_utils/utils.h>

#include <mutex>
#include <utility>

namespace srpb {
namespace logger {

class RobotLogger: public BenchmarkLogger {
public:
    /// For odometry, e.g., Z, Roll, Pitch (based on diff_drive controller)
    static constexpr auto COVARIANCE_UNKNOWN = 1000000.0;

    RobotLogger() = default;

    void init(ros::NodeHandle& nh);

    void start();

    /// Performs writes to files that this class handles, most recent robot data is used
    void update(double timestamp, const RobotData& robot);

    void finish();

    /// Converts given robot data into string description
    static std::string robotToString(const RobotData& robot);

    /// Converts given @ref str string description into robot data
    static std::pair<bool, RobotData> robotFromString(const std::string& str);

protected:
    void localizationCB(const nav_msgs::OdometryConstPtr& msg);

    FILE* log_file_;

    std::mutex cb_mutex_;
    ros::Subscriber localization_sub_;

    /// Newest pose with covariance of the robot (expressed in coordinate system given by 'target_frame_')
    geometry_msgs::PoseWithCovariance robot_pose_;
    /// Newest velocity with covariance of the robot (expressed in local coordinate system)
    geometry_msgs::PoseWithCovariance robot_vel_;
};

} // namespace logger
} // namespace srpb
