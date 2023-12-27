#pragma once

#include "benchmark_logger.h"
#include "robot_data.h"

#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64MultiArray.h>

// helper functions
#include <people_msgs_utils/utils.h>

#include <atomic>
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

    /**
     * Should be called before @ref update when a significant timing offset is expected (see details)
     *
     * The timing offset is related to the the duration between the "timestamp" passed to the @ref update and
     * the actual timestamp of the localization data that are updated asynchronously and stored as class members.
     *
     * Each "latch" should be renewed after every @ref update call.
     */
    void latch();

    /// Performs writes to files that this class handles, most recent robot data is used
    void update(double timestamp, const RobotData& robot);

    void finish();

    /// Converts given robot data into string description
    static std::string robotToString(const RobotData& robot);

    /// Converts given @ref str string description into robot data
    static std::pair<bool, RobotData> robotFromString(const std::string& str);

protected:
    void localizationCB(const nav_msgs::OdometryConstPtr& msg);

    void externalComputationTimesCB(const std_msgs::Float64MultiArrayConstPtr& msg);

    std::fstream log_file_;

    std::mutex cb_mutex_;
    ros::Subscriber localization_sub_;
    /// Subscriber that is valid once the given topic param is non-empty
    ros::Subscriber external_comp_times_sub_;

    /// When this flag is true, any incoming updates of robot pose and velocity will not be accepted
    std::atomic<bool> latched_;
    /// Newest pose with covariance of the robot (expressed in coordinate system given by 'target_frame_')
    geometry_msgs::PoseWithCovariance robot_pose_;
    /// Newest velocity with covariance of the robot (expressed in local coordinate system)
    geometry_msgs::PoseWithCovariance robot_vel_;

    /// Data index of the multi-dimension array that a computation time is stored under
    size_t external_comp_time_array_index_;
    /// Stores the newest value of the computation time obtained via the @ref external_comp_times_sub_ subscriber
    std::atomic<double> external_comp_time_;
};

} // namespace logger
} // namespace srpb
