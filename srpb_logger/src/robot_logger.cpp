#include "srpb_logger/robot_logger.h"

#include <tf2/utils.h>

#include <sstream>
#include <iomanip>

namespace srpb {
namespace logger {

void RobotLogger::init(ros::NodeHandle& nh) {
    BenchmarkLogger::init(nh);
    log_filename_ = BenchmarkLogger::appendToFilename(log_filename_, "robot");

    // by default, latching is disabled = async updates are allowed
    latched_.store(false);
    localization_sub_ = nh.subscribe<nav_msgs::Odometry>(
        "/odom",
        1,
        boost::bind(&RobotLogger::localizationCB, this, _1)
    );

    // obtain the name of the topic to retrieve the computation times of the external local planner (not a navstack
    // plugin); if empty, computation times obtained via the @ref update call will be used (default)
    external_comp_time_.store(NAN);
    std::string computation_time_topic("");
    int computation_time_array_idx = -1;
    nh.param("srpb/external_computation_times_topic", computation_time_topic, computation_time_topic);
    nh.param("srpb/external_computation_times_array_index", computation_time_array_idx, computation_time_array_idx);

    if (!computation_time_topic.empty() && computation_time_array_idx >= 0) {
        external_comp_time_array_index_ = static_cast<size_t>(computation_time_array_idx);
        external_comp_times_sub_ = nh.subscribe<std_msgs::Float64MultiArray>(
            computation_time_topic,
            10,
            boost::bind(&RobotLogger::externalComputationTimesCB, this, _1)
        );
    }
}

void RobotLogger::start() {
    if (!log_file_.is_open()) {
        BenchmarkLogger::start(log_file_, log_filename_);
        std::cout << "[ SRPB] Started the first log file for a robot" << std::endl;
    } else if (log_file_.is_open()) {
        BenchmarkLogger::finish(log_file_);
        std::cout << "[ SRPB] Finishing a robot log file" << std::endl;
        BenchmarkLogger::startNextPart(log_file_, log_filename_);
        std::cout << "[ SRPB] Started next log file for a robot" << std::endl;
    }

    // increment the counter used for numbering goals
    incrementLogPartCounter();
}

void RobotLogger::update(double timestamp, const RobotData& robot) {
    if (!log_file_) {
        throw std::runtime_error("File for RobotLogger was not properly created!");
    }

    // check if given RobotData should be extended with odometry data from logger
    RobotData robot_data = robot;
    if (robot.hasPartialData()) {
        // we'll be processing the data that may be updated asynchronously
        std::lock_guard<std::mutex> l(cb_mutex_);

        // NOTE: goal from the RobotData is assumed to be represented in the logger's frame
        robot_data = RobotData(
            robot_pose_,
            robot_vel_,
            robot.getGoal(),
            robot.getVelocityCommand(),
            robot.getDistToObstacle(),
            robot.getLocalPlanningTime()
        );
    }
    // modify local planning time, if required and valid data was received
    if (!std::isnan(external_comp_time_.load())) {
        robot_data = RobotData(
            robot_data.getPoseWithCovariance(),
            robot_data.getVelocityWithCovariance(),
            robot_data.getGoal(),
            robot_data.getVelocityCommand(),
            robot_data.getDistToObstacle(),
            external_comp_time_.load()
        );
    }

    // (enable the asynchronous updates of pose and velocity
    latched_.store(false);

    std::stringstream ss;
    ss.setf(std::ios::fixed);
    ss << std::setw(9) << std::setprecision(4) << timestamp << " ";
    ss << RobotLogger::robotToString(robot_data) << std::endl;
    log_file_ << ss.str();
}

void RobotLogger::latch() {
    latched_.store(true);
}

void RobotLogger::finish() {
    BenchmarkLogger::finish(log_file_);
}

std::string RobotLogger::robotToString(const RobotData& robot) {
    std::stringstream ss;
    ss.setf(std::ios::fixed);
    /*  0 */ ss << std::setw(9) << std::setprecision(4) << robot.getPositionX() << " ";
    /*  1 */ ss << std::setw(9) << std::setprecision(4) << robot.getPositionY() << " ";
    /*  2 */ ss << std::setw(9) << std::setprecision(4) << robot.getPositionZ() << " ";
    /*  3 */ ss << std::setw(9) << std::setprecision(4) << robot.getOrientationYaw() << " ";
    /*  4 */ ss << std::setw(9) << std::setprecision(4) << robot.getCovariancePoseXX() << " ";
    /*  5 */ ss << std::setw(9) << std::setprecision(4) << robot.getCovariancePoseXY() << " ";
    /*  6 */ ss << std::setw(9) << std::setprecision(4) << robot.getCovariancePoseYX() << " ";
    /*  7 */ ss << std::setw(9) << std::setprecision(4) << robot.getCovariancePoseYY() << " ";
    /*  8 */ ss << std::setw(9) << std::setprecision(4) << robot.getCovariancePoseYawYaw() << " ";
    /*  9 */ ss << std::setw(9) << std::setprecision(4) << robot.getVelocityX() << " ";
    /* 10 */ ss << std::setw(9) << std::setprecision(4) << robot.getVelocityY() << " ";
    /* 11 */ ss << std::setw(9) << std::setprecision(4) << robot.getVelocityZ() << " ";
    /* 12 */ ss << std::setw(9) << std::setprecision(4) << robot.getVelocityTheta() << " ";
    /* 13 */ ss << std::setw(9) << std::setprecision(4) << robot.getCovarianceVelocityXX() << " ";
    /* 14 */ ss << std::setw(9) << std::setprecision(4) << robot.getCovarianceVelocityXY() << " ";
    /* 15 */ ss << std::setw(9) << std::setprecision(4) << robot.getCovarianceVelocityYX() << " ";
    /* 16 */ ss << std::setw(9) << std::setprecision(4) << robot.getCovarianceVelocityYY() << " ";
    /* 17 */ ss << std::setw(9) << std::setprecision(4) << robot.getCovarianceVelocityThTh() << " ";
    /* 18 */ ss << std::setw(9) << std::setprecision(4) << robot.getGoalPositionX() << " ";
    /* 19 */ ss << std::setw(9) << std::setprecision(4) << robot.getGoalPositionY() << " ";
    /* 20 */ ss << std::setw(9) << std::setprecision(4) << robot.getGoalOrientationYaw() << " ";
    /* 21 */ ss << std::setw(9) << std::setprecision(4) << robot.getVelocityCommandLinearX() << " ";
    /* 22 */ ss << std::setw(9) << std::setprecision(4) << robot.getVelocityCommandLinearY() << " ";
    /* 23 */ ss << std::setw(9) << std::setprecision(4) << robot.getVelocityCommandAngularZ() << " ";
    /* 24 */ ss << std::setw(9) << std::setprecision(4) << robot.getDistToObstacle() << " ";
    /* 25 */ ss << std::setw(9) << std::setprecision(4) << robot.getLocalPlanningTime();
    return ss.str();
}

std::pair<bool, RobotData> RobotLogger::robotFromString(const std::string& str) {
    auto vals = people_msgs_utils::parseString<double>(str, " ");
    if (vals.size() != 26) {
        std::cout << "\x1B[31mFound corrupted data of a robot:\r\n\t" << str << "\x1B[0m" << std::endl;
        // dummy robot data
        auto dummy_robot = RobotData(
            geometry_msgs::PoseWithCovariance(),
            geometry_msgs::PoseWithCovariance(),
            geometry_msgs::Pose(),
            geometry_msgs::Twist(),
            0.0,
            0.0
        );
        return {false, dummy_robot};
    }

    geometry_msgs::PoseWithCovariance pose;
    pose.pose.position.x = vals.at(0);
    pose.pose.position.y = vals.at(1);
    pose.pose.position.z = vals.at(2);
    tf2::Quaternion quat;
    quat.setRPY(0.0, 0.0, vals.at(3));
    pose.pose.orientation.x = quat.getX();
    pose.pose.orientation.y = quat.getY();
    pose.pose.orientation.z = quat.getZ();
    pose.pose.orientation.w = quat.getW();

    pose.covariance.at(RobotData::COV_XX_INDEX) = vals.at(4);
    pose.covariance.at(RobotData::COV_XY_INDEX) = vals.at(5);
    pose.covariance.at(RobotData::COV_YX_INDEX) = vals.at(6);
    pose.covariance.at(RobotData::COV_YY_INDEX) = vals.at(7);
    pose.covariance.at(RobotData::COV_ZZ_INDEX) = COVARIANCE_UNKNOWN;
    pose.covariance.at(RobotData::COV_ROLLROLL_INDEX) = COVARIANCE_UNKNOWN;
    pose.covariance.at(RobotData::COV_PITCHPITCH_INDEX) = COVARIANCE_UNKNOWN;
    pose.covariance.at(RobotData::COV_YAWYAW_INDEX) = vals.at(8);

    geometry_msgs::PoseWithCovariance vel;
    vel.pose.position.x = vals.at(9);
    vel.pose.position.y = vals.at(10);
    vel.pose.position.z = vals.at(11);
    quat.setRPY(0.0, 0.0, vals.at(12));
    vel.pose.orientation.x = quat.getX();
    vel.pose.orientation.y = quat.getY();
    vel.pose.orientation.z = quat.getZ();
    vel.pose.orientation.w = quat.getW();

    vel.covariance.at(RobotData::COV_XX_INDEX) = vals.at(13);
    vel.covariance.at(RobotData::COV_XY_INDEX) = vals.at(14);
    vel.covariance.at(RobotData::COV_YX_INDEX) = vals.at(15);
    vel.covariance.at(RobotData::COV_YY_INDEX) = vals.at(16);
    vel.covariance.at(RobotData::COV_ZZ_INDEX) = COVARIANCE_UNKNOWN;
    vel.covariance.at(RobotData::COV_ROLLROLL_INDEX) = COVARIANCE_UNKNOWN;
    vel.covariance.at(RobotData::COV_PITCHPITCH_INDEX) = COVARIANCE_UNKNOWN;
    vel.covariance.at(RobotData::COV_YAWYAW_INDEX) = vals.at(17);

    geometry_msgs::Pose goal;
    goal.position.x = vals.at(18);
    goal.position.y = vals.at(19);
    quat.setRPY(0.0, 0.0, vals.at(20));
    goal.orientation.x = quat.getX();
    goal.orientation.y = quat.getY();
    goal.orientation.z = quat.getZ();
    goal.orientation.w = quat.getW();

    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = vals.at(21);
    cmd_vel.linear.y = vals.at(22);
    cmd_vel.angular.z = vals.at(23);

    double obst_dist = vals.at(24);
    double exec_time = vals.at(25);

    auto robot = RobotData(pose, vel, goal, cmd_vel, obst_dist, exec_time);
    return {true, robot};
}

void RobotLogger::localizationCB(const nav_msgs::OdometryConstPtr& msg) {
    // check if any updates should be accepted
    if (latched_.load()) {
        return;
    }

    std::lock_guard<std::mutex> l(cb_mutex_);
    // pose should be transformed to the logger's frame
    robot_pose_ = transformPose(msg->pose, msg->header.frame_id).pose;
    // prepare velocity of the robot;
    // velocity of the robot is expressed in its body frame (local frame) and is stored in that form
    geometry_msgs::PoseWithCovariance vel;
    vel.pose.position.x = msg->twist.twist.linear.x;
    vel.pose.position.y = msg->twist.twist.linear.y;
    vel.pose.position.z = msg->twist.twist.linear.z;
    tf2::Quaternion vel_quat;
    vel_quat.setRPY(msg->twist.twist.angular.x, msg->twist.twist.angular.y, msg->twist.twist.angular.z);
    vel.pose.orientation.x = vel_quat.getX();
    vel.pose.orientation.y = vel_quat.getY();
    vel.pose.orientation.z = vel_quat.getZ();
    vel.pose.orientation.w = vel_quat.getW();
    // prepare covariance
    vel.covariance = msg->twist.covariance;
    // save velocity
    robot_vel_ = vel;
}

void RobotLogger::externalComputationTimesCB(const std_msgs::Float64MultiArrayConstPtr& msg) {
    size_t required_elems = external_comp_time_array_index_ + 1;
    // verify that sufficient number of elements is available
    if (msg->data.size() < required_elems) {
        std::cout <<
            "\x1B[31m" <<
            "[ SRPB] RobotLogger did not receive a proper multi array at the " <<
            "`" << external_comp_times_sub_.getTopic().c_str() << "` topic. " <<
            "Got " << msg->data.size() << " data elements, whereas at least " <<
            required_elems << " are required" <<
            "\x1B[0m" <<
        std::endl;
        return;
    }
    external_comp_time_.store(static_cast<double>(msg->data.at(external_comp_time_array_index_)));
}

} // namespace srpb
} // namespace logger
