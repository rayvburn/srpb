#include "srpb_logger/robot_logger.h"

#include <tf2/utils.h>

namespace srpb {
namespace logger {

void RobotLogger::init(ros::NodeHandle& nh) {
    BenchmarkLogger::init(nh);

    auto log_path_base = log_filename_.substr(0, log_filename_.find_last_of(EXTENSION_SEPARATOR));
    auto log_extension = log_filename_.substr(log_filename_.find_last_of(EXTENSION_SEPARATOR) + 1);

    log_filename_ = log_path_base + "_robot" + EXTENSION_SEPARATOR + log_extension;

    localization_sub_ = nh.subscribe<nav_msgs::Odometry>(
        "/odom",
        1,
        boost::bind(&RobotLogger::localizationCB, this, _1)
    );
}

void RobotLogger::start() {
    BenchmarkLogger::start(&log_file_, log_filename_);
}

void RobotLogger::update(double timestamp, const RobotData& robot) {
    if (log_file_ == nullptr) {
        throw std::runtime_error("File descriptor for RobotLogger was not properly created!");
    }

    // check if given RobotData should be extended with odometry data from logger
    RobotData robot_data = robot;
    if (robot.hasPartialData()) {
        robot_data = RobotData(
            robot_pose_,
            robot_vel_,
            robot.getGoal(),
            robot.getDistToObstacle(),
            robot.getLocalPlanningTime()
        );
    }

    fprintf(
        log_file_,
        "%9.4f %s\n",
        timestamp,
        RobotLogger::robotToString(robot_data).c_str()
    );
}

void RobotLogger::finish() {
    BenchmarkLogger::finish(&log_file_);
}

std::string RobotLogger::robotToString(const RobotData& robot) {
    char buff[300] = {0};
    sprintf(
        buff,
        // pose: x y z yaw
        "%9.4f %9.4f %9.4f %9.4f "
        // pose cov
        "%9.4f %9.4f %9.4f %9.4f %9.4f "
        // vel: x y z theta
        "%9.4f %9.4f %9.4f %9.4f "
        // vel cov
        "%9.4f %9.4f %9.4f %9.4f %9.4f "
        // goal pose 2d
        "%9.4f %9.4f %9.4f "
        // dist to obstacle, planning time
        "%9.4f %9.4f",
        /*  0 */ robot.getPositionX(),
        /*  1 */ robot.getPositionY(),
        /*  2 */ robot.getPositionZ(),
        /*  3 */ robot.getOrientationYaw(),
        /*  4 */ robot.getCovariancePoseXX(),
        /*  5 */ robot.getCovariancePoseXY(),
        /*  6 */ robot.getCovariancePoseYX(),
        /*  7 */ robot.getCovariancePoseYY(),
        /*  8 */ robot.getCovariancePoseYawYaw(),
        /*  9 */ robot.getVelocityX(),
        /* 10 */ robot.getVelocityY(),
        /* 11 */ robot.getVelocityZ(),
        /* 12 */ robot.getVelocityTheta(),
        /* 13 */ robot.getCovarianceVelocityXX(),
        /* 14 */ robot.getCovarianceVelocityXY(),
        /* 15 */ robot.getCovarianceVelocityYX(),
        /* 16 */ robot.getCovarianceVelocityYY(),
        /* 17 */ robot.getCovarianceVelocityThTh(),
        /* 18 */ robot.getGoalPositionX(),
        /* 19 */ robot.getGoalPositionY(),
        /* 20 */ robot.getGoalOrientationYaw(),
        /* 21 */ robot.getDistToObstacle(),
        /* 22 */ robot.getLocalPlanningTime()
    );
    return std::string(buff);
}

RobotData RobotLogger::robotFromString(const std::string& str) {
    auto vals = people_msgs_utils::parseString<double>(str, " ");
    assert(vals.size() == 23);

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

    double obst_dist = vals.at(21);
    double exec_time = vals.at(22);

    return RobotData(pose, vel, goal, obst_dist, exec_time);
}

void RobotLogger::localizationCB(const nav_msgs::OdometryConstPtr& msg) {
    std::lock_guard<std::mutex> l(cb_mutex_);
    // save pose
    robot_pose_ = msg->pose;
}

} // namespace srpb
} // namespace logger
