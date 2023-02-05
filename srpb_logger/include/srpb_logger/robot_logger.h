#pragma once

#include "benchmark_logger.h"
#include "robot_data.h"
#include <tf2/utils.h>

// helper functions
#include <people_msgs_utils/utils.h>

namespace srpb {
namespace logger {

class RobotLogger: public BenchmarkLogger {
public:
    RobotLogger() = default;

    void init(ros::NodeHandle& nh) {
        BenchmarkLogger::init(nh);

        auto log_path_base = log_filename_.substr(0, log_filename_.find_last_of(EXTENSION_SEPARATOR));
        auto log_extension = log_filename_.substr(log_filename_.find_last_of(EXTENSION_SEPARATOR) + 1);

        log_filename_ = log_path_base + "_robot" + EXTENSION_SEPARATOR + log_extension;
    }

    void start() {
        BenchmarkLogger::start(&log_file_, log_filename_);
    }

    void update(double timestamp, const RobotData& robot) {
        if (log_file_ == nullptr) {
            throw std::runtime_error("File descriptor for RobotLogger was not properly created!");
        }

        fprintf(
            log_file_,
            "%9.4f %s\n",
            timestamp,
            RobotLogger::robotToString(robot).c_str()
        );
    }

    void finish() {
        BenchmarkLogger::finish(&log_file_);
    }

    /// Converts given robot data into string description
    static std::string robotToString(const RobotData& robot) {
        char buff[150];
        sprintf(
            buff,
            "%9.4f %9.4f %9.4f %9.4f %9.4f %9.4f %9.4f %9.4f %9.4f %9.4f %9.4f",
            robot.getPositionX(),
            robot.getPositionY(),
            robot.getOrientationYaw(),
            robot.getVelocityX(),
            robot.getVelocityY(),
            robot.getVelocityTheta(),
            robot.getGoalPositionX(),
            robot.getGoalPositionY(),
            robot.getGoalOrientationYaw(),
            robot.getDistToObstacle(),
            robot.getLocalPlanningTime()
        );
        return std::string(buff);
    }

    /// Converts given @ref str string description into robot data
    static RobotData robotFromString(const std::string& str) {
        auto vals = people_msgs_utils::parseString<double>(str, " ");
        assert(vals.size() == 11);

        geometry_msgs::Pose pose;
        pose.position.x = vals.at(0);
        pose.position.y = vals.at(1);
        tf2::Quaternion quat;
        quat.setRPY(0.0, 0.0, vals.at(2));
        pose.orientation.x = quat.getX();
        pose.orientation.y = quat.getY();
        pose.orientation.z = quat.getZ();
        pose.orientation.w = quat.getW();

        geometry_msgs::Pose vel;
        vel.position.x = vals.at(3);
        vel.position.y = vals.at(4);
        quat.setRPY(0.0, 0.0, vals.at(5));
        vel.orientation.x = quat.getX();
        vel.orientation.y = quat.getY();
        vel.orientation.z = quat.getZ();
        vel.orientation.w = quat.getW();

        geometry_msgs::Pose goal;
        goal.position.x = vals.at(6);
        goal.position.y = vals.at(7);
        quat.setRPY(0.0, 0.0, vals.at(8));
        goal.orientation.x = quat.getX();
        goal.orientation.y = quat.getY();
        goal.orientation.z = quat.getZ();
        goal.orientation.w = quat.getW();

        double obst_dist = vals.at(9);
        double exec_time = vals.at(10);

        return RobotData(pose, vel, goal, obst_dist, exec_time);
    }

protected:
    FILE* log_file_;
};

} // namespace logger
} // namespace srpb
