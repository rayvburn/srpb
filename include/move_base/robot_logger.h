#pragma once

#include "benchmark_logger.h"
#include <tf2/utils.h>

class RobotLogger: public BenchmarkLogger {
public:
    RobotLogger() = default;

    void start() {
        BenchmarkLogger::start(&log_file_, log_filename_);
    }

    /// Must be called prior to @ref update that takes 1 argument
    void update(
        double timestamp,
        const geometry_msgs::PoseStamped& robot_pose,
        const geometry_msgs::PoseStamped& robot_vel,
        double obstacle_distance
    ) {
        fprintf(
            log_file_,
            "%.3f %.3f %.3f %.3f %.3f %.3f %.3f ",
            timestamp,
            robot_pose.pose.position.x,
            robot_pose.pose.position.y,
            tf2::getYaw(robot_pose.pose.orientation),
            robot_vel.pose.position.x,
            tf2::getYaw(robot_vel.pose.orientation),
            obstacle_distance
        );
    }

    void update(double exec_time) {
        fprintf(log_file_, "%.3f\n", exec_time);
    }

    void finish() {
        BenchmarkLogger::finish(&log_file_);
    }

protected:
    FILE* log_file_;
};
