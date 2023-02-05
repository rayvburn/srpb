#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <tf2/utils.h>

namespace srpb {
namespace logger {

class RobotData {
public:
    RobotData(
        const geometry_msgs::PoseStamped& robot_pose,
        const geometry_msgs::PoseStamped& robot_vel,
        const geometry_msgs::PoseStamped& goal_pose,
        double obstacle_distance,
        double exec_time
    ): RobotData(robot_pose.pose, robot_vel.pose, goal_pose.pose, obstacle_distance, exec_time) {}

    RobotData(
        const geometry_msgs::Pose& robot_pose,
        const geometry_msgs::Pose& robot_vel,
        const geometry_msgs::Pose& goal_pose,
        double obstacle_distance,
        double exec_time
    ):
        pose_(robot_pose),
        vel_(robot_vel),
        goal_(goal_pose),
        obstacle_distance_(obstacle_distance),
        local_plan_exec_time_(exec_time)
    {}

    inline geometry_msgs::Pose getPose() const {
        return pose_;
    }

    inline geometry_msgs::Point getPosition() const {
        return pose_.position;
    }

    inline geometry_msgs::Quaternion getOrientation() const {
        return pose_.orientation;
    }

    inline double getPositionX() const {
        return pose_.position.x;
    }

    inline double getPositionY() const {
        return pose_.position.y;
    }

    inline double getPositionZ() const {
        return pose_.position.z;
    }

    inline double getOrientationYaw() const {
        return tf2::getYaw(pose_.orientation);
    }

    inline geometry_msgs::Pose getVelocity() const {
        return vel_;
    }

    inline double getVelocityX() const {
        return vel_.position.x;
    }

    inline double getVelocityY() const {
        return vel_.position.y;
    }

    inline double getVelocityZ() const {
        return vel_.position.z;
    }

    inline double getVelocityTheta() const {
        return tf2::getYaw(vel_.orientation);
    }

    inline geometry_msgs::Pose getGoal() const {
        return goal_;
    }

    inline double getGoalPositionX() const {
        return goal_.position.x;
    }

    inline double getGoalPositionY() const {
        return goal_.position.y;
    }

    inline double getGoalPositionZ() const {
        return goal_.position.z;
    }

    inline double getGoalOrientationYaw() const {
        return tf2::getYaw(goal_.orientation);
    }

    inline double getDistToObstacle() const {
        return obstacle_distance_;
    }

    /// Returns duration of local plan computations
    inline double getLocalPlanningTime() const {
        return local_plan_exec_time_;
    }

protected:
    geometry_msgs::Pose pose_;
    geometry_msgs::Pose vel_;
    geometry_msgs::Pose goal_;
    double obstacle_distance_;
    double local_plan_exec_time_;
};

} // namespace logger
} // namespace srpb
