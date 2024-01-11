#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <tf2/utils.h>

#include <vector>

namespace srpb {
namespace logger {

class GlobalPlannerData {
public:
    GlobalPlannerData(
        const geometry_msgs::PoseStamped& robot_pose,
        const geometry_msgs::PoseStamped& goal_pose,
        bool planning_success,
        double planning_time,
        const std::vector<geometry_msgs::PoseStamped>& plan
    ): GlobalPlannerData(robot_pose.pose, goal_pose.pose, planning_success, planning_time, plan) {}

    GlobalPlannerData(
        const geometry_msgs::Pose& robot_pose,
        const geometry_msgs::Pose& goal_pose,
        bool planning_success,
        double planning_time,
        const std::vector<geometry_msgs::PoseStamped>& plan
    ):
        pose_(robot_pose),
        goal_(goal_pose),
        planning_success_(planning_success),
        planning_time_(planning_time),
        plan_(plan)
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

    /// Whether the planning was successful (i.e. the planner did not fail and returned a non-empty plan)
    inline bool getPlanningStatus() const {
        return planning_success_;
    }

    /// Returns duration of global plan computations
    inline double getPlanningTime() const {
        return planning_time_;
    }

    /// Returns a copy of a container with the global path plan @ref plan_
    inline std::vector<geometry_msgs::PoseStamped> getPlan() const {
        return plan_;
    }

    /// Returns a const reference to a container with the global path plan @ref plan_
    inline const std::vector<geometry_msgs::PoseStamped>& getPlanCref() const {
        return plan_;
    }

    /// Number of poses the global path plan @ref plan_ is expected to contain
    inline size_t getPlanSize() const {
        return plan_.size();
    }

protected:
    bool planning_success_;
    geometry_msgs::Pose pose_;
    geometry_msgs::Pose goal_;
    double planning_time_;
    std::vector<geometry_msgs::PoseStamped> plan_;
};

} // namespace logger
} // namespace srpb
