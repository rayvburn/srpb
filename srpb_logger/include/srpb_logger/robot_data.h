#pragma once

#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <tf2/utils.h>

namespace srpb {
namespace logger {

class RobotData {
public:
    static constexpr auto COV_MAT_SIZE = 36;
    static constexpr auto COV_XX_INDEX = 0;
    static constexpr auto COV_XY_INDEX = 1;
    static constexpr auto COV_YX_INDEX = 6;
    static constexpr auto COV_YY_INDEX = 7;
    static constexpr auto COV_ZZ_INDEX = 14;
    static constexpr auto COV_ROLLROLL_INDEX = 21;
    static constexpr auto COV_PITCHPITCH_INDEX = 28;
    static constexpr auto COV_YAWYAW_INDEX = COV_MAT_SIZE - 1;

    /// Full constructor for the RobotLogger
    RobotData(
        const geometry_msgs::PoseWithCovarianceStamped& robot_pose,
        const geometry_msgs::PoseWithCovarianceStamped& robot_vel,
        const geometry_msgs::PoseStamped& goal_pose,
        double obstacle_distance,
        double exec_time
    ): RobotData(robot_pose.pose, robot_vel.pose, goal_pose.pose, obstacle_distance, exec_time) {}

    /// Full constructor for the RobotLogger
    RobotData(
        const geometry_msgs::PoseWithCovariance& robot_pose,
        const geometry_msgs::PoseWithCovariance& robot_vel,
        const geometry_msgs::Pose& goal_pose,
        double obstacle_distance,
        double exec_time
    ):
        pose_(robot_pose),
        vel_(robot_vel),
        goal_(goal_pose),
        obstacle_distance_(obstacle_distance),
        local_plan_exec_time_(exec_time),
        partial_(false)
    {}

    /// Partial constructor for the `move_base` node
    RobotData(
        const geometry_msgs::PoseStamped& goal_pose,
        double obstacle_distance,
        double exec_time
    ): RobotData(goal_pose.pose, obstacle_distance, exec_time) {}

    /// Partial constructor for the `move_base` node
    RobotData(
        const geometry_msgs::Pose& goal_pose,
        double obstacle_distance,
        double exec_time
    ):
        goal_(goal_pose),
        obstacle_distance_(obstacle_distance),
        local_plan_exec_time_(exec_time),
        partial_(true)
    {
        pose_.pose.position.x = NAN;
        pose_.pose.position.y = NAN;
        pose_.pose.position.z = NAN;
        pose_.pose.orientation.x = NAN;
        pose_.pose.orientation.y = NAN;
        pose_.pose.orientation.z = NAN;
        pose_.pose.orientation.w = NAN;
        pose_.covariance.fill(NAN);
        vel_ = pose_;
    }

    inline geometry_msgs::Pose getPose() const {
        return pose_.pose;
    }

    inline geometry_msgs::Point getPosition() const {
        return pose_.pose.position;
    }

    inline geometry_msgs::Quaternion getOrientation() const {
        return pose_.pose.orientation;
    }

    inline double getPositionX() const {
        return pose_.pose.position.x;
    }

    inline double getPositionY() const {
        return pose_.pose.position.y;
    }

    inline double getPositionZ() const {
        return pose_.pose.position.z;
    }

    inline double getOrientationYaw() const {
        return tf2::getYaw(pose_.pose.orientation);
    }

    /// @return 6x6 matrix with covariance values
	inline std::array<double, COV_MAT_SIZE> getCovariancePose() const {
		std::array<double, COV_MAT_SIZE> arr;
		std::copy(pose_.covariance.begin(), pose_.covariance.end(), arr.begin());
		return arr;
	}

    inline double getCovariancePoseXX() const {
        return pose_.covariance.at(COV_XX_INDEX);
    }

    inline double getCovariancePoseXY() const {
        return pose_.covariance.at(COV_XY_INDEX);
    }

    inline double getCovariancePoseYX() const {
        return pose_.covariance.at(COV_YX_INDEX);
    }

    inline double getCovariancePoseYY() const {
        return pose_.covariance.at(COV_YY_INDEX);
    }

    inline double getCovariancePoseYawYaw() const {
        return pose_.covariance.at(COV_YAWYAW_INDEX);
    }

    inline geometry_msgs::Pose getVelocity() const {
        return vel_.pose;
    }

    inline double getVelocityX() const {
        return vel_.pose.position.x;
    }

    inline double getVelocityY() const {
        return vel_.pose.position.y;
    }

    inline double getVelocityZ() const {
        return vel_.pose.position.z;
    }

    inline double getVelocityTheta() const {
        return tf2::getYaw(vel_.pose.orientation);
    }

    /// @return 6x6 matrix with covariance values
    inline std::array<double, COV_MAT_SIZE> getCovarianceVelocity() const {
        std::array<double, COV_MAT_SIZE> arr;
        std::copy(vel_.covariance.begin(), vel_.covariance.end(), arr.begin());
        return arr;
    }

    inline double getCovarianceVelocityXX() const {
        return vel_.covariance[COV_XX_INDEX];
    }

    inline double getCovarianceVelocityXY() const {
        return vel_.covariance[COV_XY_INDEX];
    }

    inline double getCovarianceVelocityYX() const {
        return vel_.covariance[COV_YX_INDEX];
    }

    inline double getCovarianceVelocityYY() const {
        return vel_.covariance[COV_YY_INDEX];
    }

    inline double getCovarianceVelocityThTh() const {
        return vel_.covariance[COV_YAWYAW_INDEX];
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

    // Whether valid data is present in all structures (e.g., if covariances are available)
    inline bool hasPartialData() const {
        return partial_;
    }

protected:
    geometry_msgs::PoseWithCovariance pose_;
    geometry_msgs::PoseWithCovariance vel_;
    geometry_msgs::Pose goal_;
    double obstacle_distance_;
    double local_plan_exec_time_;

    bool partial_;
};

} // namespace logger
} // namespace srpb
