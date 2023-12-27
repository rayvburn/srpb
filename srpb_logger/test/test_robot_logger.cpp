#include <gtest/gtest.h>

#include <srpb_logger/robot_logger.h>

using namespace srpb::logger;

// Test cases
TEST(ConversionTest, robot) {
    geometry_msgs::PoseWithCovarianceStamped pose;
    pose.pose.pose.position.x = 0.123;
    pose.pose.pose.position.y = 10.654;
    pose.pose.pose.position.z = 12.321;
    tf2::Quaternion pose_quat;
    pose_quat.setRPY(0.0, 0.0, 0.632);
    pose.pose.pose.orientation.x = pose_quat.getX();
    pose.pose.pose.orientation.y = pose_quat.getY();
    pose.pose.pose.orientation.z = pose_quat.getZ();
    pose.pose.pose.orientation.w = pose_quat.getW();

    geometry_msgs::PoseWithCovarianceStamped vel;
    vel.pose.pose.position.x = 8.963;
    vel.pose.pose.position.y = 6.852;
    vel.pose.pose.position.z = 1.321;
    tf2::Quaternion vel_quat;
    vel_quat.setRPY(0.0, 0.0, 0.987);
    vel.pose.pose.orientation.x = vel_quat.getX();
    vel.pose.pose.orientation.y = vel_quat.getY();
    vel.pose.pose.orientation.z = vel_quat.getZ();
    vel.pose.pose.orientation.w = vel_quat.getW();

    geometry_msgs::PoseStamped goal;
    goal.pose.position.x = 100.354;
    goal.pose.position.y = 40.123;
    goal.pose.position.z = 11.325;
    tf2::Quaternion goal_quat;
    goal_quat.setRPY(0.0, 0.0, 1.3572);
    goal.pose.orientation.x = goal_quat.getX();
    goal.pose.orientation.y = goal_quat.getY();
    goal.pose.orientation.z = goal_quat.getZ();
    goal.pose.orientation.w = goal_quat.getW();

    geometry_msgs::TwistStamped cmd;
    cmd.twist.linear.x = 0.478;
    cmd.twist.linear.y = 0.321;
    cmd.twist.angular.z = 0.741;

    double obstacle_distance = 0.135;
    double exec_time = 0.497;

    RobotData data_in(
        pose,
        vel,
        goal,
        cmd,
        obstacle_distance,
        exec_time
    );

    // 1. evaluate if attributes were saved correctly
    ASSERT_EQ(data_in.getPositionX(), pose.pose.pose.position.x);
    ASSERT_EQ(data_in.getPositionY(), pose.pose.pose.position.y);
    ASSERT_EQ(data_in.getPositionZ(), pose.pose.pose.position.z);
    ASSERT_EQ(data_in.getOrientation().x, pose.pose.pose.orientation.x);
    ASSERT_EQ(data_in.getOrientation().y, pose.pose.pose.orientation.y);
    ASSERT_EQ(data_in.getOrientation().z, pose.pose.pose.orientation.z);
    ASSERT_EQ(data_in.getOrientation().w, pose.pose.pose.orientation.w);
    ASSERT_EQ(data_in.getCovariancePoseXX(), pose.pose.covariance.at(RobotData::COV_XX_INDEX));
    ASSERT_EQ(data_in.getCovariancePoseXY(), pose.pose.covariance.at(RobotData::COV_XY_INDEX));
    ASSERT_EQ(data_in.getCovariancePoseYX(), pose.pose.covariance.at(RobotData::COV_YX_INDEX));
    ASSERT_EQ(data_in.getCovariancePoseYY(), pose.pose.covariance.at(RobotData::COV_YY_INDEX));
    ASSERT_EQ(data_in.getCovariancePoseYawYaw(), pose.pose.covariance.at(RobotData::COV_YAWYAW_INDEX));

    ASSERT_EQ(data_in.getVelocityX(), vel.pose.pose.position.x);
    ASSERT_EQ(data_in.getVelocityY(), vel.pose.pose.position.y);
    ASSERT_EQ(data_in.getVelocityZ(), vel.pose.pose.position.z);
    ASSERT_EQ(data_in.getVelocityTheta(), tf2::getYaw(vel.pose.pose.orientation));
    ASSERT_EQ(data_in.getCovarianceVelocityXX(), vel.pose.covariance.at(RobotData::COV_XX_INDEX));
    ASSERT_EQ(data_in.getCovarianceVelocityXY(), vel.pose.covariance.at(RobotData::COV_XY_INDEX));
    ASSERT_EQ(data_in.getCovarianceVelocityYX(), vel.pose.covariance.at(RobotData::COV_YX_INDEX));
    ASSERT_EQ(data_in.getCovarianceVelocityYY(), vel.pose.covariance.at(RobotData::COV_YY_INDEX));
    ASSERT_EQ(data_in.getCovarianceVelocityThTh(), vel.pose.covariance.at(RobotData::COV_YAWYAW_INDEX));

    ASSERT_EQ(data_in.getGoalPositionX(), goal.pose.position.x);
    ASSERT_EQ(data_in.getGoalPositionY(), goal.pose.position.y);
    ASSERT_EQ(data_in.getGoalPositionZ(), goal.pose.position.z);
    ASSERT_DOUBLE_EQ(data_in.getGoalOrientationYaw(), tf2::getYaw(goal_quat));

    ASSERT_EQ(data_in.getVelocityCommandLinearX(), cmd.twist.linear.x);
    ASSERT_EQ(data_in.getVelocityCommandLinearY(), cmd.twist.linear.y);
    ASSERT_EQ(data_in.getVelocityCommandLinearZ(), cmd.twist.linear.z);
    ASSERT_EQ(data_in.getVelocityCommandAngularX(), cmd.twist.angular.x);
    ASSERT_EQ(data_in.getVelocityCommandAngularY(), cmd.twist.angular.y);
    ASSERT_EQ(data_in.getVelocityCommandAngularZ(), cmd.twist.angular.z);

    ASSERT_EQ(data_in.getDistToObstacle(), obstacle_distance);
    ASSERT_EQ(data_in.getLocalPlanningTime(), exec_time);

    auto data_in_str = RobotLogger::robotToString(data_in);
    auto conversion = RobotLogger::robotFromString(data_in_str);

    // conversion successful
    ASSERT_TRUE(conversion.first);
    RobotData data_out = conversion.second;

    // 2. check if "out" attributes are equal to "in"'s
    ASSERT_EQ(data_out.getPositionX(), data_in.getPositionX());
    ASSERT_EQ(data_out.getPositionY(), data_in.getPositionY());
    ASSERT_EQ(data_out.getPositionZ(), data_in.getPositionZ());
    ASSERT_EQ(data_out.getOrientation().x, data_in.getOrientation().x);
    ASSERT_EQ(data_out.getOrientation().y, data_in.getOrientation().y);
    ASSERT_EQ(data_out.getOrientation().z, data_in.getOrientation().z);
    ASSERT_EQ(data_out.getOrientation().w, data_in.getOrientation().w);
    ASSERT_EQ(data_out.getCovariancePoseXX(), data_in.getCovariancePoseXX());
    ASSERT_EQ(data_out.getCovariancePoseXY(), data_in.getCovariancePoseXY());
    ASSERT_EQ(data_out.getCovariancePoseYX(), data_in.getCovariancePoseYX());
    ASSERT_EQ(data_out.getCovariancePoseYY(), data_in.getCovariancePoseYY());
    ASSERT_EQ(data_out.getCovariancePoseYawYaw(), data_in.getCovariancePoseYawYaw());

    ASSERT_EQ(data_out.getVelocityX(), data_in.getVelocityX());
    ASSERT_EQ(data_out.getVelocityY(), data_in.getVelocityY());
    ASSERT_EQ(data_out.getVelocityZ(), data_in.getVelocityZ());
    ASSERT_EQ(data_out.getVelocityTheta(), data_in.getVelocityTheta());
    ASSERT_EQ(data_out.getCovarianceVelocityXX(), data_in.getCovarianceVelocityXX());
    ASSERT_EQ(data_out.getCovarianceVelocityXY(), data_in.getCovarianceVelocityXY());
    ASSERT_EQ(data_out.getCovarianceVelocityYX(), data_in.getCovarianceVelocityYX());
    ASSERT_EQ(data_out.getCovarianceVelocityYY(), data_in.getCovarianceVelocityYY());
    ASSERT_EQ(data_out.getCovarianceVelocityThTh(), data_in.getCovarianceVelocityThTh());

    ASSERT_EQ(data_out.getGoalPositionX(), data_in.getGoalPositionX());
    ASSERT_EQ(data_out.getGoalPositionY(), data_in.getGoalPositionY());
    // not logged
    // ASSERT_EQ(data_out.getGoalPositionZ(), data_in.getGoalPositionZ());
    ASSERT_EQ(data_out.getGoalOrientationYaw(), data_in.getGoalOrientationYaw());

    ASSERT_EQ(data_out.getVelocityCommandLinearX(), data_in.getVelocityCommandLinearX());
    ASSERT_EQ(data_out.getVelocityCommandLinearY(), data_in.getVelocityCommandLinearY());
    ASSERT_EQ(data_out.getVelocityCommandLinearZ(), data_in.getVelocityCommandLinearZ());
    ASSERT_EQ(data_out.getVelocityCommandAngularX(), data_in.getVelocityCommandAngularX());
    ASSERT_EQ(data_out.getVelocityCommandAngularY(), data_in.getVelocityCommandAngularY());
    ASSERT_EQ(data_out.getVelocityCommandAngularZ(), data_in.getVelocityCommandAngularZ());

    ASSERT_EQ(data_out.getLocalPlanningTime(), data_in.getLocalPlanningTime());
    ASSERT_EQ(data_out.getDistToObstacle(), data_in.getDistToObstacle());
}

int main(int argc, char** argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
