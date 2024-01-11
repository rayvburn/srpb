#include <gtest/gtest.h>

#include <srpb_logger/global_planner_logger.h>
#include <angles/angles.h>

using namespace srpb::logger;

// Test cases
TEST(ConversionTest, planner) {
    geometry_msgs::Pose pose;
    pose.position.x = 0.123;
    pose.position.y = 10.654;
    pose.position.z = 12.321;
    // only `yaw` is logged
    tf2::Quaternion pose_quat;
    pose_quat.setRPY(0.0, 0.0, 0.632);
    pose.orientation.x = pose_quat.getX();
    pose.orientation.y = pose_quat.getY();
    pose.orientation.z = pose_quat.getZ();
    pose.orientation.w = pose_quat.getW();

    geometry_msgs::Pose goal;
    goal.position.x = 100.354;
    goal.position.y = 40.123;
    goal.position.z = 11.325;
    // only `yaw` is logged
    tf2::Quaternion goal_quat;
    goal_quat.setRPY(0.0, 0.0, 1.5789);
    goal.orientation.x = goal_quat.getX();
    goal.orientation.y = goal_quat.getY();
    goal.orientation.z = goal_quat.getZ();
    goal.orientation.w = goal_quat.getW();

    bool planning_success = true;
    double planning_time = 0.135;
    size_t plan_size = 497;
    // create a arbitrary path plan
    std::vector<geometry_msgs::PoseStamped> plan;
    plan.resize(plan_size);
    double x_plan = -5.159;
    double y_plan = 20.951;
    double yaw_plan = 0.357;
    for (auto& pose_stamped: plan) {
        // modify vector element
        pose_stamped.pose.position.x = x_plan;
        pose_stamped.pose.position.y = y_plan;
        // NOTE: ctor of tf2::Quaternion using Euler angles is deprecated
        tf2::Quaternion quat;
        quat.setEuler(0.0, 0.0, yaw_plan);
        pose_stamped.pose.orientation = tf2::toMsg(quat);
        // prep coords for the next iteration
        x_plan += 0.142;
        y_plan -= 0.249;
        yaw_plan = angles::normalize_angle(yaw_plan + 0.029);
    }

    GlobalPlannerData data_in(pose, goal, planning_success, planning_time, plan);

    // 1. evaluate if attributes were saved correctly
    ASSERT_EQ(data_in.getPositionX(), pose.position.x);
    ASSERT_EQ(data_in.getPositionY(), pose.position.y);
    ASSERT_EQ(data_in.getPositionZ(), pose.position.z);
    ASSERT_EQ(data_in.getOrientation().x, pose.orientation.x);
    ASSERT_EQ(data_in.getOrientation().y, pose.orientation.y);
    ASSERT_EQ(data_in.getOrientation().z, pose.orientation.z);
    ASSERT_EQ(data_in.getOrientation().w, pose.orientation.w);
    ASSERT_EQ(data_in.getGoalPositionX(), goal.position.x);
    ASSERT_EQ(data_in.getGoalPositionY(), goal.position.y);
    ASSERT_EQ(data_in.getGoalPositionZ(), goal.position.z);
    ASSERT_DOUBLE_EQ(data_in.getGoalOrientationYaw(), tf2::getYaw(goal_quat));
    ASSERT_EQ(data_in.getPlanningStatus(), planning_success);
    ASSERT_EQ(data_in.getPlanningTime(), planning_time);
    ASSERT_EQ(data_in.getPlanSize(), plan_size);
    ASSERT_EQ(data_in.getPlan(), plan);

    auto data_in_str = GlobalPlannerLogger::plannerToString(data_in);
    auto conversion = GlobalPlannerLogger::plannerFromString(data_in_str);

    // conversion successful
    ASSERT_TRUE(conversion.first);
    GlobalPlannerData data_out = conversion.second;

    // 2. check if "out" attributes are equal to "in"'s
    ASSERT_EQ(data_out.getPositionX(), data_in.getPositionX());
    ASSERT_EQ(data_out.getPositionY(), data_in.getPositionY());
    ASSERT_EQ(data_out.getPositionZ(), data_in.getPositionZ());
    ASSERT_EQ(data_out.getOrientation().x, data_in.getOrientation().x);
    ASSERT_EQ(data_out.getOrientation().y, data_in.getOrientation().y);
    ASSERT_EQ(data_out.getOrientation().z, data_in.getOrientation().z);
    ASSERT_EQ(data_out.getOrientation().w, data_in.getOrientation().w);
    ASSERT_EQ(data_out.getGoalPositionX(), data_in.getGoalPositionX());
    ASSERT_EQ(data_out.getGoalPositionY(), data_in.getGoalPositionY());
    ASSERT_EQ(data_out.getGoalPositionZ(), data_in.getGoalPositionZ());
    ASSERT_EQ(data_out.getGoalOrientationYaw(), data_in.getGoalOrientationYaw());
    ASSERT_EQ(data_out.getPlanningStatus(), data_in.getPlanningStatus());
    ASSERT_EQ(data_out.getPlanningTime(), data_in.getPlanningTime());
    ASSERT_EQ(data_out.getPlanSize(), data_in.getPlanSize());
    /* NOTE: cannot simply use:
     *   ASSERT_EQ(data_out.getPlan(), data_in.getPlan());
     * here, as it uses a serialized version (?) of the message structure; despite the same contents (with 1e-04
     * accuracy), there may be some whitespace differences
     */
    auto out_plan = data_out.getPlanCref();
    auto in_plan = data_in.getPlanCref();
    for (
        auto out_it = out_plan.cbegin(), in_it = in_plan.cbegin();
        out_it != out_plan.cend() || in_it != in_plan.cend();
        out_it++, in_it++
    ) {
        auto out_pose = *out_it;
        auto in_pose = *in_it;
        ASSERT_NEAR(out_pose.pose.position.x, in_pose.pose.position.x, 1e-04);
        ASSERT_NEAR(out_pose.pose.position.y, in_pose.pose.position.y, 1e-04);
        ASSERT_NEAR(tf2::getYaw(out_pose.pose.orientation), tf2::getYaw(in_pose.pose.orientation), 1e-04);
    }
}

int main(int argc, char** argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
