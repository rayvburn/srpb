#include <gtest/gtest.h>

#include <srpb_logger/people_logger.h>

// Test cases
TEST(ConversionTest, person) {
    std::string name = "963";
    geometry_msgs::PoseWithCovariance pose;
    pose.pose.position.x = 0.123;
    pose.pose.position.y = 10.654;
    pose.pose.position.z = 12.321;
    // only `yaw` is logged
    tf2::Quaternion pose_quat;
    pose_quat.setRPY(0.0, 0.0, 0.632);
    pose.pose.orientation.x = pose_quat.getX();
    pose.pose.orientation.y = pose_quat.getY();
    pose.pose.orientation.z = pose_quat.getZ();
    pose.pose.orientation.w = pose_quat.getW();
    // only XX, XY, YX, YY, YawYaw are logged
    pose.covariance.at(people_msgs_utils::Person::COV_XX_INDEX) = 1.321;
    pose.covariance.at(people_msgs_utils::Person::COV_XY_INDEX) = 2.456;
    pose.covariance.at(people_msgs_utils::Person::COV_YX_INDEX) = 3.654;
    pose.covariance.at(people_msgs_utils::Person::COV_YY_INDEX) = 4.357;
    pose.covariance.at(people_msgs_utils::Person::COV_YAWYAW_INDEX) = 5.951;
    geometry_msgs::PoseWithCovariance velocity;
    velocity.pose.position.x = 1.098;
    velocity.pose.position.y = 2.345;
    velocity.pose.position.z = 6.543;
    // only `theta` is logged
    tf2::Quaternion vel_quat;
    vel_quat.setRPY(0.0, 0.0, 1.489);
    velocity.pose.orientation.x = vel_quat.getX();
    velocity.pose.orientation.y = vel_quat.getY();
    velocity.pose.orientation.z = vel_quat.getZ();
    velocity.pose.orientation.w = vel_quat.getW();
    // only XX, XY, YX, YY, YawYaw are logged
    velocity.covariance.at(people_msgs_utils::Person::COV_XX_INDEX) = 5.951;
    velocity.covariance.at(people_msgs_utils::Person::COV_XY_INDEX) = 4.159;
    velocity.covariance.at(people_msgs_utils::Person::COV_YX_INDEX) = 3.846;
    velocity.covariance.at(people_msgs_utils::Person::COV_YY_INDEX) = 2.624;
    velocity.covariance.at(people_msgs_utils::Person::COV_YAWYAW_INDEX) = 1.123;
    double reliability = 0.843;
    bool occluded = true;
    bool matched = true;
    unsigned int detection_id = 87654;
    unsigned long int track_age = 3456789;
    std::string group_name = "258";

    people_msgs_utils::Person p_in(
        name,
        pose,
        velocity,
        reliability,
        occluded,
        matched,
        detection_id,
        track_age,
        group_name
    );
    std::string p_str = PeopleLogger::personToString(p_in);
    auto p_out = PeopleLogger::personFromString(p_str);

    // 1, check if p_in's attributes are saved correctly
    ASSERT_EQ(p_in.getName(), name);
    ASSERT_EQ(p_in.getPositionX(), pose.pose.position.x);
    ASSERT_EQ(p_in.getPositionY(), pose.pose.position.y);
    ASSERT_EQ(p_in.getPositionZ(), pose.pose.position.z);
    ASSERT_EQ(p_in.getOrientation().x, pose.pose.orientation.x);
    ASSERT_EQ(p_in.getOrientation().y, pose.pose.orientation.y);
    ASSERT_EQ(p_in.getOrientation().z, pose.pose.orientation.z);
    ASSERT_EQ(p_in.getOrientation().w, pose.pose.orientation.w);
    for (size_t i = 0; i < p_in.getCovariancePose().size(); i++) {
        ASSERT_EQ(p_in.getCovariancePose().at(i), pose.covariance.at(i));
    }
    ASSERT_EQ(p_in.getVelocityX(), velocity.pose.position.x);
    ASSERT_EQ(p_in.getVelocityY(), velocity.pose.position.y);
    ASSERT_EQ(p_in.getVelocityZ(), velocity.pose.position.z);
    ASSERT_EQ(p_in.getVelocityTheta(), 1.489);
    for (size_t i = 0; i < p_in.getCovarianceVelocity().size(); i++) {
        ASSERT_EQ(p_in.getCovarianceVelocity().at(i), velocity.covariance.at(i));
    }
    ASSERT_EQ(p_in.getReliability(), reliability);
    ASSERT_EQ(p_in.isOccluded(), occluded);
    ASSERT_EQ(p_in.isMatched(), matched);
    ASSERT_EQ(p_in.getDetectionID(), detection_id);
    ASSERT_EQ(p_in.getTrackAge(), track_age);
    ASSERT_EQ(p_in.getGroupName(), group_name);

    // 2, check if p_out's attributes are equal to p_in's
    ASSERT_EQ(p_out.getName(), p_in.getName());
    ASSERT_EQ(p_out.getPositionX(), p_in.getPositionX());
    ASSERT_EQ(p_out.getPositionY(), p_in.getPositionY());
    ASSERT_EQ(p_out.getPositionZ(), p_in.getPositionZ());
    ASSERT_EQ(p_out.getOrientation().x, p_in.getOrientation().x);
    ASSERT_EQ(p_out.getOrientation().y, p_in.getOrientation().y);
    ASSERT_EQ(p_out.getOrientation().z, p_in.getOrientation().z);
    ASSERT_EQ(p_out.getOrientation().w, p_in.getOrientation().w);
    for (size_t i = 0; i < p_in.getCovariancePose().size(); i++) {
        ASSERT_EQ(p_out.getCovariancePose().at(i), p_in.getCovariancePose().at(i));
    }
    ASSERT_EQ(p_out.getVelocityX(), p_in.getVelocityX());
    ASSERT_EQ(p_out.getVelocityY(), p_in.getVelocityY());
    ASSERT_EQ(p_out.getVelocityZ(), p_in.getVelocityZ());
    ASSERT_EQ(p_out.getVelocityTheta(), p_in.getVelocityTheta());
    for (size_t i = 0; i < p_in.getCovarianceVelocity().size(); i++) {
        ASSERT_EQ(p_out.getCovarianceVelocity().at(i), p_in.getCovarianceVelocity().at(i));
    }
    ASSERT_EQ(p_out.getReliability(), p_in.getReliability());
    ASSERT_EQ(p_out.isOccluded(), p_in.isOccluded());
    ASSERT_EQ(p_out.isMatched(), p_in.isMatched());
    ASSERT_EQ(p_out.getDetectionID(), p_in.getDetectionID());
    ASSERT_EQ(p_out.getTrackAge(), p_in.getTrackAge());
    ASSERT_EQ(p_out.getGroupName(), p_in.getGroupName());
}

TEST(ConversionTest, group) {
    std::string name = "852";
    unsigned long int age = 9514862;
    std::vector<people_msgs_utils::Person> members{
        people_msgs_utils::Person(
            /* only this will be checked */ "2468",
            geometry_msgs::Point(),
            geometry_msgs::Point(),
            0.987,
            std::vector<std::string>(),
            std::vector<std::string>()
        ),
        people_msgs_utils::Person(
            /* only this will be checked */ "8462",
            geometry_msgs::Point(),
            geometry_msgs::Point(),
            0.654,
            std::vector<std::string>(),
            std::vector<std::string>()
        )
    };
    std::vector<unsigned int> member_ids{2468, 8462};
    // 2 members -> 1 relation
    std::vector<std::tuple<unsigned int, unsigned int, double>> relations{{2468, 8462, 0.753}};
    geometry_msgs::Point cog;
    cog.x = 5.684;
    cog.y = 6.953;
    cog.z = 7.854;

    people_msgs_utils::Group g_in(
        name,
        age,
        members,
        member_ids,
        relations,
        cog
    );
    std::string g_str = PeopleLogger::groupToString(g_in);
    auto g_out = PeopleLogger::groupFromString(g_str);

    // 1, check if g_in's attributes are saved correctly
    ASSERT_EQ(g_in.getAge(), age);
    ASSERT_EQ(g_in.getCenterOfGravity().x, cog.x);
    ASSERT_EQ(g_in.getCenterOfGravity().y, cog.y);
    ASSERT_EQ(g_in.getCenterOfGravity().z, cog.z);
    ASSERT_EQ(g_in.getName(), name);
    ASSERT_EQ(g_in.getMemberIDs(), member_ids);
    ASSERT_EQ(g_in.getMembers().size(), members.size());
    for (size_t i = 0; i < g_in.getMembers().size(); i++) {
        ASSERT_EQ(g_in.getMembers().at(i).getName(), members.at(i).getName());
    }
    ASSERT_EQ(g_in.getSocialRelations(), relations);

    // 2, check if g_out's attributes are equal to g_in's
    ASSERT_EQ(g_out.getAge(), g_in.getAge());
    ASSERT_EQ(g_out.getCenterOfGravity().x, g_in.getCenterOfGravity().x);
    ASSERT_EQ(g_out.getCenterOfGravity().y, g_in.getCenterOfGravity().y);
    ASSERT_EQ(g_out.getCenterOfGravity().z, g_in.getCenterOfGravity().z);
    ASSERT_EQ(g_out.getName(), g_in.getName());
    ASSERT_EQ(g_out.getMemberIDs(), g_in.getMemberIDs());
    // NOTE that converted group does not contain member copies (dummy group) ...
    EXPECT_EQ(g_out.getMembers().size(), 0);
    // ... doesn't make sense to iterate through people here (as above)
    ASSERT_EQ(g_out.getSocialRelations(), g_in.getSocialRelations());
}

int main(int argc, char** argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
