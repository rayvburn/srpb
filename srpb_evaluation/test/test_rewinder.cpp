#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <srpb_evaluation/rewinder.h>

using namespace srpb;

// dummy objects to fill up the data vectors
const logger::RobotData DUMMY_ROBOT(
    geometry_msgs::PoseStamped(),
    geometry_msgs::PoseStamped(),
    geometry_msgs::PoseStamped(),
    0.5,
    0.1
);
people_msgs_utils::Person createPerson(const std::string& name) {
    return people_msgs_utils::Person(
        name,
        geometry_msgs::PoseWithCovariance(),
        geometry_msgs::PoseWithCovariance(),
        1.0,
        true,
        true,
        951,
        357,
        "123"
    );
}
const people_msgs_utils::Group DUMMY_GROUP = people_msgs_utils::EMPTY_GROUP;

/*
 * Mocking references:
 * - http://google.github.io/googletest/gmock_cook_book.html
 * - https://stackoverflow.com/questions/42521088/how-to-use-gmock-to-mock-up-a-stdfunction
 */
/*
 * General rule of how to find expected number of calls
 * - mockCbNextTimestamp.Times: num of entries of robot minus one
 * - mockCbLastTimestamp.Times: always 1 -> last entry of processing robot data
 * - mockCbNextPersonTimestamp.Times: accumulated: number of people in timestamp times groups
 * - mockCbAllPeopleTimestamp.Times: unique timestamps, times groups in timestamp if a group exists, otherwise * 1
 *     - single group in a time step does not make any difference
 * - mockCbNextGroupTimestamp.Times: total entries of groups
 * - mockCbAllGroupsTimestamp.Times: number of unique timestamps of groups
 */
TEST(TestRewinder, robot) {
    std::vector<std::pair<double, logger::RobotData>> robot_data{
        {2.50, DUMMY_ROBOT},
        {2.75, DUMMY_ROBOT},
        {3.00, DUMMY_ROBOT},
        {3.25, DUMMY_ROBOT},
        {3.50, DUMMY_ROBOT},
        {3.75, DUMMY_ROBOT},
        {4.00, DUMMY_ROBOT}
    };

    testing::MockFunction<void(void)> mockCbNextTimestamp;
    EXPECT_CALL(mockCbNextTimestamp, Call()).Times(6);
    testing::MockFunction<void(void)> mockCbLastTimestamp;
    EXPECT_CALL(mockCbLastTimestamp, Call()).Times(1);
    testing::MockFunction<void(void)> mockCbNextPersonTimestamp;
    EXPECT_CALL(mockCbNextPersonTimestamp, Call()).Times(0);
    testing::MockFunction<void(void)> mockCbAllPeopleTimestamp;
    EXPECT_CALL(mockCbAllPeopleTimestamp, Call()).Times(0);
    testing::MockFunction<void(void)> mockCbNextGroupTimestamp;
    EXPECT_CALL(mockCbNextGroupTimestamp, Call()).Times(0);
    testing::MockFunction<void(void)> mockCbAllGroupsTimestamp;
    EXPECT_CALL(mockCbAllGroupsTimestamp, Call()).Times(0);

    evaluation::Rewinder rew(robot_data);
    rew.setHandlerNextTimestamp(mockCbNextTimestamp.AsStdFunction());
    rew.setHandlerLastTimestamp(mockCbLastTimestamp.AsStdFunction());
    rew.setHandlerNextPersonTimestamp(mockCbNextPersonTimestamp.AsStdFunction());
    rew.setHandlerAllPeopleTimestamp(mockCbAllPeopleTimestamp.AsStdFunction());
    rew.setHandlerNextGroupTimestamp(mockCbNextGroupTimestamp.AsStdFunction());
    rew.setHandlerAllGroupsTimestamp(mockCbAllGroupsTimestamp.AsStdFunction());
    rew.perform();
}

TEST(TestRewinder, robotPeople) {
    std::vector<std::pair<double, logger::RobotData>> robot_data{
        {2.50, DUMMY_ROBOT},
        {2.75, DUMMY_ROBOT},
        {3.00, DUMMY_ROBOT},
        {3.25, DUMMY_ROBOT},
        {3.50, DUMMY_ROBOT},
        {3.75, DUMMY_ROBOT},
        {4.00, DUMMY_ROBOT},
        {4.50, DUMMY_ROBOT},
        {5.00, DUMMY_ROBOT}
    };
    std::vector<std::pair<double, people_msgs_utils::Person>> people_data{
        {2.50, createPerson("00")},
        {3.25, createPerson("01")},
        {3.50, createPerson("02")},
        {3.75, createPerson("03")},
        {5.00, createPerson("04")}
    };

    testing::MockFunction<void(void)> mockCbNextTimestamp;
    EXPECT_CALL(mockCbNextTimestamp, Call())
        .Times(8);
    testing::MockFunction<void(void)> mockCbLastTimestamp;
    EXPECT_CALL(mockCbLastTimestamp, Call())
        .Times(1);
    testing::MockFunction<void(void)> mockCbNextPersonTimestamp;
    EXPECT_CALL(mockCbNextPersonTimestamp, Call())
        .Times(5);
    testing::MockFunction<void(void)> mockCbAllPeopleTimestamp;
    EXPECT_CALL(mockCbAllPeopleTimestamp, Call())
        .Times(5);
    testing::MockFunction<void(void)> mockCbNextGroupTimestamp;
    EXPECT_CALL(mockCbNextGroupTimestamp, Call())
        .Times(0);
    testing::MockFunction<void(void)> mockCbAllGroupsTimestamp;
    EXPECT_CALL(mockCbAllGroupsTimestamp, Call())
        .Times(0);

    evaluation::Rewinder rew(robot_data, people_data);
    rew.setHandlerNextTimestamp(mockCbNextTimestamp.AsStdFunction());
    rew.setHandlerLastTimestamp(mockCbLastTimestamp.AsStdFunction());
    rew.setHandlerNextPersonTimestamp(mockCbNextPersonTimestamp.AsStdFunction());
    rew.setHandlerAllPeopleTimestamp(mockCbAllPeopleTimestamp.AsStdFunction());
    rew.setHandlerNextGroupTimestamp(mockCbNextGroupTimestamp.AsStdFunction());
    rew.setHandlerAllGroupsTimestamp(mockCbAllGroupsTimestamp.AsStdFunction());
    rew.perform();
}

TEST(TestRewinder, robotPeopleGroups1) {
    std::vector<std::pair<double, logger::RobotData>> robot_data{
        {2.50, DUMMY_ROBOT},
        {3.00, DUMMY_ROBOT},
        {3.25, DUMMY_ROBOT},
        {3.50, DUMMY_ROBOT},
        {3.75, DUMMY_ROBOT},
        {4.00, DUMMY_ROBOT},
        {4.50, DUMMY_ROBOT},
        {5.00, DUMMY_ROBOT}
    };
    std::vector<std::pair<double, people_msgs_utils::Person>> people_data{
        {2.50, createPerson("00")},
        {2.50, createPerson("01")},
        {3.25, createPerson("02")},
        {3.50, createPerson("03")},
        {3.75, createPerson("04")},
        {5.00, createPerson("05")}
    };
    std::vector<std::pair<double, people_msgs_utils::Group>> group_data{
        {3.25, DUMMY_GROUP},
        {3.50, DUMMY_GROUP},
        {5.00, DUMMY_GROUP}
    };

    testing::MockFunction<void(void)> mockCbNextTimestamp;
    EXPECT_CALL(mockCbNextTimestamp, Call())
        .Times(7);
    testing::MockFunction<void(void)> mockCbLastTimestamp;
    EXPECT_CALL(mockCbLastTimestamp, Call())
        .Times(1);
    testing::MockFunction<void(void)> mockCbNextPersonTimestamp;
    EXPECT_CALL(mockCbNextPersonTimestamp, Call())
        .Times(6);
    testing::MockFunction<void(void)> mockCbAllPeopleTimestamp;
    EXPECT_CALL(mockCbAllPeopleTimestamp, Call())
        .Times(5);
    testing::MockFunction<void(void)> mockCbNextGroupTimestamp;
    EXPECT_CALL(mockCbNextGroupTimestamp, Call())
        .Times(3);
    testing::MockFunction<void(void)> mockCbAllGroupsTimestamp;
    EXPECT_CALL(mockCbAllGroupsTimestamp, Call())
        .Times(3);

    evaluation::Rewinder rew(robot_data, people_data, group_data);
    rew.setHandlerNextTimestamp(mockCbNextTimestamp.AsStdFunction());
    rew.setHandlerLastTimestamp(mockCbLastTimestamp.AsStdFunction());
    rew.setHandlerNextPersonTimestamp(mockCbNextPersonTimestamp.AsStdFunction());
    rew.setHandlerAllPeopleTimestamp(mockCbAllPeopleTimestamp.AsStdFunction());
    rew.setHandlerNextGroupTimestamp(mockCbNextGroupTimestamp.AsStdFunction());
    rew.setHandlerAllGroupsTimestamp(mockCbAllGroupsTimestamp.AsStdFunction());
    rew.perform();
}

TEST(TestRewinder, robotPeopleGroups2) {
    std::vector<std::pair<double, logger::RobotData>> robot_data{
        {2.50, DUMMY_ROBOT},
        {3.00, DUMMY_ROBOT},
        {3.25, DUMMY_ROBOT},
        {3.50, DUMMY_ROBOT},
        {3.75, DUMMY_ROBOT},
        {4.00, DUMMY_ROBOT},
        {4.50, DUMMY_ROBOT},
        {5.00, DUMMY_ROBOT},
        {5.50, DUMMY_ROBOT},
        {6.00, DUMMY_ROBOT},
        {6.25, DUMMY_ROBOT},
        {6.50, DUMMY_ROBOT},
        {6.75, DUMMY_ROBOT},
        {7.00, DUMMY_ROBOT},
        {7.50, DUMMY_ROBOT},
        {8.00, DUMMY_ROBOT}
    };
    std::vector<std::pair<double, people_msgs_utils::Person>> people_data{
        {2.50, createPerson("00")},
        {3.00, createPerson("01")},
        {3.25, createPerson("02")},
        {3.50, createPerson("03")},
        {3.75, createPerson("04")},
        {4.00, createPerson("05")},
        {4.50, createPerson("06")},
        {5.00, createPerson("07")},
        {5.50, createPerson("08")},
        {6.00, createPerson("09")},
        {6.25, createPerson("10")},
        {6.50, createPerson("11")},
        {6.75, createPerson("12")},
        {7.00, createPerson("13")},
        {7.50, createPerson("14")},
        {8.00, createPerson("15")}
    };
    std::vector<std::pair<double, people_msgs_utils::Group>> group_data{
        {2.50, DUMMY_GROUP},
        {3.25, DUMMY_GROUP},
        {3.50, DUMMY_GROUP},
        {4.00, DUMMY_GROUP},
        {4.50, DUMMY_GROUP},
        {5.00, DUMMY_GROUP},
        {6.00, DUMMY_GROUP},
        {6.25, DUMMY_GROUP},
        {6.50, DUMMY_GROUP},
        {6.75, DUMMY_GROUP},
        {7.50, DUMMY_GROUP},
        {8.00, DUMMY_GROUP}
    };

    testing::MockFunction<void(void)> mockCbNextTimestamp;
    EXPECT_CALL(mockCbNextTimestamp, Call())
        .Times(15);
    testing::MockFunction<void(void)> mockCbLastTimestamp;
    EXPECT_CALL(mockCbLastTimestamp, Call())
        .Times(1);
    testing::MockFunction<void(void)> mockCbNextPersonTimestamp;
    EXPECT_CALL(mockCbNextPersonTimestamp, Call())
        .Times(16);
    testing::MockFunction<void(void)> mockCbAllPeopleTimestamp;
    EXPECT_CALL(mockCbAllPeopleTimestamp, Call())
        .Times(16);
    testing::MockFunction<void(void)> mockCbNextGroupTimestamp;
    EXPECT_CALL(mockCbNextGroupTimestamp, Call())
        .Times(12);
    testing::MockFunction<void(void)> mockCbAllGroupsTimestamp;
    EXPECT_CALL(mockCbAllGroupsTimestamp, Call())
        .Times(12);

    evaluation::Rewinder rew(robot_data, people_data, group_data);
    rew.setHandlerNextTimestamp(mockCbNextTimestamp.AsStdFunction());
    rew.setHandlerLastTimestamp(mockCbLastTimestamp.AsStdFunction());
    rew.setHandlerNextPersonTimestamp(mockCbNextPersonTimestamp.AsStdFunction());
    rew.setHandlerAllPeopleTimestamp(mockCbAllPeopleTimestamp.AsStdFunction());
    rew.setHandlerNextGroupTimestamp(mockCbNextGroupTimestamp.AsStdFunction());
    rew.setHandlerAllGroupsTimestamp(mockCbAllGroupsTimestamp.AsStdFunction());
    rew.perform();
}

TEST(TestRewinder, robotPeopleGroups3) {
    std::vector<std::pair<double, logger::RobotData>> robot_data{
        {2.50, DUMMY_ROBOT},
        {3.50, DUMMY_ROBOT},
        {4.50, DUMMY_ROBOT},
        {5.50, DUMMY_ROBOT}
    };
    std::vector<std::pair<double, people_msgs_utils::Person>> people_data{
        {2.50, createPerson("00")},
        {2.50, createPerson("01")},
        {2.50, createPerson("02")},
        {3.50, createPerson("03")},
        {4.50, createPerson("04")},
        {4.50, createPerson("05")},
        {5.50, createPerson("06")},
        {5.50, createPerson("07")},
        {5.50, createPerson("08")}
    };
    std::vector<std::pair<double, people_msgs_utils::Group>> group_data{
        {2.50, DUMMY_GROUP},
        {2.50, DUMMY_GROUP},
        {4.50, DUMMY_GROUP},
        {4.50, DUMMY_GROUP},
        {4.50, DUMMY_GROUP},
        {5.50, DUMMY_GROUP},
        {5.50, DUMMY_GROUP}
    };

    testing::MockFunction<void(void)> mockCbNextTimestamp;
    EXPECT_CALL(mockCbNextTimestamp, Call())
        .Times(3);
    testing::MockFunction<void(void)> mockCbLastTimestamp;
    EXPECT_CALL(mockCbLastTimestamp, Call())
        .Times(1);
    testing::MockFunction<void(void)> mockCbNextPersonTimestamp;
    EXPECT_CALL(mockCbNextPersonTimestamp, Call())
        .Times(3*2 + 1 + 2*3 + 3*2);
    testing::MockFunction<void(void)> mockCbAllPeopleTimestamp;
    EXPECT_CALL(mockCbAllPeopleTimestamp, Call())
        .Times(  2 + 1 +   3 +   2);
    testing::MockFunction<void(void)> mockCbNextGroupTimestamp;
    EXPECT_CALL(mockCbNextGroupTimestamp, Call())
        .Times(7);
    testing::MockFunction<void(void)> mockCbAllGroupsTimestamp;
    EXPECT_CALL(mockCbAllGroupsTimestamp, Call())
        .Times(3);

    evaluation::Rewinder rew(robot_data, people_data, group_data);
    rew.setHandlerNextTimestamp(mockCbNextTimestamp.AsStdFunction());
    rew.setHandlerLastTimestamp(mockCbLastTimestamp.AsStdFunction());
    rew.setHandlerNextPersonTimestamp(mockCbNextPersonTimestamp.AsStdFunction());
    rew.setHandlerAllPeopleTimestamp(mockCbAllPeopleTimestamp.AsStdFunction());
    rew.setHandlerNextGroupTimestamp(mockCbNextGroupTimestamp.AsStdFunction());
    rew.setHandlerAllGroupsTimestamp(mockCbAllGroupsTimestamp.AsStdFunction());
    rew.perform();
}

TEST(TestRewinder, robotPeopleGroups4) {
    std::vector<std::pair<double, logger::RobotData>> robot_data{
        {2.50, DUMMY_ROBOT},
        {2.75, DUMMY_ROBOT},
        {3.00, DUMMY_ROBOT},
        {3.25, DUMMY_ROBOT},
        {3.50, DUMMY_ROBOT},
        {3.75, DUMMY_ROBOT},
        {4.00, DUMMY_ROBOT},
        {4.25, DUMMY_ROBOT},
        {4.50, DUMMY_ROBOT},
        {4.75, DUMMY_ROBOT},
        {5.00, DUMMY_ROBOT},
        {5.25, DUMMY_ROBOT},
        {5.50, DUMMY_ROBOT}
    };
    std::vector<std::pair<double, people_msgs_utils::Person>> people_data{
        {2.50, createPerson("00")},
        {2.50, createPerson("01")},
        {2.50, createPerson("02")},
        {3.50, createPerson("03")},
        {3.50, createPerson("04")},
        {3.50, createPerson("05")},
        {3.50, createPerson("06")},
        {3.75, createPerson("07")},
        {3.75, createPerson("08")},
        {3.75, createPerson("09")},
        {3.75, createPerson("10")},
        {4.00, createPerson("11")},
        {4.50, createPerson("12")},
        {4.50, createPerson("13")},
        {5.00, createPerson("14")},
        {5.00, createPerson("15")},
        {5.00, createPerson("16")},
        {5.00, createPerson("17")},
        {5.50, createPerson("18")},
        {5.50, createPerson("19")},
        {5.50, createPerson("20")},
        {5.50, createPerson("21")},
        {5.50, createPerson("22")}
    };
    std::vector<std::pair<double, people_msgs_utils::Group>> group_data{
        {3.50, DUMMY_GROUP},
        {3.50, DUMMY_GROUP},
        {4.50, DUMMY_GROUP},
        {5.00, DUMMY_GROUP},
        {5.50, DUMMY_GROUP},
        {5.50, DUMMY_GROUP}
    };

    testing::MockFunction<void(void)> mockCbNextTimestamp;
    EXPECT_CALL(mockCbNextTimestamp, Call())
        .Times(12);
    testing::MockFunction<void(void)> mockCbLastTimestamp;
    EXPECT_CALL(mockCbLastTimestamp, Call())
        .Times(1);
    testing::MockFunction<void(void)> mockCbNextPersonTimestamp;
    EXPECT_CALL(mockCbNextPersonTimestamp, Call())
        .Times(3 + 4*2 + 4 + 1 + 2*1 + 4*1 + 5*2);
    testing::MockFunction<void(void)> mockCbAllPeopleTimestamp;
    EXPECT_CALL(mockCbAllPeopleTimestamp, Call())
        .Times(1 +   2 + 1 + 1 +   1 +   1  +  2);
    testing::MockFunction<void(void)> mockCbNextGroupTimestamp;
    EXPECT_CALL(mockCbNextGroupTimestamp, Call())
        .Times(6);
    testing::MockFunction<void(void)> mockCbAllGroupsTimestamp;
    EXPECT_CALL(mockCbAllGroupsTimestamp, Call())
        .Times(4);

    evaluation::Rewinder rew(robot_data, people_data, group_data);
    rew.setHandlerNextTimestamp(mockCbNextTimestamp.AsStdFunction());
    rew.setHandlerLastTimestamp(mockCbLastTimestamp.AsStdFunction());
    rew.setHandlerNextPersonTimestamp(mockCbNextPersonTimestamp.AsStdFunction());
    rew.setHandlerAllPeopleTimestamp(mockCbAllPeopleTimestamp.AsStdFunction());
    rew.setHandlerNextGroupTimestamp(mockCbNextGroupTimestamp.AsStdFunction());
    rew.setHandlerAllGroupsTimestamp(mockCbAllGroupsTimestamp.AsStdFunction());
    rew.perform();
}

TEST(TestRewinder, robotPeopleGroups5) {
    std::vector<std::pair<double, logger::RobotData>> robot_data{
        {2.50, DUMMY_ROBOT},
        {2.75, DUMMY_ROBOT},
        {3.00, DUMMY_ROBOT},
        {3.25, DUMMY_ROBOT},
        {3.50, DUMMY_ROBOT},
        {3.75, DUMMY_ROBOT},
        {4.00, DUMMY_ROBOT},
        {4.25, DUMMY_ROBOT},
        {4.50, DUMMY_ROBOT},
        {4.75, DUMMY_ROBOT},
        {5.00, DUMMY_ROBOT},
        {5.25, DUMMY_ROBOT},
        {5.50, DUMMY_ROBOT}
    };
    std::vector<std::pair<double, people_msgs_utils::Person>> people_data{
        {2.75, createPerson("00")},
        {2.75, createPerson("01")},
        {2.75, createPerson("02")},
        {3.50, createPerson("03")},
        {3.50, createPerson("04")},
        {3.50, createPerson("05")},
        {3.50, createPerson("06")},
        {3.75, createPerson("07")},
        {3.75, createPerson("08")},
        {3.75, createPerson("09")},
        {3.75, createPerson("10")},
        {4.25, createPerson("11")},
        {4.50, createPerson("12")},
        {4.50, createPerson("13")},
        {4.75, createPerson("14")},
        {5.00, createPerson("15")},
        {5.00, createPerson("16")},
        {5.00, createPerson("17")},
        {5.00, createPerson("18")},
        {5.50, createPerson("19")},
        {5.50, createPerson("20")},
        {5.50, createPerson("21")},
        {5.50, createPerson("22")},
        {5.50, createPerson("23")}
    };
    std::vector<std::pair<double, people_msgs_utils::Group>> group_data{
        {3.50, DUMMY_GROUP},
        {3.50, DUMMY_GROUP},
        {4.50, DUMMY_GROUP},
        {5.00, DUMMY_GROUP},
        {5.50, DUMMY_GROUP},
        {5.50, DUMMY_GROUP}
    };

    testing::MockFunction<void(void)> mockCbNextTimestamp;
    EXPECT_CALL(mockCbNextTimestamp, Call())
        .Times(12);
    testing::MockFunction<void(void)> mockCbLastTimestamp;
    EXPECT_CALL(mockCbLastTimestamp, Call())
        .Times(1);
    testing::MockFunction<void(void)> mockCbNextPersonTimestamp;
    EXPECT_CALL(mockCbNextPersonTimestamp, Call())
        .Times(3 + 4*2 + 4 + 1 + 2*1 + 1 + 4*1 + 5*2);
    testing::MockFunction<void(void)> mockCbAllPeopleTimestamp;
    EXPECT_CALL(mockCbAllPeopleTimestamp, Call())
        .Times(1 +   2 + 1 + 1 +   1 + 1 +   1 +   2);
    testing::MockFunction<void(void)> mockCbNextGroupTimestamp;
    EXPECT_CALL(mockCbNextGroupTimestamp, Call())
        .Times(6);
    testing::MockFunction<void(void)> mockCbAllGroupsTimestamp;
    EXPECT_CALL(mockCbAllGroupsTimestamp, Call())
        .Times(4);

    evaluation::Rewinder rew(robot_data, people_data, group_data);
    rew.setHandlerNextTimestamp(mockCbNextTimestamp.AsStdFunction());
    rew.setHandlerLastTimestamp(mockCbLastTimestamp.AsStdFunction());
    rew.setHandlerNextPersonTimestamp(mockCbNextPersonTimestamp.AsStdFunction());
    rew.setHandlerAllPeopleTimestamp(mockCbAllPeopleTimestamp.AsStdFunction());
    rew.setHandlerNextGroupTimestamp(mockCbNextGroupTimestamp.AsStdFunction());
    rew.setHandlerAllGroupsTimestamp(mockCbAllGroupsTimestamp.AsStdFunction());
    rew.perform();
}

TEST(TestRewinder, robotPeopleGroups6) {
    std::vector<std::pair<double, logger::RobotData>> robot_data{
        {2.50, DUMMY_ROBOT},
        {2.75, DUMMY_ROBOT},
        {3.00, DUMMY_ROBOT},
        {3.25, DUMMY_ROBOT},
        {3.50, DUMMY_ROBOT},
        {3.75, DUMMY_ROBOT},
        {4.00, DUMMY_ROBOT},
        {4.25, DUMMY_ROBOT},
        {4.50, DUMMY_ROBOT},
        {4.75, DUMMY_ROBOT},
        {5.00, DUMMY_ROBOT},
        {5.25, DUMMY_ROBOT},
        {5.50, DUMMY_ROBOT}
    };
    std::vector<std::pair<double, people_msgs_utils::Person>> people_data{
        {2.75, createPerson("00")},
        {2.75, createPerson("01")},
        {2.75, createPerson("02")},
        {2.75, createPerson("03")},
        {2.75, createPerson("04")},
        {2.75, createPerson("05")},
        {2.75, createPerson("06")},
        {2.75, createPerson("07")},
        {2.75, createPerson("08")},
        {2.75, createPerson("09")},
        {2.75, createPerson("10")},
        {2.75, createPerson("11")},
        {4.25, createPerson("12")},
        {5.00, createPerson("13")},
        {5.00, createPerson("14")},
        {5.00, createPerson("15")},
        {5.00, createPerson("16")},
        {5.00, createPerson("17")},
        {5.00, createPerson("18")},
        {5.00, createPerson("19")},
        {5.00, createPerson("20")},
        {5.00, createPerson("21")},
        {5.00, createPerson("22")},
        {5.00, createPerson("23")}
    };
    std::vector<std::pair<double, people_msgs_utils::Group>> group_data{
        {2.75, DUMMY_GROUP},
        {5.00, DUMMY_GROUP}
    };

    testing::MockFunction<void(void)> mockCbNextTimestamp;
    EXPECT_CALL(mockCbNextTimestamp, Call())
        .Times(12);
    testing::MockFunction<void(void)> mockCbLastTimestamp;
    EXPECT_CALL(mockCbLastTimestamp, Call())
        .Times(1);
    testing::MockFunction<void(void)> mockCbNextPersonTimestamp;
    EXPECT_CALL(mockCbNextPersonTimestamp, Call())
        .Times(12*1 + 1 + 11*1);
    testing::MockFunction<void(void)> mockCbAllPeopleTimestamp;
    EXPECT_CALL(mockCbAllPeopleTimestamp, Call())
        .Times(   1 + 1 +    1);
    testing::MockFunction<void(void)> mockCbNextGroupTimestamp;
    EXPECT_CALL(mockCbNextGroupTimestamp, Call())
        .Times(2);
    testing::MockFunction<void(void)> mockCbAllGroupsTimestamp;
    EXPECT_CALL(mockCbAllGroupsTimestamp, Call())
        .Times(2);

    evaluation::Rewinder rew(robot_data, people_data, group_data);
    rew.setHandlerNextTimestamp(mockCbNextTimestamp.AsStdFunction());
    rew.setHandlerLastTimestamp(mockCbLastTimestamp.AsStdFunction());
    rew.setHandlerNextPersonTimestamp(mockCbNextPersonTimestamp.AsStdFunction());
    rew.setHandlerAllPeopleTimestamp(mockCbAllPeopleTimestamp.AsStdFunction());
    rew.setHandlerNextGroupTimestamp(mockCbNextGroupTimestamp.AsStdFunction());
    rew.setHandlerAllGroupsTimestamp(mockCbAllGroupsTimestamp.AsStdFunction());
    rew.perform();
}

TEST(TestRewinder, robotPeopleGroups7) {
    std::vector<std::pair<double, logger::RobotData>> robot_data{
        {2.50, DUMMY_ROBOT},
        {2.75, DUMMY_ROBOT},
        {3.00, DUMMY_ROBOT},
        {3.25, DUMMY_ROBOT},
        {3.50, DUMMY_ROBOT},
        {3.75, DUMMY_ROBOT},
        {4.00, DUMMY_ROBOT},
        {4.25, DUMMY_ROBOT},
        {4.50, DUMMY_ROBOT},
        {4.75, DUMMY_ROBOT},
        {5.00, DUMMY_ROBOT},
        {5.25, DUMMY_ROBOT},
        {5.50, DUMMY_ROBOT}
    };
    std::vector<std::pair<double, people_msgs_utils::Person>> people_data{
        {4.75, createPerson("00")},
        {4.75, createPerson("01")},
        {4.75, createPerson("02")},
        {4.75, createPerson("03")},
        {4.75, createPerson("04")},
        {4.75, createPerson("05")},
        {4.75, createPerson("06")},
        {4.75, createPerson("07")},
        {4.75, createPerson("08")},
        {4.75, createPerson("09")},
        {4.75, createPerson("10")},
        {4.75, createPerson("11")},
        {4.75, createPerson("12")},
        {5.00, createPerson("13")},
        {5.00, createPerson("14")},
        {5.00, createPerson("15")},
        {5.00, createPerson("16")},
        {5.00, createPerson("17")},
        {5.00, createPerson("18")},
        {5.00, createPerson("19")},
        {5.00, createPerson("20")},
        {5.00, createPerson("21")},
        {5.00, createPerson("22")},
        {5.00, createPerson("23")}
    };
    std::vector<std::pair<double, people_msgs_utils::Group>> group_data{
        {5.00, DUMMY_GROUP}
    };

    testing::MockFunction<void(void)> mockCbNextTimestamp;
    EXPECT_CALL(mockCbNextTimestamp, Call())
        .Times(12);
    testing::MockFunction<void(void)> mockCbLastTimestamp;
    EXPECT_CALL(mockCbLastTimestamp, Call())
        .Times(1);
    testing::MockFunction<void(void)> mockCbNextPersonTimestamp;
    EXPECT_CALL(mockCbNextPersonTimestamp, Call())
        .Times(13*1 + 11*1);
    testing::MockFunction<void(void)> mockCbAllPeopleTimestamp;
    EXPECT_CALL(mockCbAllPeopleTimestamp, Call())
        .Times(   1 +    1);
    testing::MockFunction<void(void)> mockCbNextGroupTimestamp;
    EXPECT_CALL(mockCbNextGroupTimestamp, Call())
        .Times(1);
    testing::MockFunction<void(void)> mockCbAllGroupsTimestamp;
    EXPECT_CALL(mockCbAllGroupsTimestamp, Call())
        .Times(1);

    evaluation::Rewinder rew(robot_data, people_data, group_data);
    rew.setHandlerNextTimestamp(mockCbNextTimestamp.AsStdFunction());
    rew.setHandlerLastTimestamp(mockCbLastTimestamp.AsStdFunction());
    rew.setHandlerNextPersonTimestamp(mockCbNextPersonTimestamp.AsStdFunction());
    rew.setHandlerAllPeopleTimestamp(mockCbAllPeopleTimestamp.AsStdFunction());
    rew.setHandlerNextGroupTimestamp(mockCbNextGroupTimestamp.AsStdFunction());
    rew.setHandlerAllGroupsTimestamp(mockCbAllGroupsTimestamp.AsStdFunction());
    rew.perform();
}

TEST(TestRewinder, robotPeopleGroups8) {
    std::vector<std::pair<double, logger::RobotData>> robot_data{
        {2.50, DUMMY_ROBOT},
        {2.75, DUMMY_ROBOT},
        {3.00, DUMMY_ROBOT},
        {3.25, DUMMY_ROBOT},
        {3.50, DUMMY_ROBOT},
        {3.75, DUMMY_ROBOT},
        {4.00, DUMMY_ROBOT},
        {4.25, DUMMY_ROBOT},
        {4.50, DUMMY_ROBOT},
        {4.75, DUMMY_ROBOT},
        {5.00, DUMMY_ROBOT},
        {5.25, DUMMY_ROBOT},
        {5.50, DUMMY_ROBOT}
    };
    std::vector<std::pair<double, people_msgs_utils::Person>> people_data{
        {2.75, createPerson("00")},
        {2.75, createPerson("01")},
        {2.75, createPerson("02")},
        {2.75, createPerson("03")},
        {2.75, createPerson("04")},
        {2.75, createPerson("05")},
        {2.75, createPerson("06")},
        {2.75, createPerson("07")},
        {2.75, createPerson("08")},
        {2.75, createPerson("09")},
        {2.75, createPerson("10")},
        {2.75, createPerson("11")},
        {2.75, createPerson("12")},
        {4.00, createPerson("13")},
        {4.00, createPerson("14")},
        {4.00, createPerson("15")},
        {4.00, createPerson("16")},
        {4.00, createPerson("17")},
        {4.00, createPerson("18")},
        {4.00, createPerson("19")},
        {4.00, createPerson("20")},
        {4.00, createPerson("21")},
        {4.00, createPerson("22")},
        {4.00, createPerson("23")}
    };
    std::vector<std::pair<double, people_msgs_utils::Group>> group_data{
        {2.75, DUMMY_GROUP}
    };

    testing::MockFunction<void(void)> mockCbNextTimestamp;
    EXPECT_CALL(mockCbNextTimestamp, Call())
        .Times(12);
    testing::MockFunction<void(void)> mockCbLastTimestamp;
    EXPECT_CALL(mockCbLastTimestamp, Call())
        .Times(1);
    testing::MockFunction<void(void)> mockCbNextPersonTimestamp;
    EXPECT_CALL(mockCbNextPersonTimestamp, Call())
        .Times(13*1 + 11*1);
    testing::MockFunction<void(void)> mockCbAllPeopleTimestamp;
    EXPECT_CALL(mockCbAllPeopleTimestamp, Call())
        .Times(   1 +    1);
    testing::MockFunction<void(void)> mockCbNextGroupTimestamp;
    EXPECT_CALL(mockCbNextGroupTimestamp, Call())
        .Times(1);
    testing::MockFunction<void(void)> mockCbAllGroupsTimestamp;
    EXPECT_CALL(mockCbAllGroupsTimestamp, Call())
        .Times(1);

    evaluation::Rewinder rew(robot_data, people_data, group_data);
    rew.setHandlerNextTimestamp(mockCbNextTimestamp.AsStdFunction());
    rew.setHandlerLastTimestamp(mockCbLastTimestamp.AsStdFunction());
    rew.setHandlerNextPersonTimestamp(mockCbNextPersonTimestamp.AsStdFunction());
    rew.setHandlerAllPeopleTimestamp(mockCbAllPeopleTimestamp.AsStdFunction());
    rew.setHandlerNextGroupTimestamp(mockCbNextGroupTimestamp.AsStdFunction());
    rew.setHandlerAllGroupsTimestamp(mockCbAllGroupsTimestamp.AsStdFunction());
    rew.perform();
}

TEST(TestRewinder, robotPeopleGroups9) {
    std::vector<std::pair<double, logger::RobotData>> robot_data{
        {2.50, DUMMY_ROBOT},
        {2.75, DUMMY_ROBOT},
        {3.00, DUMMY_ROBOT},
        {3.25, DUMMY_ROBOT},
        {3.50, DUMMY_ROBOT},
        {3.75, DUMMY_ROBOT},
        {4.00, DUMMY_ROBOT},
        {4.25, DUMMY_ROBOT},
        {4.50, DUMMY_ROBOT},
        {4.75, DUMMY_ROBOT},
        {5.00, DUMMY_ROBOT},
        {5.25, DUMMY_ROBOT},
        {5.50, DUMMY_ROBOT}
    };
    std::vector<std::pair<double, people_msgs_utils::Person>> people_data{
        {2.50, createPerson("00")},
        {2.50, createPerson("01")},
        {2.75, createPerson("02")},
        {2.75, createPerson("03")},
        {3.00, createPerson("04")},
        {3.00, createPerson("05")},
        {3.25, createPerson("06")},
        {3.25, createPerson("07")},
        {3.50, createPerson("08")},
        {3.75, createPerson("09")},
        {3.75, createPerson("10")},
        {4.00, createPerson("11")},
        {4.00, createPerson("12")},
        {4.00, createPerson("13")},
        {4.00, createPerson("14")},
        {4.00, createPerson("15")},
        {4.00, createPerson("16")},
        {4.00, createPerson("17")},
        {4.00, createPerson("18")},
        {4.00, createPerson("19")},
        {4.00, createPerson("20")},
        {4.00, createPerson("21")},
        {4.00, createPerson("22")},
        {5.25, createPerson("23")}
    };
    std::vector<std::pair<double, people_msgs_utils::Group>> group_data{
        {2.50, DUMMY_GROUP},
        {2.75, DUMMY_GROUP}
    };

    testing::MockFunction<void(void)> mockCbNextTimestamp;
    EXPECT_CALL(mockCbNextTimestamp, Call())
        .Times(12);
    testing::MockFunction<void(void)> mockCbLastTimestamp;
    EXPECT_CALL(mockCbLastTimestamp, Call())
        .Times(1);
    testing::MockFunction<void(void)> mockCbNextPersonTimestamp;
    EXPECT_CALL(mockCbNextPersonTimestamp, Call())
        .Times(2*1 + 2*1 + 2 + 2 + 1 + 2 + 12 + 1);
    testing::MockFunction<void(void)> mockCbAllPeopleTimestamp;
    EXPECT_CALL(mockCbAllPeopleTimestamp, Call())
        .Times(  1 +   1 + 1 + 1 + 1 + 1 +  1 + 1);
    testing::MockFunction<void(void)> mockCbNextGroupTimestamp;
    EXPECT_CALL(mockCbNextGroupTimestamp, Call())
        .Times(2);
    testing::MockFunction<void(void)> mockCbAllGroupsTimestamp;
    EXPECT_CALL(mockCbAllGroupsTimestamp, Call())
        .Times(2);

    evaluation::Rewinder rew(robot_data, people_data, group_data);
    rew.setHandlerNextTimestamp(mockCbNextTimestamp.AsStdFunction());
    rew.setHandlerLastTimestamp(mockCbLastTimestamp.AsStdFunction());
    rew.setHandlerNextPersonTimestamp(mockCbNextPersonTimestamp.AsStdFunction());
    rew.setHandlerAllPeopleTimestamp(mockCbAllPeopleTimestamp.AsStdFunction());
    rew.setHandlerNextGroupTimestamp(mockCbNextGroupTimestamp.AsStdFunction());
    rew.setHandlerAllGroupsTimestamp(mockCbAllGroupsTimestamp.AsStdFunction());
    rew.perform();
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
