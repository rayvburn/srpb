#include <gtest/gtest.h>

#include "srpb_evaluation/utils.h"

people_msgs_utils::Person createPerson(const std::string& name, const std::string& group_name) {
    return people_msgs_utils::Person(
        name,
        geometry_msgs::PoseWithCovariance(),
        geometry_msgs::PoseWithCovariance(),
        1.0,
        true,
        true,
        951,
        357,
        group_name
    );
}

people_msgs_utils::Group createIllformedGroup(
    const std::string& name,
    const std::vector<unsigned int>& members,
    const std::vector<std::tuple<unsigned int, unsigned int, double>>& relations
) {
    return people_msgs_utils::Group(
        name,
        9518475321,
        std::vector<people_msgs_utils::Person>(),
        members,
        relations,
        geometry_msgs::Point()
    );
}

TEST(TestFillGroupsWithMembers, members1) {
    std::vector<std::pair<double, people_msgs_utils::Person>> tppl{
        {0.5, createPerson("01", "100")},
        {0.5, createPerson("02", "100")},
        {0.5, createPerson("03", ""   )},
        {0.5, createPerson("04", ""   )},
        {1.5, createPerson("02", "100")},
        {1.5, createPerson("03", "100")},
        {1.5, createPerson("04", "101")},
        {1.5, createPerson("05", "101")},
        {2.5, createPerson("03", "102")},
        {2.5, createPerson("04", ""   )},
        {2.5, createPerson("05", ""   )},
        {2.5, createPerson("06", "102")}
    };
    std::vector<std::pair<double, people_msgs_utils::Group>> tgrp{
        {0.5, createIllformedGroup(
            "100",
            std::vector<unsigned int>{01, 02},
            std::vector<std::tuple<unsigned int, unsigned int, double>>{{01, 02, 0.987}}
        )},
        {1.5, createIllformedGroup("100",
            std::vector<unsigned int>{02, 03},
            std::vector<std::tuple<unsigned int, unsigned int, double>>{{02, 03, 0.789}}
        )},
        {1.5, createIllformedGroup("101",
            std::vector<unsigned int>{04, 05},
            std::vector<std::tuple<unsigned int, unsigned int, double>>{{04, 05, 0.852}}
        )},
        {2.5, createIllformedGroup("102",
            std::vector<unsigned int>{03, 06},
            std::vector<std::tuple<unsigned int, unsigned int, double>>{{03, 06, 0.369}}
        )}
    };

    ASSERT_EQ(tppl.size(), 12);
    ASSERT_EQ(tppl.at(0).first, 0.5);
    ASSERT_EQ(tppl.at(0).second.getName(), "01");
    ASSERT_EQ(tppl.at(0).second.getGroupName(), "100");

    ASSERT_EQ(tgrp.size(), 4);
    ASSERT_EQ(tgrp.at(0).first, 0.5);
    ASSERT_EQ(tgrp.at(0).second.getName(), "100");
    ASSERT_EQ(tgrp.at(0).second.getMemberIDs().size(), 2);
    ASSERT_EQ(tgrp.at(0).second.getMembers().size(), 0);
    ASSERT_EQ(tgrp.at(0).second.getSocialRelations().size(), 1);
    ASSERT_EQ(tgrp.at(1).second.getMembers().size(), 0);
    ASSERT_EQ(tgrp.at(1).second.getSocialRelations().size(), 1);
    ASSERT_EQ(tgrp.at(2).second.getMembers().size(), 0);
    ASSERT_EQ(tgrp.at(2).second.getSocialRelations().size(), 1);
    ASSERT_EQ(tgrp.at(3).second.getMembers().size(), 0);
    ASSERT_EQ(tgrp.at(3).second.getSocialRelations().size(), 1);

    auto tgrp_filled = srpb::evaluation::fillGroupsWithMembers(tgrp, tppl);

    ASSERT_EQ(tgrp_filled.size(), tgrp.size());

    ASSERT_EQ(tgrp_filled.at(0).second.getName(), "100");
    ASSERT_EQ(tgrp_filled.at(0).second.getMembers().size(), 2);
    ASSERT_EQ(tgrp_filled.at(0).second.getMembers().at(0).getName(), "01");
    ASSERT_EQ(tgrp_filled.at(0).second.getMembers().at(1).getName(), "02");
    ASSERT_EQ(tgrp_filled.at(0).second.getSocialRelations(1).size(), 1);
    ASSERT_EQ(tgrp_filled.at(0).second.getSocialRelations(2).size(), 1);

    ASSERT_EQ(tgrp_filled.at(1).second.getName(), "100");
    ASSERT_EQ(tgrp_filled.at(1).second.getMembers().size(), 2);
    ASSERT_EQ(tgrp_filled.at(1).second.getMembers().at(0).getName(), "02");
    ASSERT_EQ(tgrp_filled.at(1).second.getMembers().at(1).getName(), "03");
    ASSERT_EQ(tgrp_filled.at(1).second.getSocialRelations(2).size(), 1);
    ASSERT_EQ(tgrp_filled.at(1).second.getSocialRelations(3).size(), 1);

    ASSERT_EQ(tgrp_filled.at(2).second.getName(), "101");
    ASSERT_EQ(tgrp_filled.at(2).second.getMembers().size(), 2);
    ASSERT_EQ(tgrp_filled.at(2).second.getMembers().at(0).getName(), "04");
    ASSERT_EQ(tgrp_filled.at(2).second.getMembers().at(1).getName(), "05");
    ASSERT_EQ(tgrp_filled.at(2).second.getSocialRelations(4).size(), 1);
    ASSERT_EQ(tgrp_filled.at(2).second.getSocialRelations(5).size(), 1);

    ASSERT_EQ(tgrp_filled.at(3).second.getName(), "102");
    ASSERT_EQ(tgrp_filled.at(3).second.getMembers().size(), 2);
    ASSERT_EQ(tgrp_filled.at(3).second.getMembers().at(0).getName(), "03");
    ASSERT_EQ(tgrp_filled.at(3).second.getMembers().at(1).getName(), "06");
    ASSERT_EQ(tgrp_filled.at(3).second.getSocialRelations(3).size(), 1);
    ASSERT_EQ(tgrp_filled.at(3).second.getSocialRelations(6).size(), 1);
}

TEST(TestFillGroupsWithMembers, members2) {
    std::vector<std::pair<double, people_msgs_utils::Person>> tppl{
        {0.5, createPerson("01", "100")},
        {0.5, createPerson("02", "100")},
        {0.5, createPerson("03", "100")},
        {0.5, createPerson("04", ""   )},
        {1.5, createPerson("02", "100")},
        {1.5, createPerson("03", "100")},
        {1.5, createPerson("04", "101")},
        {1.5, createPerson("05", "101")},
        {2.5, createPerson("03", "100")},
        {2.5, createPerson("04", ""   )},
        {2.5, createPerson("05", "100")},
        {2.5, createPerson("06", ""   )}
    };
    std::vector<std::pair<double, people_msgs_utils::Group>> tgrp{
        {0.5, createIllformedGroup(
            "100",
            std::vector<unsigned int>{01, 02, 03},
            std::vector<std::tuple<unsigned int, unsigned int, double>>{
                {01, 02, 0.987},
                {02, 03, 0.321},
                {01, 03, 0.147}
            }
        )},
        {1.5, createIllformedGroup("100",
            std::vector<unsigned int>{02, 03},
            std::vector<std::tuple<unsigned int, unsigned int, double>>{{02, 03, 0.789}}
        )},
        {1.5, createIllformedGroup("101",
            std::vector<unsigned int>{04, 05},
            std::vector<std::tuple<unsigned int, unsigned int, double>>{{04, 05, 0.741}}
        )},
        {2.5, createIllformedGroup("100",
            std::vector<unsigned int>{03, 05},
            std::vector<std::tuple<unsigned int, unsigned int, double>>{{03, 05, 0.852}}
        )}
    };

    ASSERT_EQ(tppl.size(), 12);
    ASSERT_EQ(tppl.at(0).first, 0.5);
    ASSERT_EQ(tppl.at(0).second.getName(), "01");
    ASSERT_EQ(tppl.at(0).second.getGroupName(), "100");

    ASSERT_EQ(tgrp.size(), 4);
    ASSERT_EQ(tgrp.at(0).first, 0.5);
    ASSERT_EQ(tgrp.at(0).second.getName(), "100");
    ASSERT_EQ(tgrp.at(0).second.getMemberIDs().size(), 3);
    ASSERT_EQ(tgrp.at(0).second.getMembers().size(), 0);
    ASSERT_EQ(tgrp.at(0).second.getSocialRelations().size(), 3);
    ASSERT_EQ(tgrp.at(1).second.getMembers().size(), 0);
    ASSERT_EQ(tgrp.at(1).second.getSocialRelations().size(), 1);
    ASSERT_EQ(tgrp.at(2).second.getMembers().size(), 0);
    ASSERT_EQ(tgrp.at(2).second.getSocialRelations().size(), 1);
    ASSERT_EQ(tgrp.at(3).second.getMembers().size(), 0);
    ASSERT_EQ(tgrp.at(3).second.getSocialRelations().size(), 1);

    auto tgrp_filled = srpb::evaluation::fillGroupsWithMembers(tgrp, tppl);

    ASSERT_EQ(tgrp_filled.size(), tgrp.size());

    ASSERT_EQ(tgrp_filled.at(0).second.getName(), "100");
    ASSERT_EQ(tgrp_filled.at(0).second.getMembers().size(), 3);
    ASSERT_EQ(tgrp_filled.at(0).second.getMembers().at(0).getName(), "01");
    ASSERT_EQ(tgrp_filled.at(0).second.getMembers().at(1).getName(), "02");
    ASSERT_EQ(tgrp_filled.at(0).second.getMembers().at(2).getName(), "03");
    ASSERT_EQ(tgrp_filled.at(0).second.getSocialRelations(1).size(), 2);
    ASSERT_EQ(tgrp_filled.at(0).second.getSocialRelations(2).size(), 2);
    ASSERT_EQ(tgrp_filled.at(0).second.getSocialRelations(3).size(), 2);

    ASSERT_EQ(tgrp_filled.at(1).second.getName(), "100");
    ASSERT_EQ(tgrp_filled.at(1).second.getMembers().size(), 2);
    ASSERT_EQ(tgrp_filled.at(1).second.getMembers().at(0).getName(), "02");
    ASSERT_EQ(tgrp_filled.at(1).second.getMembers().at(1).getName(), "03");
    ASSERT_EQ(tgrp_filled.at(1).second.getSocialRelations(2).size(), 1);
    ASSERT_EQ(tgrp_filled.at(1).second.getSocialRelations(3).size(), 1);

    ASSERT_EQ(tgrp_filled.at(2).second.getName(), "101");
    ASSERT_EQ(tgrp_filled.at(2).second.getMembers().size(), 2);
    ASSERT_EQ(tgrp_filled.at(2).second.getMembers().at(0).getName(), "04");
    ASSERT_EQ(tgrp_filled.at(2).second.getMembers().at(1).getName(), "05");
    ASSERT_EQ(tgrp_filled.at(2).second.getSocialRelations(4).size(), 1);
    ASSERT_EQ(tgrp_filled.at(2).second.getSocialRelations(5).size(), 1);

    ASSERT_EQ(tgrp_filled.at(3).second.getName(), "100");
    ASSERT_EQ(tgrp_filled.at(3).second.getMembers().size(), 2);
    ASSERT_EQ(tgrp_filled.at(3).second.getMembers().at(0).getName(), "03");
    ASSERT_EQ(tgrp_filled.at(3).second.getMembers().at(1).getName(), "05");
    ASSERT_EQ(tgrp_filled.at(3).second.getSocialRelations(3).size(), 1);
    ASSERT_EQ(tgrp_filled.at(3).second.getSocialRelations(5).size(), 1);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
