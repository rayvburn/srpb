#pragma once

#include <tuple>
#include <vector>

#include <srpb_logger/robot_data.h>
#include <people_msgs_utils/person.h>
#include <people_msgs_utils/group.h>

#include "srpb_postprocessing/rewinder.h"

class Metric {
public:
	Metric(
        const std::vector<std::pair<double, RobotData>>& robot_data
    ): rewinder_(robot_data) {}

    Metric(
        const std::vector<std::pair<double, RobotData>>& robot_data,
        const std::vector<std::pair<double, people_msgs_utils::Person>>& people_data
    ): rewinder_(robot_data, people_data) {}

    Metric(
        const std::vector<std::pair<double, RobotData>>& robot_data,
        const std::vector<std::pair<double, people_msgs_utils::Person>>& people_data,
        const std::vector<std::pair<double, people_msgs_utils::Group>>& groups_data
    ): rewinder_(robot_data, people_data, groups_data) {}

	virtual void printResults() const = 0;

protected:
	Rewinder rewinder_;

    virtual void compute() = 0;
};
