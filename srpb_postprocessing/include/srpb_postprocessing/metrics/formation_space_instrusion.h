#pragma once

#include "srpb_postprocessing/metric_gaussian.h"

#include <srpb_logger/robot_data.h>
#include <people_msgs_utils/person.h>
#include <people_msgs_utils/group.h>

/// Similar to personal space intrusion but related to the group space
class FormationSpaceIntrusion: public MetricGaussian {
public:
  FormationSpaceIntrusion(
    const std::vector<std::pair<double, RobotData>>& robot_data,
    const std::vector<std::pair<double, people_msgs_utils::Person>>& people_data,
    const std::vector<std::pair<double, people_msgs_utils::Group>>& groups_data,
    double group_space_threshold,
    bool max_method = true
  ):
    MetricGaussian(robot_data, people_data, groups_data),
    group_space_threshold_(group_space_threshold),
    max_method_(max_method)
  {
    compute();
  }

  /// Calculates radius of the F-formation based on positions of group members and group's center of gravity
  static double calculateRadiusFformation(
    const people_msgs_utils::Group& group,
    const std::vector<people_msgs_utils::Person>& people_group
  ) {
    double radius = 0.0;
    for (const auto& person: people_group) {
      double dist_from_cog = std::sqrt(
        std::pow(person.getPositionX() - group.getCenterOfGravity().x, 2)
        + std::pow(person.getPositionY() - group.getCenterOfGravity().y, 2)
      );
      // select the maximum value as radius
      if (radius < dist_from_cog) {
        radius = dist_from_cog;
      }
    }
    return radius;
  }

  void printResults() const override {
    printf(
      "Formation space intrusion = %.4f [%%] (min = %.4f [%%], max = %.4f [%%], violations %3u)\n",
      intrusion_total_,
      intrusion_min_,
      intrusion_max_,
      violations_num_
    );
  }

protected:
  // parameters
  double group_space_threshold_;
  bool max_method_;

  // results
  double intrusion_min_;
  double intrusion_max_;
  double intrusion_total_;
  unsigned int violations_num_;

  void compute() override {
    // store durations and Gaussians of the robot in terms of nearby people
    std::vector<std::pair<double, std::vector<double>>> timed_gaussians;
    /// prepare container for gaussian values in this `for` iteration
    std::pair<double, std::vector<double>> timed_gaussian;

    // collect people data into container that store group members
    std::vector<people_msgs_utils::Person> people_this_group;

    rewinder_.setHandlerNextTimestamp(
      [&]() {
        if (rewinder_.getTimestampCurr() == rewinder_.getTimestampLast()) {
          return;
        }
        timed_gaussian = std::make_pair(rewinder_.getTimestampNext() - rewinder_.getTimestampCurr(), std::vector<double>());
      }
    );

    rewinder_.setHandlerNextPersonTimestamp(
      [&]() {
        // add person data to the container of group members
        auto group_members = rewinder_.getGroupCurr().getMemberIDs();
        if (std::find(group_members.begin(), group_members.end(), rewinder_.getPersonCurr().getID()) != group_members.end()) {
          people_this_group.push_back(rewinder_.getPersonCurr());
        }
      }
    );

    rewinder_.setHandlerNextGroupTimestamp(
      [&]() {
        /// computations for the group
        double fformation_radius = FormationSpaceIntrusion::calculateRadiusFformation(
          rewinder_.getGroupCurr(),
          people_this_group
        );

        /*
         * Compute `cost` of the robot being located in the current position - how it affects group's ease.
         * NOTE that we do not possess the orientation of the group (yaw is set to 0)
         * so the circular model is assumed (sigmas are equal)
         *
         * Conversion from radius to stddev was mentioned, e.g., by Truong in `To Approach Humans? (..)` article (2017)
         */
        double gaussian = 0.0;
        // 2 sigma rule is used here (mean is the center of the F-formation, 2 times stddev corresponds to its span)
        double stddev_fformation = fformation_radius / 2.0;
        // perception can report 1 person in a group - do not investigate such situations further
        if (people_this_group.size() > 1) {
          gaussian = MetricGaussian::calculateGaussian(
            rewinder_.getRobotCurr().getPositionX(),
            rewinder_.getRobotCurr().getPositionY(),
            rewinder_.getGroupCurr().getCenterOfGravity().x,
            rewinder_.getGroupCurr().getCenterOfGravity().y,
            0.0,
            stddev_fformation,
            stddev_fformation,
            stddev_fformation
          );
        }

        // choose maximum reliability among group members as a reference to multiply gaussian
        double group_reliability = std::max_element(
          people_this_group.begin(),
          people_this_group.end(),
          [](const people_msgs_utils::Person& lhs, const people_msgs_utils::Person& rhs) {
            return lhs.getReliability() < rhs.getReliability();
          }
        )->getReliability();

        // count in the person's tracking reliability - we want to avoid penalizing robot that investigates `old` tracks
        double cost = group_reliability * gaussian;

        // Gaussian cost of the robot being located in the current pose; cost related to the investigated group of people
        timed_gaussian.second.push_back(cost);

        // clear set of people for the next group
        people_this_group.clear();
      }
    );

    rewinder_.setHandlerAllGroupsTimestamp(
      [&]() {
        if (!timed_gaussian.second.empty()) {
          timed_gaussians.push_back(timed_gaussian);
        }
      }
    );

    rewinder_.perform();

    std::tie(
      intrusion_min_,
      intrusion_max_,
      intrusion_total_,
      violations_num_
    ) = MetricGaussian::calculateGaussianStatistics(timed_gaussians, group_space_threshold_, max_method_);
  }
};
