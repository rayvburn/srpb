#include "srpb_postprocessing/rewinder.h"

namespace srpb {
namespace postprocessing {

// for convenience and compactness
using namespace srpb::logger;

Rewinder::Rewinder(
  const std::vector<std::pair<double, RobotData>>& robot_data
):
  robot_data_(robot_data),
  people_data_(std::vector<std::pair<double, people_msgs_utils::Person>>()),
  groups_data_(std::vector<std::pair<double, people_msgs_utils::Group>>()),
  data_people_empty_(true),
  data_groups_empty_(true)
{}

Rewinder::Rewinder(
  const std::vector<std::pair<double, RobotData>>& robot_data,
  const std::vector<std::pair<double, people_msgs_utils::Person>>& people_data
):
  robot_data_(robot_data),
  people_data_(people_data),
  groups_data_(std::vector<std::pair<double, people_msgs_utils::Group>>()),
  data_people_empty_(false),
  data_groups_empty_(true)
{}

Rewinder::Rewinder(
  const std::vector<std::pair<double, RobotData>>& robot_data,
  const std::vector<std::pair<double, people_msgs_utils::Person>>& people_data,
  const std::vector<std::pair<double, people_msgs_utils::Group>>& groups_data
):
  robot_data_(robot_data),
  people_data_(people_data),
  groups_data_(groups_data),
  data_people_empty_(false),
  data_groups_empty_(false)
{}

void Rewinder::setHandlerNextTimestamp(std::function<void(void)> fun) {
  handler_next_timestamp_ = std::move(fun);
}

void Rewinder::setHandlerLastTimestamp(std::function<void(void)> fun) {
  handler_last_timestamp_ = std::move(fun);
}

void Rewinder::setHandlerAllGroupsTimestamp(std::function<void(void)> fun) {
  handler_all_groups_timestamp_ = std::move(fun);
}

void Rewinder::setHandlerAllPeopleTimestamp(std::function<void(void)> fun) {
  handler_all_people_timestamp_ = std::move(fun);
}

void Rewinder::setHandlerNextPersonTimestamp(std::function<void(void)> fun) {
  handler_next_person_timestamp_ = std::move(fun);
}

void Rewinder::setHandlerNextGroupTimestamp(std::function<void(void)> fun) {
  handler_next_group_timestamp_ = std::move(fun);
}

bool Rewinder::perform() {
  if (robot_data_.size() < 2) {
    std::cout << "Robot data size is too small, at least 2 samples are required due to time step difference calculations" << std::endl;
    return false;
  }

  /// match people detections to logged data of the robot
  // update iterator to investigate robot data
  if (!robot_data_.empty()) {
    it_robot_ = robot_data_.cbegin();
  }
  // update iterator to investigate people data
  if (!data_people_empty_) {
    it_ppl_ = people_data_.cbegin();
  }
  // update iterator to investigate groups data
  if (!data_groups_empty_) {
    it_grp_ = groups_data_.cbegin();
  }

  // iterator for robot data (e.g. poses)
  // pose of robot in each time step must be investigated against pose of each person
  for (
    /* initialized above */;
    !robot_data_.empty() && it_robot_ != robot_data_.cend();
    it_robot_++)
  {
    // check if this is the last element of the robot data
    if (std::next(it_robot_) == robot_data_.cend()) {
      // handle event
      if (handler_last_timestamp_) {
        handler_last_timestamp_();
      }
    } else {
      // handle event
      if (handler_next_timestamp_) {
          handler_next_timestamp_();
      }
    }

    auto allPeopleCheckedFun = [&]() -> bool {
      return data_people_empty_ || (!data_people_empty_ && it_ppl_ == people_data_.cend());
    };
    auto allPeopleInTimestepCheckedFun = [&]() -> bool {
      return (!data_groups_empty_ && !data_people_empty_ && it_ppl_->first != it_robot_->first)
        || (data_groups_empty_ && !data_people_empty_ && it_ppl_->first != it_robot_->first)
        || allPeopleCheckedFun();
    };
    auto groupsAndPeopleSkippableFun = [&]() -> bool {
      return !data_groups_empty_ && (it_grp_->first > it_robot_->first)
        && !data_people_empty_ && (it_ppl_->first > it_robot_->first);
    };
    if (groupsAndPeopleSkippableFun()) {
      continue;
    }

    // iterate over all groups recognized in a given time step
    for (
      /*initialized above*/;
      (
        data_groups_empty_
        && (!data_people_empty_ && !allPeopleInTimestepCheckedFun() && !allPeopleCheckedFun())
      )
      ||
      (
        !data_groups_empty_ && it_grp_ != groups_data_.cend()
      );
      it_grp_++
    ) {
      // predicates for loops execution
      auto peopleItLaggingBehindGroupsFun = [&]() -> bool {
        return !data_groups_empty_ && !data_people_empty_ && it_ppl_->first < it_grp_->first;
      };
      auto allGroupsInTimestepCheckedFun = [&]() -> bool {
        return data_groups_empty_ || (!data_groups_empty_ && it_robot_->first != it_grp_->first);
      };
      auto nextGroupHasSameTimestampFun = [&]() -> bool {
        return !data_groups_empty_ && (std::next(it_grp_) != groups_data_.cend()) && (std::next(it_grp_)->first == it_grp_->first);
      };
      auto groupsEmptyButPeopleCheckedFun = [&]() -> bool {
        return data_groups_empty_ && allPeopleCheckedFun();
      };
      auto groupsInTimestampValidFun = [&]() -> bool {
        return !data_groups_empty_ && std::prev(it_grp_)->first == it_robot_->first;
      };
      // terminal conditions, based on them, action is taken at the end of the iteration
      auto checkingLastRobotFun = [&]() -> bool {
        return std::next(it_robot_) == robot_data_.cend();
      };
      auto checkingLastPersonFun = [&]() -> bool {
        return data_people_empty_ || std::next(it_ppl_) == people_data_.cend();
      };
      auto checkingLastGroupFun = [&]() -> bool {
        return !data_groups_empty_ && std::next(it_grp_) == groups_data_.cend();
      };
      auto lastSampleBeingCheckedFun = [&]() -> bool {
        return checkingLastRobotFun() && checkingLastGroupFun() && checkingLastPersonFun();
      };

      // call 'all groups' handler only if there were some groups and all were already processed
      if (allGroupsInTimestepCheckedFun() && allPeopleInTimestepCheckedFun()) {
        // handle event checking if there were some groups (timestamps match determines that)
        if (groupsInTimestampValidFun() && handler_all_groups_timestamp_) {
          handler_all_groups_timestamp_();
        }
        break;
      }

      // save people iterator to restore it in case of 1 person is assigned to multiple groups (must iterate over all people recognized at a specific time step)
      auto it_ppl_curr_timestamp = it_ppl_;

      // iterate over all people recognized in a given time step
      for (
        /*initialized above*/;
        !data_people_empty_ && it_ppl_ != people_data_.cend();
        it_ppl_++
      ) {
        if (allPeopleInTimestepCheckedFun()) {
          // handle event only when next timestamp is different from the current one
          if (!data_people_empty_ && handler_all_people_timestamp_) {
            handler_all_people_timestamp_();
          }
          break;
        }

        // handle event
        if (!data_people_empty_ && handler_next_person_timestamp_) {
          handler_next_person_timestamp_();
        }

        // when person overall last sample occurs in the last timestamp, it will be handled here since next person loop will not be executed
        if (checkingLastPersonFun()) {
          // handle event
          if (!data_people_empty_ && handler_all_people_timestamp_) {
            handler_all_people_timestamp_();
          }
        }

      } // iterating over people log entries

      // NOTE that it_ppl is always 1 step ahead of group at this moment
      auto foundPeopleRelatedToGroupFun = [&]() -> bool {
        return !data_groups_empty_ && it_ppl_ != people_data_.cbegin() && std::prev(it_ppl_)->first == it_grp_->first;
      };
      // handle event
      if (foundPeopleRelatedToGroupFun() && handler_next_group_timestamp_) {
        handler_next_group_timestamp_();
      }

      if (allGroupsInTimestepCheckedFun() || lastSampleBeingCheckedFun() || checkingLastGroupFun()) {
        // handle event
        if (foundPeopleRelatedToGroupFun() && handler_all_groups_timestamp_) {
          handler_all_groups_timestamp_();
        }
        break;
      }
      if (!checkingLastGroupFun() && nextGroupHasSameTimestampFun()) {
        it_ppl_ = it_ppl_curr_timestamp;
      }

    } // iterating over groups log entries

  } // iteration over robot log entries
  return true;
}

double Rewinder::getTimestampCurr() const {
  if (!isIteratorInRange(it_robot_, robot_data_.cbegin(), robot_data_.cend())) {
    throw Rewinder::createRuntimeError("current", "robot data");
  }
  return it_robot_->first;
}

double Rewinder::getTimestampPrev() const {
  if (!isIteratorInRange(std::prev(it_robot_), robot_data_.cbegin(), robot_data_.cend())) {
    throw Rewinder::createRuntimeError("previous", "robot data");
  }
  return std::prev(it_robot_)->first;
}

double Rewinder::getTimestampNext() const {
  if (!isIteratorInRange(std::next(it_robot_), robot_data_.cbegin(), robot_data_.cend())) {
    throw Rewinder::createRuntimeError("next", "robot data");
  }
  return std::next(it_robot_)->first;
}

double Rewinder::getTimestampFirst() const {
  if (robot_data_.empty()) {
    throw std::runtime_error("Cannot obtain first timestamp as robot data is empty\r\n");
  }
  return robot_data_.cbegin()->first;
}

double Rewinder::getTimestampLast() const {
  if (robot_data_.empty()) {
    throw std::runtime_error("Cannot obtain last timestamp as robot data is empty\r\n");
  }
  return (robot_data_.cend() - 1)->first;
}

RobotData Rewinder::getRobotCurr() const {
  if (!isIteratorInRange(it_robot_, robot_data_.cbegin(), robot_data_.cend())) {
    throw Rewinder::createRuntimeError("current", "robot data");
  }
  return it_robot_->second;
}

RobotData Rewinder::getRobotPrev() const {
  if (!isIteratorInRange(std::prev(it_robot_), robot_data_.cbegin(), robot_data_.cend())) {
    throw Rewinder::createRuntimeError("previous", "robot data");
  }
  return std::prev(it_robot_)->second;
}

RobotData Rewinder::getRobotNext() const {
  if (!isIteratorInRange(std::next(it_robot_), robot_data_.cbegin(), robot_data_.cend())) {
    throw Rewinder::createRuntimeError("next", "robot data");
  }
  return std::next(it_robot_)->second;
}

people_msgs_utils::Person Rewinder::getPersonCurr() const {
  if (!isIteratorInRange(it_ppl_, people_data_.cbegin(), people_data_.cend())) {
    throw Rewinder::createRuntimeError("current", "people data");
  }
  return it_ppl_->second;
}

people_msgs_utils::Person Rewinder::getPersonPrev() const {
  if (!isIteratorInRange(std::prev(it_ppl_), people_data_.cbegin(), people_data_.cend())) {
    throw Rewinder::createRuntimeError("previous", "people data");
  }
  return std::prev(it_ppl_)->second;
}

people_msgs_utils::Person Rewinder::getPersonNext() const {
  if (!isIteratorInRange(std::next(it_ppl_), people_data_.cbegin(), people_data_.cend())) {
    throw Rewinder::createRuntimeError("next", "people data");
  }
  return std::next(it_ppl_)->second;
}

people_msgs_utils::Group Rewinder::getGroupCurr() const {
  if (!isIteratorInRange(it_grp_, groups_data_.cbegin(), groups_data_.cend())) {
    throw Rewinder::createRuntimeError("current", "group data");
  }
  return it_grp_->second;
}

people_msgs_utils::Group Rewinder::getGroupPrev() const {
  if (!isIteratorInRange(std::prev(it_grp_), groups_data_.cbegin(), groups_data_.cend())) {
    throw Rewinder::createRuntimeError("previous", "group data");
  }
  return std::prev(it_grp_)->second;
}

people_msgs_utils::Group Rewinder::getGroupNext() const {
  if (!isIteratorInRange(std::next(it_grp_), groups_data_.cbegin(), groups_data_.cend())) {
    throw Rewinder::createRuntimeError("next", "group data");
  }
  return std::next(it_grp_)->second;
}

std::runtime_error Rewinder::Rewinder::createRuntimeError(const std::string& position, const std::string& data_set) const {
  std::runtime_error(
    "Cannot obtain " + position + " " + data_set + " (compared to "
    + std::to_string(getTimestampCurr())
    + " timestamp)"
  );
}

} // namespace postprocessing
} // namespace srpb
