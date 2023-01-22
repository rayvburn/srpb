#include "postprocessing/rewinder.h"

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

    // allows not checking any groups data; no need to check for lack of people data here
    bool groups_empty_but_people_checked = false;

    // prepare predicates
    auto all_people_checked_timestep_fun = [&]()->bool {
      return (!data_groups_empty_ && !data_people_empty_ && it_ppl_->first != it_grp_->first)
      || (data_groups_empty_ && !data_people_empty_ && it_ppl_->first != it_robot_->first);
    };

    auto all_people_checked_fun = [&]()->bool {
      return data_people_empty_ || (!data_people_empty_ && (std::next(it_ppl_) == people_data_.cend()));
    };

    // iterate over all groups recognized in a given time step
    for (
      /*initialized above*/;
      (
        data_groups_empty_
        && (!data_people_empty_ && !all_people_checked_timestep_fun() && !all_people_checked_fun())
      )
      ||
      (
        !data_groups_empty_ && it_grp_ != groups_data_.cend()
      );
      it_grp_++
    ) {
      bool all_groups_checked_timestep =
        groups_empty_but_people_checked
        || (!data_groups_empty_ && (it_robot_->first != it_grp_->first));
      if (all_groups_checked_timestep) {
        // handle event
        if (handler_all_groups_timestamp_) {
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
        // break if timestamp does not match group's one
        bool all_people_checked_timestep = all_people_checked_timestep_fun();
        bool people_it_lagging_behind = !data_groups_empty_ && !data_people_empty_ && it_ppl_->first < it_grp_->first;
        bool checking_last_group = !data_groups_empty_ && std::next(it_grp_) == groups_data_.cend();
        bool checking_last_person = all_people_checked_fun();
        bool last_sample_being_checked = checking_last_group && checking_last_person;
        if (people_it_lagging_behind && !last_sample_being_checked) {
          // if people timestamp hasn't progressed as the group's one, let's skip people entries w/o groups
          while (it_ppl_->first < it_grp_->first) {
            it_ppl_++;
          }
        } else if (all_people_checked_timestep || last_sample_being_checked) {
          // handle event
          if (handler_all_people_timestamp_) {
            handler_all_people_timestamp_();
          }
          break;
        }

        // handle event
        if (handler_next_person_timestamp_) {
          handler_next_person_timestamp_();
        }

      } // iterating over people log entries

      // terminal conditions, based on them, action is taken at the end of the iteration
      bool checking_last_robot = std::next(it_robot_) == robot_data_.cend();
      bool checking_last_group = data_groups_empty_ || std::next(it_grp_) == groups_data_.cend();
      bool checking_last_person = data_people_empty_ || std::next(it_ppl_) == people_data_.cend();
      bool last_sample_being_checked = checking_last_robot && checking_last_group && checking_last_person;

      // handle event
      if (handler_next_group_timestamp_) {
        handler_next_group_timestamp_();
      }

      if (all_groups_checked_timestep || last_sample_being_checked || checking_last_group) {
        // handle event
        if (handler_all_groups_timestamp_) {
          handler_all_groups_timestamp_();
        }
        break;
      }

      // safely finished group computations, let's restore people iterator for this time step -
      // only if needed for the next group
      bool last_group_in_dataset = std::next(it_grp_) == groups_data_.cend();
      bool next_group_has_same_timestamp = std::next(it_grp_)->first == it_grp_->first;
      if (!last_group_in_dataset && next_group_has_same_timestamp) {
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
