#include <move_base/people_logger.h>
#include <tf2/utils.h>

#include <people_msgs_utils/utils.h>

void PeopleLogger::init(ros::NodeHandle& nh) {
  BenchmarkLogger::init(nh);

  auto log_path_base = log_filename_.substr(0, log_filename_.find_last_of(EXTENSION_SEPARATOR));
  auto log_extension = log_filename_.substr(log_filename_.find_last_of(EXTENSION_SEPARATOR) + 1);

  log_filename_people_ = log_path_base + "_people" + EXTENSION_SEPARATOR + log_extension;
  log_filename_groups_ = log_path_base + "_groups" + EXTENSION_SEPARATOR + log_extension;

  // social navigation benchmark utils
  people_sub_ = nh.subscribe<people_msgs::People>("/people", 1, boost::bind(&PeopleLogger::peopleCB, this, _1));
}

void PeopleLogger::start() {
  BenchmarkLogger::start(&log_file_people_, log_filename_people_);
  BenchmarkLogger::start(&log_file_groups_, log_filename_groups_);
}

void PeopleLogger::update(double timestamp) {
  // no people -> no groups
  // fill up people and group entries with timestamp (indicating that no detections and tracks were present then)
  if (people_.empty()) {
    fprintf(log_file_people_, "%9.4f\n", timestamp);
    fprintf(log_file_groups_, "%9.4f\n", timestamp);
    return;
  }

  for (const auto& person: people_) {
    if (log_file_people_ == nullptr) {
        throw std::runtime_error("People file descriptor for PeopleLogger was not properly created!");
    }
    fprintf(log_file_people_, "%9.4f %s\n", timestamp, personToString(person).c_str());
  }
  for (const auto& group: groups_) {
    if (log_file_groups_ == nullptr) {
        throw std::runtime_error("Groups file descriptor for PeopleLogger was not properly created!");
    }
    fprintf(log_file_groups_, "%9.4f %s\n", timestamp, groupToString(group).c_str());
  }
}

void PeopleLogger::finish() {
  BenchmarkLogger::finish(&log_file_people_);
  BenchmarkLogger::finish(&log_file_groups_);
}

std::string PeopleLogger::personToString(const people_msgs_utils::Person& person) {
  char buff[320] = {0};
  sprintf(
    buff,
    // id, rel,
    "%4u %9.4f "
    // px,  py,   pz,   yaw
    "%9.4f %9.4f %9.4f %9.4f "
    //cxx   cxy   cyx   cyy   cthth
    "%9.6f %9.6f %9.6f %9.6f %9.6f "
    // vx    vy    vz    vth
    "%9.4f %9.4f %9.4f %9.4f "
    // cvxx  cvxy  cvyx  cvyy  cvthh
    "%9.6f %9.6f %9.6f %9.6f %9.6f "
    // det age mt oc grp
    "%6u %12lu %d %d %6u",
    /*  0 */ person.getID(),
    /*  1 */ person.getReliability(),
    /*  2 */ person.getPositionX(),
    /*  3 */ person.getPositionY(),
    /*  4 */ person.getPositionZ(),
    /*  5 */ person.getOrientationYaw(),
    /*  6 */ person.getCovariancePoseXX(),
    /*  7 */ person.getCovariancePoseXY(),
    /*  8 */ person.getCovariancePoseYX(),
    /*  9 */ person.getCovariancePoseYY(),
    /* 10 */ person.getCovariancePoseYawYaw(),
    /* 11 */ person.getVelocityX(),
    /* 12 */ person.getVelocityY(),
    /* 13 */ person.getVelocityZ(),
    /* 14 */ person.getVelocityTheta(),
    /* 15 */ person.getCovarianceVelocityXX(),
    /* 16 */ person.getCovarianceVelocityXY(),
    /* 17 */ person.getCovarianceVelocityYX(),
    /* 18 */ person.getCovarianceVelocityYY(),
    /* 19 */ person.getCovarianceVelocityThTh(),
    /* 20 */ person.getDetectionID(),
    /* 21 */ person.getTrackAge(),
    /* 22 */ person.isMatched(),
    /* 23 */ person.isOccluded(),
    /* 24 */ person.getGroupID()
  );
  return std::string(buff);
}

people_msgs_utils::Person PeopleLogger::personFromString(const std::string& str) {
  auto vals = people_msgs_utils::parseString<double>(str, " ");

  // value indexes dictionary can be found in @ref personToString method
  assert(vals.size() == 25);

  // IDs are numbers in reality
  std::string name = std::to_string(static_cast<unsigned int>(vals.at(0)));
  double reliability = vals.at(1);

  geometry_msgs::PoseWithCovariance pose;
  pose.pose.position.x = vals.at(2);
  pose.pose.position.y = vals.at(3);
  pose.pose.position.z = vals.at(4);

  tf2::Quaternion quat_orient;
  quat_orient.setRPY(0.0, 0.0, vals.at(5));
  pose.pose.orientation.x = quat_orient.getX();
  pose.pose.orientation.y = quat_orient.getY();
  pose.pose.orientation.z = quat_orient.getZ();
  pose.pose.orientation.w = quat_orient.getW();

  pose.covariance.at(people_msgs_utils::Person::COV_XX_INDEX) = vals.at(6);
  pose.covariance.at(people_msgs_utils::Person::COV_XY_INDEX) = vals.at(7);
  pose.covariance.at(people_msgs_utils::Person::COV_YX_INDEX) = vals.at(8);
  pose.covariance.at(people_msgs_utils::Person::COV_YY_INDEX) = vals.at(9);
  pose.covariance.at(people_msgs_utils::Person::COV_YAWYAW_INDEX) = vals.at(10);

  geometry_msgs::PoseWithCovariance vel;
  vel.pose.position.x = vals.at(11);
  vel.pose.position.y = vals.at(12);
  vel.pose.position.z = vals.at(13);

  tf2::Quaternion quat_vel;
  quat_vel.setRPY(0.0, 0.0, vals.at(14));
  vel.pose.orientation.x = quat_vel.getX();
  vel.pose.orientation.y = quat_vel.getY();
  vel.pose.orientation.z = quat_vel.getZ();
  vel.pose.orientation.w = quat_vel.getW();

  vel.covariance.at(people_msgs_utils::Person::COV_XX_INDEX) = vals.at(15);
  vel.covariance.at(people_msgs_utils::Person::COV_XY_INDEX) = vals.at(16);
  vel.covariance.at(people_msgs_utils::Person::COV_YX_INDEX) = vals.at(17);
  vel.covariance.at(people_msgs_utils::Person::COV_YY_INDEX) = vals.at(18);
  vel.covariance.at(people_msgs_utils::Person::COV_YAWYAW_INDEX) = vals.at(19);

  unsigned int detection_id = vals.at(20);
  unsigned long int track_age = vals.at(21);
  bool matched = static_cast<bool>(vals.at(22));
  bool occluded = static_cast<bool>(vals.at(23));
  std::string group_name = std::to_string(static_cast<unsigned int>(vals.at(24)));

  return people_msgs_utils::Person(
    name,
    pose,
    vel,
    reliability,
    occluded,
    matched,
    detection_id,
    track_age,
    group_name
  );
}

std::string PeopleLogger::groupToString(const people_msgs_utils::Group& group) {
  char buff[180] = {0};

  // 1st part: static
  sprintf(
    buff,
    "%4u %9.4f %9.4f %9.4f %12lu ",
    group.getID(),
    group.getCenterOfGravity().x,
    group.getCenterOfGravity().y,
    group.getCenterOfGravity().z,
    group.getAge()
  );

  // dynamic size of member identifiers
  for (const auto& id: group.getMemberIDs()) {
    std::size_t pos = std::strlen(buff);
    sprintf(&buff[pos], " %4u", id);
  }

  // social relations will have dynamic size too
  // add separator, to divide static + member IDs part and relations part
  sprintf(&buff[std::strlen(buff)], " / ");
  for (const auto& relation: group.getSocialRelations()) {
    std::size_t pos = std::strlen(buff);
    auto who1 = std::get<0>(relation);
    auto who2 = std::get<1>(relation);
    auto rel_strength = std::get<2>(relation);
    sprintf(&buff[pos], " %4u %4u %6.4f", who1, who2, rel_strength);
  }

  return std::string(buff);
}

people_msgs_utils::Group PeopleLogger::groupFromString(const std::string& str) {
  // divide into 2 parts considering separator
  size_t separator = str.find("/");
  auto vals1 = people_msgs_utils::parseString<double>(str.substr(0, separator - 1), " ");
  auto vals2 = people_msgs_utils::parseString<double>(str.substr(separator + 1), " ");

  if (vals1.empty()) {
    // return a dummy group without any IDs of tracked people
    return people_msgs_utils::EMPTY_GROUP;
  }

  // 5 static entries and `group` must contain at least 2 IDs
  assert(vals1.size() >= 7);

  std::string name = std::to_string(static_cast<unsigned int>(vals1.at(0)));
  geometry_msgs::Point cog;
  cog.x = vals1.at(1);
  cog.y = vals1.at(2);
  cog.z = vals1.at(3);
  unsigned long int age = vals1.at(4);

  std::vector<unsigned int> member_ids;
  // start from the first track ID
  for (size_t i = 5; i < vals1.size(); i++) {
    member_ids.push_back(static_cast<unsigned int>(vals1.at(i)));
  }

  // check if relations are available
  std::vector<std::tuple<unsigned int, unsigned int, double>> relations;
  if (!vals2.empty() && vals2.size() % 3 == 0) {
    // each relation represented by a triplet
    for (
      auto it = vals2.cbegin();
      it != vals2.cend();
      it = it + 3
    ) {
      unsigned int id1 = *it;
      unsigned int id2 = *(it+1);
      double strength = *(it+2);
      relations.push_back(std::make_tuple(id1, id2, strength));
    }
  }

  /*
   * NOTE: members with their attributes are not available here so this is kinda dummy group,
   * needs further association with people set, see @ref people_msgs_utils::fillGroupsWithMembers
   */
  return people_msgs_utils::Group(
    name,
    age,
    std::vector<people_msgs_utils::Person>(),
    member_ids,
    relations,
    cog
  );
}

void PeopleLogger::peopleCB(const people_msgs::PeopleConstPtr& msg) {
  std::lock_guard<std::mutex> l(cb_mutex_);
  std::tie(people_, groups_) = people_msgs_utils::createFromPeople(msg->people);
}
