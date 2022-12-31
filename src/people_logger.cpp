#include <move_base/people_logger.h>
#include <tf2/utils.h>

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
    char buff[175] = {0};
    sprintf(
        buff,
        "%4u %9.4f %9.4f %9.4f %9.4f %9.4f %9.4f %9.4f %9.4f %9.4f %6u %16lu %d %d",
        person.getID(),
        person.getReliability(),
        person.getPositionX(),
        person.getPositionY(),
        person.getPositionZ(),
        person.getOrientationYaw(),
        person.getVelocityX(),
        person.getVelocityY(),
        person.getVelocityZ(),
        person.getVelocityTheta(),
        person.getDetectionID(),
        person.getTrackAge(),
        person.isMatched(),
        person.isOccluded()
    );
    return std::string(buff);
}

people_msgs_utils::Person PeopleLogger::personFromString(const std::string& str) {
  auto vals = people_msgs_utils::Person::parseString<double>(str, " ");

  // see method that creates such
  assert(vals.size() == 14);

  std::string name = std::to_string(vals.at(0));
  geometry_msgs::Point pos;
  pos.x = vals.at(2);
  pos.y = vals.at(3);
  pos.z = vals.at(4);
  geometry_msgs::Point vel;
  vel.x = vals.at(6);
  vel.y = vals.at(7);
  vel.z = vals.at(8);
  double reliability = vals.at(1);

  tf2::Quaternion quat_orient;
  quat_orient.setRPY(0.0, 0.0, vals.at(5));

  // not used...
  // tf2::Quaternion quat_vel;
  // quat_vel.setRPY(0.0, 0.0, vals.at(9));

  std::vector<std::string> tagnames;
  std::vector<std::string> tags;
  tagnames.push_back("orientation");
  tags.push_back(
    std::to_string(quat_orient.getX())
    + " " + std::to_string(quat_orient.getY())
    + " " + std::to_string(quat_orient.getZ())
    + " " + std::to_string(quat_orient.getW())
  );
  tagnames.push_back("occluded");
  tags.push_back(std::to_string(vals.at(13)));
  tagnames.push_back("matched");
  tags.push_back(std::to_string(vals.at(12)));
  tagnames.push_back("detection_id");
  tags.push_back(std::to_string(vals.at(10)));
  tagnames.push_back("track_age");
  tags.push_back(std::to_string(vals.at(11)));

  // NOTE: group-related tags are ommited here: `group_id`, `group_age`, `group_track_ids`, `group_center_of_gravity`
  return people_msgs_utils::Person(name, pos, vel, reliability, tagnames, tags);
}

std::string PeopleLogger::groupToString(const people_msgs_utils::Group& group) {
  char buff[125] = {0};
  sprintf(
    buff,
    "%4u %9.4f %9.4f %9.4f %11lu ",
    group.getID(),
    group.getCenterOfGravity().x,
    group.getCenterOfGravity().y,
    group.getCenterOfGravity().z,
    group.getAge()
  );
  for (const auto& track: group.getTrackIDs()) {
    std::size_t pos = std::strlen(buff);
    sprintf(&buff[pos], "%4u ", track);
  }
  return std::string(buff);
}

people_msgs_utils::Group PeopleLogger::groupFromString(const std::string& str) {
  auto vals = people_msgs_utils::Person::parseString<double>(str, " ");

  if (vals.empty()) {
    // return a dummy group without any IDs of tracked people
    return people_msgs_utils::DUMMY_GROUP;
  }

  // `group` must contain at least 1 ID
  assert(vals.size() >= 6);

  geometry_msgs::Point cog;
  cog.x = vals.at(1);
  cog.y = vals.at(2);
  cog.z = vals.at(3);

  std::vector<unsigned int> track_ids;
  // start from the first track ID
  for (size_t i = 5; i < vals.size(); i++) {
    track_ids.push_back(static_cast<unsigned int>(vals.at(i)));
  }

  return people_msgs_utils::Group(
    std::to_string(vals.at(0)),
    static_cast<unsigned int>(vals.at(4)),
    track_ids,
    cog
  );
}

void PeopleLogger::peopleCB(const people_msgs::PeopleConstPtr& msg) {
    std::lock_guard<std::mutex> l(cb_mutex_);

    people_.clear();
    for (const auto& person: msg->people) {
        people_.emplace_back(person);
    }

    groups_ = people_msgs_utils::Group::fromPeople(people_);
}
