#include <srpb_logger/people_logger.h>
#include <tf2/utils.h>

#include <people_msgs_utils/utils.h>

#include <sstream>
#include <iomanip>

namespace srpb {
namespace logger {

const std::string PeopleLogger::GROUP_NAME_EMPTY = "none";

void PeopleLogger::init(ros::NodeHandle& nh) {
  BenchmarkLogger::init(nh);
  log_filename_people_ = BenchmarkLogger::appendToFilename(log_filename_, "people");
  log_filename_groups_ = BenchmarkLogger::appendToFilename(log_filename_, "groups");

  // social navigation benchmark utils
  people_sub_ = nh.subscribe<people_msgs::People>("/people", 1, boost::bind(&PeopleLogger::peopleCB, this, _1));
}

void PeopleLogger::start() {
  // people file management
  if (!log_file_people_.is_open()) {
    BenchmarkLogger::start(log_file_people_, log_filename_people_);
    std::cout << "[ SRPB] Started the first log file for people" << std::endl;
  } else {
    BenchmarkLogger::finish(log_file_people_);
    std::cout << "[ SRPB] Finishing a people log file" << std::endl;
    BenchmarkLogger::startNextPart(log_file_people_, log_filename_people_);
    std::cout << "[ SRPB] Started next log file for people" << std::endl;
  }

  // group file management
  if (!log_file_groups_.is_open()) {
    BenchmarkLogger::start(log_file_groups_, log_filename_groups_);
    std::cout << "[ SRPB] Started the first log file for groups" << std::endl;
  } else {
    BenchmarkLogger::finish(log_file_groups_);
    std::cout << "[ SRPB] Finishing a groups log file" << std::endl;
    BenchmarkLogger::startNextPart(log_file_groups_, log_filename_groups_);
    std::cout << "[ SRPB] Started next log file for groups" << std::endl;
  }

  // increment the counter used for numbering goals
  incrementLogPartCounter();
}

void PeopleLogger::update(double timestamp) {
  // make sure that the log files are OK
  if (!log_file_people_) {
    throw std::runtime_error("File for PeopleLogger (people) was not properly created!");
  }

  if (!log_file_groups_) {
    throw std::runtime_error("File for PeopleLogger (groups) was not properly created!");
  }

  // we'll be processing the data that may be updated asynchronously
  std::lock_guard<std::mutex> l(cb_mutex_);

  // no people -> no groups
  // fill up people and group entries with timestamp (indicating that no detections and tracks were present then)
  if (people_.empty()) {
    std::stringstream ss;
    ss.setf(std::ios::fixed);
    ss << std::setw(9) << std::setprecision(4) << timestamp << std::endl;
    log_file_people_ << ss.str();
    log_file_groups_ << ss.str();
    return;
  }

  // log received data
  for (const auto& person: people_) {
    std::stringstream ss;
    ss.setf(std::ios::fixed);
    ss << std::setw(9) << std::setprecision(4) << timestamp << " ";
    ss << personToString(person) << std::endl;
    log_file_people_ << ss.str();
  }

  // mark the timestamp of groups even if no valid data is available
  if (groups_.empty()) {
    std::stringstream ss;
    ss.setf(std::ios::fixed);
    ss << std::setw(9) << std::setprecision(4) << timestamp << std::endl;
    log_file_groups_ << ss.str();
    return;
  }
  for (const auto& group: groups_) {
    std::stringstream ss;
    ss.setf(std::ios::fixed);
    ss << std::setw(9) << std::setprecision(4) << timestamp << " ";
    ss << groupToString(group) << std::endl;
    log_file_groups_ << ss.str();
  }
}

void PeopleLogger::finish() {
  BenchmarkLogger::finish(log_file_people_);
  BenchmarkLogger::finish(log_file_groups_);
}

std::string PeopleLogger::personToString(const people_msgs_utils::Person& person) {
  std::stringstream ss;
  ss.setf(std::ios::fixed);
  /*  0 */ ss << std::setw(6) << person.getName() << " ";
  /*  1 */ ss << std::setw(9) << std::setprecision(4) << person.getReliability() << " ";
  /*  2 */ ss << std::setw(9) << std::setprecision(4) << person.getPositionX() << " ";
  /*  3 */ ss << std::setw(9) << std::setprecision(4) << person.getPositionY() << " ";
  /*  4 */ ss << std::setw(9) << std::setprecision(4) << person.getPositionZ() << " ";
  /*  5 */ ss << std::setw(9) << std::setprecision(4) << person.getOrientationYaw() << " ";
  /*  6 */ ss << std::setw(9) << std::setprecision(6) << person.getCovariancePoseXX() << " ";
  /*  7 */ ss << std::setw(9) << std::setprecision(6) << person.getCovariancePoseXY() << " ";
  /*  8 */ ss << std::setw(9) << std::setprecision(6) << person.getCovariancePoseYX() << " ";
  /*  9 */ ss << std::setw(9) << std::setprecision(6) << person.getCovariancePoseYY() << " ";
  /* 10 */ ss << std::setw(9) << std::setprecision(6) << person.getCovariancePoseYawYaw() << " ";
  /* 11 */ ss << std::setw(9) << std::setprecision(4) << person.getVelocityX() << " ";
  /* 12 */ ss << std::setw(9) << std::setprecision(4) << person.getVelocityY() << " ";
  /* 13 */ ss << std::setw(9) << std::setprecision(4) << person.getVelocityZ() << " ";
  /* 14 */ ss << std::setw(9) << std::setprecision(4) << person.getVelocityTheta() << " ";
  /* 15 */ ss << std::setw(9) << std::setprecision(6) << person.getCovarianceVelocityXX() << " ";
  /* 16 */ ss << std::setw(9) << std::setprecision(6) << person.getCovarianceVelocityXY() << " ";
  /* 17 */ ss << std::setw(9) << std::setprecision(6) << person.getCovarianceVelocityYX() << " ";
  /* 18 */ ss << std::setw(9) << std::setprecision(6) << person.getCovarianceVelocityYY() << " ";
  /* 19 */ ss << std::setw(9) << std::setprecision(6) << person.getCovarianceVelocityThTh() << " ";
  /* 20 */ ss << std::setw(6) << person.getDetectionID() << " ";
  /* 21 */ ss << std::setw(12) << person.getTrackAge() << " ";
  /* 22 */ ss << std::setw(1) << person.isMatched() << " ";
  /* 23 */ ss << std::setw(1) << person.isOccluded() << " ";
  /* 24 */ ss << std::setw(6) << (person.isAssignedToGroup() ? person.getGroupName() : GROUP_NAME_EMPTY);
  return ss.str();
}

std::pair<bool, people_msgs_utils::Person> PeopleLogger::personFromString(const std::string& str) {
  auto vals = people_msgs_utils::parseString<std::string>(str, " ");

  // value indexes dictionary can be found in @ref personToString method
  if (vals.size() != 25) {
    std::cout << "\x1B[31mFound corrupted data of a person:\r\n\t" << str << "\x1B[0m" << std::endl;
    // return a dummy person
    auto dummy_person = people_msgs_utils::Person(
      "",
      geometry_msgs::PoseWithCovariance(),
      geometry_msgs::PoseWithCovariance(),
      0.0,
      true,
      false,
      std::numeric_limits<unsigned int>::max(),
      std::numeric_limits<unsigned long int>::max(),
      ""
    );
    return {false, dummy_person};
  }

  // IDs are numbers in reality
  std::string name = vals.at(0);
  double reliability = std::stod(vals.at(1));

  geometry_msgs::PoseWithCovariance pose;
  pose.pose.position.x = std::stod(vals.at(2));
  pose.pose.position.y = std::stod(vals.at(3));
  pose.pose.position.z = std::stod(vals.at(4));

  tf2::Quaternion quat_orient;
  quat_orient.setRPY(0.0, 0.0, std::stod(vals.at(5)));
  pose.pose.orientation.x = quat_orient.getX();
  pose.pose.orientation.y = quat_orient.getY();
  pose.pose.orientation.z = quat_orient.getZ();
  pose.pose.orientation.w = quat_orient.getW();

  pose.covariance.at(people_msgs_utils::Person::COV_XX_INDEX) = std::stod(vals.at(6));
  pose.covariance.at(people_msgs_utils::Person::COV_XY_INDEX) = std::stod(vals.at(7));
  pose.covariance.at(people_msgs_utils::Person::COV_YX_INDEX) = std::stod(vals.at(8));
  pose.covariance.at(people_msgs_utils::Person::COV_YY_INDEX) = std::stod(vals.at(9));
  pose.covariance.at(people_msgs_utils::Person::COV_YAWYAW_INDEX) = std::stod(vals.at(10));

  geometry_msgs::PoseWithCovariance vel;
  vel.pose.position.x = std::stod(vals.at(11));
  vel.pose.position.y = std::stod(vals.at(12));
  vel.pose.position.z = std::stod(vals.at(13));

  tf2::Quaternion quat_vel;
  quat_vel.setRPY(0.0, 0.0, std::stod(vals.at(14)));
  vel.pose.orientation.x = quat_vel.getX();
  vel.pose.orientation.y = quat_vel.getY();
  vel.pose.orientation.z = quat_vel.getZ();
  vel.pose.orientation.w = quat_vel.getW();

  vel.covariance.at(people_msgs_utils::Person::COV_XX_INDEX) = std::stod(vals.at(15));
  vel.covariance.at(people_msgs_utils::Person::COV_XY_INDEX) = std::stod(vals.at(16));
  vel.covariance.at(people_msgs_utils::Person::COV_YX_INDEX) = std::stod(vals.at(17));
  vel.covariance.at(people_msgs_utils::Person::COV_YY_INDEX) = std::stod(vals.at(18));
  vel.covariance.at(people_msgs_utils::Person::COV_YAWYAW_INDEX) = std::stod(vals.at(19));

  unsigned int detection_id = std::stoul(vals.at(20));
  unsigned long int track_age = std::stoul(vals.at(21));
  bool matched = static_cast<bool>(std::stoi(vals.at(22)));
  bool occluded = static_cast<bool>(std::stoi(vals.at(23)));
  std::string group_name = "";
  if (vals.at(24) != GROUP_NAME_EMPTY) {
    group_name = vals.at(24);
  }

  auto person = people_msgs_utils::Person(
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
  return {true, person};
}

std::string PeopleLogger::groupToString(const people_msgs_utils::Group& group) {
  std::stringstream ss;
  ss.setf(std::ios::fixed);

  // 1st part: static
  ss << std::setw(6) << group.getName() << " ";
  ss << std::setw(9) << std::setprecision(4) << group.getCenterOfGravity().x << " ";
  ss << std::setw(9) << std::setprecision(4) << group.getCenterOfGravity().y << " ";
  ss << std::setw(9) << std::setprecision(4) << group.getCenterOfGravity().z << " ";
  ss << std::setw(12) << group.getAge() << " ";

  // dynamic size of member identifiers
  for (const auto& id: group.getMemberIDs()) {
    ss << " " << std::setw(6) << id;
  }

  // social relations will have dynamic size too
  // add separator, to divide static + member IDs part and relations part
  ss << " / ";
  for (const auto& relation: group.getSocialRelations()) {
    auto who1 = std::get<0>(relation);
    auto who2 = std::get<1>(relation);
    auto rel_strength = std::get<2>(relation);
    ss << " " << std::setw(6) << who1;
    ss << " " << std::setw(6) << who2;
    ss << " " << std::setw(6) << std::setprecision(4) << rel_strength;
  }
  return ss.str();
}

std::pair<bool, people_msgs_utils::Group> PeopleLogger::groupFromString(const std::string& str) {
  // divide into 2 parts considering separator
  size_t separator = str.find("/");
  auto vals1 = people_msgs_utils::parseString<std::string>(str.substr(0, separator - 1), " ");
  auto vals2 = people_msgs_utils::parseString<std::string>(str.substr(separator + 1), " ");

  if (vals1.empty()) {
    // return a dummy group without any IDs of tracked people
    return {false, people_msgs_utils::EMPTY_GROUP};
  }

  // 5 static entries and `group` must contain at least 2 IDs
  if (vals1.size() < 7) {
    std::cout << "\x1B[31mFound corrupted data of a group:\r\n\t" << str << "\x1B[0m" << std::endl;
    // return a dummy group
    return {false, people_msgs_utils::EMPTY_GROUP};
  }

  std::string name = vals1.at(0);
  geometry_msgs::Point cog;
  cog.x = std::stod(vals1.at(1));
  cog.y = std::stod(vals1.at(2));
  cog.z = std::stod(vals1.at(3));
  unsigned long int age = std::stoul(vals1.at(4));

  std::vector<std::string> member_ids;
  // start from the first track ID
  for (size_t i = 5; i < vals1.size(); i++) {
    member_ids.push_back(vals1.at(i));
  }

  // check if relations are available
  std::vector<std::tuple<std::string, std::string, double>> relations;
  if (!vals2.empty() && vals2.size() % 3 == 0) {
    // each relation represented by a triplet
    for (
      auto it = vals2.cbegin();
      it != vals2.cend();
      it = it + 3
    ) {
      std::string id1 = *it;
      std::string id2 = *(it+1);
      double strength = std::stod(*(it+2));
      relations.push_back(std::make_tuple(id1, id2, strength));
    }
  }

  /*
   * NOTE: members with their attributes are not available here so this is kinda dummy group,
   * needs further association with people set, see @ref people_msgs_utils::fillGroupsWithMembers
   */
  auto group = people_msgs_utils::Group(
    name,
    age,
    std::vector<people_msgs_utils::Person>(),
    member_ids,
    relations,
    cog
  );
  return {true, group};
}

void PeopleLogger::peopleCB(const people_msgs::PeopleConstPtr& msg) {
  std::lock_guard<std::mutex> l(cb_mutex_);
  std::tie(people_, groups_) = people_msgs_utils::createFromPeople(msg->people);

  // once extracted, let's transform to the common frame -
  // find a transform from the source frame (msg's) to the frame of the logger
  auto transform = getTransform(msg->header.frame_id);
  // transform people applying the transformation above
  for (auto& person: people_) {
    // TODO: consider transforming velocity to the local frame of the person,
    // Nevertheless, currently metrics don't use that information
    person.transform(transform);
  }

  // transform groups too
  for (auto& group: groups_) {
    group.transform(transform);
  }
}

} // namespace logger
} // namespace srpb
