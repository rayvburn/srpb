#include <srpb_logger/global_planner_logger.h>

// helper functions
#include <people_msgs_utils/utils.h>

namespace srpb {
namespace logger {

void GlobalPlannerLogger::init(ros::NodeHandle& nh) {
    BenchmarkLogger::init(nh);
    log_filename_ = BenchmarkLogger::appendToFilename(log_filename_, "gplanner");
}

void GlobalPlannerLogger::start() {
    if (!log_file_.is_open()) {
        BenchmarkLogger::start(log_file_, log_filename_);
        std::cout << "[ SRPB] Started the first log file for a global planner" << std::endl;
    } else if (log_file_.is_open()) {
        BenchmarkLogger::finish(log_file_);
        std::cout << "[ SRPB] Finishing a global planner log file" << std::endl;
        BenchmarkLogger::startNextPart(log_file_, log_filename_);
        std::cout << "[ SRPB] Started next log file for a global planner" << std::endl;
    }

    // increment the counter used for numbering goals
    incrementLogPartCounter();
}

void GlobalPlannerLogger::update(double timestamp, const GlobalPlannerData& planner) {
    if (!log_file_) {
        throw std::runtime_error("File for GlobalPlannerLogger was not properly created!");
    }

    std::stringstream ss;
    ss.setf(std::ios::fixed);
    ss << std::setw(9) << std::setprecision(4) << timestamp << " ";
    ss << GlobalPlannerLogger::plannerToString(planner) << std::endl;
    log_file_ << ss.str();
}

void GlobalPlannerLogger::finish() {
    BenchmarkLogger::finish(log_file_);
}

std::string GlobalPlannerLogger::plannerToString(const GlobalPlannerData& planner) {
    std::stringstream ss;
    ss.setf(std::ios::fixed);
    /*  0 */ ss << std::setw(9) << std::setprecision(4) << planner.getPositionX() << " ";
    /*  1 */ ss << std::setw(9) << std::setprecision(4) << planner.getPositionY() << " ";
    /*  2 */ ss << std::setw(9) << std::setprecision(4) << planner.getPositionZ() << " ";
    /*  3 */ ss << std::setw(9) << std::setprecision(4) << planner.getOrientationYaw() << " ";
    /*  4 */ ss << std::setw(9) << std::setprecision(4) << planner.getGoalPositionX() << " ";
    /*  5 */ ss << std::setw(9) << std::setprecision(4) << planner.getGoalPositionY() << " ";
    /*  6 */ ss << std::setw(9) << std::setprecision(4) << planner.getGoalPositionZ() << " ";
    /*  7 */ ss << std::setw(9) << std::setprecision(4) << planner.getGoalOrientationYaw() << " ";
    /*  8 */ ss << std::setw(4) << std::setprecision(1) << static_cast<double>(planner.getPlanningStatus()) << " ";
    /*  9 */ ss << std::setw(9) << std::setprecision(4) << planner.getPlanningTime() << " ";
    /* 10 */ ss << std::setw(9) << std::setprecision(1) << planner.getPlanSize();

    // dynamic size of the plan
    ss << " / ";
    const auto& plan = planner.getPlanCref();
    for (const auto& pose_stamped: plan) {
        ss << " " << std::setw(9) << std::setprecision(4) << pose_stamped.pose.position.x;
        ss << " " << std::setw(9) << std::setprecision(4) << pose_stamped.pose.position.y;
        ss << " " << std::setw(9) << std::setprecision(4) << tf2::getYaw(pose_stamped.pose.orientation);
    }
    return ss.str();
}

std::pair<bool, GlobalPlannerData> GlobalPlannerLogger::plannerFromString(const std::string& str) {
    // divide into 2 parts considering separator
    size_t separator = str.find("/");
    auto vals = people_msgs_utils::parseString<double>(str.substr(0, separator - 1), " ");
    auto vals_plan_poses = people_msgs_utils::parseString<double>(str.substr(separator + 1), " ");

    // static part of the entry has always the same length; a dynamic part must have 3 elements for each pose
    bool plan_poses_in_triplets = vals_plan_poses.size() % 3 != 0;
    if (vals.size() != 11 || plan_poses_in_triplets) {
        std::cout << "\x1B[31mFound corrupted data of a global planner:\r\n\t" << str << "\x1B[0m" << std::endl;
        // dummy planner data
        auto dummy_planner = GlobalPlannerData(
            geometry_msgs::Pose(),
            geometry_msgs::Pose(),
            false,
            0.0,
            std::vector<geometry_msgs::PoseStamped>()
        );
        return {false, dummy_planner};
    }

    geometry_msgs::Pose pose;
    pose.position.x = vals.at(0);
    pose.position.y = vals.at(1);
    pose.position.z = vals.at(2);
    tf2::Quaternion quat;
    quat.setRPY(0.0, 0.0, vals.at(3));
    pose.orientation.x = quat.getX();
    pose.orientation.y = quat.getY();
    pose.orientation.z = quat.getZ();
    pose.orientation.w = quat.getW();

    geometry_msgs::Pose goal;
    goal.position.x = vals.at(4);
    goal.position.y = vals.at(5);
    goal.position.z = vals.at(6);
    quat.setRPY(0.0, 0.0, vals.at(7));
    goal.orientation.x = quat.getX();
    goal.orientation.y = quat.getY();
    goal.orientation.z = quat.getZ();
    goal.orientation.w = quat.getW();

    bool planning_status = static_cast<bool>(static_cast<int>(vals.at(8)));
    double planning_time = vals.at(9);

    double plan_size = vals.at(10);
    std::vector<geometry_msgs::PoseStamped> plan;
    // each pose is represented by a triplet
    for (
        auto it = vals_plan_poses.cbegin();
        it != vals_plan_poses.cend();
        it = it + 3
    ) {
        double x = *it;
        double y = *(it+1);
        double yaw = *(it+2);
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = x;
        pose.pose.position.y = y;

        // NOTE: ctor of tf2::Quaternion using Euler angles is deprecated
        tf2::Quaternion quat;
        quat.setEuler(0.0, 0.0, yaw);
        pose.pose.orientation = tf2::toMsg(quat);

        plan.push_back(pose);
    }

    auto planner = GlobalPlannerData(pose, goal, planning_status, planning_time, plan);
    return {true, planner};
}

} // namespace logger
} // namespace srpb
