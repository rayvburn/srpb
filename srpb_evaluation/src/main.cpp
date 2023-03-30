#include <angles/angles.h>
#include <srpb_logger/robot_logger.h>
#include <srpb_logger/people_logger.h>

#include "srpb_evaluation/utils.h"

using namespace srpb::logger;
using namespace srpb::evaluation;

int main(int argc, char* argv[]) {
  if (argc != 5) {
    printf(
      "Please input\r\n"
      "\t* the path to the log file of the robot\r\n"
      "\t* the path to the log file of the people\r\n"
      "\t* the path to the log file of the people groups\r\n"
      "\t* and value of the safety distance [m].\r\n"
    );
    return 1;
  }

  auto file_robot = std::string(argv[1]);
  auto file_people = std::string(argv[2]);
  auto file_groups = std::string(argv[3]);
  auto safety_distance = std::atof(argv[4]);

  // goal reached values
  double goal_tolerance_xy = 0.2;
  double goal_tolerance_yaw = 0.2;

  // oscillation threshold values
  double osc_vel_lin_x_threshold = 0.05;
  double osc_vel_x_threshold = 0.05;
  double osc_vel_y_threshold = 0.05;
  double osc_vel_ang_z_threshold = 0.15;

  /*
   * Threshold of Gaussian value to detect space violations.
   * With Kirby's parameters PS in the center is approx. 0.14 (without uncertainty accounted in).
   * With percentage results (implemented) - referring to max at each step.
   */
  double personal_space_threshold = 0.50;
  double group_space_threshold = 0.50;

  // How much space (radius) is physically occupied by a human
  double person_occupancy_radius = 0.28;
  // estimated field of view of people
  double person_fov = angles::from_degrees(190.0);
    // Size of the circumradius of the evaluated mobile base
  double robot_circumradius = 0.275;
  // Maximum allowable speed for the evaluated robot
  double robot_max_speed = 0.55;
  // threshold of Gaussian value to detect significant disturbance caused by robot location or motion direction
  double heading_disturbance_threshold = 0.20;

  auto timed_robot_data = parseFile<RobotData>(file_robot, &RobotLogger::robotFromString);
  auto timed_people_data = parseFile<people_msgs_utils::Person>(file_people, &PeopleLogger::personFromString);
  auto timed_groups_data = parseFile<people_msgs_utils::Group>(file_groups, &PeopleLogger::groupFromString);
  // since Person and Group are logged in separation, so by default Group does not contain members, only their IDs
  timed_groups_data = fillGroupsWithMembers(timed_groups_data, timed_people_data);

  printf(
    "Processing %4lu samples of robot data, %4lu samples of people data and %4lu samples of groups data\r\n",
    timed_robot_data.size(),
    timed_people_data.size(),
    timed_groups_data.size()
  );

  GoalReached mgoal(timed_robot_data, goal_tolerance_xy, goal_tolerance_yaw);
  mgoal.printResults();

  ObstacleSafety safety(timed_robot_data, safety_distance);
  safety.printResults();

  MotionEfficiency mef(timed_robot_data);
  mef.printResults();

  ComputationalEfficiency cef(timed_robot_data);
  cef.printResults();

  ComputationalTimeRepeatability cre(timed_robot_data);
  cre.printResults();

  VelocitySmoothness vsm(timed_robot_data);
  vsm.printResults();

  HeadingChangeSmoothness hsm(timed_robot_data);
  hsm.printResults();

  PathLinearLength plen(timed_robot_data);
  plen.printResults();

  CumulativeHeadingChange chc(timed_robot_data);
  chc.printResults();

  Oscillations osc(
    timed_robot_data,
    osc_vel_lin_x_threshold,
    osc_vel_x_threshold,
    osc_vel_y_threshold,
    osc_vel_ang_z_threshold
  );
  osc.printResults();

  BackwardMovements bwd(timed_robot_data, osc_vel_x_threshold);
  bwd.printResults();

  InplaceRotations inplace(timed_robot_data, osc_vel_ang_z_threshold);
  inplace.printResults();

  PersonalSpaceIntrusion psi(
    timed_robot_data,
    timed_people_data,
    personal_space_threshold
  );
  psi.printResults();

  FormationSpaceIntrusion fsi(
    timed_robot_data,
    timed_people_data,
    timed_groups_data,
    group_space_threshold
  );
  fsi.printResults();

  HeadingDirectionDisturbance heading_direction(
    timed_robot_data,
    timed_people_data,
    heading_disturbance_threshold,
    person_occupancy_radius,
    person_fov,
    robot_circumradius,
    robot_max_speed
  );
  heading_direction.printResults();

  // save results file
  auto file_results = file_robot.substr(0, file_robot.find_last_of('_')) + "_results.txt";
  createResultsFile(
    file_results,
    timed_robot_data.size(),
    timed_people_data.size(),
    timed_groups_data.size(),
    mgoal,
    safety,
    mef,
    cef,
    cre,
    vsm,
    hsm,
    plen,
    chc,
    osc,
    bwd,
    inplace,
    psi,
    fsi,
    heading_direction
  );

  return 0;
}
