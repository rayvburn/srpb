#include <angles/angles.h>
#include <srpb_logger/robot_logger.h>
#include <srpb_logger/people_logger.h>

#include "srpb_evaluation/utils.h"

#include "srpb_evaluation/metrics/backward_movements.h"
#include "srpb_evaluation/metrics/computational_efficiency.h"
#include "srpb_evaluation/metrics/cumulative_heading_change.h"
#include "srpb_evaluation/metrics/formation_space_instrusion.h"
#include "srpb_evaluation/metrics/inplace_rotations.h"
#include "srpb_evaluation/metrics/motion_efficiency.h"
#include "srpb_evaluation/metrics/obstacle_safety.h"
#include "srpb_evaluation/metrics/oscillations.h"
#include "srpb_evaluation/metrics/path_linear_length.h"
#include "srpb_evaluation/metrics/person_disturbance.h"
#include "srpb_evaluation/metrics/personal_space_instrusion.h"
#include "srpb_evaluation/metrics/velocity_smoothness.h"

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

  // oscillation threshold values
  double osc_vel_lin_x_threshold = 0.05;
  double osc_vel_ang_z_threshold = 0.15;

  // personal space Gaussian model parameters
  // values from Kirby, 2010, Fig. A.1
  double sigma_h = 2.00;
  double sigma_r = 1.00;
  double sigma_s = 1.33;
  // threshold of Gaussian value to detect space violations
  double personal_space_threshold = 0.55;
  double group_space_threshold = 0.55;
  // estimated field of view of people
  double person_fov = angles::from_degrees(190.0);
  // threshold of Gaussian value to detect significant disturbance caused by robot location or motion direction
  double disturbance_threshold = 0.20;

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

  ObstacleSafety safety(timed_robot_data, safety_distance);
  safety.printResults();

  MotionEfficiency mef(timed_robot_data);
  mef.printResults();

  ComputationalEfficiency cef(timed_robot_data);
  cef.printResults();

  VelocitySmoothness vsm(timed_robot_data);
  vsm.printResults();

  PathLinearLength plen(timed_robot_data);
  plen.printResults();

  CumulativeHeadingChange chc(timed_robot_data);
  chc.printResults();

  BackwardMovements bwd(timed_robot_data);
  bwd.printResults();

  Oscillations osc(timed_robot_data, osc_vel_lin_x_threshold, osc_vel_ang_z_threshold);
  osc.printResults();

  InplaceRotations inplace(timed_robot_data, osc_vel_lin_x_threshold);
  inplace.printResults();

  PersonalSpaceIntrusion psi(
    timed_robot_data,
    timed_people_data,
    sigma_h,
    sigma_r,
    sigma_s,
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

  PersonDisturbance disturbance(
    timed_robot_data,
    timed_people_data,
    disturbance_threshold,
    person_fov
  );
  disturbance.printResults();

  return 0;
}
