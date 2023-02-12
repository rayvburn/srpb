# srpb_evaluation

Package designed for evaluation of different navigation methods, especially in terms of social robots.

Once logs with experiment data are generated, follow these steps:

```bash
source <ROS_WORKSPACE_DIRECTORY>/devel/setup.bash
rosrun srpb_evaluation srpb_evaluation <PATH_TO_ROBOT_LOG_FILE>  <PATH_TO_PEOPLE_LOG_FILE>  <PATH_TO_PEOPLE_GROUPS_LOG_FILE> <SAFETY_DISTANCE>
```
