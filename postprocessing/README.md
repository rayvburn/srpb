# Postprocessing - evaluation

To evaluate the local planner, use metrics calculator build on top of the [MRPB 1.0](https://github.com/NKU-MobFly-Robotics/local-planning-benchmark) by Wen et al. Once `csv` with experiment data is generated follow these steps:

```bash
cd <ROS_WORKSPACE_DIRECTORY>
source devel/setup.bash
cd $(rospack find move_base_benchmark)/postprocessing
mkdir build
cd build
cmake ..
make
./metric_evaluation <PATH_TO_LOG_FILE> <SAFETY_DISTANCE>
```
