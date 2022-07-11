#pragma once

#include <costmap_2d/costmap_2d_ros.h>
#include <ceres/cubic_interpolation.h>
#include <opencv2/opencv.hpp>

// A helper class implemented for calculating the distance between the robot and the closest obstacle
class ObsDistCalculator
{
public:
  ObsDistCalculator();
  ~ObsDistCalculator();

  double compute(costmap_2d::Costmap2DROS* costmap_ros) const;
};
