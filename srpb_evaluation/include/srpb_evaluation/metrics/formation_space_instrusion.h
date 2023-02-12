#pragma once

#include "srpb_evaluation/metric_gaussian.h"

#include <srpb_logger/robot_data.h>
#include <people_msgs_utils/person.h>
#include <people_msgs_utils/group.h>

#include <social_nav_utils/gaussians.h>
#include <social_nav_utils/ellipse_fitting.h>

// may prevent compilation errors calling to matrix.inverse()
#include <eigen3/Eigen/LU>

namespace srpb {
namespace evaluation {

/// Similar to personal space intrusion but related to the group space
class FormationSpaceIntrusion: public MetricGaussian {
public:
  FormationSpaceIntrusion(
    const std::vector<std::pair<double, logger::RobotData>>& robot_data,
    const std::vector<std::pair<double, people_msgs_utils::Person>>& people_data,
    const std::vector<std::pair<double, people_msgs_utils::Group>>& groups_data,
    double group_space_threshold,
    bool max_method = true
  ):
    MetricGaussian(robot_data, people_data, groups_data),
    group_space_threshold_(group_space_threshold),
    max_method_(max_method)
  {
    compute();
  }

  void printResults() const override {
    printf(
      "Formation space intrusion = %.4f [%%] (min = %.4f [%%], max = %.4f [%%], violations %.4f [%%])\n",
      intrusion_total_ * 100.0,
      intrusion_min_ * 100.0,
      intrusion_max_ * 100.0,
      violations_percentage_ * 100.0
    );
  }

  /**
   * @brief Computes value of a Gaussian (given by method parameters) at given position
   *
   * @param ospace_pos_x
   * @param ospace_pos_y
   * @param ospace_orientation
   * @param ospace_variance_x
   * @param ospace_variance_y
   * @param pos_center_variance_xx
   * @param pos_center_variance_xyyx
   * @param pos_center_variance_yy
   * @param robot_pos_x
   * @param robot_pos_y
   * @return double
   *
   * @sa computeFormationSpaceGaussian
   */
  static double computeFormationSpaceGaussian(
    double ospace_pos_x,
    double ospace_pos_y,
    double ospace_orientation,
    double ospace_variance_x,
    double ospace_variance_y,
    double pos_center_variance_xx,
    double pos_center_variance_xyyx,
    double pos_center_variance_yy,
    double robot_pos_x,
    double robot_pos_y
  ) {
    // create matrix for covariance rotation
    Eigen::MatrixXd rot(2, 2);
    rot << std::cos(ospace_orientation), -std::sin(ospace_orientation),
      std::sin(ospace_orientation), std::cos(ospace_orientation);

    // create covariance matrix of the personal zone model
    Eigen::MatrixXd cov_fsi_init(2, 2);
    cov_fsi_init << ospace_variance_x, 0.0, 0.0, ospace_variance_y;

    // rotate covariance matrix
    Eigen::MatrixXd cov_fsi(2, 2);
    cov_fsi = rot * cov_fsi_init * rot.inverse();

    // create covariance matrix of the position estimation uncertainty
    Eigen::MatrixXd cov_pos(2, 2);
    cov_pos << pos_center_variance_xx, pos_center_variance_xyyx, pos_center_variance_xyyx, pos_center_variance_yy;

    // resultant covariance matrices (variances summed up)
    Eigen::MatrixXd cov_result(2, 2);
    cov_result = cov_pos + cov_fsi;

    // prepare vectors for gaussian calculation
    // position to check Gaussian against - position of robot
    Eigen::VectorXd x_pos(2);
    x_pos << robot_pos_x, robot_pos_y;
    // mean - position of the group
    Eigen::VectorXd mean_pos(2);
    mean_pos << ospace_pos_x, ospace_pos_y;

    return social_nav_utils::calculateGaussian(x_pos, mean_pos, cov_result);
  }

protected:
  // parameters
  double group_space_threshold_;
  bool max_method_;

  // results
  double intrusion_min_;
  double intrusion_max_;
  double intrusion_total_;
  double violations_percentage_;

  void compute() override {
    // store durations and Gaussians of the robot in terms of nearby groups
    std::vector<std::pair<double, std::vector<double>>> timed_gaussians;
    /// prepare container for gaussian values in this `for` iteration
    std::pair<double, std::vector<double>> timed_gaussian;

    rewinder_.setHandlerNextTimestamp(
      [&]() {
        // prepare container for upcoming calculations related to human personal spaces
        timed_gaussian = std::make_pair(rewinder_.getTimestampNext() - rewinder_.getTimestampCurr(), std::vector<double>());
      }
    );

    rewinder_.setHandlerNextGroupTimestamp(
      [&]() {
        // computations for the group
        // take half of the span and apply 2 sigma rule
        // (mean is the center of the O-space, 2 times stddev corresponds to its span)
        double variance_ospace_x = std::pow((rewinder_.getGroupCurr().getSpanX() / 2.0) / 2.0, 2);
        double variance_ospace_y = std::pow((rewinder_.getGroupCurr().getSpanY() / 2.0) / 2.0, 2);

        double gaussian = computeFormationSpaceGaussian(
          rewinder_.getGroupCurr().getPositionX(),
          rewinder_.getGroupCurr().getPositionY(),
          rewinder_.getGroupCurr().getOrientationYaw(),
          variance_ospace_x,
          variance_ospace_y,
          rewinder_.getGroupCurr().getCovariancePoseXX(),
          rewinder_.getGroupCurr().getCovariancePoseXY(),
          rewinder_.getGroupCurr().getCovariancePoseYY(),
          rewinder_.getRobotCurr().getPositionX(),
          rewinder_.getRobotCurr().getPositionY()
        );

        double gaussian_max = computeFormationSpaceGaussian(
          rewinder_.getGroupCurr().getPositionX(),
          rewinder_.getGroupCurr().getPositionY(),
          rewinder_.getGroupCurr().getOrientationYaw(),
          variance_ospace_x,
          variance_ospace_y,
          rewinder_.getGroupCurr().getCovariancePoseXX(),
          rewinder_.getGroupCurr().getCovariancePoseXY(),
          rewinder_.getGroupCurr().getCovariancePoseYY(),
          rewinder_.getGroupCurr().getPositionX(),
          rewinder_.getGroupCurr().getPositionY()
        );

        // Gaussian cost of the robot being located in the current pose; cost related to the investigated group of people
        timed_gaussian.second.push_back(gaussian / gaussian_max);
      }
    );

    rewinder_.setHandlerAllGroupsTimestamp(
      [&]() {
        if (!timed_gaussian.second.empty()) {
          timed_gaussians.push_back(timed_gaussian);
        }
      }
    );

    rewinder_.perform();

    std::tie(
      intrusion_min_,
      intrusion_max_,
      intrusion_total_,
      violations_percentage_
    ) = MetricGaussian::calculateGaussianStatistics(timed_gaussians, group_space_threshold_, max_method_);
  }
};

} // namespace evaluation
} // namespace srpb
