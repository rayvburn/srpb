#pragma once

#include "srpb_evaluation/metric_gaussian.h"

#include <social_nav_utils/distance_vector.h>
#include <social_nav_utils/relative_location.h>
#include <social_nav_utils/gaussians.h>

// may prevent compilation errors calling to matrix.inverse()
#include <eigen3/Eigen/LU>

namespace srpb {
namespace evaluation {

/**
 * @brief Computes people personal space intrusion score
 *
 * People have their constrained area attached to the body projection. Once robot moves around people it may come
 * too closer or further. In each time step a value of a person area (modelled by an Asymmetric Gaussian) is computed.
 * This function returns scores: min Gaussian, max Gaussian and normalized to execution time. Maximum of Gaussian,
 * which is 1.0, shows that robot was located in the same position as person over the whole experiment. On the other
 * hand, normalized value of 0.0 means that robot never moved close to any person (according to the given variances).
 *
 * @param robot_data
 * @param people_data
 * @param var_h variance to the heading direction of the person (Gaussian)
 * @param var_r variance to the rear (Gaussian)
 * @param var_s variance to the side (Gaussian)
 * @param personal_space_threshold Gaussian values bigger than that will be considered as violation of personal space
 * @param max_method set to true (default) to use max element from Gaussians to normalize metrics;
 * false means averaging over all Gaussian occurrences in a current time step
 *
 * @return std::tuple<double, double, double, unsigned int> tuple with scores: min, max and normalized to execution
 * time and number of personal space violations (according to @ref personal_space_threshold)
 *
 * The closest to our method is the approach presented by Truong and Ngo in
 * “To Approach Humans?”: A Unified Framework for Approaching Pose Prediction and Socially Aware Robot Navigation
 * They called it `Social Individual Index`. Their method lacks normalization in terms of path duration.
 */
class PersonalSpaceIntrusion: public MetricGaussian {
public:
  PersonalSpaceIntrusion(
    const std::vector<std::pair<double, logger::RobotData>>& robot_data,
    const std::vector<std::pair<double, people_msgs_utils::Person>>& people_data,
    double var_h,
    double var_r,
    double var_s,
    double personal_space_threshold,
    bool max_method = true
  ):
    MetricGaussian(robot_data, people_data),
    var_front_(var_h),
    var_rear_(var_r),
    var_side_(var_s),
    personal_space_threshold_(personal_space_threshold),
    max_method_(max_method)
  {
    compute();
  }

  void printResults() const override {
    printf(
      "Personal space intrusion = %.4f [%%] (min = %.4f [%%], max = %.4f [%%], violations %.4f [%%])\n",
      intrusion_total_ * 100.0,
      intrusion_min_ * 100.0,
      intrusion_max_ * 100.0,
      violations_percentage_ * 100.0
    );
  }

  /**
   * @brief Computes value of a Gaussian modelling the personal space
   *
   * Includes pose uncertainty of the human
   *
   * @param person_pos_x
   * @param person_pos_y
   * @param person_orient_yaw
   * @param person_pos_cov_xx
   * @param person_pos_cov_xy
   * @param person_pos_cov_yx
   * @param person_pos_cov_yy
   * @param person_ps_var_front variance along the front direction of a person
   * @param person_ps_var_rear variance along the rear direction of a person
   * @param person_ps_var_side variance along the direction of person side
   * @param robot_pos_x
   * @param robot_pos_y
   * @param unify_asymmetry_scale adjusts scale of the output to avoid a step when front/rear covariances strongly differ
   * @return double
   */
  static double computePersonalSpaceGaussian(
    double person_pos_x,
    double person_pos_y,
    double person_orient_yaw,
    double person_pos_cov_xx,
    double person_pos_cov_xy,
    double person_pos_cov_yx,
    double person_pos_cov_yy,
    double person_ps_var_front,
    double person_ps_var_rear,
    double person_ps_var_side,
    double robot_pos_x,
    double robot_pos_y,
    bool unify_asymmetry_scale = false
  ) {
    // create matrix for covariance rotation
    double rot_angle = person_orient_yaw;
    Eigen::MatrixXd rot(2, 2);
    rot << std::cos(rot_angle), -std::sin(rot_angle),
      std::sin(rot_angle), std::cos(rot_angle);

    // create human position uncertainty matrix
    Eigen::MatrixXd cov_p(2, 2);
    cov_p << person_pos_cov_xx, person_pos_cov_xy, person_pos_cov_yx, person_pos_cov_yy;

    // prepare vectors for gaussian calculation
    // position to check Gaussian against - position of robot
    Eigen::VectorXd x_pos(2);
    x_pos << robot_pos_x, robot_pos_y;
    // mean - position of human
    Eigen::VectorXd mean_pos(2);
    mean_pos << person_pos_x, person_pos_y;

    /*
     * Perfect Gaussians in terms of mathematical description. Selecting `unify_asymmetry_scale`,
     * With asymmetry there will be a bump across the center axis due to different variances (thus maximums)
     */
    if (!unify_asymmetry_scale) {
      // aka phi
      social_nav_utils::DistanceVector dist_vector(
        person_pos_x,
        person_pos_y,
        robot_pos_x,
        robot_pos_y
      );

      // aka delta
      social_nav_utils::RelativeLocation rel_loc(dist_vector, person_orient_yaw);

      // choose variance
      double var_h_heading = person_ps_var_rear;
      if (rel_loc.getAngle() <= M_PI_2) {
        var_h_heading = person_ps_var_front;
      }

      // create covariance matrix of the personal zone model
      Eigen::MatrixXd cov_psi_init(2, 2);
      cov_psi_init << var_h_heading, 0.0, 0.0, person_ps_var_side;

      // rotate covariance matrix
      Eigen::MatrixXd cov_psi(2, 2);
      cov_psi = rot * cov_psi_init * rot.inverse();

      // resultant covariance matrix
      Eigen::MatrixXd cov_result(2, 2);
      cov_result = cov_p + cov_psi;

      // we already know the covariance so there is no need to evaluate the 'Asymmetrical' Gaussian case for `x_pos`
      return social_nav_utils::calculateGaussian(x_pos, mean_pos, cov_result);
    }

    /*
     * More popular version - the Gaussian with higher variance is prolonged in the uppper direction according
     * to the second one's maximum (in the mean pose)
     */
    // create covariance matrices of the personal zone model
    Eigen::MatrixXd cov_psi_init_front(2, 2);
    cov_psi_init_front << person_ps_var_front, 0.0, 0.0, person_ps_var_side;
    Eigen::MatrixXd cov_psi_init_rear(2, 2);
    cov_psi_init_rear << person_ps_var_rear, 0.0, 0.0, person_ps_var_side;

    // rotate covariance matrices
    Eigen::MatrixXd cov_psi_front(2, 2);
    cov_psi_front = rot * cov_psi_init_front * rot.inverse();
    Eigen::MatrixXd cov_psi_rear(2, 2);
    cov_psi_rear = rot * cov_psi_init_rear * rot.inverse();

    // resultant covariance matrices (variances summed up)
    Eigen::MatrixXd cov_result_front(2, 2);
    cov_result_front = cov_p + cov_psi_front;
    Eigen::MatrixXd cov_result_rear(2, 2);
    cov_result_rear = cov_p + cov_psi_rear;

    // compute value of asymmetric Gaussian
    return social_nav_utils::calculateGaussianAsymmetrical(
      x_pos,
      mean_pos,
      person_orient_yaw,
      cov_result_front,
      cov_result_rear,
      true
    );
  }

protected:
  // parameters
  double var_front_;
  double var_rear_;
  double var_side_;
  double personal_space_threshold_;
  bool max_method_;

  // results
  double intrusion_min_;
  double intrusion_max_;
  double intrusion_total_;
  double violations_percentage_;

  void compute() override {
    // store durations and Gaussians of the robot in terms of nearby people
    std::vector<std::pair<double, std::vector<double>>> timed_gaussians;
    /// prepare container for gaussian values in this `for` iteration
    std::pair<double, std::vector<double>> timed_gaussian;

    rewinder_.setHandlerNextTimestamp(
      [&]() {
        // prepare container for upcoming calculations related to human personal spaces
        timed_gaussian = std::make_pair(rewinder_.getTimestampNext() - rewinder_.getTimestampCurr(), std::vector<double>());
      }
    );

    // on event 'iterating through next person in the timestamp' - compute Gaussian of Personal Zone at robot position
    rewinder_.setHandlerNextPersonTimestamp(
      [&]() {
        // compute gaussian at position of robot
        double gaussian = computePersonalSpaceGaussian(
          rewinder_.getPersonCurr().getPositionX(),
          rewinder_.getPersonCurr().getPositionY(),
          rewinder_.getPersonCurr().getOrientationYaw(),
          rewinder_.getPersonCurr().getCovariancePoseXX(),
          rewinder_.getPersonCurr().getCovariancePoseXY(),
          rewinder_.getPersonCurr().getCovariancePoseYX(),
          rewinder_.getPersonCurr().getCovariancePoseYY(),
          var_front_,
          var_rear_,
          var_side_,
          rewinder_.getRobotCurr().getPositionX(),
          rewinder_.getRobotCurr().getPositionY(),
          true
        );

        //  find max of Gaussian knowing the current arrangement and certainty - compute gaussian at mean position
        double gaussian_max = computePersonalSpaceGaussian(
          rewinder_.getPersonCurr().getPositionX(),
          rewinder_.getPersonCurr().getPositionY(),
          rewinder_.getPersonCurr().getOrientationYaw(),
          rewinder_.getPersonCurr().getCovariancePoseXX(),
          rewinder_.getPersonCurr().getCovariancePoseXY(),
          rewinder_.getPersonCurr().getCovariancePoseYX(),
          rewinder_.getPersonCurr().getCovariancePoseYY(),
          var_front_,
          var_rear_,
          var_side_,
          rewinder_.getPersonCurr().getPositionX(),
          rewinder_.getPersonCurr().getPositionY(),
          true
        );

        // store result for later aggregation
        timed_gaussian.second.push_back(gaussian / gaussian_max);
      }
    );

    // on event 'iterated through all people in the timestamp'
    rewinder_.setHandlerAllPeopleTimestamp(
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
    ) = MetricGaussian::calculateGaussianStatistics(timed_gaussians, personal_space_threshold_, max_method_);
  }
};

} // namespace evaluation
} // namespace srpb
