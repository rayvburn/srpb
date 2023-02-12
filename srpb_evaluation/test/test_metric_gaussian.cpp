#include <gtest/gtest.h>

#include <srpb_evaluation/metric_gaussian.h>
#include <srpb_evaluation/metrics/personal_space_instrusion.h>
#include <srpb_evaluation/metrics/formation_space_instrusion.h>

using namespace srpb::evaluation;

TEST(TestMetricGaussian, gaussianStatistics) {
    // durations and gaussians
    std::vector<std::pair<double, std::vector<double>>> timed_gaussians{
        {0.25, {1.0, 0.75, 0.51, 0.24, 0.51, 0.75, 1.0, 1.25}},
        {0.50, {1.0, 0.75, 0.51, 0.25, 0.51, 0.75, 1.0, 1.27}},
        {0.75, {1.0, 0.75, 0.51, 0.26, 0.51, 0.75, 1.0, 1.26}}
    };
    // tuple with: min, max and normalized, and threshold violations
    auto stats = MetricGaussian::calculateGaussianStatistics(timed_gaussians, 0.75, true);
    ASSERT_EQ(std::get<0>(stats), 0.24);
    ASSERT_EQ(std::get<1>(stats), 1.27);
    double duration = 0.25 + 0.50 + 0.75;
    ASSERT_EQ(
        std::get<2>(stats),
        1.25 * (0.25 / duration)
        + 1.27 * (0.50 / duration)
        + 1.26 * (0.75 / duration)
    );
    ASSERT_EQ(std::get<3>(stats), 9.0 / (3.0 * 8.0));
}

TEST(TestMetricGaussian, personalSpaceGaussian) {
    // Based on Matlab implementation: check pos, pcov, variance_*, then after computations look at X3mv, Y3mv and G3mv
    double gaussian1 = PersonalSpaceIntrusion::computePersonalSpaceGaussian(
        3.13779826339, /* person_pos_x */
        5.19603112355, /* person_pos_y */
        2.53422628911270, /* person_orient_yaw */
        0.127649644158897, /* person_pos_cov_xx */
        -9.01197659836358e-08, /* person_pos_cov_xy */
        -9.01197659853060e-08, /* person_pos_cov_yx */
        0.127649597818208, /* person_pos_cov_yy */
        3.00, /* person_ps_var_front */
        0.75, /* person_ps_var_rear */
        1.33, /* person_ps_var_side */
        2.68779826339000, /* robot_pos_x */
        4.74603112355000, /* robot_pos_y */
        true /* unify_asymmetry_scale */
    );
    EXPECT_NEAR(gaussian1, 0.122746777488856, 1e-05);

    double gaussian2 = PersonalSpaceIntrusion::computePersonalSpaceGaussian(
        3.13779826339, /* person_pos_x */
        5.19603112355, /* person_pos_y */
        2.53422628911270, /* person_orient_yaw */
        0.127649644158897, /* person_pos_cov_xx */
        -9.01197659836358e-08, /* person_pos_cov_xy */
        -9.01197659853060e-08, /* person_pos_cov_yx */
        0.127649597818208, /* person_pos_cov_yy */
        3.00, /* person_ps_var_front */
        0.75, /* person_ps_var_rear */
        1.33, /* person_ps_var_side */
        3.03779826339000, /* robot_pos_x */
        8.24603112355000, /* robot_pos_y */
        true /* unify_asymmetry_scale */
    );
    EXPECT_NEAR(gaussian2, 0.0106003940784025, 1e-05);
}

TEST(TestMetricGaussian, formationSpaceGaussian) {
    // Based on Matlab implementation
    double gaussian1 = FormationSpaceIntrusion::computeFormationSpaceGaussian(
        2.0000, /* ospace_pos_x */
        2.7500, /* ospace_pos_y */
        0.0, /* ospace_orientation */
        0.255208333333333, /* ospace_variance_x */
        0.765625000000000, /* ospace_variance_y */
        0.427649644158897, /* pos_center_variance_xx */
        0.0, /* pos_center_variance_xyyx */
        0.487649597818208, /* pos_center_variance_yy */
        2.10000000000000, /* robot_pos_x */
        2.85000000000000 /* robot_pos_y */
    );
    EXPECT_NEAR(gaussian1, 0.170105832109089, 1e-05);

    double gaussian2 = FormationSpaceIntrusion::computeFormationSpaceGaussian(
        2.0000, /* ospace_pos_x */
        2.7500, /* ospace_pos_y */
        0.0, /* ospace_orientation */
        0.255208333333333, /* ospace_variance_x */
        0.765625000000000, /* ospace_variance_y */
        0.427649644158897, /* pos_center_variance_xx */
        0.0, /* pos_center_variance_xyyx */
        0.487649597818208, /* pos_center_variance_yy */
        3.30000000000000, /* robot_pos_x */
        4.05000000000000 /* robot_pos_y */
    );
    EXPECT_NEAR(gaussian2, 0.0254331283458186, 1e-05);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
