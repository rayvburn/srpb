#include <gtest/gtest.h>

#include <srpb_evaluation/metric_statistics.h>

using namespace srpb::evaluation;

TEST(TestMetricStatistics, statistics) {
    // durations and, e.g., Gaussians
    std::vector<std::pair<double, std::vector<double>>> timed_gaussians{
        {0.25, {1.0, 0.75, 0.51, 0.24, 0.51, 0.75, 1.0, 1.25}},
        {0.50, {1.0, 0.75, 0.51, 0.25, 0.51, 0.75, 1.0, 1.27}},
        {0.75, {1.0, 0.75, 0.51, 0.26, 0.51, 0.75, 1.0, 1.26}}
    };
    // tuple with: min, max and normalized, and threshold violations
    auto stats = MetricStatistics::calculateStatistics(timed_gaussians, 0.75, true, true);
    ASSERT_EQ(std::get<0>(stats), 0.24);
    ASSERT_EQ(std::get<1>(stats), 1.27);
    double duration = 0.25 + 0.50 + 0.75;
    ASSERT_EQ(
        std::get<2>(stats),
        1.25 * (0.25 / duration)
        + 1.27 * (0.50 / duration)
        + 1.26 * (0.75 / duration)
    );
    // violations are computed with regards to all Gaussians, i.e., the number of violating samples is divided
    // by the total number of samples
    ASSERT_EQ(std::get<3>(stats), 9.0 / (3.0 * 8.0));
}

TEST(TestMetricStatistics, statisticsObstacles1) {
    // durations and, e.g., distances to obstacles
    std::vector<std::pair<double, std::vector<double>>> timed_dists{
        {0.24, {0.2}},
        {0.26, {0.6}},
        {0.25, {0.5}},
        {0.23, {0.3}}
    };
    // tuple with: min, max and normalized, and threshold violations
    auto stats = MetricStatistics::calculateStatistics(timed_dists, 0.5, false, true);
    ASSERT_EQ(std::get<0>(stats), 0.2);
    ASSERT_EQ(std::get<1>(stats), 0.6);
    double duration = 0.24 + 0.26 + 0.25 + 0.23;
    ASSERT_EQ(
        std::get<2>(stats),
        0.2 * (0.24 / duration)
        + 0.6 * (0.26 / duration)
        + 0.5 * (0.25 / duration)
        + 0.3 * (0.23 / duration)
    );
    // 2 out of 4 samples are less than the threshold
    ASSERT_EQ(std::get<3>(stats), 2.0 / 4.0);
}

TEST(TestMetricStatistics, statisticsObstacles2) {
    // durations and, e.g., distances to obstacles
    std::vector<std::pair<double, std::vector<double>>> timed_dists{
        {0.25, {0.51}},
        {0.25, {0.51}},
        {0.25, {0.51}},
        {0.25, {0.51}}
    };
    // tuple with: min, max and normalized, and threshold violations
    auto stats = MetricStatistics::calculateStatistics(timed_dists, 0.5, false, true);
    ASSERT_EQ(std::get<0>(stats), 0.51);
    ASSERT_EQ(std::get<1>(stats), 0.51);
    double duration = 0.25 + 0.25 + 0.25 + 0.25;
    ASSERT_EQ(
        std::get<2>(stats),
        0.51 * (0.25 / duration)
        + 0.51 * (0.25 / duration)
        + 0.51 * (0.25 / duration)
        + 0.51 * (0.25 / duration)
    );
    ASSERT_EQ(std::get<3>(stats), 0.0 / 4.0);
}

TEST(TestMetricStatistics, statisticsObstacles3) {
    // durations and, e.g., distances to obstacles
    std::vector<std::pair<double, std::vector<double>>> timed_dists{
        {0.22, {0.48}},
        {0.23, {0.49}},
        {0.24, {0.50}},
    };
    // tuple with: min, max and normalized, and threshold violations
    auto stats = MetricStatistics::calculateStatistics(timed_dists, 0.5, false, true);
    ASSERT_EQ(std::get<0>(stats), 0.48);
    ASSERT_EQ(std::get<1>(stats), 0.50);
    double duration = 0.22 + 0.23 + 0.24;
    ASSERT_EQ(
        std::get<2>(stats),
        0.48 * (0.22 / duration)
        + 0.49 * (0.23 / duration)
        + 0.50 * (0.24 / duration)
    );
    // 2 out of 3 are less than the threshold
    ASSERT_EQ(std::get<3>(stats), 2.0 / 3.0);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
