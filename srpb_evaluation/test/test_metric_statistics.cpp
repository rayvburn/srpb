#include <gtest/gtest.h>

#include <srpb_evaluation/metric_statistics.h>

using namespace srpb::evaluation;

TEST(TestMetricStatistics, statistics) {
    // durations and, e.g., Gaussians
    std::vector<std::pair<double, std::vector<double>>> timed_gaussians{
        {0.25, {1.0, 0.75, 0.51, 0.24, 0.51, 0.75, 1.0, 1.25}},
        {0.50, {1.0, 0.75, 0.51, 0.25, 0.51, 0.75, 1.0, 1.27}},
        {0.75, {1.0, 0.75, 0.51, 0.26, 0.51, 0.75, 1.0, 1.26}},
        // {1.00, {}} // empty "second" - container, will force metrics to be 0.0
    };
    // tuple with: min, max and normalized, and threshold violations
    auto stats = MetricStatistics::calculateStatistics(timed_gaussians, 0.75, true, true);
    ASSERT_EQ(std::get<0>(stats), 0.24);
    ASSERT_EQ(std::get<1>(stats), 1.27);
    double duration = 0.25 + 0.50 + 0.75;
    // max_method is set to true, thus max values are pointed out below
    ASSERT_DOUBLE_EQ(
        std::get<2>(stats),
        1.25 * (0.25 / duration)
        + 1.27 * (0.50 / duration)
        + 1.26 * (0.75 / duration)
    );

    // violations ocurred in each time step, i.e., sum up the subsequent durations and divide them by a total duration
    // the metric is 1.0 here
    ASSERT_DOUBLE_EQ(
        std::get<3>(stats),
        (0.25 + 0.50 + 0.75) / duration
    );
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
    ASSERT_DOUBLE_EQ(
        std::get<2>(stats),
        0.2 * (0.24 / duration)
        + 0.6 * (0.26 / duration)
        + 0.5 * (0.25 / duration)
        + 0.3 * (0.23 / duration)
    );

    // sum up only a duration when the threshold was violated (distances less than a threshold)
    ASSERT_DOUBLE_EQ(
        std::get<3>(stats),
        (0.24 + 0.23) / duration
    );
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
    ASSERT_DOUBLE_EQ(
        std::get<2>(stats),
        0.51 * (0.25 / duration)
        + 0.51 * (0.25 / duration)
        + 0.51 * (0.25 / duration)
        + 0.51 * (0.25 / duration)
    );

    // violations did not occur
    ASSERT_DOUBLE_EQ(
        std::get<3>(stats),
        0.0 / duration
    );
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
    ASSERT_DOUBLE_EQ(
        std::get<2>(stats),
        0.48 * (0.22 / duration)
        + 0.49 * (0.23 / duration)
        + 0.50 * (0.24 / duration)
    );

    // 2 out of 3 samples are less than the threshold (they violate)
    ASSERT_DOUBLE_EQ(
        std::get<3>(stats),
        (0.22 + 0.23) / duration
    );
}

TEST(TestMetricStatistics, statisticsUnequal) {
    // durations and values (e.g., Gaussians)
    std::vector<std::pair<double, std::vector<double>>> timed_data{
        {0.22, {0.48, 0.51, 0.68, 0.12}},
        {0.23, {0.49, 0.34, 0.54, 0.86, 0.76}},
        {0.24, {0.50, 0.20}},
        {0.38, {0.10, 0.12}},
        {0.32, {0.97}}
    };
    // tuple with: min, max and normalized, and threshold violations
    // violation occurs above the threshold
    auto stats = MetricStatistics::calculateStatistics(timed_data, 0.5, true, true);
    ASSERT_EQ(std::get<0>(stats), 0.10);
    ASSERT_EQ(std::get<1>(stats), 0.97);
    double duration = 0.22 + 0.23 + 0.24 + 0.38 + 0.32;
    ASSERT_DOUBLE_EQ(
        std::get<2>(stats),
        0.68 * (0.22 / duration)
        + 0.86 * (0.23 / duration)
        + 0.50 * (0.24 / duration)
        + 0.12 * (0.38 / duration)
        + 0.97 * (0.32 / duration)
    );

    // in 3 out of 5 sets the threshold was violated
    ASSERT_DOUBLE_EQ(
        std::get<3>(stats),
        (0.22 + 0.23 + 0.32) / duration
    );
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
