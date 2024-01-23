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
    ASSERT_EQ(std::get<3>(stats), 9.0 / (3.0 * 8.0));
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
