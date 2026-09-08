#include "jaka_planner/trajectory_selector.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

TEST(TrajectorySelector, ComputesAccumulatedJointTravel)
{
  const std::vector<std::vector<double>> positions = {
    {0.0, 0.0}, {1.0, -2.0}, {0.5, -1.0}};
  EXPECT_DOUBLE_EQ(jaka_planner::computeJointTravel(positions), 4.5);
}

TEST(TrajectorySelector, RejectsInconsistentPointDimensions)
{
  const std::vector<std::vector<double>> positions = {{0.0}, {1.0, 2.0}};
  EXPECT_THROW(jaka_planner::computeJointTravel(positions), std::invalid_argument);
}

TEST(TrajectorySelector, HigherCostHasLowerScore)
{
  jaka_planner::SelectionConfig config;
  const double preferred = jaka_planner::computeScore(2.0, 3.0, 0.1, config);
  const double slower = jaka_planner::computeScore(3.0, 3.0, 0.1, config);
  const double more_motion = jaka_planner::computeScore(2.0, 4.0, 0.1, config);
  const double nearer_singularity = jaka_planner::computeScore(2.0, 3.0, 0.05, config);
  EXPECT_GT(preferred, slower);
  EXPECT_GT(preferred, more_motion);
  EXPECT_GT(preferred, nearer_singularity);
}

TEST(TrajectorySelector, InterpolatesAtConfiguredMaximumStep)
{
  const std::vector<double> from = {0.0, 0.0};
  const std::vector<double> to = {2.1, 0.2};
  EXPECT_EQ(jaka_planner::interpolationSteps(from, to, 1.0), 3);
  EXPECT_THROW(jaka_planner::interpolationSteps(from, {1.0}, 1.0), std::invalid_argument);
}

TEST(TrajectorySelector, EnforcesSingularityThresholds)
{
  jaka_planner::SelectionConfig config;
  EXPECT_TRUE(jaka_planner::passesSingularityLimits(0.01, 200.0, config));
  EXPECT_FALSE(jaka_planner::passesSingularityLimits(0.009, 100.0, config));
  EXPECT_FALSE(jaka_planner::passesSingularityLimits(0.02, 201.0, config));
  EXPECT_FALSE(jaka_planner::passesSingularityLimits(
    std::numeric_limits<double>::quiet_NaN(), 10.0, config));
}

TEST(TrajectorySelector, RequiresEnoughAttemptsForAllCandidates)
{
  jaka_planner::SelectionConfig config;
  config.candidate_count = 10;
  config.max_attempts = 9;
  EXPECT_FALSE(jaka_planner::validSelectionConfig(config));
  config.max_attempts = 30;
  EXPECT_TRUE(jaka_planner::validSelectionConfig(config));
}
