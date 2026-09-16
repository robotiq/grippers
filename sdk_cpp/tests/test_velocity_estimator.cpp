// Copyright (c) 2026 Robotiq, Inc.
//
// Licensed under the BSD-3-Clause license; see LICENSE for details.

#include <gtest/gtest.h>

#include <chrono>

#include <Robotiq/gripper/device_profile.hpp>
#include <Robotiq/gripper/velocity_estimator.hpp>

namespace Robotiq::test {

namespace {
using namespace std::chrono_literals;

constexpr std::chrono::nanoseconds kTimeConstant = 100ms;

// One gPO count of opening on a 2F-85.
constexpr double kCount = profiles::k2F85.openingRange() / profiles::k2F85.registerPositionRange();
constexpr double kSlowest = profiles::k2F85.minSpeed;

using Seconds = std::chrono::duration<double>;

// Positions advancing at velocity from zero, period apart, for samples periods.
double ramp(VelocityEstimator& estimator, double velocity, std::chrono::nanoseconds period, int samples)
{
   double result = 0.0;
   for(int sample = 0; sample <= samples; ++sample)
   {
      const std::chrono::nanoseconds now = sample * period;
      result = estimator.update(velocity * Seconds(now).count(), now);
   }
   return result;
}
} // namespace

TEST(TestVelocityEstimator, the_first_sample_only_establishes_a_reference)
{
   VelocityEstimator estimator(kTimeConstant);
   EXPECT_DOUBLE_EQ(estimator.update(0.3, 1s), 0.0);
   EXPECT_DOUBLE_EQ(estimator.velocity(), 0.0);
}

TEST(TestVelocityEstimator, a_constant_rate_settles_on_that_rate)
{
   VelocityEstimator estimator(kTimeConstant);
   EXPECT_NEAR(ramp(estimator, kSlowest, 10ms, 50), kSlowest, 0.001);
}

TEST(TestVelocityEstimator, the_same_motion_reads_the_same_whatever_the_sampling_rate)
{
   VelocityEstimator slow(kTimeConstant);
   VelocityEstimator fast(kTimeConstant);
   EXPECT_NEAR(ramp(slow, 0.050, 10ms, 50), ramp(fast, 0.050, 2ms, 250), 0.001);
}

TEST(TestVelocityEstimator, standing_still_decays_to_nothing)
{
   VelocityEstimator estimator(kTimeConstant);
   constexpr std::chrono::nanoseconds kMoved = 500ms;
   ASSERT_NEAR(ramp(estimator, 0.050, 10ms, 50), 0.050, 0.002);

   const double stopped = 0.050 * Seconds(kMoved).count();
   double velocity = 0.0;
   for(int sample = 1; sample <= 50; ++sample)
   {
      velocity = estimator.update(stopped, kMoved + sample * 10ms);
   }
   EXPECT_NEAR(velocity, 0.0, 0.001);
}

TEST(TestVelocityEstimator, one_count_at_rest_reads_as_that_count_over_the_time_constant)
{
   VelocityEstimator estimator(kTimeConstant);
   for(int sample = 0; sample < 20; ++sample)
   {
      estimator.update(0.040, sample * 10ms);
   }
   ASSERT_DOUBLE_EQ(estimator.velocity(), 0.0);

   const double bump = estimator.update(0.040 + kCount, 200ms);
   EXPECT_GT(bump, 0.0);
   EXPECT_LT(bump, kCount / Seconds(kTimeConstant).count());
   EXPECT_LT(bump, kSlowest);
}

TEST(TestVelocityEstimator, a_quantized_ramp_reads_as_the_rate_it_averages)
{
   // Byte-quantized position sampled at 500 Hz: most samples repeat, about
   // every fourth jumps a count.
   VelocityEstimator estimator(kTimeConstant);
   constexpr double kRate = 0.050;
   double velocity = 0.0;
   for(int sample = 0; sample <= 800; ++sample)
   {
      const std::chrono::nanoseconds now = sample * 2ms;
      const double exact = kRate * Seconds(now).count();
      velocity = estimator.update(kCount * static_cast<int>(exact / kCount), now);
   }
   EXPECT_NEAR(velocity, kRate, 0.005);
}

TEST(TestVelocityEstimator, two_samples_the_clock_cannot_separate_carry_no_rate)
{
   VelocityEstimator estimator(kTimeConstant);
   estimator.update(0.0, 100ms);
   const double moved = estimator.update(0.001, 110ms);
   ASSERT_GT(moved, 0.0);

   EXPECT_DOUBLE_EQ(estimator.update(0.002, 110ms), moved);
   EXPECT_DOUBLE_EQ(estimator.update(0.003, 90ms), moved);
   // Neither refused sample became the reference.
   EXPECT_GT(estimator.update(0.002, 120ms), moved);
}

TEST(TestVelocityEstimator, a_longer_time_constant_smooths_the_same_count_further)
{
   VelocityEstimator brief(10ms);
   VelocityEstimator patient(1s);
   brief.update(0.0, 0ms);
   patient.update(0.0, 0ms);
   EXPECT_GT(brief.update(kCount, 10ms), 10 * patient.update(kCount, 10ms));
}

TEST(TestVelocityEstimator, reset_forgets_the_gap_the_gripper_was_away_for)
{
   VelocityEstimator estimator(kTimeConstant);
   ramp(estimator, 0.050, 10ms, 50);
   ASSERT_GT(estimator.velocity(), 0.0);

   estimator.reset();
   EXPECT_DOUBLE_EQ(estimator.velocity(), 0.0);
   EXPECT_DOUBLE_EQ(estimator.update(profiles::k2F85.openingRange(), 1min), 0.0);
}

} // namespace Robotiq::test
