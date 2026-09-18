// Copyright (c) 2026 Robotiq, Inc.
//
// Licensed under the BSD-3-Clause license; see LICENSE for details.

#include <gtest/gtest.h>

#include <cstdint>
#include <limits>

#include <Robotiq/gripper/device_profile.hpp>
#include <Robotiq/gripper/units.hpp>

namespace Robotiq::test {

namespace {
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();
const DeviceProfile& k2F85 = profiles::k2F85;
const DeviceProfile& kHandE = profiles::kHandE;
using namespace Robotiq::units;
} // namespace

TEST(TestDeviceProfile, the_2f85_carries_the_manual_ranges_and_the_measured_register_band)
{
   EXPECT_DOUBLE_EQ(k2F85.minSpeed, 0.020);
   EXPECT_DOUBLE_EQ(k2F85.maxSpeed, 0.150);
   EXPECT_DOUBLE_EQ(k2F85.minOpening, 0.0);
   EXPECT_DOUBLE_EQ(k2F85.maxOpening, 0.085);
   EXPECT_EQ(k2F85.openPosition, 3);
   EXPECT_EQ(k2F85.closedPosition, 230);
   EXPECT_DOUBLE_EQ(k2F85.registerPositionRange(), 227.0);
   EXPECT_DOUBLE_EQ(k2F85.knuckleAngleRange(), 0.7929);
}

TEST(TestDeviceProfile, the_hand_e_carries_the_manual_ranges_and_the_measured_register_band)
{
   EXPECT_DOUBLE_EQ(kHandE.minSpeed, 0.020);
   EXPECT_DOUBLE_EQ(kHandE.maxSpeed, 0.150);
   EXPECT_DOUBLE_EQ(kHandE.minOpening, 0.0);
   EXPECT_DOUBLE_EQ(kHandE.maxOpening, 0.050);
   EXPECT_EQ(kHandE.openPosition, 3);
   EXPECT_EQ(kHandE.closedPosition, 250);
   EXPECT_DOUBLE_EQ(kHandE.registerPositionRange(), 247.0);
   EXPECT_DOUBLE_EQ(kHandE.knuckleAngleRange(), 0.0); // a slider: no knuckle
}

TEST(TestDeviceProfile, the_ranges_read_the_profile_s_ends)
{
   EXPECT_DOUBLE_EQ(k2F85.speed().lower(), k2F85.minSpeed);
   EXPECT_DOUBLE_EQ(k2F85.speed().upper(), k2F85.maxSpeed);
   EXPECT_DOUBLE_EQ(k2F85.opening().lower(), k2F85.minOpening);
   EXPECT_DOUBLE_EQ(k2F85.opening().upper(), k2F85.maxOpening);
   EXPECT_EQ(k2F85.position().lower(), k2F85.openPosition);
   EXPECT_EQ(k2F85.position().upper(), k2F85.closedPosition);
}

TEST(TestDeviceProfile, a_profile_written_without_a_knuckle_slope_has_no_angle)
{
   constexpr DeviceProfile custom{0.020, 0.150, 0.0, 0.085, 5, 226};
   EXPECT_DOUBLE_EQ(custom.knuckleAnglePerCount, 0.0);
   EXPECT_FALSE(knuckleAngleToRegister(0.4, custom).has_value());
   EXPECT_EQ(openingToRegister(0.0, custom), 226);
}

TEST(TestSpeedToRegister, the_hand_e_range_spans_the_whole_byte)
{
   EXPECT_EQ(speedToRegister(0.020, kHandE), 0);
   EXPECT_EQ(speedToRegister(0.150, kHandE), 255);
   EXPECT_EQ(speedToRegister(0.3, kHandE), 255);
   EXPECT_FALSE(speedToRegister(-0.1, kHandE).has_value());
}

TEST(TestOpeningToRegister, the_hand_e_stroke_spans_its_register_band)
{
   EXPECT_EQ(openingToRegister(0.050, kHandE), 3);
   EXPECT_EQ(openingToRegister(0.0, kHandE), 250);
   EXPECT_EQ(openingToRegister(0.025, kHandE), 127); // 126.5 steps
   EXPECT_EQ(openingToRegister(0.060, kHandE), 3);
   EXPECT_EQ(openingToRegister(-1e-4, kHandE), 250);
}

TEST(TestOpeningFromRegister, the_hand_e_band_spans_its_stroke)
{
   EXPECT_DOUBLE_EQ(openingFromRegister(3, kHandE).value(), 0.050);
   EXPECT_DOUBLE_EQ(openingFromRegister(250, kHandE).value(), 0.0);
   EXPECT_DOUBLE_EQ(openingFromRegister(0, kHandE).value(), 0.050);
   EXPECT_DOUBLE_EQ(openingFromRegister(255, kHandE).value(), 0.0);
}

TEST(TestRegisterFromSpan, maps_the_span_onto_the_whole_byte)
{
   EXPECT_EQ(registerFromSpan(20.0, {20.0, 235.0}), 0);
   EXPECT_EQ(registerFromSpan(127.5, {20.0, 235.0}), 128); // 127.5 steps
   EXPECT_EQ(registerFromSpan(235.0, {20.0, 235.0}), 255);
}

TEST(TestRegisterFromSpan, floors_below_the_minimum_and_saturates_above_full_scale)
{
   EXPECT_EQ(registerFromSpan(0.0, {20.0, 235.0}), 0);
   EXPECT_EQ(registerFromSpan(10.0, {20.0, 235.0}), 0);
   EXPECT_EQ(registerFromSpan(300.0, {20.0, 235.0}), 255);
}

TEST(TestRegisterFromSpan, rejects_what_has_no_register_value)
{
   EXPECT_FALSE(registerFromSpan(-1.0, {20.0, 235.0}).has_value());
   EXPECT_FALSE(registerFromSpan(kNaN, {20.0, 235.0}).has_value());
   EXPECT_FALSE(registerFromSpan(kInf, {20.0, 235.0}).has_value());
   EXPECT_FALSE(registerFromSpan(50.0, {20.0, 20.0}).has_value());
   EXPECT_FALSE(registerFromSpan(50.0, {235.0, 20.0}).has_value());
   EXPECT_FALSE(registerFromSpan(50.0, {kNaN, 235.0}).has_value());
   EXPECT_FALSE(registerFromSpan(50.0, {20.0, kInf}).has_value());
}

#if defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#elif defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable : 4996)
#endif
TEST(TestRegisterFromSpan, the_deprecated_scale_pair_maps_like_the_range)
{
   for(const double value : {-1.0, 0.0, 20.0, 127.5, 235.0, 300.0, kNaN})
   {
      EXPECT_EQ(registerFromSpan(value, 20.0, 235.0), registerFromSpan(value, {20.0, 235.0})) << value;
   }
}
#if defined(__GNUC__)
#pragma GCC diagnostic pop
#elif defined(_MSC_VER)
#pragma warning(pop)
#endif

TEST(TestSpeedToRegister, the_manual_range_spans_the_whole_byte)
{
   EXPECT_EQ(speedToRegister(0.020, k2F85), 0);
   EXPECT_EQ(speedToRegister(0.085, k2F85), 128); // 127.5 steps
   EXPECT_EQ(speedToRegister(0.150, k2F85), 255);
}

TEST(TestSpeedToRegister, floors_and_saturates_outside_the_range)
{
   EXPECT_EQ(speedToRegister(0.0, k2F85), 0);
   EXPECT_EQ(speedToRegister(0.3, k2F85), 255);
}

TEST(TestSpeedToRegister, rejects_what_has_no_register_value)
{
   EXPECT_FALSE(speedToRegister(-0.1, k2F85).has_value());
   EXPECT_FALSE(speedToRegister(kNaN, k2F85).has_value());

   DeviceProfile broken = k2F85;
   broken.minSpeed = 0.020;
   broken.maxSpeed = 0.020;
   EXPECT_FALSE(speedToRegister(0.1, broken).has_value());
}

TEST(TestOpeningToRegister, the_position_ends_land_on_the_register_band)
{
   EXPECT_EQ(openingToRegister(0.085, k2F85), 3); // fully open
   EXPECT_EQ(openingToRegister(0.0, k2F85), 230); // fully closed
}

TEST(TestOpeningToRegister, rounds_to_the_nearest_value)
{
   EXPECT_EQ(openingToRegister(0.020, k2F85), 177); // 176.59 steps
}

TEST(TestOpeningToRegister, clamps_controller_overshoot_to_the_position_ends)
{
   EXPECT_EQ(openingToRegister(0.085 + 1e-4, k2F85), 3);
   EXPECT_EQ(openingToRegister(-1e-4, k2F85), 230);
   EXPECT_EQ(openingToRegister(0.100, k2F85), 3);
}

TEST(TestOpeningToRegister, supports_a_profile_whose_minimum_position_is_not_zero)
{
   DeviceProfile offset = k2F85;
   offset.minOpening = 0.010;
   offset.maxOpening = 0.095; // e.g. fingers that never fully meet
   EXPECT_EQ(openingToRegister(0.095, offset), 3); // fully open
   EXPECT_EQ(openingToRegister(0.010, offset), 230); // fully closed
   EXPECT_EQ(openingToRegister(0.005, offset), 230); // below minOpening clamps closed
}

TEST(TestOpeningToRegister, rejects_what_has_no_register_value)
{
   EXPECT_FALSE(openingToRegister(kNaN, k2F85).has_value());
   EXPECT_FALSE(openingToRegister(kInf, k2F85).has_value());

   DeviceProfile noSpan = k2F85;
   noSpan.minOpening = 0.0;
   noSpan.maxOpening = 0.0;
   EXPECT_FALSE(openingToRegister(0.02, noSpan).has_value());

   DeviceProfile noBand = k2F85;
   noBand.openPosition = 3;
   noBand.closedPosition = 3;
   EXPECT_FALSE(openingToRegister(0.02, noBand).has_value());
}

TEST(TestOpeningFromRegister, the_register_band_ends_land_on_the_position_span)
{
   EXPECT_DOUBLE_EQ(openingFromRegister(3, k2F85).value(), 0.085);
   EXPECT_DOUBLE_EQ(openingFromRegister(230, k2F85).value(), 0.0);
   EXPECT_NEAR(openingFromRegister(177, k2F85).value(), 0.019846, 1e-6);
}

TEST(TestOpeningFromRegister, values_outside_the_band_read_as_the_position_ends)
{
   EXPECT_DOUBLE_EQ(openingFromRegister(0, k2F85).value(), 0.085);
   EXPECT_DOUBLE_EQ(openingFromRegister(255, k2F85).value(), 0.0);
}

TEST(TestOpeningFromRegister, a_closed_gripper_settling_short_of_closed_register_reads_slightly_open)
{
   EXPECT_NEAR(openingFromRegister(228, k2F85).value(), 0.00075, 1e-5);
}

TEST(TestOpeningFromRegister, supports_a_profile_whose_minimum_position_is_not_zero)
{
   DeviceProfile offset = k2F85;
   offset.minOpening = 0.010;
   offset.maxOpening = 0.095;
   EXPECT_DOUBLE_EQ(openingFromRegister(3, offset).value(), 0.095); // fully open
   EXPECT_DOUBLE_EQ(openingFromRegister(230, offset).value(), 0.010); // fully closed
}

TEST(TestOpeningFromRegister, rejects_a_profile_with_no_register_band)
{
   DeviceProfile broken = k2F85;
   broken.openPosition = 3;
   broken.closedPosition = 3;
   EXPECT_FALSE(openingFromRegister(broken.openPosition, broken).has_value());
   EXPECT_FALSE(openingFromRegister(255, broken).has_value());
}

TEST(TestOpeningFromRegister, inside_the_band_a_position_read_back_commands_the_same_value)
{
   for(int count = k2F85.openPosition; count <= k2F85.closedPosition; ++count)
   {
      const auto raw = static_cast<uint8_t>(count);
      EXPECT_EQ(openingToRegister(openingFromRegister(raw, k2F85).value(), k2F85), raw) << "count " << count;
   }
}

TEST(TestKnuckleAngleToRegister, the_sweep_ends_land_on_the_register_band)
{
   EXPECT_EQ(knuckleAngleToRegister(0.0, k2F85), 3); // fully open
   EXPECT_EQ(knuckleAngleToRegister(0.7929, k2F85), 230); // fully closed
}

TEST(TestKnuckleAngleToRegister, rounds_to_the_nearest_value)
{
   EXPECT_EQ(knuckleAngleToRegister(0.39645, k2F85), 117); // 116.5 steps
}

TEST(TestKnuckleAngleToRegister, clamps_controller_overshoot_to_the_sweep_ends)
{
   EXPECT_EQ(knuckleAngleToRegister(-1e-4, k2F85), 3);
   EXPECT_EQ(knuckleAngleToRegister(0.7929 + 1e-4, k2F85), 230);
   EXPECT_EQ(knuckleAngleToRegister(2.0, k2F85), 230);
}

TEST(TestKnuckleAngleToRegister, rejects_what_has_no_register_value)
{
   EXPECT_FALSE(knuckleAngleToRegister(kNaN, k2F85).has_value());
   EXPECT_FALSE(knuckleAngleToRegister(kInf, k2F85).has_value());

   DeviceProfile noBand = k2F85;
   noBand.openPosition = 3;
   noBand.closedPosition = 3;
   EXPECT_FALSE(knuckleAngleToRegister(0.4, noBand).has_value());
}

TEST(TestKnuckleAngleFromRegister, the_register_band_ends_land_on_the_sweep)
{
   EXPECT_DOUBLE_EQ(knuckleAngleFromRegister(3, k2F85).value(), 0.0);
   EXPECT_DOUBLE_EQ(knuckleAngleFromRegister(230, k2F85).value(), 0.7929);
}

TEST(TestKnuckleAngleFromRegister, values_outside_the_band_read_as_the_sweep_ends)
{
   EXPECT_DOUBLE_EQ(knuckleAngleFromRegister(0, k2F85).value(), 0.0);
   EXPECT_DOUBLE_EQ(knuckleAngleFromRegister(255, k2F85).value(), 0.7929);
}

TEST(TestKnuckleAngleFromRegister, closes_the_opposite_way_round_to_the_opening)
{
   // The knuckle sweeps up as the fingers close, so the two conversions run in
   // opposite directions over the same band.
   EXPECT_GT(knuckleAngleFromRegister(200, k2F85).value(), knuckleAngleFromRegister(100, k2F85).value());
   EXPECT_LT(openingFromRegister(200, k2F85).value(), openingFromRegister(100, k2F85).value());
}

TEST(TestKnuckleAngleFromRegister, inside_the_band_an_angle_read_back_commands_the_same_value)
{
   for(int count = k2F85.openPosition; count <= k2F85.closedPosition; ++count)
   {
      const auto raw = static_cast<uint8_t>(count);
      EXPECT_EQ(knuckleAngleToRegister(knuckleAngleFromRegister(raw, k2F85).value(), k2F85), raw) << "count " << count;
   }
}

TEST(TestKnuckleAngle, the_slope_survives_fingertips_that_meet_early)
{
   // knuckleAnglePerCount belongs to the linkage, so a band that stops early
   // reaches a smaller angle rather than stretching the same sweep over fewer
   // counts.
   DeviceProfile thickTips = k2F85;
   thickTips.openPosition = 3;
   thickTips.closedPosition = 200;
   EXPECT_DOUBLE_EQ(knuckleAngleFromRegister(200, thickTips).value(),
                    k2F85.knuckleAnglePerCount * (200 - k2F85.openPosition));
   EXPECT_NEAR(knuckleAngleFromRegister(200, thickTips).value(), 0.688, 1e-3);
}

TEST(TestKnuckleAngle, a_gripper_with_no_knuckle_has_no_angle_either_way)
{
   // The Hand-E, a slider, ships a zero slope, and both directions then report that
   // they have no answer rather than collapsing every count onto one angle.
   EXPECT_FALSE(knuckleAngleFromRegister(100, kHandE).has_value());
   EXPECT_FALSE(knuckleAngleToRegister(0.0, kHandE).has_value());
}

TEST(TestEffortToRegister, the_unit_span_maps_onto_the_whole_byte)
{
   EXPECT_EQ(effortToRegister(0.0), 0);
   EXPECT_EQ(effortToRegister(0.5), 128); // 127.5 steps
   EXPECT_EQ(effortToRegister(1.0), 255);
}

TEST(TestEffortToRegister, saturates_above_one)
{
   EXPECT_EQ(effortToRegister(1.5), 255);
}

TEST(TestEffortToRegister, rejects_what_has_no_register_value)
{
   EXPECT_FALSE(effortToRegister(-0.1).has_value());
   EXPECT_FALSE(effortToRegister(kNaN).has_value());
   EXPECT_FALSE(effortToRegister(kInf).has_value());
}

TEST(TestMotorCurrentFromRegister, ten_milliamperes_per_step)
{
   EXPECT_DOUBLE_EQ(motorCurrentFromRegister(0), 0.0);
   EXPECT_DOUBLE_EQ(motorCurrentFromRegister(20), 0.2);
}
} // namespace Robotiq::test
