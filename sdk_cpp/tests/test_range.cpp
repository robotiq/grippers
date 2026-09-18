// Copyright (c) 2026 Robotiq, Inc.
//
// Licensed under the BSD-3-Clause license; see LICENSE for details.

#include <gtest/gtest.h>

#include <cstdint>
#include <limits>

#include <Robotiq/gripper/range.hpp>

namespace Robotiq::test {
namespace {
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();
} // namespace

TEST(TestRange, carries_its_limits_and_their_span)
{
   constexpr Range<double> range{0.020, 0.150};
   static_assert(range.span() == 0.130, "a profile has to be a compile-time constant");
   EXPECT_DOUBLE_EQ(range.lower(), 0.020);
   EXPECT_DOUBLE_EQ(range.upper(), 0.150);
}

TEST(TestRange, an_integral_range_still_spans_a_real_distance)
{
   constexpr Range<uint8_t> band{3, 230};
   EXPECT_DOUBLE_EQ(band.span(), 227.0);
   EXPECT_DOUBLE_EQ(band.ratioAt(230), 1.0);
   EXPECT_DOUBLE_EQ(band.valueAt(0.5), 116.5);
}

TEST(TestRange, contains_its_own_limits)
{
   constexpr Range<double> range{0.0, 0.085};
   EXPECT_TRUE(range.contains(0.0));
   EXPECT_TRUE(range.contains(0.085));
   EXPECT_FALSE(range.contains(-1e-9));
   EXPECT_FALSE(range.contains(0.086));
}

TEST(TestRange, clamps_to_the_nearest_limit)
{
   constexpr Range<double> range{0.0, 0.085};
   EXPECT_DOUBLE_EQ(range.clamp(-1.0), 0.0);
   EXPECT_DOUBLE_EQ(range.clamp(1.0), 0.085);
   EXPECT_DOUBLE_EQ(range.clamp(0.040), 0.040);
}

TEST(TestRange, the_ratio_and_the_value_invert_one_another)
{
   constexpr Range<double> range{0.020, 0.150};
   EXPECT_DOUBLE_EQ(range.ratioAt(range.valueAt(0.25)), 0.25);
   EXPECT_DOUBLE_EQ(range.valueAt(range.ratioAt(0.085)), 0.085);
}

TEST(TestRange, neither_the_ratio_nor_the_value_clamps)
{
   // Callers clamp where clamping is right; the conversions want the raw ratio
   // so they can complement it.
   constexpr Range<double> range{0.0, 1.0};
   EXPECT_DOUBLE_EQ(range.ratioAt(2.0), 2.0);
   EXPECT_DOUBLE_EQ(range.valueAt(-1.0), -1.0);
}

TEST(TestRange, rejects_an_interval_the_arithmetic_cannot_use)
{
   EXPECT_TRUE((Range<double>{0.0, 1.0}.isValid()));
   EXPECT_FALSE((Range<double>{1.0, 1.0}.isValid())) << "an empty interval has no ratio";
   EXPECT_FALSE((Range<double>{1.0, 0.0}.isValid())) << "an inverted interval";
}

TEST(TestRange, a_non_finite_limit_leaves_no_usable_span)
{
   EXPECT_FALSE((Range<double>{kNaN, 1.0}.isValid()));
   EXPECT_FALSE((Range<double>{0.0, kNaN}.isValid()));
   EXPECT_FALSE((Range<double>{-kInf, 1.0}.isValid()));
   EXPECT_FALSE((Range<double>{0.0, kInf}.isValid()));
   EXPECT_FALSE((Range<double>{-kInf, kInf}.isValid()));
}
} // namespace Robotiq::test
