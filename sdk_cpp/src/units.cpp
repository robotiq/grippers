// Copyright (c) 2026 Robotiq, Inc.
//
// Licensed under the BSD-3-Clause license; see LICENSE for details.

#include <Robotiq/gripper/units.hpp>

#include <algorithm>
#include <cmath>

namespace Robotiq::units {

namespace {
uint8_t roundedRegister(double steps)
{
   return static_cast<uint8_t>(std::lround(steps));
}

//! Where a gPO reading sits in the closing travel: 0 at the open end of the
//! band, 1 at the closed end. Nothing when the band is unusable; a reading off
//! either end of it reads as that end.
std::optional<double> closingRatio(uint8_t value, const DeviceProfile& profile)
{
   if(!profile.position().isValid())
   {
      return std::nullopt;
   }
   return std::clamp(profile.position().ratioAt(value), 0.0, 1.0);
}

//! The register \p ratio of the way along the closing travel.
std::optional<uint8_t> registerAtClosingRatio(double ratio, const DeviceProfile& profile)
{
   if(!profile.position().isValid())
   {
      return std::nullopt;
   }
   return roundedRegister(profile.position().valueAt(std::clamp(ratio, 0.0, 1.0)));
}
} // namespace

std::optional<uint8_t> registerFromSpan(double value, const Range<double>& span)
{
   if(!std::isfinite(value) || value < 0.0 || !span.isValid())
   {
      return std::nullopt;
   }
   return roundedRegister(std::clamp(span.ratioAt(value), 0.0, 1.0) * 0xFF);
}

std::optional<uint8_t> registerFromSpan(double value, double minimum, double maximum)
{
   return registerFromSpan(value, Range<double>{minimum, maximum});
}

std::optional<uint8_t> speedToRegister(double speed, const DeviceProfile& profile)
{
   return registerFromSpan(speed, profile.speed());
}

std::optional<uint8_t> openingToRegister(double opening, const DeviceProfile& profile)
{
   const Range<double> span = profile.opening();
   if(!std::isfinite(opening) || !span.isValid())
   {
      return std::nullopt;
   }
   // The opening shrinks as the register grows, so the two ratios are
   // complements of one another.
   return registerAtClosingRatio(1.0 - std::clamp(span.ratioAt(opening), 0.0, 1.0), profile);
}

std::optional<double> openingFromRegister(uint8_t value, const DeviceProfile& profile)
{
   const Range<double> span = profile.opening();
   const std::optional<double> closing = closingRatio(value, profile);
   if(!closing || !span.isValid())
   {
      return std::nullopt;
   }
   return span.valueAt(1.0 - *closing);
}

std::optional<uint8_t> knuckleAngleToRegister(double angle, const DeviceProfile& profile)
{
   const Range<double> sweep{0.0, profile.knuckleAngleRange()};
   if(!std::isfinite(angle) || !sweep.isValid())
   {
      return std::nullopt;
   }
   return registerAtClosingRatio(std::clamp(sweep.ratioAt(angle), 0.0, 1.0), profile);
}

std::optional<double> knuckleAngleFromRegister(uint8_t value, const DeviceProfile& profile)
{
   const Range<double> sweep{0.0, profile.knuckleAngleRange()};
   const std::optional<double> closing = closingRatio(value, profile);
   if(!closing || !sweep.isValid())
   {
      return std::nullopt;
   }
   return sweep.valueAt(*closing);
}

std::optional<uint8_t> effortToRegister(double effort)
{
   return registerFromSpan(effort, {0.0, 1.0});
}

} // namespace Robotiq::units
