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
} // namespace

std::optional<uint8_t> registerFromSpan(double value, double minimum, double fullScale)
{
   if(!std::isfinite(value) || value < 0.0 || !std::isfinite(minimum) || !std::isfinite(fullScale)
      || fullScale <= minimum)
   {
      return std::nullopt;
   }
   const double fraction = std::clamp((value - minimum) / (fullScale - minimum), 0.0, 1.0);
   return roundedRegister(fraction * 0xFF);
}

std::optional<uint8_t> speedToRegister(double metresPerSecond, const DeviceProfile& profile)
{
   return registerFromSpan(metresPerSecond, profile.minSpeed, profile.maxSpeed);
}

std::optional<uint8_t> openingToRegister(double openingMetres, const DeviceProfile& profile)
{
   const double span = profile.maxOpening - profile.minOpening;
   if(!std::isfinite(openingMetres) || !std::isfinite(span) || span <= 0.0 || profile.registerBand() <= 0.0)
   {
      return std::nullopt;
   }
   const double openFraction = std::clamp((openingMetres - profile.minOpening) / span, 0.0, 1.0);
   return roundedRegister(profile.openPosition + (1.0 - openFraction) * profile.registerBand());
}

std::optional<double> openingFromRegister(uint8_t value, const DeviceProfile& profile)
{
   const double band = profile.registerBand();
   const double span = profile.maxOpening - profile.minOpening;
   if(band <= 0.0 || !std::isfinite(span) || span <= 0.0)
   {
      return std::nullopt;
   }
   const double closedFraction = std::clamp((value - profile.openPosition) / band, 0.0, 1.0);
   return profile.minOpening + (1.0 - closedFraction) * span;
}

std::optional<uint8_t> effortToRegister(double effort)
{
   return registerFromSpan(effort, 0.0, 1.0);
}

} // namespace Robotiq::units
