// Copyright (c) 2026 Robotiq, Inc.
//
// Licensed under the BSD-3-Clause license; see LICENSE for details.

//! \brief Conversions between SI quantities and the 0..255 register values of the
//!        command and status blocks, scaled by a DeviceProfile. Every
//!        mapping here is linear and the gripper is not: the manual
//!        gives the 2F speed scale as approximate, and a register value is a
//!        setpoint, not a measurement.
//!
//!        Speed maps [minSpeed, maxSpeed] onto register values 0..255,
//!        because value 0 is the gripper's minimum, not zero. A value
//!        below minSpeed floors at register 0 and a value above maxSpeed
//!        saturates at 255. NaN, infinity and a negative value yield
//!        nothing: no benign caller produces them.
//!
//!        Opening maps [minOpening, maxOpening] onto the profile's
//!        register band and clamps to those ends rather than rejecting,
//!        because position setpoints from interpolating controllers
//!        overshoot the ends by a hair and holding the previous command
//!        there would be the wrong outcome. Values round to nearest, so
//!        inside the band a position read back commands the same value.
//!        A closed gripper settles a few steps either side of
//!        closedPosition, so a reading there converts to an opening a
//!        touch off minOpening; values past the band read as the
//!        nearest end.
//!
//!        A profile the arithmetic cannot use (a non-positive position
//!        span or register band, or maxSpeed at or below minSpeed)
//!        yields nothing.
//!
//!        \warning The opening conversions (openingToRegister,
//!        openingFromRegister) are only valid while the gripper stays in
//!        parallel-finger mode. In encompassing mode gPO can exceed
//!        closedPosition; openingFromRegister has no way to detect that
//!        and clamps the reading to the nearest end instead, which does
//!        not reflect the real opening.

#pragma once

#include <cstdint>
#include <optional>

#include <Robotiq/gripper/device_profile.hpp>

namespace Robotiq::units {

//! \cond DOXYGEN_EXCLUDE
// The manual's approximate current equivalent for the 2F gCU byte.
inline constexpr double kAmperesPerRegisterStep = 0.010;
//! \endcond

//! \ingroup units
//! Linear map of [minimum, fullScale] onto register values 0..255, the rule
//! speedToRegister applies. Public for callers that carry
//! their own scales instead of a DeviceProfile.
//! \param value The quantity to convert. A value below \p minimum floors
//!        at register 0; a value above \p fullScale saturates at 255.
//! \param minimum The value that maps to register 0.
//! \param fullScale The value that maps to register 255.
//! \return The mapped register, or std::nullopt if \p value is
//!         non-finite or negative, \p minimum or \p fullScale is
//!         non-finite, or \p fullScale is at or below \p minimum.
[[nodiscard]] std::optional<uint8_t> registerFromSpan(double value, double minimum, double fullScale);

//! \ingroup units
//! Speed in m/s -> rSP.
//! \param metresPerSecond Requested speed. A value below profile.minSpeed
//!        floors at register 0; a value above profile.maxSpeed
//!        saturates at 255.
//! \param profile The device profile to scale against.
//! \return rSP, or std::nullopt if \p metresPerSecond is non-finite or
//!         negative, or \p profile is one the arithmetic can't use
//!         (maxSpeed at or below minSpeed).
//! \snippet snippets.cpp si-unit-conversion
[[nodiscard]] std::optional<uint8_t> speedToRegister(double metresPerSecond, const DeviceProfile& profile);

//! \ingroup units
//! Opening in m -> rPR.
//! \warning Valid only in parallel-finger mode; see the file-level note above.
//! \param openingMetres Target opening. Clamped to
//!        [profile.minOpening, profile.maxOpening] rather than
//!        rejected — a value outside that range reads as the nearest
//!        end. Values round to the nearest register.
//! \param profile The device profile to scale against.
//! \return rPR, or std::nullopt if \p openingMetres is non-finite, or
//!         \p profile is one the arithmetic can't use (non-positive
//!         position span or register band).
[[nodiscard]] std::optional<uint8_t> openingToRegister(double openingMetres, const DeviceProfile& profile);

//! \ingroup units
//! gPO -> opening in m.
//! \warning Valid only in parallel-finger mode; see the file-level note
//!          above. A gPO reading taken in encompassing mode can exceed
//!          profile.closedPosition and is clamped to the nearest end
//!          below instead of reflecting the real opening.
//! \param value The raw register value (gPO). A value outside the
//!        profile's register band reads as the nearest end.
//! \param profile The device profile to scale against.
//! \return The opening in metres, or std::nullopt if \p profile is one
//!         the arithmetic can't use (non-positive position span or
//!         register band).
[[nodiscard]] std::optional<double> openingFromRegister(uint8_t value, const DeviceProfile& profile);

//! \ingroup units
//! Effort in [0, 1] -> rFR. 0 is the gripper's minimum force, 1 its
//! maximum; a value above 1 saturates at register 255.
//! \param effort Requested gripping effort, as a fraction of the
//!        gripper's force range.
//! \return rFR, or std::nullopt if \p effort is non-finite or negative.
[[nodiscard]] std::optional<uint8_t> effortToRegister(double effort);

//! \ingroup units
//! gCU -> motor current in A.
//! \param value The raw register value (gCU).
//! \return The motor current in amperes.
[[nodiscard]] inline constexpr double motorCurrentFromRegister(uint8_t value)
{
   return kAmperesPerRegisterStep * value;
}

} // namespace Robotiq::units
