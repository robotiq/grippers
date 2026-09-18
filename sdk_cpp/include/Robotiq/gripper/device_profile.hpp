// Copyright (c) 2026 Robotiq, Inc.
//
// Licensed under the BSD-3-Clause license; see LICENSE for details.

#pragma once

#include <cstdint>

#include <Robotiq/gripper/range.hpp>

namespace Robotiq {

//! \ingroup units
//! \brief Gripper model specific profile gathering specifications such as
//!        opening range and speed.
//!
//! openPosition and closedPosition are measured, not taken from the
//! manual: with nothing between the fingers, command rPR 0 and read gPO
//! back, then command rPR 255 and read gPO back. Start from the shipped
//! profile closest to your gripper and override only what you measured —
//! the speed range and knuckleAnglePerCount are Robotiq's to supply, and a
//! profile written from scratch has nowhere to get them.
//!
//! \warning Position <-> register conversion are valid only while
//!          the gripper stays in parallel-finger mode. In encompassing
//!          mode gPO can go past closedPosition; this profile has no way
//!          to represent that, so a reading taken in encompassing mode
//!          does not convert to a meaningful opening.
//!
//! \warning minOpening, maxOpening, openPosition and closedPosition
//!          depend on which fingers are installed on the gripper:
//!          swapping fingers changes the physical opening range, and
//!          can shift the measured register endpoints too.
//!          knuckleAnglePerCount does not — it belongs to the linkage, so
//!          the knuckle simply stops earlier when the fingers meet earlier.
struct DeviceProfile
{
   double minSpeed; //!< m/s, from the manual
   double maxSpeed; //!< m/s, from the manual
   double minOpening; //!< m — opening at closedPosition
   double maxOpening; //!< m — opening at openPosition
   uint8_t openPosition; //!< gPO after commanding rPR 0, measured
   uint8_t closedPosition; //!< gPO after commanding rPR 255, measured

   //! rad/count — how far the knuckle the fingers hang off rotates per
   //! register step. A constant of the linkage, so it ships with the profile
   //! and is never the integrator's to measure. Zero on a gripper that has
   //! no knuckle, and in a profile that leaves it out, which is how the angle
   //! conversions report that they have no answer instead of inventing one.
   double knuckleAnglePerCount = 0.0;

   //! [minSpeed, maxSpeed], in m/s.
   [[nodiscard]] constexpr Range<double> speed() const { return {minSpeed, maxSpeed}; }

   //! [minOpening, maxOpening], in m; lower() is the opening at the closed end.
   [[nodiscard]] constexpr Range<double> opening() const { return {minOpening, maxOpening}; }

   //! [openPosition, closedPosition], the measured register band.
   [[nodiscard]] constexpr Range<uint8_t> position() const { return {openPosition, closedPosition}; }

   //! Register steps from full opening to full closure, in parallel mode.
   [[nodiscard]] constexpr double registerPositionRange() const { return position().span(); }

   //! Metres from full closure to full opening.
   [[nodiscard]] constexpr double openingRange() const { return opening().span(); }

   //! Radians the knuckle sweeps over the whole register band.
   [[nodiscard]] constexpr double knuckleAngleRange() const { return knuckleAnglePerCount * registerPositionRange(); }
};

namespace profiles {
//! \ingroup units
//! \brief The measured profile for the 2F-85
//!
//! \warning Assumes that the gripper is in parallel-finger mode.
//! \warning knuckleAnglePerCount is derived from the value the ROS
//!          descriptions have always used — 0.7929 rad over 227 counts —
//!          not one Robotiq measured; confirm it on the bench before
//!          relying on it.
inline constexpr DeviceProfile k2F85{0.020, 0.150, 0.0, 0.085, 3, 230, 0.7929 / 227.0};

//! \ingroup units
//! \brief The measured profile for the Hand-E
//!
//! \warning Assumes that the gripper is in parallel-finger mode.
inline constexpr DeviceProfile kHandE{0.020, 0.150, 0.0, 0.050, 3, 250, 0.0};

} // namespace profiles

} // namespace Robotiq
