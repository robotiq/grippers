// Copyright (c) 2026 Robotiq, Inc.
//
// Licensed under the BSD-3-Clause license; see LICENSE for details.

#pragma once

#include <chrono>
#include <cstdint>
#include <memory>

#include <Robotiq/gripper/status.hpp>

namespace Robotiq {
class Gripper;

namespace detail {
class ProcessImage;
} // namespace detail

//! \ingroup core_api
//! \brief An object that allows a user to synchronize with the communication
//! exchange cycle.
class GripperSync
{
public:
   //! \brief Synchronize with \p gripper's exchange cycle.
   //! \param gripper The gripper to follow.
   explicit GripperSync(const Gripper& gripper);

   //! \brief Block until the next exchange cycle.
   //! \param timeout How long to wait for the next exchange cycle.
   //! \return true when a fresh status landed, now in getStampedStatus();
   //!         false when \p timeout elapsed with nothing arriving — a
   //!         stalled bus — or the Gripper is gone. A status published just
   //!         before the Gripper's destruction may still be delivered by one
   //!         more wait(); every wait() after it returns false at once.
   [[nodiscard]] bool wait(std::chrono::milliseconds timeout);

   //! \return The timestamped status the last successful wait() woke on;
   //!         the image at construction before any.
   [[nodiscard]] const StampedStatus& getStampedStatus() const noexcept { return _stamped; }

   //! \return Cycles that landed between the two most recent successful
   //!         wait() calls; 0 after a wait() that returned false.
   [[nodiscard]] uint64_t getSkipped() const noexcept { return _skipped; }

private:
   std::shared_ptr<const detail::ProcessImage> _image;
   StampedStatus _stamped;
   uint64_t _skipped = 0;
};

} // namespace Robotiq
