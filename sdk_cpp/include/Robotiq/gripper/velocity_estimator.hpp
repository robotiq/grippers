// Copyright (c) 2026 Robotiq, Inc.
//
// Licensed under the BSD-3-Clause license; see LICENSE for details.

//! \brief Finger velocity from successive position readings. The status
//!        block carries position and motor current only, so a caller that
//!        needs a rate has to derive one, and the raw difference between
//!        two readings is unusable: gPO is a byte over the whole stroke,
//!        so at any exchange rate worth running most cycles report no
//!        change at all and the occasional count flip reads as a large
//!        spike. This filters that difference into a signal that settles
//!        on the real rate and cannot be crossed by a single count.

#pragma once

#include <chrono>

namespace Robotiq {

//! \ingroup status
//! \brief First-order low-pass filter over the difference between
//!        successive position samples.
//!
//! Unit-agnostic: velocity comes back in the unit of the positions fed in,
//! per second. Feed register counts and get counts per second; feed the
//! metres of \ref Robotiq::units::openingFromRegister and get metres per
//! second.
//!
//! The caller supplies the clock, so the estimate is reproducible in a test
//! and the class stays freestanding. Any monotonic source will do: only the
//! difference between consecutive timestamps is used.
//!
//! Sampling faster than the gripper exchanges is fine and needs no special
//! handling. Repeat samples carry no position change and simply pull the
//! estimate toward zero for their share of the elapsed time, which is what
//! they mean.
//!
//! Sizing the time constant is a trade between two quantities:
//!   - a position step of one count bumps the estimate by
//!     (count / timeConstant) and decays from there, so the constant sets
//!     how large a single spurious count reads;
//!   - after the fingers stop, the estimate falls by roughly a factor of
//!     three per time constant.
//! A constant of 100 ms on a 2F-85, whose count is about 0.37 mm, gives a
//! 3.7 mm/s bump per count against a 20 mm/s slowest commanded speed.
//!
//! \snippet snippets.cpp velocity-estimate
class VelocityEstimator
{
public:
   //! \param timeConstant Filter time constant, strictly positive.
   explicit VelocityEstimator(std::chrono::nanoseconds timeConstant)
      : _timeConstant(timeConstant)
   {
   }

   //! Take one position sample and return the estimate it produces.
   //! \param position Finite, in the unit the estimate is wanted in.
   //! \param timestamp Reading of a monotonic clock at that sample.
   //! \return The filtered velocity, per second. Zero for the first sample,
   //!         which establishes the reference the next one differs from, and
   //!         unchanged for a timestamp no later than the previous one: two
   //!         readings the clock cannot separate carry no rate.
   double update(double position, std::chrono::nanoseconds timestamp)
   {
      const std::chrono::nanoseconds elapsed = timestamp - _timestamp;
      if(_seeded && elapsed.count() <= 0)
      {
         return _velocity;
      }
      if(_seeded)
      {
         const double seconds = toSeconds(elapsed);
         // The filtered form of (position - _position) / seconds, rearranged
         // so the sample interval never divides anything: a caller sampling
         // far faster than the gripper moves would otherwise turn one count
         // into an arbitrarily large spike before the filter ever saw it.
         _velocity += (position - _position - _velocity * seconds) / (seconds + toSeconds(_timeConstant));
      }
      _seeded = true;
      _position = position;
      _timestamp = timestamp;
      return _velocity;
   }

   //! \return The estimate the last sample produced, without taking a new one.
   [[nodiscard]] double velocity() const noexcept { return _velocity; }

   //! Forget every sample taken so far. The next one starts over at zero, as
   //! the first one did. For a gripper that has been away — reactivated,
   //! reconnected — where the position either side of the gap is a rate that
   //! never happened.
   void reset() noexcept
   {
      _seeded = false;
      _velocity = 0.0;
      _position = 0.0;
      _timestamp = std::chrono::nanoseconds::zero();
   }

private:
   [[nodiscard]] static constexpr double toSeconds(std::chrono::nanoseconds duration)
   {
      return static_cast<double>(duration.count()) * 1e-9;
   }

   std::chrono::nanoseconds _timeConstant;
   std::chrono::nanoseconds _timestamp{};
   double _position = 0.0;
   double _velocity = 0.0;
   bool _seeded = false;
};

} // namespace Robotiq
