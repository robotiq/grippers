// Copyright (c) 2026 Robotiq, Inc.
//
// Licensed under the BSD-3-Clause license; see LICENSE for details.

#pragma once

#include <chrono>

namespace Robotiq {

//! \ingroup status
//! \brief First-order low-pass filter over the difference between
//!        successive position samples.
//!
//! Velocity comes back in the unit of the positions fed in, per second.
//! The caller supplies the clock; only the difference between consecutive
//! timestamps is used, so any monotonic source will do.
//!
//! The time constant trades two things: a one-count position step bumps the
//! estimate by about count/timeConstant, and after the fingers stop the
//! estimate decays by a factor of e per time constant.
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

   //! \param position Finite, in the unit the estimate is wanted in.
   //! \param timestamp Reading of a monotonic clock at that sample.
   //! \return The filtered velocity. Zero for the first sample, which only
   //!         establishes the reference; unchanged for a timestamp no later
   //!         than the previous one.
   double update(double position, std::chrono::nanoseconds timestamp)
   {
      const Seconds elapsed = timestamp - _timestamp;
      if(_seeded && elapsed.count() <= 0)
      {
         return _velocity;
      }
      if(_seeded)
      {
         // Rearranged so the sample interval never divides anything: a caller
         // sampling far faster than the gripper moves cannot turn one count
         // into a spike.
         _velocity += (position - _position - _velocity * elapsed.count()) / (elapsed + _timeConstant).count();
      }
      _seeded = true;
      _position = position;
      _timestamp = timestamp;
      return _velocity;
   }

   //! \return The estimate the last sample produced.
   [[nodiscard]] double velocity() const noexcept { return _velocity; }

   //! Forget every sample, so a gap (reactivation, reconnection) does not
   //! read as travel.
   void reset() noexcept
   {
      _seeded = false;
      _velocity = 0.0;
      _position = 0.0;
      _timestamp = std::chrono::nanoseconds::zero();
   }

private:
   using Seconds = std::chrono::duration<double>;

   Seconds _timeConstant;
   std::chrono::nanoseconds _timestamp{};
   double _position = 0.0;
   double _velocity = 0.0;
   bool _seeded = false;
};

} // namespace Robotiq
