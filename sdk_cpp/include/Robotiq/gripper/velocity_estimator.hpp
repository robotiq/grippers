// Copyright (c) 2026 Robotiq, Inc.
//
// Licensed under the BSD-3-Clause license; see LICENSE for details.

#pragma once

#include <chrono>

namespace Robotiq {

//! \ingroup status
//! \brief Time constant of the estimate the exchange cycle publishes in
//!        StampedExchange::velocity.
//!
//! The trade is ripple against lag: each gPO count flip bumps the estimate
//! by about one count per constant, and the estimate trails the fingers by
//! about three quarters of one. Chosen by replaying a 2F-85 bench recording
//! (slowest and fastest moves) through 20 to 300 ms: ripple on slow travel
//! falls steeply until about 70 ms and slowly after, so 80 ms is where an
//! equal weight of ripple and lag bottoms out, at 8 % ripple and 60 ms lag.
inline constexpr std::chrono::milliseconds kVelocityTimeConstant{80};

//! \ingroup status
//! \brief First-order low-pass filter over the difference between
//!        successive position samples.
//!
//! The exchange cycle runs one over gPO and publishes the result with each
//! record; this class is public for a caller that filters another quantity.
//! Velocity comes back in the unit of the positions fed in, per second. It
//! estimates speed, not state: whether the fingers stopped, and why, is
//! gOBJ's report.
//!
//! It is the filtered derivative V(s)/X(s) = s/(1 + tau*s), discretized
//! with a backward difference over the sample interval dt:
//! \verbatim
//!   v[k] = (tau * v[k-1] + (x[k] - x[k-1])) / (tau + dt)
//! \endverbatim
//! dt never divides anything, so sampling faster than the position changes
//! cannot turn one count into a spike, and tau/(tau + dt) stays in [0, 1)
//! for every dt > 0, so the recursion is stable at any rate, regular or
//! not. A gap between samples reads as the average rate over the gap.
//! One step of one count bumps the estimate by about count/tau; after the
//! position stops, the estimate decays by a factor of e per tau.
//!
//! References: Åström and Murray, <em>Feedback Systems</em>, 2nd ed.,
//! section 11.5, equation (11.18); Eckner, "Algorithms for Unevenly
//! Spaced Time Series", 2019, section 4.
class VelocityEstimator
{
public:
   //! \param timeConstant Filter time constant, strictly positive.
   explicit VelocityEstimator(std::chrono::nanoseconds timeConstant)
      : _timeConstant(timeConstant)
   {
   }

   //! \param position Finite, in the unit the estimate is wanted in.
   //! \param timestamp When \p position was read.
   //! \return The filtered velocity. Zero for the first sample, which only
   //!         establishes the reference; unchanged for a timestamp no later
   //!         than the previous one.
   double update(double position, std::chrono::steady_clock::time_point timestamp)
   {
      const Seconds elapsed = timestamp - _timestamp;
      if(_seeded && elapsed.count() <= 0)
      {
         return _velocity;
      }
      if(_seeded)
      {
         _velocity += (position - _position - _velocity * elapsed.count()) / (elapsed + _timeConstant).count();
      }
      _seeded = true;
      _position = position;
      _timestamp = timestamp;
      return _velocity;
   }

   //! \return The estimate the last sample produced.
   [[nodiscard]] double velocity() const noexcept { return _velocity; }

private:
   using Seconds = std::chrono::duration<double>;

   Seconds _timeConstant;
   std::chrono::steady_clock::time_point _timestamp{};
   double _position = 0.0;
   double _velocity = 0.0;
   bool _seeded = false;
};

} // namespace Robotiq
