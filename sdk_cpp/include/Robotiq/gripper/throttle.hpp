// Copyright (c) 2026 Robotiq, Inc.
//
// Licensed under the BSD-3-Clause license; see LICENSE for details.

#pragma once

#include <chrono>
#include <utility>

namespace Robotiq {

//! \ingroup utilities
//! \brief Rate limiter for a single call site (e.g. a periodic log statement).
//!
//! Owned by the call site, so independent call sites throttle
//! independently. Not thread-safe: one instance, one thread.
//!
//! \par Example
//! \snippet snippets.cpp throttle-usage
class Throttle
{
public:
   //! \param period Minimum time between two accepted calls.
   explicit Throttle(std::chrono::milliseconds period)
      : _period(period)
   {
   }

   //! \brief Run \p callable at most once per period.
   //!
   //! Suppressed calls are dropped.
   //! \param callable A callable taking no arguments.
   template <typename Callable>
   void executeIfAllowed(Callable&& callable)
   {
      executeIfAllowed(std::chrono::steady_clock::now(), std::forward<Callable>(callable));
   }

   //! \brief Deterministic variant for callers that already hold a timestamp.
   //! \param now The current time, as the caller sees it.
   //! \param callable A callable taking no arguments.
   template <typename Callable>
   void executeIfAllowed(std::chrono::steady_clock::time_point now, Callable&& callable)
   {
      if(now - _last < _period)
      {
         return;
      }
      _last = now;
      callable();
   }

private:
   std::chrono::milliseconds _period;
   std::chrono::steady_clock::time_point _last{};
};
} // namespace Robotiq
