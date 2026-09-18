// Copyright (c) 2026 Robotiq, Inc.
//
// Licensed under the BSD-3-Clause license; see LICENSE for details.

#include <Robotiq/gripper.hpp>
#include <Robotiq/gripper/sync.hpp>

#include "gripper_state.hpp"
#include "process_image.hpp"

#include <algorithm>
#include <chrono>

namespace Robotiq {
namespace {
std::chrono::steady_clock::time_point deadlineAfter(std::chrono::milliseconds timeout)
{
   using namespace std::chrono;
   // Capped so that the sum, and a platform's conversion of the deadline to
   // another clock, cannot overflow when a caller passes milliseconds::max().
   constexpr auto longest = duration_cast<milliseconds>(steady_clock::duration::max()) / 2;
   return steady_clock::now() + std::min(timeout, longest);
}
} // namespace

GripperSync::GripperSync(const Gripper& gripper)
   : _image(gripper._impl->image())
   , _stamped(_image->stampedStatus())
{
}

bool GripperSync::wait(std::chrono::milliseconds timeout)
{
   const StampedStatus fresh = _image->sync(_stamped.exchangeCount, deadlineAfter(timeout));
   if(fresh.exchangeCount <= _stamped.exchangeCount)
   {
      _skipped = 0;
      return false;
   }

   _skipped = fresh.exchangeCount - _stamped.exchangeCount - 1;
   _stamped = fresh;
   return true;
}

} // namespace Robotiq
