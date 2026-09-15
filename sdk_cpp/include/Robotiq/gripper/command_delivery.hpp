// Copyright (c) 2026 Robotiq, Inc.
//
// Licensed under the BSD-3-Clause license; see LICENSE for details.

#pragma once

namespace Robotiq {
//! \ingroup core_api
//! Fate of a command block, from GripperSync::waitForCommand().
enum class CommandDelivery
{
   Transmitted, //!< the block went out on an exchange the gripper acked
   Superseded, //!< a later setCommand() replaced it before any cycle
               //!< latched it; it never reached the wire
   Timeout, //!< nothing was acked before the timeout — a stalled bus
};
} // namespace Robotiq
