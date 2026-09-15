// Copyright (c) 2026 Robotiq, Inc.
//
// Licensed under the BSD-3-Clause license; see LICENSE for details.

//! \brief Internal: the link, the exchange thread and the process image
//! behind Gripper. Owns the image lock, so no caller has to know there is
//! one — reaching past it would be a data race waiting to happen.

#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>

#include <Robotiq/gripper/command.hpp>
#include <Robotiq/gripper/connection_state.hpp>
#include <Robotiq/gripper/logger.hpp>
#include <Robotiq/gripper/platform.hpp>
#include <Robotiq/gripper/serial.hpp>
#include <Robotiq/gripper/status.hpp>
#include <Robotiq/detail/throttle.hpp>
#include <Robotiq/gripper/modbus_client.hpp>

namespace Robotiq::detail {

class GripperState
{
public:
   GripperState(std::unique_ptr<Serial> serial,
                uint8_t slaveAddress,
                std::chrono::microseconds exchangePeriod,
                std::shared_ptr<Platform> os,
                std::shared_ptr<Logger> log);

   // RAII: the state can never outlive its exchange thread.
   ~GripperState();

   GripperState(const GripperState&) = delete;
   GripperState& operator=(const GripperState&) = delete;

   // Initialize the command image using the status echoes. Speed and force
   // have no echoes; they are initialized at maximum values, matching the
   // urcap driver behavior.
   void initializeImage();

   void start();
   // Also wakes every waiter, so a cursor's wait() returns as soon as the
   // cycle is gone rather than when its timeout runs out.
   void stop() noexcept;

   // The image's count and status, read under one lock: the status a count
   // names, and nothing newer.
   struct Snapshot
   {
      uint64_t count;
      GripperStatus status;
   };
   [[nodiscard]] Snapshot snapshot() const;

   // Returns the ticket naming this block; see Gripper::setCommand().
   uint64_t setCommand(const GripperCommand& command);
   [[nodiscard]] GripperCommand command() const;
   [[nodiscard]] GripperStatus status() const;

   // Completed exchange cycles: one per successful transaction.
   [[nodiscard]] uint64_t exchangeCount() const;

   // Block until a command at least as new as \p ticket has been acked, the
   // cycle stops or \p deadline elapses, and return the ticket last put on
   // the wire. GripperSync is the only caller.
   [[nodiscard]] uint64_t waitForCommand(uint64_t ticket, std::chrono::steady_clock::time_point deadline) const;

   // Block until the count passes \p count, the cycle stops or \p deadline
   // elapses, and return the snapshot reached. GripperSync is the only caller.
   [[nodiscard]] Snapshot sync(uint64_t count, std::chrono::steady_clock::time_point deadline) const;

   [[nodiscard]] ConnectionState connectionState() const { return _connectionState.load(); }
   [[nodiscard]] Platform& platform() const noexcept { return *_platform; }

private:
   void exchangeOnce();

   std::shared_ptr<Logger> _logger;
   std::shared_ptr<Platform> _platform;
   GripperModbusClient _client;
   detail::Throttle _failureLogThrottle{std::chrono::milliseconds(1000)};
   std::chrono::microseconds _period;

   const std::unique_ptr<Mutex> _imageMutex;
   const std::unique_ptr<ConditionVariable> _imageRefreshed;
   GripperCommand _command{};
   GripperStatus _status{};
   // Guarded by _imageMutex, not an atomic: publishing the status and the
   // count that names it in one critical section is what lets sync() hand
   // a waiter both as one snapshot. It also keeps a 64-bit counter off
   // targets with no native 8-byte atomic (see _consecutiveFailures below).
   uint64_t _exchangeCount = 0;
   // Tickets, under the same lock and for the same reason: which block the
   // wire carried has to be published with the status it answered.
   // _commandSeq names the image's current block, _sentSeq the last one an
   // exchange actually carried. Equal means the image has reached the
   // gripper; apart means a block is still waiting for a cycle to start.
   uint64_t _commandSeq = 0;
   uint64_t _sentSeq = 0;

   std::atomic<ConnectionState> _connectionState{ConnectionState::Connecting};
   std::atomic<bool> _running{false};
   // 32-bit: a 64-bit atomic needs __atomic_*_8 (no native 8-byte atomic on a
   // 32-bit MCU); a failure counter never needs more than 32 bits.
   std::atomic<uint32_t> _consecutiveFailures{0};
   std::unique_ptr<Thread> _exchangeThread;
};

} // namespace Robotiq::detail
