// Copyright (c) 2026 Robotiq, Inc.
//
// Licensed under the BSD-3-Clause license; see LICENSE for details.

//! Synchronization example — a control loop running in step with the
//! exchange cycle through a GripperSync object built over the gripper.
//! It closes the gripper from fully open at maximum speed and force, then
//! eases off to the minimum for the last quarter of the stroke: a fast
//! approach and a gentle contact. Deciding when to ease off is what the
//! synchronization is for — the position it keys on must be seen exactly
//! once, on the cycle it arrives.
//! Needs real hardware to show anything: makeFakeGripper()'s fingers
//! teleport, so they never travel through the threshold.
//! The gripper's fingers move (activation sweep, open, close): keep the
//! jaws clear.
//! Usage: exchange_sync <port>

#include <chrono>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>

#include <Robotiq/gripper.hpp>
#include <Robotiq/gripper/stderr_logger.hpp>
#include <Robotiq/gripper/sync.hpp>

using namespace std::chrono_literals;
using Robotiq::ActionRequestBit;
using Robotiq::ActivationResult;
using Robotiq::Gripper;
using Robotiq::GripperCommand;
using Robotiq::GripperStatus;
using Robotiq::GripperSync;
using Robotiq::Logger;
using Robotiq::ObjectDetection;

namespace {
constexpr uint8_t kOpen = 0;
constexpr uint8_t kClosed = 255;
// Start of the last quarter of the stroke, closing. Approximate: gPO's
// fully-closed reading depends on the fingertips — 230, not 255, on the
// bench — so this sits a little past three quarters of the real travel.
constexpr uint8_t kLastQuarter = 191;
// Where the ease-off is asked for. rSP applies to the motion already
// running, but the fingers coast down rather than braking — a steady ~0.1
// counts per cycle per cycle on the bench — so full scale to minimum needs
// the ~80 counts of lead used here. Asked for at kLastQuarter, they would
// still be slowing through the quarter meant to be slow.
constexpr uint8_t kEaseOffFrom = kLastQuarter - 80;
constexpr uint8_t kMaxSpeed = 0xFF;
constexpr uint8_t kMaxForce = 0xFF;
// Minimum speed is a crawl, not a stop. Minimum force is a different kind
// of setting — the final gripping force, capping motor current — and it
// changes nothing about travel speed: the slow finish below is rSP alone.
// It stops the fingers and reports an object once the cap is exceeded.
constexpr uint8_t kMinSpeed = 0x00;
constexpr uint8_t kMinForce = 0x00;
// Generous next to any supported exchange rate, so exceeding it means the
// bus has stopped, not that this loop was early.
constexpr auto kCycleTimeout = 500ms;
constexpr auto kMoveTimeout = 5s;

std::string positionOf(const GripperStatus& status)
{
   return std::to_string(status.position) + "/255";
}

// What the close ran into. An object met before the ease-off point gets
// the full speed and force: keying the ramp on position is the simple
// strategy, not a universal one.
std::string outcomeOf(const GripperStatus& status, bool easedOff)
{
   if(status.gripperStatus.objectDetection() != ObjectDetection::DetectedWhileClosing)
   {
      return "closed at " + positionOf(status);
   }
   return easedOff
           ? "object gripped gently at " + positionOf(status)
           : "object gripped at " + positionOf(status) + " — before the threshold, so it got the full speed and force";
}

// A plain move, waiting only for the end state: nothing here has to see
// each cycle, so the polling helper is the right tool.
bool openFully(Gripper& gripper, GripperCommand& command, Logger& logger)
{
   command.positionRequest = kOpen;
   command.speed = kMaxSpeed;
   command.force = kMaxForce;
   command.action.set(ActionRequestBit::GoTo, true);
   gripper.setCommand(command);

   if(!Robotiq::waitFor([&] { return gripper.getStatus().positionRequestEcho == kOpen; }, 1s))
   {
      logger.log(Logger::Level::Error, "the gripper never echoed the open request");
      return false;
   }
   if(!Robotiq::waitFor([&] { return gripper.getStatus().gripperStatus.objectDetection() != ObjectDetection::Moving; },
                        kMoveTimeout))
   {
      logger.log(Logger::Level::Error, "the open never settled");
      return false;
   }
   return true;
}

// Fast to the threshold, gently to the end: one pass of the loop per
// exchange cycle, so the ease-off goes out on the cycle after the position
// that called for it.
bool closeFastThenGently(Gripper& gripper, GripperCommand& command, Logger& logger)
{
   command.positionRequest = kClosed;
   command.speed = kMaxSpeed;
   command.force = kMaxForce;
   command.action.set(ActionRequestBit::GoTo, true);
   gripper.setCommand(command);

   // The sync object keeps the place, and hands each status out with the wake
   // that named it: every status below is one this loop has not acted on
   // yet, and none is acted on twice.
   //! [sync-loop]
   GripperSync sync(gripper);
   bool easedOff = false;
   bool requestAcknowledged = false;
   const auto deadline = std::chrono::steady_clock::now() + kMoveTimeout;

   while(std::chrono::steady_clock::now() < deadline)
   {
      if(!sync.wait(kCycleTimeout))
      {
         logger.log(Logger::Level::Error, "no exchange completed in time: the link has stalled mid-close");
         return false;
      }
      const GripperStatus& status = sync.getStampedStatus().status;
      if(sync.getSkipped() > 0)
      {
         // Not fatal — that status is still the newest — but a loop that
         // keeps skipping cannot hold the bus rate.
         logger.log(Logger::Level::Debug, "fell behind by " + std::to_string(sync.getSkipped()) + " cycles");
      }
      //! [sync-loop]

      if(!easedOff && status.position >= kEaseOffFrom)
      {
         // Speed and force ride along on every command block the cycle
         // writes, so this reaches the gripper on the next one and the
         // motion picks it up — rGTO and rPR untouched, nothing stopped.
         // (An rGTO edge would apply it at once instead of over a ramp,
         // holding full speed to kLastQuarter: half a second quicker on the
         // bench, at a ~110 ms dead stop.)
         command.speed = kMinSpeed;
         command.force = kMinForce;
         gripper.setCommand(command);
         easedOff = true;
         logger.log(Logger::Level::Info, "at " + positionOf(status) + ": easing off for the last quarter");
         continue;
      }

      // Until the gripper echoes the request, its motion state still
      // describes the move before this one — which has settled.
      requestAcknowledged = requestAcknowledged || status.positionRequestEcho == kClosed;
      if(requestAcknowledged && status.gripperStatus.objectDetection() != ObjectDetection::Moving)
      {
         logger.log(Logger::Level::Info, outcomeOf(status, easedOff));
         return true;
      }
   }
   logger.log(Logger::Level::Error, "the close never settled");
   return false;
}
} // namespace

int main(int argc, char* argv[])
{
   if(argc < 2)
   {
      std::cerr << "Usage: " << argv[0] << " <port>\n";
      return EXIT_FAILURE;
   }

   Robotiq::ConnectionConfig config;
   config.serial.port = argv[1];

   auto logger = std::make_shared<Robotiq::StderrLogger>("example");

   std::unique_ptr<Gripper> gripper;
   try
   {
      gripper = std::make_unique<Gripper>(config, std::make_shared<Robotiq::StderrLogger>("robotiq"));
   }
   catch(const std::exception& ex)
   {
      std::cerr << "Error: " << ex.what() << "\n\n"
                << "Could not open a gripper on '" << argv[1] << "'; move_gripper's error message "
                << "lists what to check.\n";
      return EXIT_FAILURE;
   }

   logger->log(Logger::Level::Info, "Activating...");
   ActivationResult activation = Robotiq::activate(*gripper);
   if(activation == ActivationResult::FaultLatched)
   {
      logger->log(Logger::Level::Warn, "fault latched; recovering (the fingers will move)");
      activation = Robotiq::recoverFromFault(*gripper);
   }
   if(activation != ActivationResult::Activated && activation != ActivationResult::AlreadyActive)
   {
      logger->log(Logger::Level::Error, "activation failed or timed out");
      return EXIT_FAILURE;
   }

   GripperCommand command = GripperCommand::defaults();

   // Open first: "the last quarter of the stroke" only means something
   // from a known starting point.
   logger->log(Logger::Level::Info, "Opening...");
   if(!openFully(*gripper, command, *logger))
   {
      return EXIT_FAILURE;
   }

   logger->log(Logger::Level::Info, "Closing fast, finishing gently...");
   if(!closeFastThenGently(*gripper, command, *logger))
   {
      return EXIT_FAILURE;
   }
   return EXIT_SUCCESS;
}
