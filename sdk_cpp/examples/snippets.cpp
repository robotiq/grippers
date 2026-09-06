// Copyright (c) 2026 Robotiq, Inc.
//
// Licensed under the BSD-3-Clause license; see LICENSE for details.

// Doc-only source: every //! [tag] region below backs a \snippet reference
// from a header's \code{.cpp} example (see EXAMPLE_PATH in ../Doxyfile).
// Nothing here is called from main() below — main() only exists so this
// compiles to a program; being compiled by GRIPPERS_BUILD_EXAMPLES, and thus
// checked by every normal build, is the whole point. Concepts already
// demonstrated by quick_start.cpp/move_gripper.cpp are tagged there instead;
// this file is only for the ones with no other home.

#include <Robotiq/gripper.hpp>
#include <Robotiq/gripper/fake/gripper_factory.hpp>
#include <Robotiq/gripper/stderr_logger.hpp>

#include <chrono>
#include <cstdint>
#include <memory>
#include <string_view>

using namespace std::chrono_literals;

// Every function and local variable below exists only for its //! [tag]
// region to be extracted as a doc example (see the file header comment) —
// none of it is ever called or read, by design.
#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic ignored "-Wunused-function"
#pragma GCC diagnostic ignored "-Wunused-variable"
#endif

namespace {

void configureConnection()
{
   //! [configure-connection]
   Robotiq::ConnectionConfig config;
   config.modbusSlaveAddress = 0x09;
   config.serial.port = "/dev/ttyUSB0";
   config.serial.baudrate = 115200;
   config.connectionFrequency = 100.0; // Hz
   //! [configure-connection]
}

void commandActionBits()
{
   //! [command-action-bits]
   // Command object use to interact with gripper holding registers
   Robotiq::GripperCommand command = Robotiq::GripperCommand::defaults();

   // Command building blocks
   // ACTION - Activate - rACT
   command.action.set(Robotiq::ActionRequestBit::Activate, true);
   // ACTION - Goto - rGTO
   command.action.set(Robotiq::ActionRequestBit::GoTo, true);
   // ACTION - Auto Release - rATR
   command.action.set(Robotiq::ActionRequestBit::AutoRelease, false);
   // ACTION - Auto Release Open Direction - rARD
   command.action.set(Robotiq::ActionRequestBit::AutoReleaseOpenDirection, true);
   // POSITION REQUEST - Position Request - rPR
   command.positionRequest = 100;
   // SPEED - Speed - rSP
   command.speed = 255;
   // FORCE - Force - rFR
   command.force = 255;
   //! [command-action-bits]
}

void statusGripperStatusFields(Robotiq::Gripper& gripper)
{
   //! [status-gripper-status-fields]
   // Status object retrieved from gripper input registers
   Robotiq::GripperStatus status = gripper.getStatus();

   // Status building blocks
   // GRIPPER STATUS - Object detection - gOBJ
   Robotiq::ObjectDetection gOBJ = status.gripperStatus.objectDetection();
   // GRIPPER STATUS - Activation State - gSTA
   Robotiq::ActivationState gSTA = status.gripperStatus.activationState();
   // GRIPPER STATUS - Goto Enabled - gGTO
   bool gGTO = status.gripperStatus.goToEnabled();
   // GRIPPER STATUS - Activated - gACT
   bool gACT = status.gripperStatus.activated();
   // FAULT STATUS - Controller Fault - kFLT
   Robotiq::ControllerFault kFLT = status.faultStatus.controllerFault();
   // FAULT STATUS - Gripper Fault - gFLT
   Robotiq::GripperFault gFLT = status.faultStatus.gripperFault();
   // POS REQUEST ECHO - Position Request Eco - gPR
   uint8_t gPR = status.positionRequestEcho;
   // POSITION - Position - gPO
   uint8_t gPO = status.position;
   // CURRENT - Current - gCU
   uint8_t gCU = status.current;
   //! [status-gripper-status-fields]
}

void actionRequestBits()
{
   //! [action-request-bits]
   Robotiq::GripperCommand command = Robotiq::GripperCommand::defaults();
   command.action.set(Robotiq::ActionRequestBit::GoTo); // start moving (same as set(GoTo, true))
   command.action.set(Robotiq::ActionRequestBit::AutoRelease, false); // ...but not an emergency release
   //! [action-request-bits]
}

//! [gripper-status-flags]
void gripperStatusFlags(Robotiq::Gripper& gripper)
{
   Robotiq::GripperStatus status = gripper.getStatus();
   bool activated = status.gripperStatus.activated(); // gACT flag
   bool goTo = status.gripperStatus.goToEnabled(); // gGTO flag
   if(status.gripperStatus.activationState() == Robotiq::ActivationState::Complete
      && status.gripperStatus.objectDetection() != Robotiq::ObjectDetection::Moving)
   {
      // motion has settled
   }
}
//! [gripper-status-flags]

//! [motion-settled-example]
void motionSettledExample(Robotiq::Gripper& gripper)
{
   Robotiq::GripperStatus status = gripper.getStatus();
   if(status.gripperStatus.objectDetection() != Robotiq::ObjectDetection::Moving)
   {
      // motion has settled
   }
}
//! [motion-settled-example]

void faultSeverityCheck(Robotiq::Gripper& gripper)
{
   //! [fault-severity-check]
   Robotiq::FaultStatus fault = gripper.getStatus().faultStatus;
   if(Robotiq::severity(fault.gripperFault()) == Robotiq::FaultSeverity::Major)
   {
      Robotiq::recoverFromFault(gripper);
   }
   //! [fault-severity-check]
}

//! [uart-logger]
void uartWrite(std::string_view) {} // stands in for a real UART driver

class UartLogger : public Robotiq::Logger
{
public:
   void log(Level level, std::string_view message) override
   {
      if(level >= Level::Warn)
      {
         uartWrite("!! ");
      }
      uartWrite(message);
   }
};

void useCustomLogger()
{
   auto logger = std::make_shared<UartLogger>();
   Robotiq::ConnectionConfig config;
   config.serial.port = "/dev/ttyUSB0";
   Robotiq::Gripper gripper(config, logger);
}
//! [uart-logger]

void makeFakeGripperUsage()
{
   //! [make-fake-gripper]
   auto gripper = Robotiq::makeFakeGripper(); // no hardware needed
   Robotiq::activate(*gripper);
   //! [make-fake-gripper]
}

//! [wait-with-platform]
void waitWithPlatform(Robotiq::Gripper& gripper, uint8_t target)
{
   bool settled = Robotiq::waitFor([&] { return gripper.getStatus().positionRequestEcho == target; },
                                   gripper.platform(),
                                   std::chrono::seconds(1));
}
//! [wait-with-platform]

void autoreleaseThenMove(Robotiq::Gripper& gripper)
{
   //! [autorelease-then-move]
   // Build and set an autorelease command
   Robotiq::GripperCommand command = Robotiq::GripperCommand::defaults();
   command.action.set(Robotiq::ActionRequestBit::AutoRelease);
   gripper.setCommand(command);

   // Build and set a command to move the gripper to the position 100
   command.action.set(Robotiq::ActionRequestBit::GoTo);
   command.positionRequest = 100;
   gripper.setCommand(command);
   //! [autorelease-then-move]
}

void autoreleaseThenMoveWithWait(Robotiq::Gripper& gripper)
{
   //! [autorelease-then-move-with-wait]
   // Build and set an autorelease command
   Robotiq::GripperCommand command = Robotiq::GripperCommand::defaults();
   command.action.set(Robotiq::ActionRequestBit::AutoRelease);
   gripper.setCommand(command);

   // Wait
   Robotiq::waitFor(
      [&] {
         return (gripper.getStatus().faultStatus.gripperFault() == Robotiq::GripperFault::AutomaticReleaseInProgress);
      },
      10s);

   // Build and set a command to move the gripper to the position 100
   command.action.set(Robotiq::ActionRequestBit::GoTo);
   command.positionRequest = 100;
   gripper.setCommand(command);
   //! [autorelease-then-move-with-wait]
}

void waitForMotionSettled(Robotiq::Gripper& gripper)
{
   //! [wait-for-motion-settled]
   bool settled = Robotiq::waitFor(
      [&] { return gripper.getStatus().gripperStatus.objectDetection() != Robotiq::ObjectDetection::Moving; },
      10s);
   //! [wait-for-motion-settled]
}

void activateOnly(Robotiq::Gripper& gripper)
{
   //! [activate-only]
   Robotiq::ActivationResult result = Robotiq::activate(gripper);
   //! [activate-only]
}

void recoverFromFaultOnly(Robotiq::Gripper& gripper)
{
   //! [recover-from-fault-only]
   Robotiq::ActivationResult result = Robotiq::recoverFromFault(gripper);
   //! [recover-from-fault-only]
}

void basicLoggerInjection(Robotiq::ConnectionConfig& config)
{
   //! [basic-logger-injection]
   auto logger = std::make_shared<Robotiq::StderrLogger>();
   Robotiq::Gripper gripper(config, logger);
   //! [basic-logger-injection]
}

} // namespace

int main()
{
   return 0;
}
