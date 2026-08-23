// Copyright (c) 2026 Robotiq, Inc.
//
// Licensed under the BSD-3-Clause license; see LICENSE for details.

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cstring>
#include <functional>
#include <memory>
#include <optional>
#include <thread>
#include <utility>
#include <vector>

#include <Robotiq/gripper/platform.hpp>

#include <Robotiq/gripper.hpp>
#include <Robotiq/gripper/connection_state.hpp>
#include <Robotiq/gripper/command.hpp>
#include <Robotiq/gripper/driver_exception.hpp>
#include <Robotiq/gripper/status.hpp>
#include <Robotiq/gripper/logger.hpp>
#include <Robotiq/gripper/serial_io_exception.hpp>
#include <Robotiq/detail/modbus_constants.hpp>

#include "fake/status_writer.hpp"
#include "fake_gripper_fixture.hpp"
#include "test_utils.hpp"

namespace Robotiq::test {

namespace {
namespace mc = Robotiq::detail::modbus_constants;

constexpr uint8_t kSlave = 0x09;
constexpr std::chrono::milliseconds kFastPeriod{1};
//! Slow enough that the next cycle is always still ahead when a wait
//! starts, so what a sync test observes is a real wait.
constexpr std::chrono::milliseconds kSyncPeriod{20};

GripperCommand activateCommand()
{
   GripperCommand command;
   std::memset(command.data(), 0, command.size());
   command.action.set(ActionRequestBit::Activate, true);
   return command;
}

//! GripperSerial whose writes fail while failing is set — a link
//! that starts healthy, drops out, and comes back. Failing on write
//! keeps the fake's reply streams free of stale responses.
class WriteFailingSerial : public fake::GripperSerial
{
public:
   using fake::GripperSerial::GripperSerial;

   void write(const std::vector<uint8_t>& data) override
   {
      if(failing.load())
      {
         throw SerialIOException("injected wire failure");
      }
      GripperSerial::write(data);
   }

   std::atomic<bool> failing{false};
};

//! GripperSerial whose replies are lost while dropReplies is set:
//! requests still reach the gripper, but reads fail.
class ReplyDroppingSerial : public fake::GripperSerial
{
public:
   using fake::GripperSerial::GripperSerial;

   std::vector<uint8_t> read(size_t size, std::chrono::milliseconds timeout) override
   {
      if(dropReplies.load())
      {
         _gripperServer.discardPendingReply();
         throw SerialIOException("injected reply loss");
      }
      return GripperSerial::read(size, timeout);
   }

   std::atomic<bool> dropReplies{false};
};

//! Thread that counts its joins on the way through to the real one.
class JoinCountingThread : public Thread
{
public:
   JoinCountingThread(std::unique_ptr<Thread> real, std::atomic<int>& joins)
      : _real(std::move(real))
      , _joins(joins)
   {
   }

   void join() override
   {
      _real->join();
      ++_joins;
   }

private:
   std::unique_ptr<Thread> _real;
   std::atomic<int>& _joins;
};

//! Condition variable that counts the waits it serves, on the way through
//! to the real one.
class WaitCountingConditionVariable : public ConditionVariable
{
public:
   WaitCountingConditionVariable(std::unique_ptr<ConditionVariable> real, std::atomic<int>& waits)
      : _real(std::move(real))
      , _waits(waits)
   {
   }

   void waitUntil(Mutex& mutex, std::chrono::steady_clock::time_point timePoint) override
   {
      ++_waits;
      _real->waitUntil(mutex, timePoint);
   }

   void notifyAll() override { _real->notifyAll(); }

private:
   std::unique_ptr<ConditionVariable> _real;
   std::atomic<int>& _waits;
};

//! Platform that delegates to the default std-backed one but counts
//! what the gripper asks of it — the seam a real RTOS port implements.
class InstrumentedPlatform : public Platform
{
public:
   std::atomic<int> mutexesCreated{0};
   std::atomic<int> conditionVariablesCreated{0};
   std::atomic<int> conditionWaits{0};
   std::atomic<int> threadsSpawned{0};
   std::atomic<int> threadsJoined{0};
   std::atomic<int> sleepUntils{0};
   std::atomic<int> sleepFors{0};

   std::unique_ptr<Mutex> makeMutex() override
   {
      ++mutexesCreated;
      return _real->makeMutex();
   }

   std::unique_ptr<ConditionVariable> makeConditionVariable() override
   {
      ++conditionVariablesCreated;
      auto counting = std::make_unique<WaitCountingConditionVariable>(_real->makeConditionVariable(), conditionWaits);
      _created.store(counting.get());
      return counting;
   }

   std::unique_ptr<Thread> spawn(std::function<void()> fn) override
   {
      ++threadsSpawned;
      return std::make_unique<JoinCountingThread>(_real->spawn(std::move(fn)), threadsJoined);
   }

   void sleepUntil(std::chrono::steady_clock::time_point timePoint) override
   {
      ++sleepUntils;
      _real->sleepUntil(timePoint);
   }

   void sleepFor(std::chrono::milliseconds duration) override
   {
      ++sleepFors;
      _real->sleepFor(duration);
   }

   //! Notify the gripper's condition variable with nothing behind it — the
   //! spurious wake every waiter has to be ready for.
   void notifyWithNoNews()
   {
      if(ConditionVariable* const condition = _created.load())
      {
         condition->notifyAll();
      }
   }

private:
   std::atomic<ConditionVariable*> _created{nullptr};
   std::shared_ptr<Platform> _real = makeDefaultPlatform();
};
} // namespace

class TestGripper : public ::testing::Test
{
protected:
   TestGripper()
      : gripper(std::make_unique<fake::GripperSerial>(fakeServer.server),
                kSlave,
                kFastPeriod,
                makeDefaultPlatform(),
                std::make_shared<NullLogger>())
   {
   }

   template <typename Predicate>
   bool waitFor(Predicate predicate, std::chrono::seconds timeout = std::chrono::seconds(2))
   {
      return Robotiq::waitFor(predicate, timeout, std::chrono::milliseconds(1));
   }

   InstrumentedFakeGripperServer fakeServer;
   Gripper gripper;
};

TEST_F(TestGripper, construction_reaches_the_gripper)
{
   // The initial read happened in the constructor: no waiting.
   EXPECT_EQ(gripper.connectionState(), ConnectionState::Operational);
}

TEST_F(TestGripper, activate_blocks_until_the_gripper_reports_complete)
{
   EXPECT_EQ(activate(gripper, std::chrono::seconds(2)), ActivationResult::Activated);
   EXPECT_TRUE(gripper.getStatus().gripperStatus.activated());
   // Idempotent: a second call reports already-active without acting.
   EXPECT_EQ(activate(gripper, std::chrono::seconds(2)), ActivationResult::AlreadyActive);
}

TEST(TestGripperActivate, default_speed_and_force_reach_the_gripper)
{
   InstrumentedFakeGripperServer fakeServer;
   {
      Gripper gripper(std::make_unique<fake::GripperSerial>(fakeServer.server),
                      kSlave,
                      kFastPeriod,
                      makeDefaultPlatform(),
                      std::make_shared<NullLogger>());
      EXPECT_EQ(activate(gripper, std::chrono::seconds(2)), ActivationResult::Activated);
   }
   const GripperCommand defaults = GripperCommand::defaults();
   const GripperCommand written = fakeServer.model.command();
   EXPECT_EQ(written.speed, defaults.speed);
   EXPECT_EQ(written.force, defaults.force);
}

TEST(TestGripperActivate, redundant_activate_keeps_the_command_and_never_resets)
{
   InstrumentedFakeGripperServer fakeServer;
   Gripper gripper(std::make_unique<fake::GripperSerial>(fakeServer.server),
                   kSlave,
                   kFastPeriod,
                   makeDefaultPlatform(),
                   std::make_shared<NullLogger>());
   ASSERT_EQ(activate(gripper, std::chrono::seconds(2)), ActivationResult::Activated);

   GripperCommand command = GripperCommand::defaults();
   command.action.set(ActionRequestBit::GoTo, true);
   command.positionRequest = 0x42;
   command.speed = 0x21;
   gripper.setCommand(command);

   EXPECT_EQ(activate(gripper, std::chrono::seconds(2)), ActivationResult::AlreadyActive);

   // The application's command survives, and no reset request went out.
   EXPECT_EQ(fakeServer.model.resets.load(), 0);
   const GripperCommand kept = gripper.getCommand();
   EXPECT_EQ(kept.positionRequest, 0x42);
   EXPECT_EQ(kept.speed, 0x21);
   EXPECT_TRUE(kept.action.get(ActionRequestBit::GoTo));
   EXPECT_TRUE(kept.action.get(ActionRequestBit::Activate));
}

TEST(TestGripperActivate, already_activated_gripper_is_left_undisturbed)
{
   InstrumentedFakeGripperServer fakeServer;
   // Activation retained from a previous session (it survives com loss,
   // only power loss clears it).
   fakeServer.model.setActivated();
   {
      Gripper gripper(std::make_unique<fake::GripperSerial>(fakeServer.server),
                      kSlave,
                      kFastPeriod,
                      makeDefaultPlatform(),
                      std::make_shared<NullLogger>());

      EXPECT_EQ(activate(gripper, std::chrono::seconds(2)), ActivationResult::AlreadyActive);

      // The cycle only echoes state the gripper already holds: after
      // command writes land, it is still activated and was never reset.
      ASSERT_TRUE(Robotiq::waitFor([&] { return fakeServer.model.commandWrites.load() > 0; },
                                   std::chrono::seconds(2),
                                   std::chrono::milliseconds(1)));
      EXPECT_EQ(fakeServer.model.resets.load(), 0);
      EXPECT_TRUE(gripper.getStatus().gripperStatus.activated());
   }
   // The register file is safe to inspect only once the exchange thread
   // is gone (the counters above are atomic; the registers are not).
   EXPECT_TRUE(fakeServer.model.command().action.get(ActionRequestBit::Activate));
}

TEST(TestGripperActivate, latched_major_fault_refuses_activation_until_explicit_recovery)
{
   InstrumentedFakeGripperServer fakeServer;
   // An activated gripper latching a major fault: per the manual, only
   // an rACT falling edge (reset) clears the fault status — but that
   // reset releases any grip and sweeps the fingers, so activate() must
   // refuse and leave the decision to the application.
   fakeServer.model.setActivated();
   fakeServer.model.setFault(GripperFault::Overcurrent);
   Gripper gripper(std::make_unique<fake::GripperSerial>(fakeServer.server),
                   kSlave,
                   kFastPeriod,
                   makeDefaultPlatform(),
                   std::make_shared<NullLogger>());

   const GripperCommand before = gripper.getCommand();
   EXPECT_EQ(activate(gripper, std::chrono::seconds(2)), ActivationResult::FaultLatched);
   EXPECT_EQ(fakeServer.model.resets.load(), 0);
   EXPECT_EQ(gripper.getCommand(), before) << "refusing must not touch the command either";
   EXPECT_EQ(gripper.getStatus().faultStatus.gripperFault(), GripperFault::Overcurrent);

   // The explicit recovery runs the documented reset and clears it.
   EXPECT_EQ(recoverFromFault(gripper, std::chrono::seconds(2)), ActivationResult::Activated);
   EXPECT_GE(fakeServer.model.resets.load(), 1);
   EXPECT_EQ(gripper.getStatus().faultStatus.gripperFault(), GripperFault::None);
   EXPECT_TRUE(gripper.getStatus().gripperStatus.activated());
}

TEST(TestGripperActivate, minor_fault_does_not_trigger_a_reset)
{
   InstrumentedFakeGripperServer fakeServer;
   // gFLT 0x09 ("no communication for 1 s") is inevitably latched when
   // connecting after idle time: it must not cost a reset and a
   // recalibration sweep.
   fakeServer.model.setActivated();
   fakeServer.model.setFault(GripperFault::NoCommunication);
   Gripper gripper(std::make_unique<fake::GripperSerial>(fakeServer.server),
                   kSlave,
                   kFastPeriod,
                   makeDefaultPlatform(),
                   std::make_shared<NullLogger>());

   EXPECT_EQ(activate(gripper, std::chrono::seconds(2)), ActivationResult::AlreadyActive);
   EXPECT_EQ(fakeServer.model.resets.load(), 0);
   // The fake's latched fault would have cleared on a reset: it still
   // being there is hardware-level proof none was sent.
   EXPECT_EQ(gripper.getStatus().faultStatus.gripperFault(), GripperFault::NoCommunication);
   EXPECT_TRUE(gripper.getStatus().gripperStatus.activated());
}

TEST(TestGripperActivate, power_cycled_gripper_needs_the_full_handshake)
{
   InstrumentedFakeGripperServer fakeServer;
   // A power-cycled gripper comes back unactivated yet echoing gACT set
   // while gSTA reports reset (bench-observed), so gACT alone must not
   // be trusted.
   GripperStatus powerCycled;
   fake::setActivated(powerCycled, true); // gACT echoed...
   fake::setActivationState(powerCycled, ActivationState::Reset); // ...but gSTA says reset
   fakeServer.model.setStatus(powerCycled);
   fakeServer.model.setActivationHighButIncomplete();
   Gripper gripper(std::make_unique<fake::GripperSerial>(fakeServer.server),
                   kSlave,
                   kFastPeriod,
                   makeDefaultPlatform(),
                   std::make_shared<NullLogger>());

   EXPECT_EQ(activate(gripper, std::chrono::seconds(2)), ActivationResult::Activated);
   // The full clear-then-set handshake ran: a reset request went out.
   EXPECT_GE(fakeServer.model.resets.load(), 1);
   EXPECT_EQ(gripper.getStatus().gripperStatus.activationState(), ActivationState::Complete);
}

TEST(TestGripperCommandImage, is_seeded_from_the_status_echoes_at_construction)
{
   InstrumentedFakeGripperServer fakeServer;
   // Retained state from a previous session: activated, GoTo held,
   // position request 0x55.
   fakeServer.model.setActivated();
   fakeServer.model.setGoToEcho();
   fakeServer.model.setPositionRequestEcho(0x55);
   Gripper gripper(std::make_unique<fake::GripperSerial>(fakeServer.server),
                   kSlave,
                   kFastPeriod,
                   makeDefaultPlatform(),
                   std::make_shared<NullLogger>());

   // The construction-time read already seeded the image: no waiting.
   EXPECT_EQ(gripper.connectionState(), ConnectionState::Operational);
   const GripperCommand seeded = gripper.getCommand();
   EXPECT_TRUE(seeded.action.get(ActionRequestBit::Activate));
   EXPECT_TRUE(seeded.action.get(ActionRequestBit::GoTo));
   EXPECT_EQ(seeded.positionRequest, 0x55);
   // No echo exists for speed and force: they seed at the defaults()
   // full-scale values.
   EXPECT_EQ(seeded.speed, GripperCommand::defaults().speed);
   EXPECT_EQ(seeded.force, GripperCommand::defaults().force);
}

TEST(TestGripperConstruction, fails_when_no_gripper_answers_and_never_writes)
{
   InstrumentedFakeGripperServer fakeServer;
   auto serial = std::make_unique<ReplyDroppingSerial>(fakeServer.server);
   serial->dropReplies.store(true);

   // Requests reach the gripper but no reply ever arrives: construction
   // must fail like a dead serial link would...
   EXPECT_THROW(Gripper(std::move(serial), kSlave, kFastPeriod, makeDefaultPlatform(), std::make_shared<NullLogger>()),
                DriverException);

   // ...after retrying the status read, without ever writing a command.
   EXPECT_GE(fakeServer.model.statusReads.load(), 2);
   EXPECT_EQ(fakeServer.model.commandWrites.load(), 0);
}

TEST(TestGripperConstruction, fails_on_a_null_platform_before_touching_the_bus)
{
   InstrumentedFakeGripperServer fakeServer;
   auto serial = std::make_unique<ReplyDroppingSerial>(fakeServer.server);

   EXPECT_THROW(Gripper(std::move(serial), kSlave, kFastPeriod, nullptr, std::make_shared<NullLogger>()),
                DriverException);
   EXPECT_EQ(fakeServer.model.statusReads.load(), 0);
}

TEST(TestGripperPlatform, exchange_runs_entirely_on_the_injected_platform)
{
   InstrumentedFakeGripperServer fakeServer;
   const auto platform = std::make_shared<InstrumentedPlatform>();
   {
      Gripper gripper(std::make_unique<fake::GripperSerial>(fakeServer.server),
                      kSlave,
                      kFastPeriod,
                      platform,
                      std::make_shared<NullLogger>());
      // The gripper runs on — and reports — the platform it was given.
      EXPECT_EQ(&gripper.platform(), platform.get());
      // One exchange thread, one image lock, one condition variable to
      // announce the image with — and nothing else.
      EXPECT_EQ(platform->threadsSpawned.load(), 1);
      EXPECT_EQ(platform->mutexesCreated.load(), 1);
      EXPECT_EQ(platform->conditionVariablesCreated.load(), 1);
      // The loop paces every cycle through the platform's sleep.
      ASSERT_TRUE(Robotiq::waitFor([&] { return platform->sleepUntils.load() >= 3; },
                                   std::chrono::seconds(2),
                                   std::chrono::milliseconds(1)));
      EXPECT_EQ(platform->threadsJoined.load(), 0);
   }
   // Destruction joined the exchange thread it spawned.
   EXPECT_EQ(platform->threadsJoined.load(), 1);
}

TEST(TestGripperPlatform, activate_sleeps_on_the_grippers_own_platform)
{
   // A gripper stuck mid-activation forces activate() to poll: every sleep
   // between those polls must come from the injected platform — on an RTOS a
   // std sleep here would be the app-task starvation bug all over again.
   InstrumentedFakeGripperServer fakeServer;
   GripperStatus stuck;
   fake::setActivated(stuck, true);
   fake::setActivationState(stuck, ActivationState::InProgress);
   fakeServer.model.pinnedStatus = stuck;
   const auto platform = std::make_shared<InstrumentedPlatform>();
   Gripper gripper(std::make_unique<fake::GripperSerial>(fakeServer.server),
                   kSlave,
                   kFastPeriod,
                   platform,
                   std::make_shared<NullLogger>());

   EXPECT_EQ(activate(gripper, std::chrono::milliseconds(30)), ActivationResult::Timeout);
   EXPECT_GT(platform->sleepFors.load(), 0);
}

TEST_F(TestGripper, a_cursor_hands_out_a_fresh_status_every_time)
{
   // What a control loop does: wait, act, wait again.
   auto cursor = gripper.makeSync();
   for(int cycle = 0; cycle < 5; ++cycle)
   {
      // Only that each wait lands a status this loop has not had. Whether
      // it also kept up — skipped() == 0 — is the scheduler's business, not
      // the SDK's; skipped() is asserted where it is made to happen, in
      // a_control_loop_busy_acting_on_a_status_... below.
      ASSERT_TRUE(cursor.wait(std::chrono::seconds(2))) << "cycle " << cycle << " never landed";
   }
}

TEST(TestGripperSync, waits_on_the_platforms_condition_variable_only_when_it_has_to)
{
   InstrumentedFakeGripperServer fakeServer;
   const auto platform = std::make_shared<InstrumentedPlatform>();
   Gripper gripper(std::make_unique<fake::GripperSerial>(fakeServer.server),
                   kSlave,
                   kSyncPeriod,
                   platform,
                   std::make_shared<NullLogger>());

   auto cursor = gripper.makeSync();
   ASSERT_TRUE(cursor.wait(std::chrono::seconds(2)));
   EXPECT_GT(platform->conditionWaits.load(), 0);
   // A native condition variable blocks; a wait never falls back to sleeping.
   EXPECT_EQ(platform->sleepFors.load(), 0);

   // A cycle that already landed costs no wait: let some pile up behind the
   // cursor, then take one.
   ASSERT_TRUE(Robotiq::waitFor([&] { return gripper.exchangeCount() > 3; },
                                std::chrono::seconds(2),
                                std::chrono::milliseconds(1)));
   const int waitsSoFar = platform->conditionWaits.load();
   EXPECT_TRUE(cursor.wait(std::chrono::seconds(2)));
   EXPECT_EQ(platform->conditionWaits.load(), waitsSoFar);
}

TEST(TestGripperSync, a_wake_with_nothing_behind_it_is_re_checked_not_trusted)
{
   InstrumentedFakeGripperServer fakeServer;
   const auto platform = std::make_shared<InstrumentedPlatform>();
   Gripper gripper(std::make_unique<fake::GripperSerial>(fakeServer.server),
                   kSlave,
                   kSyncPeriod,
                   platform,
                   std::make_shared<NullLogger>());

   auto cursor = gripper.makeSync();
   ASSERT_TRUE(cursor.wait(std::chrono::seconds(2))); // catch up to the cycle

   // Notifications carrying no new status, which is what a spurious wake is
   // and what the ThreadX emulation can produce from a leftover token. A
   // wait that returns on one of these without looking at the count would
   // hand back a status it never received.
   std::atomic<bool> stop{false};
   std::thread noise([&] {
      while(!stop.load())
      {
         platform->notifyWithNoNews();
         std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
   });

   const int waitsBefore = platform->conditionWaits.load();
   EXPECT_TRUE(cursor.wait(std::chrono::seconds(2)));
   // Woken many times for one cycle: each one was re-checked and resumed.
   EXPECT_GT(platform->conditionWaits.load(), waitsBefore + 1);

   stop.store(true);
   noise.join();
}

TEST(TestGripperSync, a_control_loop_busy_acting_on_a_status_never_holds_the_exchange_cycle_up)
{
   // Why this is a wait and not a callback: a callback on the exchange
   // thread — or a wait that handed back the image lock — would stall every
   // cycle for as long as the application thinks.
   InstrumentedFakeGripperServer fakeServer;
   Gripper gripper(std::make_unique<fake::GripperSerial>(fakeServer.server),
                   kSlave,
                   kFastPeriod,
                   makeDefaultPlatform(),
                   std::make_shared<NullLogger>());

   std::atomic<bool> busy{false};
   std::atomic<bool> release{false};
   std::atomic<uint64_t> skippedWhileBusy{0};
   auto cursor = gripper.makeSync();
   std::thread controlLoop([&] {
      (void)cursor.wait(std::chrono::seconds(2));
      // Stands in for arbitrarily slow application work on that status.
      busy.store(true);
      while(!release.load())
      {
         std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
      // Back from the "work": resumes on the newest status, reporting what
      // went by.
      (void)cursor.wait(std::chrono::seconds(2));
      skippedWhileBusy.store(cursor.skipped());
   });
   // A failed assertion returns early; the loop must not outlive the
   // Gripper it waits on.
   struct Releaser
   {
      std::atomic<bool>& release;
      std::thread& loop;

      ~Releaser()
      {
         release.store(true);
         if(loop.joinable())
         {
            loop.join();
         }
      }
   } const releaser{release, controlLoop};

   ASSERT_TRUE(Robotiq::waitFor([&] { return busy.load(); }, std::chrono::seconds(2), std::chrono::milliseconds(1)));

   // That cycles keep completing is the assertion; how fast is not.
   const uint64_t whenBusy = gripper.exchangeCount();
   EXPECT_TRUE(Robotiq::waitFor([&] { return gripper.exchangeCount() >= whenBusy + 100; },
                                std::chrono::seconds(2),
                                std::chrono::milliseconds(1)));

   // Nothing queued up for it either: it resumed on the newest status and
   // counted what went by. Read after the join, which publishes the count.
   release.store(true);
   controlLoop.join();
   // Half of what went by, to stay clear of where the cursor's baseline sat
   // relative to whenBusy: the point is that they are counted, not the
   // exact count.
   EXPECT_GE(skippedWhileBusy.load(), 50u);
}

TEST(TestGripperSync, a_cursors_status_is_the_snapshot_its_wake_named)
{
   InstrumentedFakeGripperServer fakeServer;
   Gripper gripper(std::make_unique<fake::GripperSerial>(fakeServer.server),
                   kSlave,
                   kSyncPeriod,
                   makeDefaultPlatform(),
                   std::make_shared<NullLogger>());

   // Activated first: the fake, like the gripper, only latches a fault
   // while rACT is set.
   ASSERT_EQ(activate(gripper, std::chrono::seconds(2)), ActivationResult::Activated);
   auto cursor = gripper.makeSync();
   ASSERT_TRUE(cursor.wait(std::chrono::seconds(2)));
   ASSERT_EQ(cursor.status().faultStatus.gripperFault(), GripperFault::None);

   // The gripper moves on and the image follows; the cursor's snapshot does
   // not, so the loop that woke on it acts on what it was woken for.
   fakeServer.model.setFault(GripperFault::Overcurrent);
   ASSERT_TRUE(
      Robotiq::waitFor([&] { return gripper.getStatus().faultStatus.gripperFault() == GripperFault::Overcurrent; },
                       std::chrono::seconds(2),
                       std::chrono::milliseconds(1)));
   EXPECT_EQ(cursor.status().faultStatus.gripperFault(), GripperFault::None);

   // The next wake is the one that carries it.
   ASSERT_TRUE(cursor.wait(std::chrono::seconds(2)));
   EXPECT_EQ(cursor.status().faultStatus.gripperFault(), GripperFault::Overcurrent);
}

TEST(TestGripperSync, a_cursor_outliving_its_gripper_returns_false_instead_of_dangling)
{
   InstrumentedFakeGripperServer fakeServer;
   std::optional<GripperSync> cursor;
   {
      Gripper gripper(std::make_unique<fake::GripperSerial>(fakeServer.server),
                      kSlave,
                      kFastPeriod,
                      makeDefaultPlatform(),
                      std::make_shared<NullLogger>());
      cursor = gripper.makeSync();
      ASSERT_TRUE(cursor->wait(std::chrono::seconds(2)));
   }

   // Defined rather than dangling: the state outlived the gripper. Returning
   // false is also the proof that destruction stopped the cycle instead of
   // leaving it running on that shared state.
   EXPECT_FALSE(cursor->wait(std::chrono::seconds(2)));
}

TEST(TestGripperSync, destroying_the_gripper_wakes_a_blocked_cursor_at_once)
{
   InstrumentedFakeGripperServer fakeServer;
   auto serial = std::make_unique<WriteFailingSerial>(fakeServer.server);
   WriteFailingSerial& link = *serial;
   auto gripper = std::make_unique<Gripper>(std::move(serial),
                                            kSlave,
                                            kFastPeriod,
                                            makeDefaultPlatform(),
                                            std::make_shared<NullLogger>());
   // Nothing will ever wake the cursor but the stop.
   link.failing.store(true);
   ASSERT_TRUE(Robotiq::waitFor([&] { return gripper->connectionState() == ConnectionState::Faulted; },
                                std::chrono::seconds(2),
                                std::chrono::milliseconds(1)));

   auto cursor = gripper->makeSync();
   std::atomic<bool> returned{false};
   std::thread controlLoop([&] {
      // Far longer than the test may take: only the stop can end this wait.
      EXPECT_FALSE(cursor.wait(std::chrono::seconds(30)));
      returned.store(true);
   });

   gripper.reset();
   EXPECT_TRUE(
      Robotiq::waitFor([&] { return returned.load(); }, std::chrono::seconds(2), std::chrono::milliseconds(1)));
   controlLoop.join();
}

TEST(TestGripperSync, a_stalled_bus_freezes_the_count_and_times_the_wait_out)
{
   InstrumentedFakeGripperServer fakeServer;
   auto serial = std::make_unique<WriteFailingSerial>(fakeServer.server);
   WriteFailingSerial& link = *serial;
   Gripper gripper(std::move(serial), kSlave, kFastPeriod, makeDefaultPlatform(), std::make_shared<NullLogger>());

   link.failing.store(true);
   ASSERT_TRUE(Robotiq::waitFor([&] { return gripper.connectionState() == ConnectionState::Faulted; },
                                std::chrono::seconds(2),
                                std::chrono::milliseconds(1)));

   // No exchange can complete, so the count stands still and the wait
   // returns it unchanged — a caller sees the stall here without waiting
   // for connectionState() to degrade.
   const uint64_t stalled = gripper.exchangeCount();
   auto cursor = gripper.makeSync();
   EXPECT_FALSE(cursor.wait(std::chrono::milliseconds(30)));
   EXPECT_EQ(gripper.exchangeCount(), stalled);
}

TEST_F(TestGripper, commands_reach_the_gripper_and_status_returns)
{
   gripper.setCommand(activateCommand());

   EXPECT_TRUE(waitFor([&] { return gripper.getStatus().gripperStatus.activated(); }));
   EXPECT_EQ(gripper.connectionState(), ConnectionState::Operational);
}

TEST_F(TestGripper, command_read_modify_write_round_trips)
{
   GripperCommand command;
   command.positionRequest = 0x42;
   gripper.setCommand(command);

   GripperCommand readBack = gripper.getCommand();
   EXPECT_EQ(readBack.positionRequest, 0x42);

   readBack.speed = 0x21;
   gripper.setCommand(readBack);

   EXPECT_EQ(gripper.getCommand().speed, 0x21);
   EXPECT_EQ(gripper.getCommand().positionRequest, 0x42);
}

TEST_F(TestGripper, typed_layers_compose_over_the_image)
{
   GripperCommand command = GripperCommand::defaults();
   command.action.set(ActionRequestBit::GoTo, true);
   command.positionRequest = 0x42;
   command.speed = 0x21;
   command.force = 0x11;
   gripper.setCommand(command);

   EXPECT_TRUE(waitFor([&] { return gripper.getStatus().positionRequestEcho == 0x42; }));
   EXPECT_TRUE(gripper.getStatus().gripperStatus.activated());
   EXPECT_EQ(gripper.getStatus().position, 0x42);
   EXPECT_EQ(gripper.getStatus().gripperStatus.objectDetection(), ObjectDetection::AtRequestedPosition);
   EXPECT_EQ(gripper.getStatus().current, fake::RegisterModel::kReportedCurrent);
   EXPECT_EQ(gripper.getStatus().faultStatus.raw(), 0);
   EXPECT_EQ(gripper.getStatus().positionRequestEcho, 0x42);
   EXPECT_TRUE(gripper.getStatus().gripperStatus.goToEnabled());
   EXPECT_EQ(gripper.getStatus().gripperStatus.activationState(), ActivationState::Complete);
}

TEST(TestGripperActivate, handshake_keeps_the_callers_speed_force_and_position)
{
   InstrumentedFakeGripperServer fakeServer;
   // A power-cycled gripper: gACT echoed, gSTA reset, so the handshake runs.
   GripperStatus powerCycled;
   fake::setActivated(powerCycled, true);
   fake::setActivationState(powerCycled, ActivationState::Reset);
   fakeServer.model.setStatus(powerCycled);
   fakeServer.model.setActivationHighButIncomplete();
   Gripper gripper(std::make_unique<fake::GripperSerial>(fakeServer.server),
                   kSlave,
                   kFastPeriod,
                   makeDefaultPlatform(),
                   std::make_shared<NullLogger>());

   // An application that has tuned its grip must not silently get full scale
   // back because a power-cycled gripper needed a handshake.
   GripperCommand gentle = gripper.getCommand();
   gentle.speed = 0x20;
   gentle.force = 0x10;
   gentle.positionRequest = 0x40;
   gentle.action.set(ActionRequestBit::GoTo, true);
   gripper.setCommand(gentle);

   ASSERT_EQ(activate(gripper, std::chrono::seconds(2)), ActivationResult::Activated);

   const GripperCommand after = gripper.getCommand();
   EXPECT_EQ(after.speed, 0x20);
   EXPECT_EQ(after.force, 0x10);
   EXPECT_EQ(after.positionRequest, 0x40);
   EXPECT_TRUE(after.action.get(ActionRequestBit::Activate));
   EXPECT_FALSE(after.action.get(ActionRequestBit::GoTo)); // the handshake never commands motion
}

TEST(TestGripperActivateFailure, recover_from_fault_times_out_while_the_link_is_down)
{
   InstrumentedFakeGripperServer fakeServer;
   auto serial = std::make_unique<WriteFailingSerial>(fakeServer.server);
   WriteFailingSerial& link = *serial;
   Gripper gripper(std::move(serial), kSlave, kFastPeriod, makeDefaultPlatform(), std::make_shared<NullLogger>());
   const GripperCommand before = gripper.getCommand();

   link.failing.store(true);
   ASSERT_TRUE(Robotiq::waitFor([&] { return gripper.connectionState() == ConnectionState::Faulted; },
                                std::chrono::seconds(2),
                                std::chrono::milliseconds(1)));

   // No status to judge and no way to send: the reset must not be attempted.
   EXPECT_EQ(recoverFromFault(gripper, std::chrono::milliseconds(30)), ActivationResult::Timeout);
   EXPECT_EQ(gripper.getCommand(), before);
}

TEST(TestGripperExchange, a_zero_period_free_runs_instead_of_stalling)
{
   // 0 Hz means free-run: sleep_until on an already-past deadline every
   // iteration. The loop must still exchange, and still be joinable.
   InstrumentedFakeGripperServer fakeServer;
   {
      Gripper gripper(std::make_unique<fake::GripperSerial>(fakeServer.server),
                      kSlave,
                      std::chrono::microseconds{0},
                      makeDefaultPlatform(),
                      std::make_shared<NullLogger>());
      ASSERT_TRUE(Robotiq::waitFor([&] { return fakeServer.model.commandWrites.load() > 10; },
                                   std::chrono::seconds(2),
                                   std::chrono::milliseconds(1)));
      EXPECT_EQ(gripper.connectionState(), ConnectionState::Operational);
   }
   EXPECT_GT(fakeServer.model.statusReads.load(), 0);
}

TEST(TestGripperActivateFailure, times_out_while_the_link_is_down)
{
   InstrumentedFakeGripperServer fakeServer;
   auto serial = std::make_unique<WriteFailingSerial>(fakeServer.server);
   WriteFailingSerial& link = *serial;
   Gripper gripper(std::move(serial), kSlave, kFastPeriod, makeDefaultPlatform(), std::make_shared<NullLogger>());

   // The link dies after construction: activate() must not judge the
   // gripper through a stale image.
   link.failing.store(true);
   ASSERT_TRUE(Robotiq::waitFor([&] { return gripper.connectionState() == ConnectionState::Faulted; },
                                std::chrono::seconds(2),
                                std::chrono::milliseconds(1)));
   EXPECT_EQ(activate(gripper, std::chrono::milliseconds(30)), ActivationResult::Timeout);
}

TEST(TestGripperActivateFailure, times_out_when_activation_never_completes)
{
   // A gripper stuck reporting "activated, calibration in progress":
   // activation complete never arrives.
   InstrumentedFakeGripperServer fakeServer;
   GripperStatus stuck;
   fake::setActivated(stuck, true);
   fake::setActivationState(stuck, ActivationState::InProgress);
   fakeServer.model.pinnedStatus = stuck;
   Gripper gripper(std::make_unique<fake::GripperSerial>(fakeServer.server),
                   kSlave,
                   kFastPeriod,
                   makeDefaultPlatform(),
                   std::make_shared<NullLogger>());

   EXPECT_EQ(activate(gripper, std::chrono::milliseconds(30)), ActivationResult::Timeout);
}

TEST(TestGripperConfigCtor, delivers_serial_logs_to_the_injected_logger)
{
   // Regression: the delegating constructor once moved the logger before
   // makeSerial read it (argument evaluation order is unspecified), so
   // the serial layer silently fell back to the default stderr logger.
   auto logger = std::make_shared<CollectingLogger>();
   ConnectionConfig config;
   config.serial.port = "/dev/this_should_not_exist";

   EXPECT_THROW(Gripper(config, logger), SerialIOException);
   EXPECT_TRUE(logger->contains("opening serial port '/dev/this_should_not_exist'"));
}

TEST(TestGripperHealth, repeated_exchange_failures_degrade_the_state_to_faulted)
{
   InstrumentedFakeGripperServer fakeServer;
   auto serial = std::make_unique<WriteFailingSerial>(fakeServer.server);
   WriteFailingSerial& link = *serial;
   Gripper gripper(std::move(serial), kSlave, kFastPeriod, makeDefaultPlatform(), std::make_shared<NullLogger>());
   ASSERT_EQ(gripper.connectionState(), ConnectionState::Operational);

   // The link dies: every exchange fails from here on.
   link.failing.store(true);
   EXPECT_TRUE(Robotiq::waitFor([&] { return gripper.connectionState() == ConnectionState::Faulted; },
                                std::chrono::seconds(2),
                                std::chrono::milliseconds(1)));
}

TEST(TestGripperHealth, faulted_state_recovers_to_operational_on_the_next_success)
{
   InstrumentedFakeGripperServer fakeServer;
   auto serial = std::make_unique<WriteFailingSerial>(fakeServer.server);
   WriteFailingSerial& link = *serial;
   Gripper gripper(std::move(serial), kSlave, kFastPeriod, makeDefaultPlatform(), std::make_shared<NullLogger>());

   link.failing.store(true);
   ASSERT_TRUE(Robotiq::waitFor([&] { return gripper.connectionState() == ConnectionState::Faulted; },
                                std::chrono::seconds(2),
                                std::chrono::milliseconds(1)));

   link.failing.store(false);
   EXPECT_TRUE(Robotiq::waitFor([&] { return gripper.connectionState() == ConnectionState::Operational; },
                                std::chrono::seconds(2),
                                std::chrono::milliseconds(1)));
}
} // namespace Robotiq::test
