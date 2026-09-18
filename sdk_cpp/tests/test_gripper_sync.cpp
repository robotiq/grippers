// Copyright (c) 2026 Robotiq, Inc.
//
// Licensed under the BSD-3-Clause license; see LICENSE for details.

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <memory>
#include <optional>
#include <thread>
#include <utility>

#include <Robotiq/gripper.hpp>
#include <Robotiq/gripper/fault_status.hpp>
#include <Robotiq/gripper/platform.hpp>
#include <Robotiq/gripper/status.hpp>
#include <Robotiq/gripper/sync.hpp>

#include "fake_gripper_fixture.hpp"
#include "gripper_test_helpers.hpp"
#include "instrumented_platform.hpp"

namespace Robotiq::test {
namespace {
constexpr std::chrono::seconds kWait{2};

// Returned as a prvalue: Gripper is neither copyable nor movable, and C++17's
// guaranteed elision is what lets a factory hand one back by value anyway.
Gripper makeGripper(InstrumentedFakeGripperServer& fakeServer,
                    std::chrono::milliseconds period,
                    std::shared_ptr<Platform> platform = makeDefaultPlatform())
{
   return Gripper(std::make_unique<fake::GripperSerial>(fakeServer.server),
                  kSlave,
                  period,
                  std::move(platform),
                  std::make_shared<NullLogger>());
}

//! A gripper whose link has already failed for good: no exchange will ever
//! complete again, so nothing but the stop can end a wait on it.
std::unique_ptr<Gripper> makeStalledGripper(InstrumentedFakeGripperServer& fakeServer)
{
   auto serial = std::make_unique<WriteFailingSerial>(fakeServer.server);
   WriteFailingSerial& link = *serial;
   auto gripper = std::make_unique<Gripper>(std::move(serial),
                                            kSlave,
                                            kFastPeriod,
                                            makeDefaultPlatform(),
                                            std::make_shared<NullLogger>());
   link.failing.store(true);
   EXPECT_TRUE(Robotiq::waitFor([&] { return gripper->connectionState() == ConnectionState::Faulted; },
                                kWait,
                                std::chrono::milliseconds(1)));
   return gripper;
}

class TestGripperSync : public ::testing::Test
{
protected:
   InstrumentedFakeGripperServer fakeServer;
   Gripper gripper = makeGripper(fakeServer, kSyncPeriod);
};

TEST_F(TestGripperSync, a_sync_objects_status_is_the_snapshot_its_wake_named)
{
   // Activated first: the fake, like the gripper, only latches a fault
   // while rACT is set.
   ASSERT_EQ(activate(gripper, kWait), ActivationResult::Activated);
   GripperSync sync(gripper);
   ASSERT_TRUE(sync.wait(kWait));
   ASSERT_EQ(sync.getStampedStatus().status.faultStatus.gripperFault(), GripperFault::None);

   // The gripper moves on and the image follows; the sync object's snapshot
   // does not, so the stamped status is stale.
   fakeServer.model.setFault(GripperFault::Overcurrent);
   ASSERT_TRUE(
      Robotiq::waitFor([&] { return gripper.getStatus().faultStatus.gripperFault() == GripperFault::Overcurrent; },
                       kWait,
                       std::chrono::milliseconds(1)));
   EXPECT_EQ(sync.getStampedStatus().status.faultStatus.gripperFault(), GripperFault::None);

   // The next wake is the one that carries it.
   ASSERT_TRUE(sync.wait(kWait));
   EXPECT_EQ(sync.getStampedStatus().status.faultStatus.gripperFault(), GripperFault::Overcurrent);
}

TEST_F(TestGripperSync, timestamps_advance_with_the_cycle_count)
{
   GripperSync sync(gripper);
   ASSERT_TRUE(sync.wait(kWait));

   StampedStatus previous = sync.getStampedStatus();
   for(int wake = 0; wake < 5; ++wake)
   {
      ASSERT_TRUE(sync.wait(kWait));
      const StampedStatus stamped = sync.getStampedStatus();
      EXPECT_GT(stamped.exchangeCount, previous.exchangeCount) << "wake " << wake;
      EXPECT_GT(stamped.timestamp, previous.timestamp) << "wake " << wake;
      previous = stamped;
   }
}

TEST_F(TestGripperSync, a_fresh_sync_object_carries_the_seed_reads_timestamp)
{
   // Before any wake the sync object shows the seeding read, not a zero
   // clock: a derived parameter seeded from it starts with a usable interval.
   const GripperSync sync(gripper);
   EXPECT_GT(sync.getStampedStatus().timestamp, std::chrono::steady_clock::time_point{});
   EXPECT_LE(sync.getStampedStatus().timestamp, std::chrono::steady_clock::now());
}

TEST_F(TestGripperSync, the_largest_timeout_waits_for_the_cycle_instead_of_expiring_at_once)
{
   GripperSync sync(gripper);
   EXPECT_TRUE(sync.wait(std::chrono::milliseconds::max()));
}

TEST(TestGripperSyncPlatform, waits_on_the_platforms_condition_variable_not_a_sleep)
{
   InstrumentedFakeGripperServer fakeServer;
   const auto platform = std::make_shared<InstrumentedPlatform>();
   const Gripper gripper = makeGripper(fakeServer, kSyncPeriod, platform);

   GripperSync sync(gripper);
   // Twice: the first publish may land before the first wait even starts.
   ASSERT_TRUE(sync.wait(kWait));
   ASSERT_TRUE(sync.wait(kWait));
   EXPECT_GT(platform->conditionWaits.load(), 0);
   // A native condition variable blocks; a wait never falls back to sleeping.
   EXPECT_EQ(platform->sleepFors.load(), 0);
}

TEST(TestGripperSyncPlatform, a_control_loop_busy_acting_on_a_status_never_holds_the_exchange_cycle_up)
{
   InstrumentedFakeGripperServer fakeServer;
   const Gripper gripper = makeGripper(fakeServer, kFastPeriod);

   std::atomic<bool> busy{false};
   std::atomic<bool> release{false};
   std::atomic<uint64_t> skippedWhileBusy{0};
   GripperSync sync(gripper);
   std::thread controlLoop([&] {
      (void)sync.wait(kWait);
      // Stands in for arbitrarily slow application work on that status.
      busy.store(true);
      while(!release.load())
      {
         std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
      // Back from the "work": resumes on the newest status, reporting what
      // went by.
      (void)sync.wait(kWait);
      skippedWhileBusy.store(sync.getSkipped());
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

   ASSERT_TRUE(Robotiq::waitFor([&] { return busy.load(); }, kWait, std::chrono::milliseconds(1)));

   // That cycles keep completing is the assertion; how fast is not.
   GripperSync pacer(gripper);
   for(int cycle = 0; cycle < 100; ++cycle)
   {
      ASSERT_TRUE(pacer.wait(kWait));
   }

   // Read after the join, which publishes the count.
   release.store(true);
   controlLoop.join();
   // Half of what went by, to stay clear of where the sync object's baseline
   // sat: the point is that they are counted, not the exact count.
   EXPECT_GE(skippedWhileBusy.load(), 50u);
}

TEST(TestGripperSyncLifetime, a_sync_object_outliving_its_gripper_returns_false_instead_of_dangling)
{
   InstrumentedFakeGripperServer fakeServer;
   std::optional<GripperSync> sync;
   {
      const Gripper gripper = makeGripper(fakeServer, kFastPeriod);
      sync.emplace(gripper);
      ASSERT_TRUE(sync->wait(kWait));
   }

   // A publish that landed between the wake above and the destruction may
   // still be delivered once; nothing follows it.
   (void)sync->wait(kWait);
   EXPECT_FALSE(sync->wait(kWait));
}

TEST(TestGripperSyncLifetime, destroying_the_gripper_wakes_a_blocked_sync_object_at_once)
{
   InstrumentedFakeGripperServer fakeServer;
   auto gripper = makeStalledGripper(fakeServer);

   GripperSync sync(*gripper);
   std::atomic<bool> returned{false};
   std::thread controlLoop([&] {
      // Far longer than the test may take: only the stop can end this wait.
      EXPECT_FALSE(sync.wait(std::chrono::seconds(30)));
      returned.store(true);
   });

   gripper.reset();
   EXPECT_TRUE(Robotiq::waitFor([&] { return returned.load(); }, kWait, std::chrono::milliseconds(1)));
   controlLoop.join();
}

TEST(TestGripperSyncLifetime, a_stalled_bus_freezes_the_count_and_times_the_wait_out)
{
   InstrumentedFakeGripperServer fakeServer;
   const auto gripper = makeStalledGripper(fakeServer);

   // No exchange can complete, so the count stands still and the wait
   // returns it unchanged — a caller sees the stall here without waiting
   // for connectionState() to degrade.
   GripperSync sync(*gripper);
   const uint64_t stalled = sync.getStampedStatus().exchangeCount;
   EXPECT_FALSE(sync.wait(std::chrono::milliseconds(30)));
   EXPECT_EQ(sync.getStampedStatus().exchangeCount, stalled);
}

} // namespace
} // namespace Robotiq::test
