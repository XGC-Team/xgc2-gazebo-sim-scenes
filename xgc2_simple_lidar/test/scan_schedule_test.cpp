#include "xgc2_simple_lidar/scan_schedule.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <gtest/gtest.h>
#include <random>
#include <vector>

namespace {

using xgc2_simple_lidar::ScanSchedule;

// The CPU plugin's frame gate as of xgc2-gazebo-sim-scenes 1.4.0-2, inline
// in Frame(): on every world update the sensor and its parent link were
// checked first, then the schedule. (That version also returned first when the
// cloud had no subscriber; no gate below takes a subscriber input.)
struct InlineGate {
    double period, next_scan{0}, previous_time{0};
    bool Frame(double now, bool sensor_and_parent) {
        if (!sensor_and_parent)
            return false;
        if (now < previous_time)
            next_scan = now;
        previous_time = now;
        if (now + 1e-9 < next_scan)
            return false;
        const double elapsed = std::max(0.0, now - next_scan);
        next_scan += (std::floor(elapsed / period) + 1) * period;
        return true;
    }
};

// The plugin's frame gate now: the sensor and parent are looked up only
// when the schedule says a scan is due. The gate has no subscriber input: the
// sensor scans at its rate whether or not the cloud has a consumer.
struct CurrentGate {
    ScanSchedule schedule;
    unsigned lookups{0};
    bool Frame(double now, bool sensor_and_parent) {
        if (!schedule.Due(now))
            return false;
        ++lookups;
        if (!sensor_and_parent)
            return false;
        schedule.Taken(now);
        return true;
    }
};

// Negative control: the 1.4.1-2 plugin returned from every update while the
// cloud had no subscriber. The tests below must tell this gate from the
// current one, or they would also pass if the subscriber check came back.
struct SubscriberGatedGate {
    CurrentGate gate;
    bool Frame(double now, bool subscribed, bool sensor_and_parent) {
        return subscribed && gate.Frame(now, sensor_and_parent);
    }
};

std::uint64_t Bits(double value) {
    std::uint64_t bits = 0;
    std::memcpy(&bits, &value, sizeof(bits));
    return bits;
}

bool SameBits(double a, double b) {
    return Bits(a) == Bits(b);
}

// Random runs: physics steps of 1 or 4 ms, several rates, world resets. Every
// update must take the same decision and leave the same next scan time, bit
// for bit, as the schedule that was inline in the plugin.
TEST(ScanSchedule, MatchesThePerUpdateGateOnRandomRuns) {
    std::mt19937 random(20261001);
    std::uniform_real_distribution<double> unit(0.0, 1.0);
    const double steps[] = {0.001, 0.004}, rates[] = {10, 20, 30, 7.3, 100};
    unsigned scans = 0, updates = 0, resets = 0;
    for (int run = 0; run < 40; ++run) {
        const double step = steps[run % 2], period = 1.0 / rates[run % 5];
        InlineGate inline_gate{period};
        CurrentGate current{ScanSchedule(period)};
        long tick = 0;
        for (int update = 0; update < 20000; ++update, ++tick) {
            if (unit(random) < 0.0005) {
                tick = unit(random) < 0.5 ? 0 : static_cast<long>(unit(random) * static_cast<double>(tick));
                ++resets;
            }
            const double now = static_cast<double>(tick) * step;
            const bool expected = inline_gate.Frame(now, true);
            ASSERT_EQ(current.Frame(now, true), expected);
            ASSERT_TRUE(SameBits(current.schedule.Next(), inline_gate.next_scan));
            scans += expected ? 1u : 0u;
            ++updates;
        }
    }
    EXPECT_EQ(updates, 800000u);
    EXPECT_TRUE(scans > 10000u);
    EXPECT_TRUE(resets > 100u);
}

// A sensor or parent link that is briefly missing (model teardown) delays
// the scan to the next update in both, while sim time only advances.
TEST(ScanSchedule, MissingParentDelaysTheScanAsBefore) {
    std::mt19937 random(7);
    std::uniform_real_distribution<double> unit(0.0, 1.0);
    InlineGate inline_gate{0.1};
    CurrentGate current{ScanSchedule(0.1)};
    for (long tick = 0; tick < 50000; ++tick) {
        const double now = static_cast<double>(tick) * 0.004;
        const bool present = unit(random) > 0.2;
        ASSERT_EQ(current.Frame(now, present), inline_gate.Frame(now, present));
        ASSERT_TRUE(SameBits(current.schedule.Next(), inline_gate.next_scan));
    }
}

// The per-update cost: at 250 Hz physics and a 10 Hz scan the sensor and
// parent are looked up 10 times per second instead of 250. No update is
// skipped for want of a subscriber: the 601 scans of 15,000 updates are the
// schedule rate (60 s of sim time at 10 Hz, both ends included) with zero
// subscribers.
TEST(ScanSchedule, ScansAtTheScheduleRateWithoutAnySubscriber) {
    CurrentGate current{ScanSchedule(0.1)};
    unsigned scans = 0;
    for (long tick = 0; tick <= 15000; ++tick)
        scans += current.Frame(static_cast<double>(tick) * 0.004, true) ? 1u : 0u;
    EXPECT_EQ(scans, 601u);
    EXPECT_EQ(current.lookups, scans);
}

// Fails if a subscriber check comes back in front of the schedule: the same
// 15,000 updates with zero subscribers must give the scheduled scans, and the
// gate that consults a subscriber count must not. With one subscriber both
// agree, so only the zero-subscriber run tells them apart.
TEST(ScanSchedule, ASubscriberGateScansNothingWhereTheScheduleScansAtRate) {
    CurrentGate current{ScanSchedule(0.1)};
    SubscriberGatedGate without_subscriber{CurrentGate{ScanSchedule(0.1)}};
    SubscriberGatedGate with_subscriber{CurrentGate{ScanSchedule(0.1)}};
    unsigned scheduled = 0, gated_none = 0, gated_one = 0;
    for (long tick = 0; tick <= 15000; ++tick) {
        const double now = static_cast<double>(tick) * 0.004;
        scheduled += current.Frame(now, true) ? 1u : 0u;
        gated_none += without_subscriber.Frame(now, false, true) ? 1u : 0u;
        gated_one += with_subscriber.Frame(now, true, true) ? 1u : 0u;
    }
    EXPECT_EQ(scheduled, 601u);
    EXPECT_EQ(gated_one, scheduled);
    EXPECT_EQ(gated_none, 0u);
}

TEST(ScanSchedule, AResetRestartsTheGridAtTheNextScan) {
    ScanSchedule schedule(0.1);
    EXPECT_TRUE(schedule.Due(0.0));
    schedule.Taken(0.0);
    EXPECT_FALSE(schedule.Due(0.05));
    EXPECT_TRUE(schedule.Due(0.1));
    schedule.Taken(0.1);
    // A late update skips the missed slots instead of bursting.
    EXPECT_TRUE(schedule.Due(0.537));
    schedule.Taken(0.537);
    EXPECT_NEAR(schedule.Next(), 0.6, 1e-12);
    // World reset: sim time runs backwards, the next scan restarts the grid.
    EXPECT_TRUE(schedule.Due(0.012));
    EXPECT_TRUE(schedule.Due(0.016));
    schedule.Taken(0.016);
    EXPECT_NEAR(schedule.Next(), 0.116, 1e-12);
    EXPECT_FALSE(schedule.Due(0.1));
}

} // namespace

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
