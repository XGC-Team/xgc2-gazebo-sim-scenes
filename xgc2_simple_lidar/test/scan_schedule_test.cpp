#include "xgc2_simple_lidar/scan_schedule.hpp"
#include <algorithm>
#include <gtest/gtest.h>
#include <string>
#include <vector>

using xgc2_simple_lidar::ScanSchedule;

TEST(ScanSchedule, PreservesEachSensorsRateWithoutFleetBursts) {
    const double period = 0.25, step = 0.004;
    std::vector<ScanSchedule> schedules;
    for (int slot = 1; slot <= 20; ++slot)
        schedules.emplace_back(period, ScanSchedule::SensorPhase(period, "uav" + std::to_string(slot) + "::base_link"));
    std::vector<int> counts(20, 0);
    unsigned peak = 0;
    for (int tick = 0; tick < 2500; ++tick) {
        unsigned frame = 0;
        for (std::size_t i = 0; i < schedules.size(); ++i) {
            const double now = tick * step;
            if (!schedules[i].Due(now))
                continue;
            schedules[i].Taken(now);
            ++counts[i];
            ++frame;
        }
        peak = std::max(peak, frame);
    }
    for (int count : counts)
        EXPECT_EQ(count, 40);
    EXPECT_LE(peak, 3u);
}

TEST(ScanSchedule, LateSensorLoadAndWorldResetKeepIndependentPhases) {
    const double period = 0.25;
    std::vector<ScanSchedule> schedules;
    for (int slot = 1; slot <= 20; ++slot)
        schedules.emplace_back(period, ScanSchedule::SensorPhase(period, "uav" + std::to_string(slot) + "::base_link"));
    for (double start : {37.012, 0.012}) {
        std::vector<int> counts(20, 0);
        unsigned peak = 0;
        for (int tick = 0; tick < 63; ++tick) {
            unsigned frame = 0;
            for (std::size_t i = 0; i < schedules.size(); ++i) {
                const double now = start + tick * 0.004;
                if (!schedules[i].Due(now))
                    continue;
                schedules[i].Taken(now);
                ++counts[i];
                ++frame;
            }
            peak = std::max(peak, frame);
        }
        for (int count : counts)
            EXPECT_EQ(count, 1);
        EXPECT_LE(peak, 3u);
    }
}

TEST(ScanSchedule, MissingParentDefersOneScanAndSkipsMissedPeriods) {
    ScanSchedule schedule(0.1, 0.03);
    EXPECT_FALSE(schedule.Due(0.0));
    EXPECT_FALSE(schedule.Due(0.02));
    EXPECT_TRUE(schedule.Due(0.032));
    // No Taken call while a model is unavailable.
    EXPECT_TRUE(schedule.Due(0.537));
    schedule.Taken(0.537);
    EXPECT_NEAR(schedule.Next(), 0.63, 1e-12);
    EXPECT_FALSE(schedule.Due(0.54));
    EXPECT_TRUE(schedule.Due(0.632));
}

TEST(ScanSchedule, NamesAndPeriodsDeterminePhaseIndependentlyOfLoadOrder) {
    const double phase = ScanSchedule::SensorPhase(0.25, "uav7::base_link");
    EXPECT_GE(phase, 0.0);
    EXPECT_LT(phase, 0.25);
    EXPECT_DOUBLE_EQ(phase * 2, ScanSchedule::SensorPhase(0.5, "uav7::base_link"));
    EXPECT_DOUBLE_EQ(phase, ScanSchedule::SensorPhase(0.25, "uav7::base_link"));
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
