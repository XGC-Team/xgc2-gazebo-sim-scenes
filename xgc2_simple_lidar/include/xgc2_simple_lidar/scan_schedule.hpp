#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>

namespace xgc2_simple_lidar {

// Sim-time schedule of the CPU lidar. Gazebo calls the plugin after every
// world update (250-1000 Hz) and a scan is due at the sensor rate. Due() is
// the only work on the updates in between; whatever else gates a scan (the
// sensor and its parent link still exist) runs only when one is due, and
// Taken() then advances the schedule.
//
// Each sensor keeps its own periodic phase. Spreading those phases avoids
// doing an entire fleet of full ray scans inside the same physics update.
// A world reset retains the phase separation; missed scans never burst.
class ScanSchedule {
  public:
    explicit ScanSchedule(double period, double phase = 0.0)
        : period_(period), phase_(phase), next_(phase) {}

    static double SensorPhase(double period, const std::string& identity) {
        std::uint32_t key = 2166136261u;
        for (const unsigned char byte : identity) { key ^= byte; key *= 16777619u; }
        key ^= key >> 16;
        return period * static_cast<double>(key & 0xffffu) / 65536.0;
    }

    // Call on every ready world update. Newly loaded sensors join their own
    // next phase rather than all scanning immediately after a slow model load.
    bool Due(double now) {
        if (!initialized_ || now < previous_) {
            next_ = std::floor(now / period_) * period_ + phase_;
            if (next_ + 1e-9 < now)
                next_ += period_;
            initialized_ = true;
        }
        previous_ = now;
        return now + 1e-9 >= next_;
    }

    void Taken(double now) {
        const double elapsed = std::max(0.0, now - next_);
        next_ += (std::floor(elapsed / period_) + 1) * period_;
    }

    double Next() const { return next_; }

  private:
    double period_, phase_, next_, previous_{0};
    bool initialized_{false};
};

} // namespace xgc2_simple_lidar
