#pragma once

#include <algorithm>
#include <cmath>

namespace xgc2_simple_lidar {

// Sim-time schedule of the CPU lidar. Gazebo calls the plugin after every
// world update (250-1000 Hz) and a scan is due at the sensor rate. Due() is
// the only work on the updates in between; whatever else gates a scan (the
// sensor and its parent link still exist) runs only when one is due, and
// Taken() then advances the schedule.
//
// Scans stay on the grid k * period anchored at sim time 0. When sim time
// runs backwards (a world reset), the next scan taken restarts the grid at
// its own time. This is the schedule the plugin kept inline before; Due and
// Taken only split it so the scan gates are not evaluated on every update.
class ScanSchedule {
  public:
    explicit ScanSchedule(double period) : period_(period) {}

    // Call on every world update while the cloud has subscribers.
    bool Due(double now) {
        if (now < previous_)
            reset_ = true;
        previous_ = now;
        return reset_ || now + 1e-9 >= next_;
    }

    // Call when a due scan is taken at `now`.
    void Taken(double now) {
        if (reset_) {
            next_ = now;
            reset_ = false;
        }
        const double elapsed = std::max(0.0, now - next_);
        next_ += (std::floor(elapsed / period_) + 1) * period_;
    }

    double Next() const { return next_; }

  private:
    double period_, next_{0}, previous_{0};
    bool reset_{false};
};

} // namespace xgc2_simple_lidar
