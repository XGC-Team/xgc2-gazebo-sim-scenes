// Minimal ros::Time stand-in for the standalone scan projection test.
#pragma once

#include <cstdint>

namespace ros {

struct Time {
    Time() = default;
    explicit Time(double seconds)
        : sec(static_cast<uint32_t>(seconds)),
          nsec(static_cast<uint32_t>((seconds - static_cast<uint32_t>(seconds)) * 1e9)) {}
    Time(uint32_t seconds, uint32_t nanoseconds) : sec(seconds), nsec(nanoseconds) {}
    bool operator==(const Time& other) const { return sec == other.sec && nsec == other.nsec; }
    uint32_t sec{0}, nsec{0};
};

} // namespace ros
