#pragma once

#include <cstdlib>
#include <stdexcept>
#include <string>

#include <boost/filesystem.hpp>
#include <gazebo/common/Console.hh>

namespace xgc2_gazebo_scene {

// Called by the first selected native SystemPlugin, before world/transport
// startup. The process supervisor already owns stdout/stderr and its limits.
inline void ConfigureNativeServerLogging() {
    const char* supplied = std::getenv("XGC_GAZEBO_LOG_OUTPUT");
    const std::string output = supplied ? supplied : "stream";
    if (output == "file")
        return; // Explicit native diagnostics retain their caller's owner.
    if (output != "stream")
        throw std::runtime_error("XGC_GAZEBO_LOG_OUTPUT must be stream or file");

    auto& logger = gazebo::common::Console::log;
    const auto directory = logger.GetLogDirectory();
    if (!directory.empty()) {
        // Focal's Boost Filesystem concatenates an absolute rhs in operator/=.
        // FileLogger::Init uses that operator, so an absolute /dev/null would
        // name a file below HOME instead. Resolve from its actual native dir.
        const auto relative = boost::filesystem::path("/dev/null").lexically_relative(
            boost::filesystem::canonical(directory));
        logger.Init("server-", relative.string());
        logger.clear();
        logger << "Native diagnostics use supervised stdout/stderr.\n" << std::flush;
        if (!logger)
            throw std::runtime_error("could not disable the native Gazebo file logger");
    }
    // Without HOME, Gazebo never opened a file. Its console remains usable.
    gazebo::common::Console::SetQuiet(false);
}

} // namespace xgc2_gazebo_scene
