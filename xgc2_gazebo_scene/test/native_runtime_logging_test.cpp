#include <cstdlib>
#include <fstream>
#include <iterator>
#include <string>
#include <tuple>
#include <vector>
#include <unistd.h>

#include <boost/filesystem.hpp>
#include <gazebo/common/Console.hh>
#include <gtest/gtest.h>

#include "xgc2_gazebo_scene/native_runtime_logging.hpp"

namespace {

class NativeRuntimeLogging : public testing::Test {
  protected:
    void SetUp() override {
        for (const auto* key : {"HOME", "GAZEBO_MASTER_URI", "XGC_GAZEBO_LOG_OUTPUT"})
            previous_.emplace_back(key, std::getenv(key) ? std::getenv(key) : "", std::getenv(key) != nullptr);
        char path[] = "/tmp/xgc-native-log-XXXXXX";
        ASSERT_NE(mkdtemp(path), nullptr);
        root_ = path;
        boost::filesystem::create_directory(root_ / "actual-home");
        boost::filesystem::create_directory_symlink(root_ / "actual-home", root_ / "home-alias");
        setenv("HOME", (root_ / "home-alias").c_str(), 1);
        setenv("GAZEBO_MASTER_URI", "http://127.0.0.1:29345", 1);
        unsetenv("XGC_GAZEBO_LOG_OUTPUT");
        gazebo::common::Console::SetQuiet(true);
        gazebo::common::Console::log.Init("server-", "default.log");
        gazebo::common::Console::log.clear();
        initial_ = boost::filesystem::path(gazebo::common::Console::log.GetLogDirectory()) / "default.log";
    }

    void TearDown() override {
        for (const auto& previous : previous_) {
            if (std::get<2>(previous))
                setenv(std::get<0>(previous).c_str(), std::get<1>(previous).c_str(), 1);
            else
                unsetenv(std::get<0>(previous).c_str());
        }
        boost::filesystem::remove_all(root_);
    }

    static std::string Read(const boost::filesystem::path& path) {
        std::ifstream input(path.string());
        return {std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
    }

    boost::filesystem::path root_, initial_;
    std::vector<std::tuple<std::string, std::string, bool>> previous_;
};

TEST_F(NativeRuntimeLogging, StreamKeepsOnlyTheInitialHeaderAndUsesTheRealNullDevice) {
    const auto header = Read(initial_);
    ASSERT_FALSE(header.empty());
    xgc2_gazebo_scene::ConfigureNativeServerLogging();
    EXPECT_FALSE(gazebo::common::Console::GetQuiet());
    EXPECT_EQ(boost::filesystem::canonical(
                  boost::filesystem::path(gazebo::common::Console::log.GetLogDirectory()) / "null"),
              boost::filesystem::path("/dev/null"));
    testing::internal::CaptureStdout();
    testing::internal::CaptureStderr();
    for (unsigned index = 0; index < 20000; ++index)
        gzwarn << ("bounded-native-message-" + std::to_string(index) + "\n");
    const auto console = testing::internal::GetCapturedStderr() + testing::internal::GetCapturedStdout();
    EXPECT_NE(console.find("bounded-native-message-19999"), std::string::npos);
    EXPECT_EQ(Read(initial_), header);
    EXPECT_TRUE(gazebo::common::Console::log.good());
}

TEST_F(NativeRuntimeLogging, ExplicitFileOutputKeepsNativeDiagnosticOwnership) {
    setenv("XGC_GAZEBO_LOG_OUTPUT", "file", 1);
    xgc2_gazebo_scene::ConfigureNativeServerLogging();
    EXPECT_TRUE(gazebo::common::Console::GetQuiet());
    gzwarn << "explicit-file-diagnostic\n";
    EXPECT_NE(Read(initial_).find("explicit-file-diagnostic"), std::string::npos);
}

TEST_F(NativeRuntimeLogging, UnknownPolicyIsAnExplicitFailure) {
    setenv("XGC_GAZEBO_LOG_OUTPUT", "unknown", 1);
    EXPECT_THROW(xgc2_gazebo_scene::ConfigureNativeServerLogging(), std::runtime_error);
}

} // namespace
