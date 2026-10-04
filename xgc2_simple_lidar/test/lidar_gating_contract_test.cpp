// Source contract of the simple lidar plugins: nothing about the cloud's
// consumers may decide whether a sensor scans or publishes. Neither plugin
// can be built here without Gazebo and roscpp, so the scan schedule test
// covers the decision logic and this test pins what it cannot see: the
// plugin sources do not look at subscribers, and the sensor model declares
// the sensor always on.
//
// The 1.4.1-2 plugins read the subscriber count in Frame and drove
// SetActive(count > 0) from the connect and disconnect callbacks. The
// negative controls below are those lines: the checks must reject each of
// them, or they would pass on a source in which the gating had returned.
#include <gtest/gtest.h>

#include <fstream>
#include <iostream>
#include <iterator>
#include <regex>
#include <string>
#include <utility>
#include <vector>

#ifndef XGC2_SIMPLE_LIDAR_DIR
#error "XGC2_SIMPLE_LIDAR_DIR must name the xgc2_simple_lidar package directory"
#endif

namespace {

std::string ReadFile(const std::string& relative) {
    std::ifstream stream(std::string(XGC2_SIMPLE_LIDAR_DIR) + "/" + relative, std::ios::binary);
    return std::string(std::istreambuf_iterator<char>(stream), std::istreambuf_iterator<char>());
}

// The source without its comments; string and character literals stay.
std::string StripComments(const std::string& source) {
    enum class Mode { Code, Line, Block, String, Character };
    std::string code;
    Mode mode = Mode::Code;
    for (std::size_t i = 0; i < source.size(); ++i) {
        const char c = source[i], next = i + 1 < source.size() ? source[i + 1] : '\0';
        switch (mode) {
        case Mode::Code:
            if (c == '/' && next == '/') {
                mode = Mode::Line;
                ++i;
            } else if (c == '/' && next == '*') {
                mode = Mode::Block;
                ++i;
            } else {
                mode = c == '"' ? Mode::String : c == '\'' ? Mode::Character : Mode::Code;
                code += c;
            }
            break;
        case Mode::Line:
            if (c == '\n') {
                mode = Mode::Code;
                code += c;
            }
            break;
        case Mode::Block:
            if (c == '*' && next == '/') {
                mode = Mode::Code;
                ++i;
            }
            break;
        case Mode::String:
        case Mode::Character:
            code += c;
            if (c == '\\' && i + 1 < source.size())
                code += source[++i];
            else if ((mode == Mode::String && c == '"') || (mode == Mode::Character && c == '\''))
                mode = Mode::Code;
            break;
        }
    }
    return code;
}

// What a plugin must not contain: any use of roscpp's subscriber API (the
// count, the connect and disconnect callbacks, a subscriber of its own),
// advertise options (their only use here is those callbacks), and a SetActive
// whose argument is computed instead of the literal true or false.
std::vector<std::string> GatingViolations(const std::string& source) {
    static const std::vector<std::pair<std::string, std::regex>> rules = {
        {"a subscriber count, callback or subscriber", std::regex("[Ss]ubscri")},
        {"publisher connect callbacks (AdvertiseOptions)", std::regex("AdvertiseOptions")},
        {"a SetActive argument that is not true or false", std::regex("SetActive\\s*\\(\\s*(?!true\\b|false\\b)\\S")},
    };
    const std::string code = StripComments(source);
    std::vector<std::string> violations;
    for (const auto& rule : rules)
        if (std::regex_search(code, rule.second))
            violations.push_back(rule.first);
    return violations;
}

// The values of the always_on elements of an SDF or xacro text.
std::vector<std::string> AlwaysOnValues(const std::string& text) {
    static const std::regex element("<always_on>\\s*([A-Za-z0-9]*)\\s*</always_on>");
    const std::string code = std::regex_replace(text, std::regex("<!--[\\s\\S]*?-->"), "");
    std::vector<std::string> values;
    for (std::sregex_iterator it(code.begin(), code.end(), element), end; it != end; ++it)
        values.push_back((*it)[1]);
    return values;
}

const char* const kPluginSources[] = {
    "src/gpu_lidar_plugin.cpp",
    "src/cpu_lidar_plugin.cpp",
    "include/xgc2_simple_lidar/scan_schedule.hpp",
    "include/xgc2_simple_lidar/scan_projection.hpp",
};

TEST(LidarGatingContract, PluginSourcesDoNotLookAtSubscribers) {
    for (const char* path : kPluginSources) {
        const std::string source = ReadFile(path);
        ASSERT_TRUE(!source.empty());
        const auto violations = GatingViolations(source);
        for (const auto& violation : violations)
            std::cerr << path << " contains " << violation << "\n";
        EXPECT_TRUE(violations.empty());
    }
}

// The GPU plugin does not rely on the SDF flag either: it activates the
// sensor itself once it is complete, and deactivates it only while loading
// and on teardown.
TEST(LidarGatingContract, TheGpuPluginActivatesItsSensorItself) {
    const std::string code = StripComments(ReadFile("src/gpu_lidar_plugin.cpp"));
    EXPECT_TRUE(std::regex_search(code, std::regex("SetActive\\s*\\(\\s*true\\s*\\)")));
}

TEST(LidarGatingContract, TheSensorModelIsAlwaysOn) {
    for (const char* path : {"models/sensor.xacro", "test/enclosure.world"}) {
        const std::string text = ReadFile(path);
        ASSERT_TRUE(!text.empty());
        const auto values = AlwaysOnValues(text);
        EXPECT_EQ(values.size(), 1u);
        EXPECT_TRUE(values.size() == 1 && values[0] == "true");
    }
}

// Negative controls: the gating lines of 1.4.1-2 and the other ways to
// make a scan depend on a subscriber must be rejected, so the checks above
// cannot pass on a source in which the gating had returned.
TEST(LidarGatingContract, TheChecksRejectTheGatingOf141x2) {
    const char* const gated[] = {
        "current->SetActive(publisher.getNumSubscribers() > 0);",
        "if (!publisher.getNumSubscribers()) return;",
        "subscribed = publisher.getNumSubscribers() > 0;",
        "if (closing || !subscribed) return;",
        "const auto activity = [weak_state](const ros::SingleSubscriberPublisher&) { current->UpdateActivity(); };",
        "auto options = ros::AdvertiseOptions::create<sensor_msgs::PointCloud2>(topic, 1, connect, disconnect);",
        "ros::Subscriber probe = node.subscribe(topic, 1, callback);",
        "sensor->SetActive(count > 0);",
        "sensor->SetActive( wanted() );",
    };
    for (const char* line : gated) {
        std::string source = "void f() {\n";
        source += line;
        source += "\n}\n";
        const auto violations = GatingViolations(source);
        if (violations.empty())
            std::cerr << "not rejected: " << line << "\n";
        EXPECT_FALSE(violations.empty());
    }
    // Comments and literal SetActive calls are not gating.
    const std::string clean = "// getNumSubscribers is not consulted\n/* SetActive(subscribed) */\n"
                              "void f() { gpu->SetActive(false); gpu->SetActive( true ); }\n";
    EXPECT_TRUE(GatingViolations(clean).empty());
    // A comment marker inside a string literal does not hide what follows it.
    EXPECT_FALSE(GatingViolations("auto s = \"//\"; int n = publisher.getNumSubscribers();\n").empty());

    const auto disabled = AlwaysOnValues("<always_on>false</always_on>");
    EXPECT_TRUE(disabled.size() == 1 && disabled[0] == "false");
    EXPECT_TRUE(AlwaysOnValues("<!-- <always_on>false</always_on> -->").empty());
}

} // namespace

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
