// A small subset of the GoogleTest API, so scan_projection_test.cpp also
// builds with only g++ and the stand-in headers in test/standalone. The
// catkin build uses the real GoogleTest; this header is never on its path.
#pragma once

#include <cmath>
#include <functional>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

namespace testing {

struct Registry {
    std::vector<std::pair<std::string, std::function<void()>>> tests;
    int failures{0};
    static Registry& Get() {
        static Registry registry;
        return registry;
    }
};

struct Registrar {
    Registrar(const std::string& name, std::function<void()> body) {
        Registry::Get().tests.emplace_back(name, std::move(body));
    }
};

inline void InitGoogleTest(int*, char**) {}

inline bool Report(bool passed, const char* expression, const char* file, int line) {
    if (!passed) {
        ++Registry::Get().failures;
        std::cerr << file << ":" << line << ": expectation failed: " << expression << "\n";
    }
    return passed;
}

} // namespace testing

#define TEST(suite, name)                                                                                              \
    static void suite##_##name##_body();                                                                               \
    static const ::testing::Registrar suite##_##name##_registrar(#suite "." #name, suite##_##name##_body);             \
    static void suite##_##name##_body()

#define XGC2_CHECK(condition, text) ::testing::Report(static_cast<bool>(condition), text, __FILE__, __LINE__)
#define EXPECT_TRUE(value) XGC2_CHECK((value), #value)
#define EXPECT_FALSE(value) XGC2_CHECK(!(value), "!(" #value ")")
#define EXPECT_EQ(a, b) XGC2_CHECK((a) == (b), #a " == " #b)
#define EXPECT_NE(a, b) XGC2_CHECK((a) != (b), #a " != " #b)
#define EXPECT_LT(a, b) XGC2_CHECK((a) < (b), #a " < " #b)
#define EXPECT_LE(a, b) XGC2_CHECK((a) <= (b), #a " <= " #b)
#define EXPECT_NEAR(a, b, tolerance) XGC2_CHECK(std::abs((a) - (b)) <= (tolerance), #a " ~= " #b)
#define EXPECT_THROW(statement, exception)                                                                             \
    do {                                                                                                               \
        bool thrown = false;                                                                                           \
        try {                                                                                                          \
            statement;                                                                                                 \
        } catch (const exception&) {                                                                                   \
            thrown = true;                                                                                             \
        }                                                                                                              \
        XGC2_CHECK(thrown, #statement " throws " #exception);                                                          \
    } while (false)
#define EXPECT_NO_THROW(statement)                                                                                     \
    do {                                                                                                               \
        bool thrown = false;                                                                                           \
        try {                                                                                                          \
            statement;                                                                                                 \
        } catch (...) {                                                                                                \
            thrown = true;                                                                                             \
        }                                                                                                              \
        XGC2_CHECK(!thrown, #statement " does not throw");                                                             \
    } while (false)
#define ASSERT_TRUE(value)                                                                                             \
    if (!EXPECT_TRUE(value))                                                                                           \
    return
#define ASSERT_EQ(a, b)                                                                                                \
    if (!EXPECT_EQ(a, b))                                                                                              \
    return

inline int RUN_ALL_TESTS() {
    auto& registry = ::testing::Registry::Get();
    for (const auto& test : registry.tests) {
        const int before = registry.failures;
        test.second();
        std::cout << (registry.failures == before ? "[ OK ] " : "[FAIL] ") << test.first << "\n";
    }
    std::cout << registry.tests.size() << " tests, " << registry.failures << " failed expectations\n";
    return registry.failures == 0 ? 0 : 1;
}
