// The SDF parsing the scene adapter does for each obstacle, without Gazebo.
//
// Applying a scene parses every obstacle's SDF twice: CompileScene validates
// it and ApplySceneModelParameters restores its parameters. "new sdf::SDF" is
// what 1.4.1-1 did for each parse: a new sdf::SDF, sdf::init() (the whole SDF
// specification) and sdf::readString(). "parser" is ModelSdfParser, which
// reads the specification once. The last columns are the parsing of one
// application of 500 obstacles (1,000 parses); half of the old time ran with
// the physics update mutex held.
//
// Build and run: test/run_sdf_parser_benchmark.sh
#include "obstacle_sdf.hpp"
#include "xgc2_gazebo_scene/model_sdf_parser.hpp"

#include <chrono>
#include <cstdio>
#include <string>
#include <vector>

namespace {

using Clock = std::chrono::steady_clock;

double Milliseconds(Clock::time_point start, int count) {
    return std::chrono::duration<double, std::milli>(Clock::now() - start).count() / count;
}

} // namespace

int Run() {
    using xgc2_gazebo_scene::ModelSdfParser;
    using xgc2_gazebo_scene::ObstacleSdf;
    const int obstacles = 100;
    int rejected = 0;
    std::printf("%6s | %22s %12s %8s | %26s\n", "parts", "new sdf::SDF ms/parse", "parser ms", "ratio",
                "500 obstacles s (old -> new)");
    for (const int parts : {1, 2, 8, 32}) {
        std::vector<std::string> strings;
        strings.reserve(obstacles);
        for (int index = 0; index < obstacles; ++index)
            strings.push_back(ObstacleSdf(index, parts));

        auto start = Clock::now();
        for (const auto& sdf_string : strings) {
            sdf::SDFPtr parsed(new sdf::SDF());
            sdf::init(parsed);
            rejected += sdf::readString(sdf_string, parsed) ? 0 : 1;
        }
        const double fresh = Milliseconds(start, obstacles);

        ModelSdfParser parser;
        parser.Parse(strings.front()); // the specification, once per application
        start = Clock::now();
        for (const auto& sdf_string : strings)
            rejected += parser.Parse(sdf_string) ? 0 : 1;
        const double reused = Milliseconds(start, obstacles);

        std::printf("%6d | %22.2f %12.2f %7.1fx | %12.1f -> %.2f\n", parts, fresh, reused, fresh / reused,
                    1000 * fresh / 1000.0, (2 * fresh + 1000 * reused) / 1000.0);
    }
    return rejected == 0 ? 0 : 1;
}

int main() {
    try {
        return Run();
    } catch (...) {
        std::fputs("benchmark failed\n", stderr);
        return 2;
    }
}
