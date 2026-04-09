#include "benchmark_cases.h"
#include "cbs.h"
#include "collision_detector.h"
#include "grid_renderer.h"

#include <chrono>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

// Returns the total path length across all robots in a solved benchmark instance.
int totalPathCost(const std::vector<Robot>& robots) {
  int cost = 0;
  for (const Robot& robot : robots) {
    cost += static_cast<int>(robot.path.size());
  }
  return cost;
}

// Runs one benchmark case, verifies the solution, optionally writes SVGs, and prints the timing result.
void runBenchmarkCase(const BenchmarkCase& benchmark_case,
                      const std::optional<std::string>& svg_dir) {
  CBSPlanner planner(benchmark_case.rows, benchmark_case.cols);
  CollisionDetector collision_detector;
  CBSStats stats;

  const auto start_time = std::chrono::steady_clock::now();
  const auto solution = planner.findPaths(benchmark_case.robots, &stats);
  const auto end_time = std::chrono::steady_clock::now();

  if (!solution.has_value()) {
    throw std::runtime_error(benchmark_case.name + ": CBS failed to find a solution");
  }

  const std::vector<Collision> collisions =
      collision_detector.detectCollisions(*solution);
  if (!collisions.empty()) {
    throw std::runtime_error(benchmark_case.name + ": solution still contains collisions");
  }

  const auto elapsed_ms = std::chrono::duration<double, std::milli>(
      end_time - start_time);

  std::string input_svg_output;
  std::string solution_svg_output;
  if (svg_dir.has_value()) {
    std::filesystem::create_directories(*svg_dir);
    input_svg_output = *svg_dir + "/" + benchmark_case.name + "_input.svg";
    solution_svg_output = *svg_dir + "/" + benchmark_case.name + "_solution.svg";

    GridRenderer input_renderer(
        benchmark_case.rows, benchmark_case.cols, benchmark_case.robots);
    input_renderer.writeSvg(input_svg_output);

    GridRenderer solution_renderer(
        benchmark_case.rows, benchmark_case.cols, *solution);
    solution_renderer.writeSvg(solution_svg_output);
  }

  std::cout << benchmark_case.name
            << " | grid=" << benchmark_case.rows << "x" << benchmark_case.cols
            << " | robots=" << benchmark_case.robot_count
            << " | total_cost=" << totalPathCost(*solution)
            << " | time_ms=" << std::fixed << std::setprecision(3)
            << elapsed_ms.count()
            << " | cbs_nodes=" << stats.high_level_nodes_expanded
            << " | a_star_calls=" << stats.low_level_searches
            << " | collisions=" << stats.collisions_resolved
            << " | duplicates=" << stats.duplicate_nodes_pruned
            << " | bypasses=" << stats.bypasses_applied
            << " | states_expanded=" << stats.low_level_states_expanded
            << " | states_generated=" << stats.low_level_states_generated;
  if (!input_svg_output.empty()) {
    std::cout << " | input_svg=" << input_svg_output
              << " | solution_svg=" << solution_svg_output;
  }
  std::cout << "\n";
}

void printUsage(const char* program_name) {
  std::cerr << "Usage: " << program_name
            << " <few|medium|abundant|few_8|few_16|medium_16|medium_32|abundant_64|abundant_128>"
            << " [svg_output_dir]\n";
  std::cerr << "Benchmark case store: " << benchmarkCaseStorePath() << "\n";
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 2 && argc != 3) {
    printUsage(argv[0]);
    return 1;
  }

  try {
    const std::vector<BenchmarkCaseDefinition> definitions =
        selectBenchmarkCaseDefinitions(argv[1]);
    const std::optional<std::string> svg_dir =
        argc == 3 ? std::optional<std::string>(argv[2]) : std::nullopt;

    for (const BenchmarkCaseDefinition& definition : definitions) {
      runBenchmarkCase(buildBenchmarkCase(definition), svg_dir);
    }
  } catch (const std::exception& error) {
    std::cerr << "benchmark error: " << error.what() << "\n";
    return 1;
  }

  return 0;
}
