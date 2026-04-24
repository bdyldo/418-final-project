#include "benchmark_cases.h"
#include "collision_detector.h"
#include "greedy_repair.h"
#include "grid_renderer.h"
#include "solution_snapshot.h"

#include <filesystem>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <string>

#ifdef _OPENMP
#include <omp.h>
#endif

namespace {

int parsePositiveInt(const char* value, const std::string& name) {
  try {
    size_t parsed_chars = 0;
    const int parsed_value = std::stoi(value, &parsed_chars);
    if (parsed_chars != std::string(value).size() || parsed_value <= 0) {
      throw std::invalid_argument("not a positive integer");
    }
    return parsed_value;
  } catch (const std::exception&) {
    throw std::invalid_argument(name + " must be a positive integer");
  }
}

void printUsage(const char* program_name) {
  std::cerr << "Usage: " << program_name
            << " <few_8|few_16|few_32|small_32|small_64|wide_1024|medium_16|medium_32|medium_64|abundant_64|abundant_128>"
            << " [output.svg] [--force-resolve] [-N threads]\n";
}

StoredSolution solveCase(const std::string& case_name) {
  const std::vector<BenchmarkCaseDefinition> definitions =
      selectBenchmarkCaseDefinitions(case_name);
  if (definitions.size() != 1) {
    throw std::invalid_argument("animation tool requires one exact benchmark case");
  }

  const BenchmarkCase benchmark_case = buildBenchmarkCase(definitions.front());
  GreedyRepairStats stats;
  GreedyRepairPlanner planner(benchmark_case.rows, benchmark_case.cols);
  const auto solution = planner.findPaths(benchmark_case.robots, &stats);
  if (!solution.has_value()) {
    throw std::runtime_error(case_name +
                             ": greedy repair failed to find a solution");
  }

  CollisionDetector collision_detector;
  const std::vector<Collision> collisions =
      collision_detector.detectCollisions(*solution);
  if (!collisions.empty()) {
    throw std::runtime_error(case_name + ": solution still contains collisions");
  }

  return StoredSolution{
      benchmark_case.name,
      benchmark_case.rows,
      benchmark_case.cols,
      *solution,
  };
}

void requireCollisionFree(const StoredSolution& stored_solution,
                          const std::string& source_label) {
  CollisionDetector collision_detector;
  const std::vector<Collision> collisions =
      collision_detector.detectCollisions(stored_solution.robots);
  if (!collisions.empty()) {
    throw std::runtime_error(
        source_label + " contains " +
        std::to_string(collisions.size()) +
        " collision(s); rerun the case with a collision-free planner");
  }
}

}  // namespace

int main(int argc, char** argv) {
  try {
    std::optional<std::string> case_name;
    std::optional<std::string> output_path;
    std::optional<int> thread_count;
    bool force_resolve = false;

    for (int i = 1; i < argc; ++i) {
      const std::string arg = argv[i];
      if (arg == "--force-resolve") {
        force_resolve = true;
      } else if (arg == "-N") {
        if (i + 1 >= argc) {
          printUsage(argv[0]);
          return 1;
        }
        thread_count = parsePositiveInt(argv[++i], "thread count");
      } else if (!case_name.has_value()) {
        case_name = arg;
      } else if (!output_path.has_value()) {
        output_path = arg;
      } else {
        printUsage(argv[0]);
        return 1;
      }
    }

    if (!case_name.has_value()) {
      printUsage(argv[0]);
      return 1;
    }

#ifdef _OPENMP
    if (thread_count.has_value()) {
      omp_set_num_threads(*thread_count);
    }
#else
    if (thread_count.has_value()) {
      std::cerr << "warning: OpenMP is not enabled, ignoring -N "
                << *thread_count << "\n";
    }
#endif

    const std::string snapshot_path = defaultSolutionSnapshotPath(*case_name);
    StoredSolution stored_solution;

    if (!force_resolve && std::filesystem::exists(snapshot_path)) {
      stored_solution = readSolutionSnapshot(snapshot_path);
      requireCollisionFree(stored_solution, snapshot_path);
      std::cout << "Loaded saved solution snapshot: " << snapshot_path << "\n";
    } else {
      stored_solution = solveCase(*case_name);
      requireCollisionFree(stored_solution, *case_name);
      std::filesystem::create_directories(
          std::filesystem::path(snapshot_path).parent_path());
      writeSolutionSnapshot(stored_solution, snapshot_path);
      std::cout << "Solved benchmark case and saved snapshot: " << snapshot_path
                << "\n";
    }

    const std::string final_output_path = output_path.has_value()
        ? *output_path
        : "benchmark_svgs/" + stored_solution.case_name + "/" +
              stored_solution.case_name + "_animation.svg";
    std::filesystem::create_directories(
        std::filesystem::path(final_output_path).parent_path());

    GridRenderer renderer(
        stored_solution.rows, stored_solution.cols, stored_solution.robots);
    renderer.writeAnimatedSvg(final_output_path);

    std::cout << "Animated SVG: " << final_output_path << "\n";
  } catch (const std::exception& error) {
    std::cerr << "animation error: " << error.what() << "\n";
    return 1;
  }

  return 0;
}
