#include "benchmark_cases.h"
#include "bitset_wavefront.h"
#include "collision_detector.h"
#include "greedy_repair.h"
#include "grid_renderer.h"
#include "parallel_greedy_repair.h"
#include "solution_snapshot.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#ifdef _OPENMP
#include <omp.h>
#endif

namespace {

enum class PlannerKind {
  Greedy,
  Parallel,
  Bitset,
  BitsetIndependent,
};

struct ParallelPlannerOptions {
  int conflict_pool_size = 128;
  int beam_width = 0;
  int lookahead_depth = 1;
};

std::string plannerKindName(PlannerKind planner_kind) {
  if (planner_kind == PlannerKind::BitsetIndependent) {
    return "bitset-wavefront-independent";
  }
  if (planner_kind == PlannerKind::Bitset) {
    return "bitset-wavefront-mapf";
  }
  if (planner_kind == PlannerKind::Parallel) {
    return "parallel-speculative";
  }
  return "greedy-batch";
}

std::string plannerArtifactCaseName(const std::string& case_name,
                                    PlannerKind planner_kind) {
  if (planner_kind == PlannerKind::BitsetIndependent) {
    return case_name + "_bitset_independent";
  }
  if (planner_kind == PlannerKind::Bitset) {
    return case_name + "_bitset";
  }
  return case_name;
}

const Robot* finalRobot(const std::vector<Robot>& robots) {
  const Robot* final_robot = nullptr;
  int last_finish_time = -1;

  for (const Robot& robot : robots) {
    const int finish_time = robot.path.empty()
        ? -1
        : static_cast<int>(robot.path.size()) - 1;
    if (finish_time > last_finish_time) {
      last_finish_time = finish_time;
      final_robot = &robot;
    }
  }

  return final_robot;
}

int manhattanDistance(const Point& lhs, const Point& rhs) {
  return std::abs(lhs.row - rhs.row) + std::abs(lhs.col - rhs.col);
}

// Returns the total path length across all robots in a solved benchmark instance.
int totalPathCost(const std::vector<Robot>& robots) {
  int cost = 0;
  for (const Robot& robot : robots) {
    cost += static_cast<int>(robot.path.size());
  }
  return cost;
}

// Returns the timestep when the last robot reaches its goal.
int totalTime(const std::vector<Robot>& robots) {
  int last_finish_time = 0;
  for (const Robot& robot : robots) {
    if (robot.path.empty()) {
      continue;
    }
    last_finish_time = std::max(
        last_finish_time, static_cast<int>(robot.path.size()) - 1);
  }
  return last_finish_time;
}

// Returns the amount of waiting performed by the robot that determines total time.
int finalRobotWaitTime(const std::vector<Robot>& robots) {
  const Robot* final_robot = finalRobot(robots);
  if (final_robot == nullptr) {
    return 0;
  }

  int wait_time = 0;
  for (int i = 1; i < static_cast<int>(final_robot->path.size()); ++i) {
    if (final_robot->path[i] == final_robot->path[i - 1]) {
      ++wait_time;
    }
  }
  return wait_time;
}

// Returns the number of non-wait steps by the last-finishing robot that do not reduce distance to goal.
int finalRobotDetourCount(const std::vector<Robot>& robots) {
  const Robot* final_robot = finalRobot(robots);
  if (final_robot == nullptr) {
    return 0;
  }

  int detour_count = 0;
  for (int i = 1; i < static_cast<int>(final_robot->path.size()); ++i) {
    const Point& previous = final_robot->path[i - 1];
    const Point& current = final_robot->path[i];
    if (current == previous) {
      continue;
    }

    if (manhattanDistance(current, final_robot->goal) >=
        manhattanDistance(previous, final_robot->goal)) {
      ++detour_count;
    }
  }
  return detour_count;
}

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

PlannerKind parsePlannerKind(const std::string& value) {
  if (value == "greedy") {
    return PlannerKind::Greedy;
  }
  if (value == "parallel") {
    return PlannerKind::Parallel;
  }
  if (value == "bitset") {
    return PlannerKind::Bitset;
  }
  if (value == "bitset-independent") {
    return PlannerKind::BitsetIndependent;
  }
  throw std::invalid_argument(
      "planner must be 'greedy', 'parallel', 'bitset', or 'bitset-independent'");
}

int availableWorkerCount() {
#ifdef _OPENMP
  return omp_get_max_threads();
#else
  return 1;
#endif
}

// Runs one benchmark case, verifies the solution, optionally writes SVGs, and prints the timing result.
void runBenchmarkCase(const BenchmarkCase& benchmark_case,
                      const std::optional<std::string>& svg_dir,
                      PlannerKind planner_kind,
                      const ParallelPlannerOptions& parallel_options) {
  CollisionDetector collision_detector;

  const auto start_time = std::chrono::steady_clock::now();
  GreedyRepairStats greedy_stats;
  int workers = 1;
  int top_k_conflicts = 0;
  int conflict_pool_size = 0;
  int beam_width = 0;
  int lookahead_depth = 0;
  int collision_count = 0;
  std::optional<std::vector<Robot>> solution;
  if (planner_kind == PlannerKind::Bitset) {
    GreedyRepairPlanner planner(
        benchmark_case.rows,
        benchmark_case.cols,
        10000,
        32,
        LowLevelPlannerKind::BitsetWavefront);
    workers = planner.maxWorkerCount();
    top_k_conflicts = planner.topKConflicts();
    solution = planner.findPaths(benchmark_case.robots, &greedy_stats);
  } else if (planner_kind == PlannerKind::BitsetIndependent) {
    BitsetWavefrontPlanner planner(benchmark_case.rows, benchmark_case.cols);
    std::vector<Robot> low_level_solution = benchmark_case.robots;
    const int robot_count = static_cast<int>(low_level_solution.size());
    const int available_workers = availableWorkerCount();
    const int robot_workers = std::max(
        1,
        std::min(available_workers, robot_count));
    // Use robot-level parallelism only on very robot-dense cases. Otherwise the
    // per-search row-parallel bitset kernel scales better.
    const bool parallelize_across_robots =
        robot_count >= 4 * benchmark_case.rows;
    workers = parallelize_across_robots ? robot_workers : available_workers;
    std::vector<AStarStats> bitset_robot_stats(robot_count);
    std::atomic<bool> bitset_failed{false};
    std::string bitset_error;

    auto runBitsetRobot = [&](int robot_index) {
      if (bitset_failed.load(std::memory_order_relaxed)) {
        return;
      }

      Robot& robot = low_level_solution[robot_index];
      try {
        const auto path =
            planner.findPath(robot, {}, &bitset_robot_stats[robot_index]);
        if (!path.has_value()) {
#ifdef _OPENMP
#pragma omp critical
#endif
          {
            if (!bitset_failed.load(std::memory_order_relaxed)) {
              bitset_failed.store(true, std::memory_order_relaxed);
              bitset_error =
                  benchmark_case.name +
                  ": bitset-wavefront failed to find a path for robot " +
                  std::to_string(robot.id);
            }
          }
          return;
        }
        robot.path = *path;
      } catch (const std::exception& error) {
#ifdef _OPENMP
#pragma omp critical
#endif
        {
          if (!bitset_failed.load(std::memory_order_relaxed)) {
            bitset_failed.store(true, std::memory_order_relaxed);
            bitset_error =
                benchmark_case.name +
                ": bitset-wavefront threw for robot " +
                std::to_string(robot.id) + ": " + error.what();
          }
        }
      }
    };

    if (parallelize_across_robots) {
#ifdef _OPENMP
#pragma omp parallel for if(robot_workers > 1) num_threads(robot_workers) schedule(static)
#endif
      for (int robot_index = 0; robot_index < robot_count; ++robot_index) {
        runBitsetRobot(robot_index);
      }
    } else {
      for (int robot_index = 0; robot_index < robot_count; ++robot_index) {
        runBitsetRobot(robot_index);
      }
    }

    if (bitset_failed.load(std::memory_order_relaxed)) {
      throw std::runtime_error(bitset_error);
    }

    greedy_stats.low_level_searches = robot_count;
    for (const AStarStats& robot_stats : bitset_robot_stats) {
      greedy_stats.low_level_states_expanded += robot_stats.states_expanded;
      greedy_stats.low_level_states_generated += robot_stats.states_generated;
    }
    solution = std::move(low_level_solution);
  } else if (planner_kind == PlannerKind::Parallel) {
    ParallelGreedyRepairPlanner planner(
        benchmark_case.rows,
        benchmark_case.cols,
        10000,
        parallel_options.conflict_pool_size,
        parallel_options.beam_width,
        parallel_options.lookahead_depth);
    workers = planner.maxWorkerCount();
    conflict_pool_size = planner.conflictPoolSize();
    beam_width = planner.beamWidth();
    lookahead_depth = planner.lookaheadDepth();
    solution = planner.findPaths(benchmark_case.robots, &greedy_stats);
  } else {
    GreedyRepairPlanner planner(benchmark_case.rows, benchmark_case.cols);
    workers = planner.maxWorkerCount();
    top_k_conflicts = planner.topKConflicts();
    solution = planner.findPaths(benchmark_case.robots, &greedy_stats);
  }

  if (!solution.has_value()) {
    throw std::runtime_error(benchmark_case.name +
                             ": " + plannerKindName(planner_kind) +
                             " failed to find a solution");
  }
  const auto end_time = std::chrono::steady_clock::now();

  const std::vector<Collision> collisions =
      collision_detector.detectCollisions(*solution);
  collision_count = static_cast<int>(collisions.size());
  if (planner_kind != PlannerKind::BitsetIndependent && !collisions.empty()) {
    throw std::runtime_error(benchmark_case.name + ": solution still contains collisions");
  }

  const auto elapsed_ms = std::chrono::duration<double, std::milli>(
      end_time - start_time);

  const std::string artifact_case_name =
      plannerArtifactCaseName(benchmark_case.name, planner_kind);
  const std::string solution_snapshot_output =
      defaultSolutionSnapshotPath(artifact_case_name);
  std::filesystem::create_directories(
      std::filesystem::path(solution_snapshot_output).parent_path());
  writeSolutionSnapshot(
      StoredSolution{
          artifact_case_name,
          benchmark_case.rows,
          benchmark_case.cols,
          *solution,
      },
      solution_snapshot_output);

  std::string input_svg_output;
  std::string solution_svg_output;
  std::string animation_svg_output;
  if (svg_dir.has_value()) {
    std::filesystem::create_directories(*svg_dir);
    input_svg_output = *svg_dir + "/" + artifact_case_name + "_input.svg";
    solution_svg_output = *svg_dir + "/" + artifact_case_name + "_solution.svg";
    animation_svg_output = *svg_dir + "/" + artifact_case_name + "_animation.svg";

    GridRenderer input_renderer(
        benchmark_case.rows, benchmark_case.cols, benchmark_case.robots);
    input_renderer.writeSvg(input_svg_output);

    GridRenderer solution_renderer(
        benchmark_case.rows, benchmark_case.cols, *solution);
    solution_renderer.writeSvg(solution_svg_output);
    solution_renderer.writeAnimatedSvg(animation_svg_output);
  }

  std::cout << benchmark_case.name << "\n"
            << "  Grid: " << benchmark_case.rows << "x" << benchmark_case.cols
            << " | Robots: " << benchmark_case.robot_count
            << " | Planner: " << plannerKindName(planner_kind);
  if (planner_kind == PlannerKind::Parallel) {
    std::cout << " | Conflict pool: " << conflict_pool_size
              << " | Beam width: " << beam_width
              << " | Lookahead: " << lookahead_depth;
  } else if (planner_kind == PlannerKind::BitsetIndependent) {
    std::cout << " | Mode: independent-low-level";
  } else if (planner_kind == PlannerKind::Bitset) {
    std::cout << " | Top-K disjoint conflicts: " << top_k_conflicts
              << " | Low-level: bitset-wavefront";
  } else {
    std::cout << " | Top-K disjoint conflicts: " << top_k_conflicts;
  }
  std::cout << " | Workers: " << workers << "\n"
            << "  Total cost: " << totalPathCost(*solution)
            << " | Total time: " << totalTime(*solution)
            << " | Final robot wait time: " << finalRobotWaitTime(*solution)
            << " | Final robot detours: " << finalRobotDetourCount(*solution)
            << " | Runtime: " << std::fixed << std::setprecision(3)
            << elapsed_ms.count() << " ms\n";
  if (planner_kind == PlannerKind::BitsetIndependent) {
    std::cout << "  Independent planning collisions: " << collision_count
              << " | Low-level searches: " << greedy_stats.low_level_searches
              << "\n"
              << "  States expanded: " << greedy_stats.low_level_states_expanded
              << " | States generated: " << greedy_stats.low_level_states_generated;
  } else {
    std::cout << "  Repair iterations: " << greedy_stats.repair_iterations
              << " | Conflicts considered: " << greedy_stats.conflicts_considered
              << " | Candidate repairs: "
              << greedy_stats.candidate_repairs_evaluated << "\n"
              << "  Low-level searches: " << greedy_stats.low_level_searches
              << " | Successful repairs: " << greedy_stats.successful_repairs
              << " | Failed repairs: " << greedy_stats.failed_repairs
              << " | Stagnant repairs: " << greedy_stats.stagnant_repairs << "\n"
              << "  States expanded: " << greedy_stats.low_level_states_expanded
              << " | States generated: " << greedy_stats.low_level_states_generated;
  }
  if (!input_svg_output.empty()) {
    std::cout << "\n"
              << "  Input SVG: " << input_svg_output
              << " | Solution SVG: " << solution_svg_output
              << " | Animation SVG: " << animation_svg_output;
  }
  std::cout << "\n"
            << "  Solution snapshot: " << solution_snapshot_output;
  std::cout << "\n";
}

void printUsage(const char* program_name) {
  std::cerr << "Usage: " << program_name
            << " <few|medium|abundant|few_8|few_16|few_32|small_32|small_64|wide_1024|medium_16|medium_32|medium_64|abundant_64|abundant_128>"
            << " [svg_output_dir] [-N threads] [--planner greedy|parallel|bitset|bitset-independent]"
            << " [--conflict-pool count] [--beam-width count] [--lookahead 1|2]\n";
  std::cerr << "Benchmark case store: " << benchmarkCaseStorePath() << "\n";
}

}  // namespace

int main(int argc, char** argv) {
  try {
    std::optional<std::string> case_name;
    std::optional<std::string> svg_dir;
    std::optional<int> thread_count;
    PlannerKind planner_kind = PlannerKind::Greedy;
    ParallelPlannerOptions parallel_options;

    for (int i = 1; i < argc; ++i) {
      const std::string arg = argv[i];
      if (arg == "-N") {
        if (i + 1 >= argc) {
          printUsage(argv[0]);
          return 1;
        }
        thread_count = parsePositiveInt(argv[++i], "thread count");
      } else if (arg == "--planner") {
        if (i + 1 >= argc) {
          printUsage(argv[0]);
          return 1;
        }
        planner_kind = parsePlannerKind(argv[++i]);
      } else if (arg == "--conflict-pool") {
        if (i + 1 >= argc) {
          printUsage(argv[0]);
          return 1;
        }
        parallel_options.conflict_pool_size =
            parsePositiveInt(argv[++i], "conflict pool size");
      } else if (arg == "--beam-width") {
        if (i + 1 >= argc) {
          printUsage(argv[0]);
          return 1;
        }
        parallel_options.beam_width =
            parsePositiveInt(argv[++i], "beam width");
      } else if (arg == "--lookahead") {
        if (i + 1 >= argc) {
          printUsage(argv[0]);
          return 1;
        }
        parallel_options.lookahead_depth =
            parsePositiveInt(argv[++i], "lookahead depth");
      } else if (!case_name.has_value()) {
        case_name = arg;
      } else if (!svg_dir.has_value()) {
        svg_dir = arg;
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

    const std::vector<BenchmarkCaseDefinition> definitions =
        selectBenchmarkCaseDefinitions(*case_name);

    for (const BenchmarkCaseDefinition& definition : definitions) {
      runBenchmarkCase(
          buildBenchmarkCase(definition), svg_dir, planner_kind, parallel_options);
    }
  } catch (const std::exception& error) {
    std::cerr << "benchmark error: " << error.what() << "\n";
    return 1;
  }

  return 0;
}
