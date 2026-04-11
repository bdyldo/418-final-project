#include "collision_detector.h"
#include "greedy_repair.h"
#include "grid_planner.h"
#include "grid_renderer.h"

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <exception>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#ifdef _OPENMP
#include <omp.h>
#endif

namespace {

enum class Strategy {
  Straight,
  Greedy,
};

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

Strategy parseStrategy(const std::string& value) {
  if (value == "straight") {
    return Strategy::Straight;
  }
  if (value == "greedy") {
    return Strategy::Greedy;
  }
  throw std::invalid_argument("strategy must be 'straight' or 'greedy'");
}

std::string strategyName(Strategy strategy) {
  if (strategy == Strategy::Straight) {
    return "straight";
  }
  return "greedy";
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

void printUsage(const char* program_name) {
  std::cerr << "Usage: " << program_name
            << " [rows] [cols] [output.svg] [--print] [--strategy straight|greedy] [-N threads]\n";
  std::cerr << "Example: " << program_name
            << " 5 10 grid_paths.svg --strategy greedy -N 8\n";
}

}  // namespace

int main(int argc, char** argv) {
  int rows = 5;
  int cols = 10;
  std::string output_path = "grid_paths.svg";
  bool print_terminal_grid = false;
  Strategy strategy = Strategy::Greedy;
  int greedy_top_k_conflicts = 0;
  int greedy_workers = 1;
  std::optional<int> thread_count;

  try {
    int positional_index = 0;
    for (int i = 1; i < argc; ++i) {
      const std::string arg = argv[i];

      if (arg == "--print") {
        print_terminal_grid = true;
      } else if (arg == "--strategy") {
        if (i + 1 >= argc) {
          printUsage(argv[0]);
          return EXIT_FAILURE;
        }
        strategy = parseStrategy(argv[++i]);
      } else if (arg == "-N") {
        if (i + 1 >= argc) {
          printUsage(argv[0]);
          return EXIT_FAILURE;
        }
        thread_count = parsePositiveInt(argv[++i], "thread count");
      } else if (arg.rfind("--", 0) == 0) {
        printUsage(argv[0]);
        return EXIT_FAILURE;
      } else if (positional_index == 0) {
        rows = parsePositiveInt(argv[i], "rows");
        ++positional_index;
      } else if (positional_index == 1) {
        cols = parsePositiveInt(argv[i], "cols");
        ++positional_index;
      } else if (positional_index == 2) {
        output_path = argv[i];
        ++positional_index;
      } else {
        printUsage(argv[0]);
        return EXIT_FAILURE;
      }
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

    GridPlanner planner(rows, cols);
    std::vector<Robot> solved_robots;

    const auto start_time = std::chrono::steady_clock::now();
    if (strategy == Strategy::Straight) {
      planner.createRowRobots();
      solved_robots = planner.robots();
    } else {
      GreedyRepairPlanner greedy_planner(rows, cols);
      greedy_top_k_conflicts = greedy_planner.topKConflicts();
      greedy_workers = greedy_planner.maxWorkerCount();
      const auto solution = greedy_planner.findPaths(planner.createRowRobotSpecs());
      if (!solution.has_value()) {
        std::cerr << "error: greedy repair failed to find a solution for this scenario\n";
        return EXIT_FAILURE;
      }
      solved_robots = *solution;
    }
    const auto end_time = std::chrono::steady_clock::now();
    const auto elapsed_ms = std::chrono::duration<double, std::milli>(
        end_time - start_time);

    GridRenderer renderer(rows, cols, solved_robots);
    renderer.printRobotSummary(std::cout);
    std::cout << "\nStrategy: " << strategyName(strategy) << "\n";
    if (strategy == Strategy::Greedy) {
      std::cout << "Top-K conflicts: " << greedy_top_k_conflicts
                << " | Workers: " << greedy_workers << "\n";
    }
    std::cout << "Planning time: " << std::fixed << std::setprecision(3)
              << elapsed_ms.count() << " ms\n";
    std::cout << "Total time: " << totalTime(solved_robots) << "\n";
    std::cout << "Final robot wait time: " << finalRobotWaitTime(solved_robots) << "\n";
    std::cout << "Final robot detours: " << finalRobotDetourCount(solved_robots) << "\n";

    CollisionDetector collision_detector;
    const std::vector<Collision> collisions =
        collision_detector.detectCollisions(solved_robots);
    collision_detector.printCollisions(collisions, std::cout);

    renderer.writeSvg(output_path);
    std::cout << "\nWrote SVG visualization to " << output_path << "\n";

    if (print_terminal_grid) {
      renderer.printPathOverlay(std::cout);
      renderer.printTimeSteps(std::cout);
    }
  } catch (const std::exception& error) {
    std::cerr << "error: " << error.what() << "\n";
    printUsage(argv[0]);
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
