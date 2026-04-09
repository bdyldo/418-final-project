#include "cbs.h"
#include "collision_detector.h"
#include "grid_planner.h"
#include "grid_renderer.h"

#include <chrono>
#include <cstdlib>
#include <exception>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

enum class Strategy {
  Straight,
  CBS,
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
  if (value == "cbs") {
    return Strategy::CBS;
  }
  throw std::invalid_argument("strategy must be 'straight' or 'cbs'");
}

std::string strategyName(Strategy strategy) {
  return strategy == Strategy::Straight ? "straight" : "cbs";
}

void printUsage(const char* program_name) {
  std::cerr << "Usage: " << program_name
            << " [rows] [cols] [output.svg] [--print] [--strategy straight|cbs]\n";
  std::cerr << "Example: " << program_name
            << " 5 10 grid_paths.svg --strategy cbs\n";
}

}  // namespace

int main(int argc, char** argv) {
  int rows = 5;
  int cols = 10;
  std::string output_path = "grid_paths.svg";
  bool print_terminal_grid = false;
  Strategy strategy = Strategy::CBS;

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

    GridPlanner planner(rows, cols);
    std::vector<Robot> solved_robots;

    const auto start_time = std::chrono::steady_clock::now();
    if (strategy == Strategy::Straight) {
      planner.createRowRobots();
      solved_robots = planner.robots();
    } else {
      CBSPlanner cbs_planner(rows, cols);
      const auto solution = cbs_planner.findPaths(planner.createRowRobotSpecs());
      if (!solution.has_value()) {
        std::cerr << "error: CBS failed to find a solution for this scenario\n";
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
    std::cout << "Planning time: " << std::fixed << std::setprecision(3)
              << elapsed_ms.count() << " ms\n";

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
