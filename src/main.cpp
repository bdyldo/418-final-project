#include "grid_planner.h"
#include "grid_renderer.h"

#include <cstdlib>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <string>

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
  std::cerr << "Usage: " << program_name << " [rows] [cols] [output.svg] [--print]\n";
  std::cerr << "Example: " << program_name << " 5 10 grid_paths.svg\n";
}

}  // namespace

int main(int argc, char** argv) {
  int rows = 5;
  int cols = 10;
  std::string output_path = "grid_paths.svg";
  bool print_terminal_grid = false;

  try {
    if (argc > 5) {
      printUsage(argv[0]);
      return EXIT_FAILURE;
    }
    if (argc >= 2) {
      rows = parsePositiveInt(argv[1], "rows");
    }
    if (argc >= 3) {
      cols = parsePositiveInt(argv[2], "cols");
    }
    if (argc >= 4) {
      output_path = argv[3];
    }
    if (argc == 5) {
      if (std::string(argv[4]) != "--print") {
        printUsage(argv[0]);
        return EXIT_FAILURE;
      }
      print_terminal_grid = true;
    }

    GridPlanner planner(rows, cols);
    planner.createRowRobots();

    GridRenderer renderer(planner);
    renderer.printRobotSummary(std::cout);
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
