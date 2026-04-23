#include "solution_snapshot.h"

#include <fstream>
#include <stdexcept>
#include <string>

namespace {

void requireToken(const std::string& actual, const std::string& expected) {
  if (actual != expected) {
    throw std::runtime_error("expected token '" + expected + "', found '" +
                             actual + "'");
  }
}

}  // namespace

std::string defaultSolutionSnapshotPath(const std::string& case_name) {
  return "benchmark_solutions/" + case_name + "_solution.txt";
}

void writeSolutionSnapshot(const StoredSolution& solution,
                           const std::string& path) {
  std::ofstream output(path);
  if (!output) {
    throw std::runtime_error("could not open solution snapshot for writing: " +
                             path);
  }

  output << "CASE " << solution.case_name << "\n";
  output << "GRID " << solution.rows << " " << solution.cols << "\n";
  output << "ROBOTS " << solution.robots.size() << "\n";

  for (const Robot& robot : solution.robots) {
    output << "ROBOT " << robot.id << " "
           << robot.start.row << " " << robot.start.col << " "
           << robot.goal.row << " " << robot.goal.col << " "
           << robot.path.size() << "\n";
    output << "PATH";
    for (const Point& point : robot.path) {
      output << " " << point.row << " " << point.col;
    }
    output << "\n";
  }
}

StoredSolution readSolutionSnapshot(const std::string& path) {
  std::ifstream input(path);
  if (!input) {
    throw std::runtime_error("could not open solution snapshot: " + path);
  }

  StoredSolution solution;
  std::string token;

  input >> token;
  requireToken(token, "CASE");
  input >> solution.case_name;

  input >> token;
  requireToken(token, "GRID");
  input >> solution.rows >> solution.cols;

  size_t robot_count = 0;
  input >> token;
  requireToken(token, "ROBOTS");
  input >> robot_count;

  if (!input || solution.rows <= 0 || solution.cols <= 0) {
    throw std::runtime_error("solution snapshot header is malformed: " + path);
  }

  solution.robots.clear();
  solution.robots.reserve(robot_count);

  for (size_t i = 0; i < robot_count; ++i) {
    Robot robot;
    size_t path_length = 0;

    input >> token;
    requireToken(token, "ROBOT");
    input >> robot.id
          >> robot.start.row >> robot.start.col
          >> robot.goal.row >> robot.goal.col
          >> path_length;

    input >> token;
    requireToken(token, "PATH");

    robot.path.clear();
    robot.path.reserve(path_length);
    for (size_t step = 0; step < path_length; ++step) {
      Point point;
      input >> point.row >> point.col;
      robot.path.push_back(point);
    }

    if (!input) {
      throw std::runtime_error("solution snapshot robot entry is malformed: " +
                               path);
    }

    solution.robots.push_back(robot);
  }

  return solution;
}
