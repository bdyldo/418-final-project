#include "grid_planner.h"

#include <stdexcept>

// Constructs a planner for a grid with the given number of rows and columns.
GridPlanner::GridPlanner(int rows, int cols) : rows_(rows), cols_(cols) {
  if (rows_ <= 0 || cols_ <= 0) {
    throw std::invalid_argument("grid dimensions must be positive");
  }
}

// Returns the number of rows in the grid.
int GridPlanner::rows() const {
  return rows_;
}

// Returns the number of columns in the grid.
int GridPlanner::cols() const {
  return cols_;
}

// Returns the generated robots and their paths.
const std::vector<Robot>& GridPlanner::robots() const {
  return robots_;
}

// Creates one robot specification per row with starts and goals but no path yet.
std::vector<Robot> GridPlanner::createRowRobotSpecs() const {
  std::vector<Robot> robots;
  robots.reserve(rows_);

  for (int row = 0; row < rows_; ++row) {
    robots.push_back({row + 1, {row, 0}, {row, cols_ - 1}, {}});
  }

  return robots;
}

// Creates one robot per row, moving from the left edge to the right edge.
void GridPlanner::createRowRobots() {
  robots_ = createRowRobotSpecs();

  for (Robot& robot : robots_) {
    robot.path = makeStraightRightPath(robot.start.row);
  }
}

// Builds the current placeholder path: a straight horizontal path to the right.
std::vector<Point> GridPlanner::makeStraightRightPath(int row) const {
  std::vector<Point> path;
  path.reserve(cols_);

  for (int col = 0; col < cols_; ++col) {
    path.push_back({row, col});
  }

  return path;
}
