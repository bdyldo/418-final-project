#include "grid_planner.h"

#include <array>
#include <fstream>
#include <iomanip>
#include <stdexcept>

namespace {

constexpr int kCellSize = 28;
constexpr int kMargin = 28;

const std::array<const char*, 12> kRobotColors = {
    "#dc2626", "#2563eb", "#16a34a", "#ea580c", "#0891b2", "#9333ea",
    "#be123c", "#4d7c0f", "#0f766e", "#b45309", "#1d4ed8", "#c026d3"};

const char* robotColor(int id) {
  return kRobotColors[(id - 1) % kRobotColors.size()];
}

int cellCenterX(int col) {
  return kMargin + col * kCellSize + kCellSize / 2;
}

int cellCenterY(int row) {
  return kMargin + row * kCellSize + kCellSize / 2;
}

}  // namespace

GridPlanner::GridPlanner(int rows, int cols) : rows_(rows), cols_(cols) {
  if (rows_ <= 0 || cols_ <= 0) {
    throw std::invalid_argument("grid dimensions must be positive");
  }
}

int GridPlanner::rows() const {
  return rows_;
}

int GridPlanner::cols() const {
  return cols_;
}

const std::vector<Robot>& GridPlanner::robots() const {
  return robots_;
}

void GridPlanner::createRowRobots() {
  robots_.clear();
  robots_.reserve(rows_);

  for (int row = 0; row < rows_; ++row) {
    Robot robot;
    robot.id = row + 1;
    robot.start = {row, 0};
    robot.goal = {row, cols_ - 1};
    robot.path = makeStraightRightPath(row);
    robots_.push_back(robot);
  }
}

void GridPlanner::printRobotSummary(std::ostream& out) const {
  out << "Grid: " << rows_ << " x " << cols_ << "\n";
  out << "Robots: " << robots_.size() << " (one robot per row)\n\n";

  for (const Robot& robot : robots_) {
    out << robotLabel(robot.id) << ": start=(" << robot.start.row << ","
        << robot.start.col << "), goal=(" << robot.goal.row << ","
        << robot.goal.col << ")\n";
  }
}

void GridPlanner::printPathOverlay(std::ostream& out) const {
  out << "\nFull path overlay:\n";

  std::vector<std::vector<std::string>> grid(
      rows_, std::vector<std::string>(cols_, "."));

  for (const Robot& robot : robots_) {
    for (const Point& point : robot.path) {
      grid[point.row][point.col] = robotLabel(robot.id);
    }
  }

  for (int row = 0; row < rows_; ++row) {
    for (int col = 0; col < cols_; ++col) {
      out << std::setw(4) << grid[row][col];
    }
    out << "\n";
  }
}

void GridPlanner::printTimeSteps(std::ostream& out) const {
  out << "\nRobot positions over time:\n";

  for (int time_step = 0; time_step < cols_; ++time_step) {
    out << "\nt = " << time_step << "\n";
    printGridAtTime(out, time_step);
  }
}

void GridPlanner::writeSvg(const std::string& path) const {
  std::ofstream svg(path);
  if (!svg) {
    throw std::runtime_error("could not open SVG output file: " + path);
  }

  const int width = cols_ * kCellSize + 2 * kMargin;
  const int height = rows_ * kCellSize + 2 * kMargin;

  svg << "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"" << width
      << "\" height=\"" << height << "\" viewBox=\"0 0 " << width << " "
      << height << "\">\n";
  svg << "  <rect width=\"100%\" height=\"100%\" fill=\"#f8fafc\"/>\n";

  for (int row = 0; row <= rows_; ++row) {
    const int y = kMargin + row * kCellSize;
    svg << "  <line x1=\"" << kMargin << "\" y1=\"" << y << "\" x2=\""
        << kMargin + cols_ * kCellSize << "\" y2=\"" << y
        << "\" stroke=\"#cbd5e1\" stroke-width=\"1\"/>\n";
  }

  for (int col = 0; col <= cols_; ++col) {
    const int x = kMargin + col * kCellSize;
    svg << "  <line x1=\"" << x << "\" y1=\"" << kMargin << "\" x2=\"" << x
        << "\" y2=\"" << kMargin + rows_ * kCellSize
        << "\" stroke=\"#cbd5e1\" stroke-width=\"1\"/>\n";
  }

  for (const Robot& robot : robots_) {
    const int y = cellCenterY(robot.start.row);
    const int start_x = cellCenterX(robot.start.col);
    const int goal_x = cellCenterX(robot.goal.col);
    const char* color = robotColor(robot.id);

    svg << "  <line x1=\"" << start_x << "\" y1=\"" << y << "\" x2=\""
        << goal_x << "\" y2=\"" << y << "\" stroke=\"" << color
        << "\" stroke-width=\"5\" stroke-linecap=\"round\" opacity=\"0.78\"/>\n";
    svg << "  <circle cx=\"" << start_x << "\" cy=\"" << y
        << "\" r=\"8\" fill=\"" << color << "\"/>\n";
    svg << "  <rect x=\"" << goal_x - 8 << "\" y=\"" << y - 8
        << "\" width=\"16\" height=\"16\" fill=\"" << color
        << "\" stroke=\"#0f172a\" stroke-width=\"2\"/>\n";
    svg << "  <text x=\"" << start_x << "\" y=\"" << y + 4
        << "\" text-anchor=\"middle\" font-size=\"9\" font-family=\"Arial\""
        << " font-weight=\"700\" fill=\"#ffffff\">" << robot.id << "</text>\n";
  }

  svg << "</svg>\n";
}

std::vector<Point> GridPlanner::makeStraightRightPath(int row) const {
  std::vector<Point> path;
  path.reserve(cols_);

  for (int col = 0; col < cols_; ++col) {
    path.push_back({row, col});
  }

  return path;
}

std::string GridPlanner::robotLabel(int id) const {
  return "R" + std::to_string(id);
}

void GridPlanner::printGridAtTime(std::ostream& out, int time_step) const {
  std::vector<std::vector<std::string>> grid(
      rows_, std::vector<std::string>(cols_, "."));

  for (const Robot& robot : robots_) {
    const Point& point = robot.path[time_step];
    grid[point.row][point.col] = robotLabel(robot.id);
  }

  for (int row = 0; row < rows_; ++row) {
    for (int col = 0; col < cols_; ++col) {
      out << std::setw(4) << grid[row][col];
    }
    out << "\n";
  }
}
