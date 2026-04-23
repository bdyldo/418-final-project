#include "grid_renderer.h"

#include <algorithm>
#include <array>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <stdexcept>
#include <vector>

namespace {

constexpr int kCellSize = 28;
constexpr int kMargin = 28;
constexpr int kRobotRadius = 7;

const std::array<const char*, 12> kRobotColors = {
    "#dc2626", "#2563eb", "#16a34a", "#ea580c", "#0891b2", "#9333ea",
    "#be123c", "#4d7c0f", "#0f766e", "#b45309", "#1d4ed8", "#c026d3"};

// Picks a repeatable display color for a robot id.
const char* robotColor(int id) {
  return kRobotColors[(id - 1) % kRobotColors.size()];
}

// Converts a grid column into the x-coordinate of that cell's center.
int cellCenterX(int col) {
  return kMargin + col * kCellSize + kCellSize / 2;
}

// Converts a grid row into the y-coordinate of that cell's center.
int cellCenterY(int row) {
  return kMargin + row * kCellSize + kCellSize / 2;
}

}  // namespace

// Stores grid dimensions and the robot paths that will be rendered.
GridRenderer::GridRenderer(int rows, int cols, const std::vector<Robot>& robots)
    : rows_(rows), cols_(cols), robots_(robots) {}

// Stores a reference to the planner data that will be rendered.
GridRenderer::GridRenderer(const GridPlanner& planner)
    : rows_(planner.rows()),
      cols_(planner.cols()),
      robots_(planner.robots()) {}

// Prints the grid dimensions and start/goal for each robot.
void GridRenderer::printRobotSummary(std::ostream& out) const {
  out << "Grid: " << rows_ << " x " << cols_ << "\n";
  out << "Robots: " << robots_.size() << " (one robot per row)\n\n";

  for (const Robot& robot : robots_) {
    out << robotLabel(robot.id) << ": start=(" << robot.start.row << ","
        << robot.start.col << "), goal=(" << robot.goal.row << ","
        << robot.goal.col << ")\n";
  }
}

// Prints a terminal view of every cell touched by each robot path.
void GridRenderer::printPathOverlay(std::ostream& out) const {
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

// Prints the robot positions at each timestep.
void GridRenderer::printTimeSteps(std::ostream& out) const {
  out << "\nRobot positions over time:\n";

  int max_time_steps = 0;
  for (const Robot& robot : robots_) {
    max_time_steps = std::max(max_time_steps, static_cast<int>(robot.path.size()));
  }

  for (int time_step = 0; time_step < max_time_steps; ++time_step) {
    out << "\nt = " << time_step << "\n";
    printGridAtTime(out, time_step);
  }
}

// Writes an SVG visualization of the grid, robot starts, goals, and paths.
void GridRenderer::writeSvg(const std::string& path) const {
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
    const int start_x = cellCenterX(robot.start.col);
    const int start_y = cellCenterY(robot.start.row);
    const int goal_x = cellCenterX(robot.goal.col);
    const int goal_y = cellCenterY(robot.goal.row);
    const char* color = robotColor(robot.id);

    svg << "  <polyline fill=\"none\" stroke=\"" << color
        << "\" stroke-width=\"4\" stroke-linecap=\"round\" stroke-linejoin=\"round\""
        << " opacity=\"0.78\" points=\"";
    if (!robot.path.empty()) {
      for (const Point& point : robot.path) {
        svg << cellCenterX(point.col) << "," << cellCenterY(point.row) << " ";
      }
    } else {
      svg << start_x << "," << start_y << " "
          << goal_x << "," << goal_y << " ";
    }
    svg << "\"/>\n";

    svg << "  <circle cx=\"" << start_x << "\" cy=\"" << start_y
        << "\" r=\"8\" fill=\"" << color
        << "\" stroke=\"#0f172a\" stroke-width=\"1.5\"/>\n";
    svg << "  <rect x=\"" << goal_x - 8 << "\" y=\"" << goal_y - 8
        << "\" width=\"16\" height=\"16\" fill=\"" << color
        << "\" stroke=\"#0f172a\" stroke-width=\"2\"/>\n";
    svg << "  <text x=\"" << start_x << "\" y=\"" << start_y - 11
        << "\" text-anchor=\"middle\" font-size=\"10\" font-family=\"Arial\""
        << " font-weight=\"700\" fill=\"#0f172a\">R" << robot.id << "</text>\n";
    svg << "  <text x=\"" << goal_x << "\" y=\"" << goal_y + 20
        << "\" text-anchor=\"middle\" font-size=\"10\" font-family=\"Arial\""
        << " font-weight=\"700\" fill=\"#0f172a\">R" << robot.id << "</text>\n";
  }

  svg << "</svg>\n";
}

// Writes an animated SVG that shows the robot positions at each timestep.
void GridRenderer::writeAnimatedSvg(const std::string& path,
                                    double seconds_per_step) const {
  if (seconds_per_step <= 0.0) {
    throw std::invalid_argument("animation seconds per step must be positive");
  }

  std::ofstream svg(path);
  if (!svg) {
    throw std::runtime_error("could not open animated SVG output file: " + path);
  }

  const int width = cols_ * kCellSize + 2 * kMargin;
  const int height = rows_ * kCellSize + 2 * kMargin + 48;
  const int time_steps = std::max(1, maxTimeSteps());
  const double duration_seconds = time_steps * seconds_per_step;

  svg << "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"" << width
      << "\" height=\"" << height << "\" viewBox=\"0 0 " << width << " "
      << height << "\">\n";
  svg << "  <rect width=\"100%\" height=\"100%\" fill=\"#f8fafc\"/>\n";
  svg << "  <text x=\"" << kMargin << "\" y=\"20\" font-size=\"14\" "
      << "font-family=\"Arial\" font-weight=\"700\" fill=\"#0f172a\">"
      << "Robot animation: " << time_steps << " time step(s), "
      << std::fixed << std::setprecision(2) << seconds_per_step
      << "s per step</text>\n";

  for (int row = 0; row <= rows_; ++row) {
    const int y = kMargin + row * kCellSize + 20;
    svg << "  <line x1=\"" << kMargin << "\" y1=\"" << y << "\" x2=\""
        << kMargin + cols_ * kCellSize << "\" y2=\"" << y
        << "\" stroke=\"#cbd5e1\" stroke-width=\"1\"/>\n";
  }

  for (int col = 0; col <= cols_; ++col) {
    const int x = kMargin + col * kCellSize;
    svg << "  <line x1=\"" << x << "\" y1=\"" << kMargin + 20 << "\" x2=\"" << x
        << "\" y2=\"" << kMargin + rows_ * kCellSize + 20
        << "\" stroke=\"#cbd5e1\" stroke-width=\"1\"/>\n";
  }

  for (const Robot& robot : robots_) {
    const char* color = robotColor(robot.id);
    const int start_x = cellCenterX(robot.start.col);
    const int start_y = cellCenterY(robot.start.row) + 20;
    const int goal_x = cellCenterX(robot.goal.col);
    const int goal_y = cellCenterY(robot.goal.row) + 20;

    svg << "  <rect x=\"" << goal_x - 8 << "\" y=\"" << goal_y - 8
        << "\" width=\"16\" height=\"16\" fill=\"none\" stroke=\"" << color
        << "\" stroke-width=\"2\" opacity=\"0.75\"/>\n";
    svg << "  <circle cx=\"" << start_x << "\" cy=\"" << start_y
        << "\" r=\"" << kRobotRadius << "\" fill=\"" << color
        << "\" stroke=\"#0f172a\" stroke-width=\"1.2\">\n";

    std::ostringstream x_values;
    std::ostringstream y_values;
    std::ostringstream key_times;
    x_values << std::fixed << std::setprecision(1);
    y_values << std::fixed << std::setprecision(1);
    key_times << std::fixed << std::setprecision(6);

    for (int time_step = 0; time_step < time_steps; ++time_step) {
      const Point point = robotPositionAtTime(robot, time_step);
      if (time_step > 0) {
        x_values << ";";
        y_values << ";";
        key_times << ";";
      }
      x_values << cellCenterX(point.col);
      y_values << cellCenterY(point.row) + 20;
      key_times << static_cast<double>(time_step) / time_steps;
    }
    x_values << ";" << cellCenterX(robotPositionAtTime(robot, 0).col);
    y_values << ";" << cellCenterY(robotPositionAtTime(robot, 0).row) + 20;
    key_times << ";1";

    svg << "    <animate attributeName=\"cx\" dur=\"" << duration_seconds
        << "s\" repeatCount=\"indefinite\" calcMode=\"linear\" values=\""
        << x_values.str() << "\" keyTimes=\"" << key_times.str()
        << "\"/>\n";
    svg << "    <animate attributeName=\"cy\" dur=\"" << duration_seconds
        << "s\" repeatCount=\"indefinite\" calcMode=\"linear\" values=\""
        << y_values.str() << "\" keyTimes=\"" << key_times.str()
        << "\"/>\n";
    svg << "  </circle>\n";
  }

  svg << "</svg>\n";
}

// Formats a robot id for terminal grid output.
std::string GridRenderer::robotLabel(int id) const {
  return "R" + std::to_string(id);
}

int GridRenderer::maxTimeSteps() const {
  int max_time_steps = 0;
  for (const Robot& robot : robots_) {
    max_time_steps = std::max(max_time_steps, static_cast<int>(robot.path.size()));
  }
  return max_time_steps;
}

Point GridRenderer::robotPositionAtTime(const Robot& robot, int time_step) const {
  if (robot.path.empty()) {
    return robot.start;
  }
  if (time_step < static_cast<int>(robot.path.size())) {
    return robot.path[time_step];
  }
  return robot.path.back();
}

// Prints the grid state for one timestep.
void GridRenderer::printGridAtTime(std::ostream& out, int time_step) const {
  std::vector<std::vector<std::string>> grid(
      rows_, std::vector<std::string>(cols_, "."));

  for (const Robot& robot : robots_) {
    const Point point = robotPositionAtTime(robot, time_step);
    grid[point.row][point.col] = robotLabel(robot.id);
  }

  for (int row = 0; row < rows_; ++row) {
    for (int col = 0; col < cols_; ++col) {
      out << std::setw(4) << grid[row][col];
    }
    out << "\n";
  }
}
