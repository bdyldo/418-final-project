#include "grid_renderer.h"

#include <array>
#include <fstream>
#include <iomanip>
#include <stdexcept>
#include <vector>

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

GridRenderer::GridRenderer(const GridPlanner& planner) : planner_(planner) {}

void GridRenderer::printRobotSummary(std::ostream& out) const {
  out << "Grid: " << planner_.rows() << " x " << planner_.cols() << "\n";
  out << "Robots: " << planner_.robots().size() << " (one robot per row)\n\n";

  for (const Robot& robot : planner_.robots()) {
    out << robotLabel(robot.id) << ": start=(" << robot.start.row << ","
        << robot.start.col << "), goal=(" << robot.goal.row << ","
        << robot.goal.col << ")\n";
  }
}

void GridRenderer::printPathOverlay(std::ostream& out) const {
  out << "\nFull path overlay:\n";

  std::vector<std::vector<std::string>> grid(
      planner_.rows(), std::vector<std::string>(planner_.cols(), "."));

  for (const Robot& robot : planner_.robots()) {
    for (const Point& point : robot.path) {
      grid[point.row][point.col] = robotLabel(robot.id);
    }
  }

  for (int row = 0; row < planner_.rows(); ++row) {
    for (int col = 0; col < planner_.cols(); ++col) {
      out << std::setw(4) << grid[row][col];
    }
    out << "\n";
  }
}

void GridRenderer::printTimeSteps(std::ostream& out) const {
  out << "\nRobot positions over time:\n";

  for (int time_step = 0; time_step < planner_.cols(); ++time_step) {
    out << "\nt = " << time_step << "\n";
    printGridAtTime(out, time_step);
  }
}

void GridRenderer::writeSvg(const std::string& path) const {
  std::ofstream svg(path);
  if (!svg) {
    throw std::runtime_error("could not open SVG output file: " + path);
  }

  const int width = planner_.cols() * kCellSize + 2 * kMargin;
  const int height = planner_.rows() * kCellSize + 2 * kMargin;

  svg << "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"" << width
      << "\" height=\"" << height << "\" viewBox=\"0 0 " << width << " "
      << height << "\">\n";
  svg << "  <rect width=\"100%\" height=\"100%\" fill=\"#f8fafc\"/>\n";

  for (int row = 0; row <= planner_.rows(); ++row) {
    const int y = kMargin + row * kCellSize;
    svg << "  <line x1=\"" << kMargin << "\" y1=\"" << y << "\" x2=\""
        << kMargin + planner_.cols() * kCellSize << "\" y2=\"" << y
        << "\" stroke=\"#cbd5e1\" stroke-width=\"1\"/>\n";
  }

  for (int col = 0; col <= planner_.cols(); ++col) {
    const int x = kMargin + col * kCellSize;
    svg << "  <line x1=\"" << x << "\" y1=\"" << kMargin << "\" x2=\"" << x
        << "\" y2=\"" << kMargin + planner_.rows() * kCellSize
        << "\" stroke=\"#cbd5e1\" stroke-width=\"1\"/>\n";
  }

  for (const Robot& robot : planner_.robots()) {
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

std::string GridRenderer::robotLabel(int id) const {
  return "R" + std::to_string(id);
}

void GridRenderer::printGridAtTime(std::ostream& out, int time_step) const {
  std::vector<std::vector<std::string>> grid(
      planner_.rows(), std::vector<std::string>(planner_.cols(), "."));

  for (const Robot& robot : planner_.robots()) {
    const Point& point = robot.path[time_step];
    grid[point.row][point.col] = robotLabel(robot.id);
  }

  for (int row = 0; row < planner_.rows(); ++row) {
    for (int col = 0; col < planner_.cols(); ++col) {
      out << std::setw(4) << grid[row][col];
    }
    out << "\n";
  }
}
