#ifndef GRID_RENDERER_H
#define GRID_RENDERER_H

#include "grid_planner.h"

#include <ostream>
#include <string>

class GridRenderer {
 public:
  GridRenderer(int rows, int cols, const std::vector<Robot>& robots);
  explicit GridRenderer(const GridPlanner& planner);

  void printRobotSummary(std::ostream& out) const;
  void printPathOverlay(std::ostream& out) const;
  void printTimeSteps(std::ostream& out) const;
  void writeSvg(const std::string& path) const;
  void writeAnimatedSvg(const std::string& path,
                        double seconds_per_step = 0.55) const;

 private:
  int rows_;
  int cols_;
  const std::vector<Robot>& robots_;

  std::string robotLabel(int id) const;
  int maxTimeSteps() const;
  Point robotPositionAtTime(const Robot& robot, int time_step) const;
  void printGridAtTime(std::ostream& out, int time_step) const;
};

#endif
