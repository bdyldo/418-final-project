#ifndef GRID_PLANNER_H
#define GRID_PLANNER_H

#include <ostream>
#include <string>
#include <vector>

struct Point {
  int row;
  int col;
};

struct Robot {
  int id;
  Point start;
  Point goal;
  std::vector<Point> path;
};

class GridPlanner {
 public:
  GridPlanner(int rows, int cols);

  int rows() const;
  int cols() const;
  const std::vector<Robot>& robots() const;

  void createRowRobots();
  void printRobotSummary(std::ostream& out) const;
  void printPathOverlay(std::ostream& out) const;
  void printTimeSteps(std::ostream& out) const;
  void writeSvg(const std::string& path) const;

 private:
  int rows_;
  int cols_;
  std::vector<Robot> robots_;

  std::vector<Point> makeStraightRightPath(int row) const;
  std::string robotLabel(int id) const;
  void printGridAtTime(std::ostream& out, int time_step) const;
};

#endif
