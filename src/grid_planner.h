#ifndef GRID_PLANNER_H
#define GRID_PLANNER_H

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

 private:
  int rows_;
  int cols_;
  std::vector<Robot> robots_;

  std::vector<Point> makeStraightRightPath(int row) const;
};

#endif
