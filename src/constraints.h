#ifndef CONSTRAINTS_H
#define CONSTRAINTS_H

#include "grid_planner.h"

enum class ConstraintType {
  Vertex,
  Edge,
};

struct Constraint {
  ConstraintType type;
  int robot_id;
  int time_step;
  Point location;
  Point from;
  Point to;
};

#endif
