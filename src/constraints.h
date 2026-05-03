#ifndef CONSTRAINTS_H
#define CONSTRAINTS_H

#include "grid_planner.h"

enum class ConstraintType {
  Vertex, // Two vertices landing on the same grid at the smae time
  Edge, // When two robots switch positions in both vertices at one time. 
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
