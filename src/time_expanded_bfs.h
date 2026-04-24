#ifndef TIME_EXPANDED_BFS_H
#define TIME_EXPANDED_BFS_H

#include "a_star.h"
#include "constraints.h"
#include "grid_planner.h"

#include <optional>
#include <vector>

class TimeExpandedBFSPlanner {
 public:
  TimeExpandedBFSPlanner(int rows, int cols);

  std::optional<std::vector<Point>> findPath(
      const Robot& robot,
      const std::vector<Constraint>& constraints,
      AStarStats* stats = nullptr) const;

 private:
  struct ConstraintIndex;

  int rows_;
  int cols_;

  bool isInBounds(const Point& point) const;
  int manhattanDistance(const Point& lhs, const Point& rhs) const;
  ConstraintIndex buildConstraintIndex(
      int robot_id,
      const std::vector<Constraint>& constraints) const;
  int searchHorizon(const Robot& robot,
                    const ConstraintIndex& constraint_index) const;
  bool violatesVertexConstraint(
      const Point& location,
      int time_step,
      const ConstraintIndex& constraint_index) const;
  bool violatesEdgeConstraint(
      const Point& from,
      const Point& to,
      int time_step,
      const ConstraintIndex& constraint_index) const;
};

#endif
