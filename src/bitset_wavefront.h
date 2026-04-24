#ifndef BITSET_WAVEFRONT_H
#define BITSET_WAVEFRONT_H

#include "a_star.h"
#include "constraints.h"
#include "grid_planner.h"

#include <optional>
#include <vector>

class BitsetWavefrontPlanner {
 public:
  BitsetWavefrontPlanner(int rows, int cols);

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
      const std::vector<Constraint>& constraints,
      int horizon) const;
  int searchHorizon(const Robot& robot,
                    const ConstraintIndex& constraint_index) const;
};

#endif
