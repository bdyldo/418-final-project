#ifndef CBS_H
#define CBS_H

#include "a_star.h"
#include "collision_detector.h"
#include "constraints.h"
#include "grid_planner.h"

#include <optional>
#include <vector>

struct CBSStats {
  long long high_level_nodes_expanded = 0;
  long long low_level_searches = 0;
  long long collisions_resolved = 0;
  long long low_level_states_expanded = 0;
  long long low_level_states_generated = 0;
  long long duplicate_nodes_pruned = 0;
  long long bypasses_applied = 0;
};

struct CBSNode {
  std::vector<Constraint> constraints;
  std::vector<Robot> robots;
  int cost;
};

class CBSPlanner {
 public:
  CBSPlanner(int rows, int cols);

  std::optional<std::vector<Robot>> findPaths(
      const std::vector<Robot>& robots,
      CBSStats* stats = nullptr) const;

 private:
  int rows_;
  int cols_;
  AStarPlanner a_star_;
  CollisionDetector collision_detector_;

  int totalCost(const std::vector<Robot>& robots) const;
  std::vector<Constraint> branchConstraints(const Collision& collision) const;
};

#endif
