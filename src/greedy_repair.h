#ifndef GREEDY_REPAIR_H
#define GREEDY_REPAIR_H

#include "a_star.h"
#include "bitset_wavefront.h"
#include "collision_detector.h"
#include "constraints.h"
#include "grid_planner.h"

#include <optional>
#include <vector>

struct GreedyRepairStats {
  long long repair_iterations = 0;
  long long conflicts_considered = 0;
  long long candidate_repairs_evaluated = 0;
  long long low_level_searches = 0;
  long long low_level_states_expanded = 0;
  long long low_level_states_generated = 0;
  long long successful_repairs = 0;
  long long failed_repairs = 0;
  long long stagnant_repairs = 0;
};

enum class LowLevelPlannerKind {
  AStar,
  BitsetWavefront,
};

class GreedyRepairPlanner {
 public:
  GreedyRepairPlanner(int rows,
                      int cols,
                      int max_repairs = 10000,
                      int top_k_conflicts = 32,
                      LowLevelPlannerKind low_level_planner_kind =
                          LowLevelPlannerKind::AStar);

  std::optional<std::vector<Robot>> findPaths(
      const std::vector<Robot>& robots,
      GreedyRepairStats* stats = nullptr) const;

  int topKConflicts() const;
  int maxWorkerCount() const;

 private:
  int rows_;
  int cols_;
  int max_repairs_;
  int top_k_conflicts_;
  LowLevelPlannerKind low_level_planner_kind_;
  AStarPlanner a_star_;
  BitsetWavefrontPlanner bitset_wavefront_;
  CollisionDetector collision_detector_;

  int totalCost(const std::vector<Robot>& robots) const;
  std::optional<std::vector<Point>> findLowLevelPath(
      const Robot& robot,
      const std::vector<Constraint>& constraints,
      AStarStats* stats) const;
  std::vector<Constraint> repairConstraints(const Collision& collision) const;
};

#endif
