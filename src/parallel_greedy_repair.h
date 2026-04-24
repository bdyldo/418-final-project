#ifndef PARALLEL_GREEDY_REPAIR_H
#define PARALLEL_GREEDY_REPAIR_H

#include "a_star.h"
#include "collision_detector.h"
#include "constraints.h"
#include "greedy_repair.h"
#include "grid_planner.h"

#include <optional>
#include <vector>

class ParallelGreedyRepairPlanner {
 public:
  ParallelGreedyRepairPlanner(int rows,
                              int cols,
                              int max_repairs = 10000,
                              int conflict_pool_size = 128,
                              int beam_width = 0,
                              int lookahead_depth = 1);

  std::optional<std::vector<Robot>> findPaths(
      const std::vector<Robot>& robots,
      GreedyRepairStats* stats = nullptr) const;

  int conflictPoolSize() const;
  int beamWidth() const;
  int lookaheadDepth() const;
  int maxWorkerCount() const;

 private:
  int rows_;
  int cols_;
  int max_repairs_;
  int conflict_pool_size_;
  int beam_width_;
  int lookahead_depth_;
  AStarPlanner a_star_;
  CollisionDetector collision_detector_;

  int effectiveBeamWidth() const;
  int totalCost(const std::vector<Robot>& robots) const;
  std::vector<Constraint> repairConstraints(const Collision& collision) const;
};

#endif
