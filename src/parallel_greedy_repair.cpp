/*
 * ParallelGreedyRepairPlanner is the project’s high-throughput MAPF variant of
 * greedy repair. It generates speculative repair candidates for a pool of
 * disjoint conflicts, evaluates them concurrently, optionally performs one-step
 * lookahead, and merges compatible improvements with beam-style selection. The
 * goal is to reduce wall-clock time and improve repair quality by exploiting
 * candidate-level parallelism while preserving the same collision-reduction
 * objective (primary: fewer collisions, secondary: lower total path cost).
 */
#include "parallel_greedy_repair.h"

#include <algorithm>
#include <cstddef>
#include <optional>
#include <stdexcept>
#include <tuple>
#include <unordered_set>
#include <utility>
#include <vector>

#ifdef _OPENMP
#include <omp.h>
#endif

namespace {

struct SpeculativeTask {
  int conflict_index;
  Constraint first_constraint;
};

struct LookaheadTask {
  int parent_candidate_index;
  Constraint second_constraint;
};

struct SpeculativeCandidate {
  int conflict_index;
  std::vector<Constraint> new_constraints;
  std::vector<Robot> repaired_robots;
  int collision_count;
  int total_cost;
};

bool betterMetrics(int lhs_collision_count,
                   int lhs_total_cost,
                   int rhs_collision_count,
                   int rhs_total_cost) {
  return std::tie(lhs_collision_count, lhs_total_cost) <
      std::tie(rhs_collision_count, rhs_total_cost);
}

bool betterCandidate(const SpeculativeCandidate& lhs,
                     const SpeculativeCandidate& rhs) {
  return betterMetrics(lhs.collision_count,
                       lhs.total_cost,
                       rhs.collision_count,
                       rhs.total_cost);
}

bool earlierCollision(const Collision& lhs, const Collision& rhs) {
  return std::tie(lhs.time_step,
                  lhs.type,
                  lhs.robot_a,
                  lhs.robot_b,
                  lhs.location.row,
                  lhs.location.col,
                  lhs.from_a.row,
                  lhs.from_a.col,
                  lhs.to_a.row,
                  lhs.to_a.col,
                  lhs.from_b.row,
                  lhs.from_b.col,
                  lhs.to_b.row,
                  lhs.to_b.col) <
      std::tie(rhs.time_step,
               rhs.type,
               rhs.robot_a,
               rhs.robot_b,
               rhs.location.row,
               rhs.location.col,
               rhs.from_a.row,
               rhs.from_a.col,
               rhs.to_a.row,
               rhs.to_a.col,
               rhs.from_b.row,
               rhs.from_b.col,
               rhs.to_b.row,
               rhs.to_b.col);
}

bool sameConstraint(const Constraint& lhs, const Constraint& rhs) {
  return std::tie(lhs.type,
                  lhs.robot_id,
                  lhs.time_step,
                  lhs.location.row,
                  lhs.location.col,
                  lhs.from.row,
                  lhs.from.col,
                  lhs.to.row,
                  lhs.to.col) ==
      std::tie(rhs.type,
               rhs.robot_id,
               rhs.time_step,
               rhs.location.row,
               rhs.location.col,
               rhs.from.row,
               rhs.from.col,
               rhs.to.row,
               rhs.to.col);
}

bool containsConstraint(const std::vector<Constraint>& constraints,
                        const Constraint& candidate) {
  return std::any_of(constraints.begin(),
                     constraints.end(),
                     [&](const Constraint& constraint) {
                       return sameConstraint(constraint, candidate);
                     });
}

int workerCountForWork(int work_items) {
  if (work_items < 4) {
    return 1;
  }

#ifdef _OPENMP
  return std::max(1, std::min(omp_get_max_threads(), work_items / 2));
#else
  return 1;
#endif
}

std::vector<Collision> selectDisjointConflicts(
    const std::vector<Collision>& collisions,
    int limit) {
  std::vector<Collision> selected;
  std::unordered_set<int> used_robot_ids;

  for (const Collision& collision : collisions) {
    if (static_cast<int>(selected.size()) >= limit) {
      break;
    }
    if (used_robot_ids.find(collision.robot_a) != used_robot_ids.end() ||
        used_robot_ids.find(collision.robot_b) != used_robot_ids.end()) {
      continue;
    }

    selected.push_back(collision);
    used_robot_ids.insert(collision.robot_a);
    used_robot_ids.insert(collision.robot_b);
  }

  return selected;
}

Robot* findRobotById(std::vector<Robot>& robots, int robot_id) {
  for (Robot& robot : robots) {
    if (robot.id == robot_id) {
      return &robot;
    }
  }
  return nullptr;
}

const Robot* findRobotById(const std::vector<Robot>& robots, int robot_id) {
  for (const Robot& robot : robots) {
    if (robot.id == robot_id) {
      return &robot;
    }
  }
  return nullptr;
}

std::optional<Robot> repairRobot(const std::vector<Robot>& base_robots,
                                 int robot_id,
                                 const std::vector<Constraint>& constraints,
                                 int rows,
                                 int cols,
                                 AStarStats* stats) {
  const Robot* robot = findRobotById(base_robots, robot_id);
  if (robot == nullptr) {
    throw std::runtime_error("repair task referenced an unknown robot");
  }

  AStarPlanner a_star(rows, cols);
  const auto path = a_star.findPath(*robot, constraints, stats);
  if (!path.has_value()) {
    return std::nullopt;
  }

  Robot repaired_robot = *robot;
  repaired_robot.path = *path;
  return repaired_robot;
}

void applyRepairedRobot(std::vector<Robot>& robots, const Robot& repaired_robot) {
  Robot* target = findRobotById(robots, repaired_robot.id);
  if (target == nullptr) {
    throw std::runtime_error("repair candidate referenced an unknown robot");
  }
  *target = repaired_robot;
}

void upsertRepairedRobot(std::vector<Robot>& repaired_robots,
                         const Robot& repaired_robot) {
  Robot* existing = findRobotById(repaired_robots, repaired_robot.id);
  if (existing == nullptr) {
    repaired_robots.push_back(repaired_robot);
    return;
  }
  *existing = repaired_robot;
}

bool hasChangedRobot(const std::vector<Robot>& repaired_robots,
                     const std::unordered_set<int>& changed_robot_ids) {
  for (const Robot& robot : repaired_robots) {
    if (changed_robot_ids.find(robot.id) != changed_robot_ids.end()) {
      return true;
    }
  }
  return false;
}

void markChangedRobots(const std::vector<Robot>& repaired_robots,
                       std::unordered_set<int>& changed_robot_ids) {
  for (const Robot& robot : repaired_robots) {
    changed_robot_ids.insert(robot.id);
  }
}

}  // namespace

ParallelGreedyRepairPlanner::ParallelGreedyRepairPlanner(
    int rows,
    int cols,
    int max_repairs,
    int conflict_pool_size,
    int beam_width,
    int lookahead_depth)
    : rows_(rows),
      cols_(cols),
      max_repairs_(max_repairs),
      conflict_pool_size_(conflict_pool_size),
      beam_width_(beam_width),
      lookahead_depth_(lookahead_depth),
      a_star_(rows, cols) {
  if (rows_ <= 0 || cols_ <= 0) {
    throw std::invalid_argument("grid dimensions must be positive");
  }
  if (max_repairs_ <= 0) {
    throw std::invalid_argument("max repairs must be positive");
  }
  if (conflict_pool_size_ <= 0) {
    throw std::invalid_argument("conflict pool size must be positive");
  }
  if (beam_width_ < 0) {
    throw std::invalid_argument("beam width cannot be negative");
  }
  if (lookahead_depth_ <= 0 || lookahead_depth_ > 2) {
    throw std::invalid_argument("lookahead depth must be 1 or 2");
  }
}

std::optional<std::vector<Robot>> ParallelGreedyRepairPlanner::findPaths(
    const std::vector<Robot>& robots,
    GreedyRepairStats* stats) const {
  if (stats != nullptr) {
    *stats = {};
  }

  std::vector<Constraint> constraints;
  std::vector<Robot> current_robots = robots;
  int initial_failures = 0;
  long long initial_searches = 0;
  long long initial_states_expanded = 0;
  long long initial_states_generated = 0;
  const int robot_count = static_cast<int>(current_robots.size());
  const int initial_worker_count = workerCountForWork(robot_count);

#ifdef _OPENMP
#pragma omp parallel for if(initial_worker_count > 1) num_threads(initial_worker_count) schedule(dynamic) reduction(+ : initial_failures, initial_searches, initial_states_expanded, initial_states_generated)
#endif
  for (int robot_index = 0; robot_index < robot_count; ++robot_index) {
    AStarStats a_star_stats;
    const auto path =
        a_star_.findPath(current_robots[robot_index], constraints, &a_star_stats);
    ++initial_searches;
    initial_states_expanded += a_star_stats.states_expanded;
    initial_states_generated += a_star_stats.states_generated;

    if (!path.has_value()) {
      ++initial_failures;
      continue;
    }

    current_robots[robot_index].path = *path;
  }

  if (stats != nullptr) {
    stats->low_level_searches += initial_searches;
    stats->low_level_states_expanded += initial_states_expanded;
    stats->low_level_states_generated += initial_states_generated;
  }
  if (initial_failures > 0) {
    return std::nullopt;
  }

  int repair_iterations = 0;

  while (true) {
    std::vector<Collision> current_collisions =
        collision_detector_.detectCollisions(current_robots);
    if (current_collisions.empty()) {
      return current_robots;
    }

    std::sort(current_collisions.begin(),
              current_collisions.end(),
              earlierCollision);

    ++repair_iterations;
    if (stats != nullptr) {
      ++stats->repair_iterations;
    }
    if (repair_iterations > max_repairs_) {
      return std::nullopt;
    }

    const int current_collision_count =
        static_cast<int>(current_collisions.size());
    const std::vector<Collision> selected_conflicts =
        selectDisjointConflicts(current_collisions, conflict_pool_size_);
    if (stats != nullptr) {
      stats->conflicts_considered +=
          static_cast<long long>(selected_conflicts.size());
    }

    std::vector<SpeculativeTask> tasks;
    tasks.reserve(selected_conflicts.size() * 2);
    for (int i = 0; i < static_cast<int>(selected_conflicts.size()); ++i) {
      for (const Constraint& new_constraint :
           repairConstraints(selected_conflicts[i])) {
        if (containsConstraint(constraints, new_constraint)) {
          continue;
        }
        tasks.push_back({i, new_constraint});
      }
    }

    if (tasks.empty()) {
      return std::nullopt;
    }

    std::vector<std::optional<SpeculativeCandidate>> candidates(tasks.size());
    long long local_candidate_repairs = 0;
    long long local_searches = 0;
    long long local_states_expanded = 0;
    long long local_states_generated = 0;
    long long local_failed_repairs = 0;
    const int task_count = static_cast<int>(tasks.size());
    const int task_worker_count = workerCountForWork(task_count);

#ifdef _OPENMP
#pragma omp parallel for if(task_worker_count > 1) num_threads(task_worker_count) schedule(dynamic) reduction(+ : local_candidate_repairs, local_searches, local_states_expanded, local_states_generated, local_failed_repairs)
#endif
    for (int task_index = 0; task_index < task_count; ++task_index) {
      const SpeculativeTask& task = tasks[task_index];
      CollisionDetector collision_detector;

      std::vector<Constraint> first_constraints = constraints;
      first_constraints.push_back(task.first_constraint);
      const std::vector<Constraint> first_new_constraints = {
          task.first_constraint,
      };

      AStarStats first_stats;
      ++local_candidate_repairs;
      ++local_searches;
      const std::optional<Robot> first_repaired_robot =
          repairRobot(current_robots,
                      task.first_constraint.robot_id,
                      first_constraints,
                      rows_,
                      cols_,
                      &first_stats);
      local_states_expanded += first_stats.states_expanded;
      local_states_generated += first_stats.states_generated;
      if (!first_repaired_robot.has_value()) {
        ++local_failed_repairs;
        continue;
      }

      std::vector<Robot> first_robots = current_robots;
      applyRepairedRobot(first_robots, *first_repaired_robot);
      const std::vector<Robot> first_repaired_robots = {
          *first_repaired_robot,
      };

      SpeculativeCandidate best_candidate{
          task.conflict_index,
          first_new_constraints,
          first_repaired_robots,
          static_cast<int>(collision_detector.detectCollisions(first_robots).size()),
          totalCost(first_robots),
      };

      candidates[task_index] = std::move(best_candidate);
    }

    if (lookahead_depth_ >= 2) {
      std::vector<LookaheadTask> lookahead_tasks;
      for (int parent_index = 0;
           parent_index < static_cast<int>(candidates.size());
           ++parent_index) {
        const std::optional<SpeculativeCandidate>& parent_candidate =
            candidates[parent_index];
        if (!parent_candidate.has_value() ||
            parent_candidate->collision_count == 0) {
          continue;
        }

        CollisionDetector collision_detector;
        std::vector<Constraint> parent_constraints = constraints;
        parent_constraints.insert(parent_constraints.end(),
                                  parent_candidate->new_constraints.begin(),
                                  parent_candidate->new_constraints.end());

        std::vector<Robot> parent_robots = current_robots;
        for (const Robot& repaired_robot : parent_candidate->repaired_robots) {
          applyRepairedRobot(parent_robots, repaired_robot);
        }

        std::vector<Collision> child_collisions =
            collision_detector.detectCollisions(parent_robots);
        std::sort(child_collisions.begin(),
                  child_collisions.end(),
                  earlierCollision);

        const std::vector<Collision> selected_child_conflicts =
            selectDisjointConflicts(child_collisions, conflict_pool_size_);
        for (const Collision& child_collision : selected_child_conflicts) {
          for (const Constraint& second_constraint :
               repairConstraints(child_collision)) {
            if (containsConstraint(parent_constraints, second_constraint)) {
              continue;
            }
            lookahead_tasks.push_back({parent_index, second_constraint});
          }
        }
      }

      std::vector<std::optional<SpeculativeCandidate>> lookahead_candidates(
          lookahead_tasks.size());
      const int lookahead_task_count =
          static_cast<int>(lookahead_tasks.size());
      const int lookahead_worker_count =
          workerCountForWork(lookahead_task_count);

#ifdef _OPENMP
#pragma omp parallel for if(lookahead_worker_count > 1) num_threads(lookahead_worker_count) schedule(dynamic) reduction(+ : local_candidate_repairs, local_searches, local_states_expanded, local_states_generated, local_failed_repairs)
#endif
      for (int task_index = 0; task_index < lookahead_task_count; ++task_index) {
        const LookaheadTask& lookahead_task = lookahead_tasks[task_index];
        const SpeculativeCandidate& parent_candidate =
            *candidates[lookahead_task.parent_candidate_index];

        std::vector<Constraint> second_constraints = constraints;
        second_constraints.insert(second_constraints.end(),
                                  parent_candidate.new_constraints.begin(),
                                  parent_candidate.new_constraints.end());
        second_constraints.push_back(lookahead_task.second_constraint);

        std::vector<Robot> first_robots = current_robots;
        for (const Robot& repaired_robot : parent_candidate.repaired_robots) {
          applyRepairedRobot(first_robots, repaired_robot);
        }

        AStarStats second_stats;
        ++local_candidate_repairs;
        ++local_searches;
        const std::optional<Robot> second_repaired_robot =
            repairRobot(first_robots,
                        lookahead_task.second_constraint.robot_id,
                        second_constraints,
                        rows_,
                        cols_,
                        &second_stats);
        local_states_expanded += second_stats.states_expanded;
        local_states_generated += second_stats.states_generated;
        if (!second_repaired_robot.has_value()) {
          ++local_failed_repairs;
          continue;
        }

        std::vector<Robot> second_robots = first_robots;
        applyRepairedRobot(second_robots, *second_repaired_robot);

        std::vector<Constraint> second_new_constraints =
            parent_candidate.new_constraints;
        second_new_constraints.push_back(lookahead_task.second_constraint);

        CollisionDetector collision_detector;
        SpeculativeCandidate second_candidate{
            parent_candidate.conflict_index,
            std::move(second_new_constraints),
            parent_candidate.repaired_robots,
            static_cast<int>(
                collision_detector.detectCollisions(second_robots).size()),
            totalCost(second_robots),
        };
        upsertRepairedRobot(second_candidate.repaired_robots,
                            *second_repaired_robot);

        lookahead_candidates[task_index] = std::move(second_candidate);
      }

      for (int candidate_index = 0;
           candidate_index < static_cast<int>(lookahead_candidates.size());
           ++candidate_index) {
        std::optional<SpeculativeCandidate>& lookahead_candidate =
            lookahead_candidates[candidate_index];
        if (!lookahead_candidate.has_value()) {
          continue;
        }

        std::optional<SpeculativeCandidate>& parent_candidate =
            candidates[lookahead_tasks[candidate_index].parent_candidate_index];
        if (!parent_candidate.has_value() ||
            betterCandidate(*lookahead_candidate, *parent_candidate)) {
          parent_candidate = std::move(*lookahead_candidate);
        }
      }
    }

    if (stats != nullptr) {
      stats->candidate_repairs_evaluated += local_candidate_repairs;
      stats->low_level_searches += local_searches;
      stats->low_level_states_expanded += local_states_expanded;
      stats->low_level_states_generated += local_states_generated;
      stats->failed_repairs += local_failed_repairs;
    }

    std::optional<SpeculativeCandidate> best_candidate;
    std::vector<std::optional<SpeculativeCandidate>> best_candidates_by_conflict(
        selected_conflicts.size());
    for (std::optional<SpeculativeCandidate>& candidate : candidates) {
      if (!candidate.has_value()) {
        continue;
      }

      if (!best_candidate.has_value() ||
          betterCandidate(*candidate, *best_candidate)) {
        best_candidate = *candidate;
      }

      std::optional<SpeculativeCandidate>& conflict_best =
          best_candidates_by_conflict[candidate->conflict_index];
      if (!conflict_best.has_value() ||
          betterCandidate(*candidate, *conflict_best)) {
        conflict_best = std::move(*candidate);
      }
    }

    if (!best_candidate.has_value()) {
      return std::nullopt;
    }

    std::vector<SpeculativeCandidate> batch_candidates;
    batch_candidates.reserve(best_candidates_by_conflict.size());
    for (std::optional<SpeculativeCandidate>& candidate :
         best_candidates_by_conflict) {
      if (candidate.has_value()) {
        batch_candidates.push_back(std::move(*candidate));
      }
    }

    std::sort(batch_candidates.begin(),
              batch_candidates.end(),
              betterCandidate);

    const int merge_limit =
        std::min(effectiveBeamWidth(), static_cast<int>(batch_candidates.size()));
    std::vector<Constraint> merged_constraints = constraints;
    std::vector<Robot> merged_robots = current_robots;
    std::unordered_set<int> changed_robot_ids;
    int merged_collision_count = current_collision_count;
    int merged_total_cost = totalCost(current_robots);
    long long merged_successful_repairs = 0;
    long long merged_stagnant_repairs = 0;

    for (int i = 0; i < merge_limit; ++i) {
      const SpeculativeCandidate& candidate = batch_candidates[i];
      if (hasChangedRobot(candidate.repaired_robots, changed_robot_ids)) {
        continue;
      }

      bool duplicate_constraint = false;
      for (const Constraint& new_constraint : candidate.new_constraints) {
        if (containsConstraint(merged_constraints, new_constraint)) {
          duplicate_constraint = true;
          break;
        }
      }
      if (duplicate_constraint) {
        continue;
      }

      std::vector<Constraint> trial_constraints = merged_constraints;
      trial_constraints.insert(trial_constraints.end(),
                               candidate.new_constraints.begin(),
                               candidate.new_constraints.end());

      std::vector<Robot> trial_robots = merged_robots;
      for (const Robot& repaired_robot : candidate.repaired_robots) {
        applyRepairedRobot(trial_robots, repaired_robot);
      }

      const int previous_collision_count = merged_collision_count;
      const int trial_collision_count = static_cast<int>(
          collision_detector_.detectCollisions(trial_robots).size());
      const int trial_total_cost = totalCost(trial_robots);

      if (!betterMetrics(trial_collision_count,
                         trial_total_cost,
                         merged_collision_count,
                         merged_total_cost)) {
        continue;
      }

      if (trial_collision_count >= previous_collision_count) {
        ++merged_stagnant_repairs;
      }
      merged_successful_repairs +=
          static_cast<long long>(candidate.repaired_robots.size());
      markChangedRobots(candidate.repaired_robots, changed_robot_ids);
      merged_constraints = std::move(trial_constraints);
      merged_robots = std::move(trial_robots);
      merged_collision_count = trial_collision_count;
      merged_total_cost = trial_total_cost;
    }

    if (merged_successful_repairs > 0) {
      constraints = std::move(merged_constraints);
      current_robots = std::move(merged_robots);
      if (stats != nullptr) {
        stats->successful_repairs += merged_successful_repairs;
        stats->stagnant_repairs += merged_stagnant_repairs;
      }
      continue;
    }

    const SpeculativeCandidate& fallback_candidate = *best_candidate;
    constraints.insert(constraints.end(),
                       fallback_candidate.new_constraints.begin(),
                       fallback_candidate.new_constraints.end());
    for (const Robot& repaired_robot : fallback_candidate.repaired_robots) {
      applyRepairedRobot(current_robots, repaired_robot);
    }
    if (stats != nullptr) {
      stats->successful_repairs +=
          static_cast<long long>(fallback_candidate.repaired_robots.size());
      if (fallback_candidate.collision_count >= current_collision_count) {
        stats->stagnant_repairs +=
            static_cast<long long>(fallback_candidate.repaired_robots.size());
      }
    }
  }
}

int ParallelGreedyRepairPlanner::conflictPoolSize() const {
  return conflict_pool_size_;
}

int ParallelGreedyRepairPlanner::beamWidth() const {
  return effectiveBeamWidth();
}

int ParallelGreedyRepairPlanner::lookaheadDepth() const {
  return lookahead_depth_;
}

int ParallelGreedyRepairPlanner::maxWorkerCount() const {
#ifdef _OPENMP
  return omp_get_max_threads();
#else
  return 1;
#endif
}

int ParallelGreedyRepairPlanner::effectiveBeamWidth() const {
  if (beam_width_ > 0) {
    return beam_width_;
  }
  return std::max(8, maxWorkerCount() * 4);
}

int ParallelGreedyRepairPlanner::totalCost(
    const std::vector<Robot>& robots) const {
  int cost = 0;
  for (const Robot& robot : robots) {
    cost += static_cast<int>(robot.path.size());
  }
  return cost;
}

std::vector<Constraint> ParallelGreedyRepairPlanner::repairConstraints(
    const Collision& collision) const {
  if (collision.type == CollisionType::Vertex) {
    return {
        {ConstraintType::Vertex,
         collision.robot_a,
         collision.time_step,
         collision.location,
         collision.location,
         collision.location},
        {ConstraintType::Vertex,
         collision.robot_b,
         collision.time_step,
         collision.location,
         collision.location,
         collision.location},
    };
  }

  return {
      {ConstraintType::Edge,
       collision.robot_a,
       collision.time_step,
       collision.to_a,
       collision.from_a,
       collision.to_a},
      {ConstraintType::Edge,
       collision.robot_b,
       collision.time_step,
       collision.to_b,
       collision.from_b,
       collision.to_b},
  };
}
