#include "greedy_repair.h"

#include <algorithm>
#include <cstddef>
#include <stdexcept>
#include <tuple>
#include <unordered_set>
#include <utility>
#include <vector>

#ifdef _OPENMP
#include <omp.h>
#endif

namespace {

struct RepairCandidate {
  int conflict_index;
  Constraint new_constraint;
  Robot repaired_robot;
  int collision_count;
  int total_cost;
};

struct RepairTask {
  int conflict_index;
  std::vector<Constraint> constraints;
  Constraint new_constraint;
};

bool betterMetrics(int lhs_collision_count,
                   int lhs_total_cost,
                   int rhs_collision_count,
                   int rhs_total_cost) {
  return std::tie(lhs_collision_count, lhs_total_cost) <
      std::tie(rhs_collision_count, rhs_total_cost);
}

bool betterCandidate(const RepairCandidate& lhs, const RepairCandidate& rhs) {
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

}  // namespace

// Creates a one-sided repair planner for a grid of the given size.
GreedyRepairPlanner::GreedyRepairPlanner(int rows,
                                         int cols,
                                         int max_repairs,
                                         int top_k_conflicts)
    : rows_(rows),
      cols_(cols),
      max_repairs_(max_repairs),
      top_k_conflicts_(top_k_conflicts),
      a_star_(rows, cols) {
  if (rows_ <= 0 || cols_ <= 0) {
    throw std::invalid_argument("grid dimensions must be positive");
  }
  if (max_repairs_ <= 0) {
    throw std::invalid_argument("max repairs must be positive");
  }
  if (top_k_conflicts_ <= 0) {
    throw std::invalid_argument("top-k conflicts must be positive");
  }
}

// Greedily resolves collisions by batching disjoint repairs and committing
// every compatible improvement found in the round.
std::optional<std::vector<Robot>> GreedyRepairPlanner::findPaths(
    const std::vector<Robot>& robots,
    GreedyRepairStats* stats) const {
  if (stats != nullptr) {
    *stats = {};
  }

  std::vector<Constraint> constraints;
  std::vector<Robot> current_robots = robots;
  int repair_iterations = 0;

  for (Robot& robot : current_robots) {
    AStarStats a_star_stats;
    const auto path = a_star_.findPath(robot, constraints, &a_star_stats);
    if (stats != nullptr) {
      ++stats->low_level_searches;
      stats->low_level_states_expanded += a_star_stats.states_expanded;
      stats->low_level_states_generated += a_star_stats.states_generated;
    }
    if (!path.has_value()) {
      return std::nullopt;
    }
    robot.path = *path;
  }

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
        selectDisjointConflicts(current_collisions, top_k_conflicts_);
    if (stats != nullptr) {
      stats->conflicts_considered +=
          static_cast<long long>(selected_conflicts.size());
    }

    std::vector<RepairTask> tasks;
    tasks.reserve(selected_conflicts.size() * 2);

    for (int i = 0; i < static_cast<int>(selected_conflicts.size()); ++i) {
      for (const Constraint& new_constraint :
           repairConstraints(selected_conflicts[i])) {
        if (containsConstraint(constraints, new_constraint)) {
          continue;
        }

        std::vector<Constraint> child_constraints = constraints;
        child_constraints.push_back(new_constraint);
        tasks.push_back({i, std::move(child_constraints), new_constraint});
      }
    }

    if (stats != nullptr) {
      stats->candidate_repairs_evaluated += static_cast<long long>(tasks.size());
    }
    if (tasks.empty()) {
      return std::nullopt;
    }

    std::vector<std::optional<RepairCandidate>> candidates(tasks.size());
    long long local_searches = 0;
    long long local_states_expanded = 0;
    long long local_states_generated = 0;
    long long local_failed_repairs = 0;

#ifdef _OPENMP
#pragma omp parallel for schedule(dynamic) reduction(+ : local_searches, local_states_expanded, local_states_generated, local_failed_repairs)
#endif
    for (int task_index = 0; task_index < static_cast<int>(tasks.size());
         ++task_index) {
      const RepairTask& task = tasks[task_index];

      std::vector<Robot> child_robots = current_robots;
      AStarPlanner a_star(rows_, cols_);
      CollisionDetector collision_detector;
      AStarStats a_star_stats;
      std::optional<Robot> repaired_robot;

      for (Robot& robot : child_robots) {
        if (robot.id != task.new_constraint.robot_id) {
          continue;
        }

        const auto path =
            a_star.findPath(robot, task.constraints, &a_star_stats);
        ++local_searches;
        local_states_expanded += a_star_stats.states_expanded;
        local_states_generated += a_star_stats.states_generated;
        if (!path.has_value()) {
          ++local_failed_repairs;
          break;
        }

        robot.path = *path;
        repaired_robot = robot;
        break;
      }

      if (!repaired_robot.has_value()) {
        continue;
      }

      const int child_collision_count = static_cast<int>(
          collision_detector.detectCollisions(child_robots).size());
      const int child_total_cost = totalCost(child_robots);

      candidates[task_index] = RepairCandidate{
          task.conflict_index,
          task.new_constraint,
          *repaired_robot,
          child_collision_count,
          child_total_cost,
      };
    }

    if (stats != nullptr) {
      stats->low_level_searches += local_searches;
      stats->low_level_states_expanded += local_states_expanded;
      stats->low_level_states_generated += local_states_generated;
      stats->failed_repairs += local_failed_repairs;
    }

    std::optional<RepairCandidate> best_candidate;
    std::vector<std::optional<RepairCandidate>> best_candidates_by_conflict(
        selected_conflicts.size());
    for (std::optional<RepairCandidate>& candidate : candidates) {
      if (!candidate.has_value()) {
        continue;
      }

      if (!best_candidate.has_value() ||
          betterCandidate(*candidate, *best_candidate)) {
        best_candidate = candidate;
      }

      std::optional<RepairCandidate>& conflict_best =
          best_candidates_by_conflict[candidate->conflict_index];
      if (!conflict_best.has_value() ||
          betterCandidate(*candidate, *conflict_best)) {
        conflict_best = candidate;
      }
    }

    std::vector<RepairCandidate> batch_candidates;
    batch_candidates.reserve(best_candidates_by_conflict.size());
    for (const std::optional<RepairCandidate>& candidate :
         best_candidates_by_conflict) {
      if (candidate.has_value()) {
        batch_candidates.push_back(*candidate);
      }
    }
    std::sort(batch_candidates.begin(),
              batch_candidates.end(),
              betterCandidate);

    std::vector<Constraint> merged_constraints = constraints;
    std::vector<Robot> merged_robots = current_robots;
    int merged_collision_count = current_collision_count;
    int merged_total_cost = totalCost(current_robots);
    long long merged_successful_repairs = 0;
    long long merged_stagnant_repairs = 0;

    for (const RepairCandidate& candidate : batch_candidates) {
      if (containsConstraint(merged_constraints, candidate.new_constraint)) {
        continue;
      }

      std::vector<Constraint> trial_constraints = merged_constraints;
      trial_constraints.push_back(candidate.new_constraint);

      std::vector<Robot> trial_robots = merged_robots;
      Robot* repaired_robot =
          findRobotById(trial_robots, candidate.repaired_robot.id);
      if (repaired_robot == nullptr) {
        throw std::runtime_error("repair candidate referenced an unknown robot");
      }
      *repaired_robot = candidate.repaired_robot;

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
      ++merged_successful_repairs;
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

    if (!best_candidate.has_value()) {
      return std::nullopt;
    }

    constraints.push_back(best_candidate->new_constraint);
    Robot* repaired_robot =
        findRobotById(current_robots, best_candidate->repaired_robot.id);
    if (repaired_robot == nullptr) {
      throw std::runtime_error("best repair candidate referenced an unknown robot");
    }
    *repaired_robot = best_candidate->repaired_robot;
    if (stats != nullptr) {
      ++stats->successful_repairs;
      if (best_candidate->collision_count >= current_collision_count) {
        ++stats->stagnant_repairs;
      }
    }
  }
}

// Returns the maximum number of earliest disjoint conflicts evaluated per round.
int GreedyRepairPlanner::topKConflicts() const {
  return top_k_conflicts_;
}

// Returns the number of worker threads available to the planner.
int GreedyRepairPlanner::maxWorkerCount() const {
#ifdef _OPENMP
  return omp_get_max_threads();
#else
  return 1;
#endif
}

// Returns the sum of all robot path lengths used as the secondary greedy-repair score.
int GreedyRepairPlanner::totalCost(const std::vector<Robot>& robots) const {
  int cost = 0;
  for (const Robot& robot : robots) {
    cost += static_cast<int>(robot.path.size());
  }
  return cost;
}

// Converts a collision into the two possible single-robot repair constraints.
std::vector<Constraint> GreedyRepairPlanner::repairConstraints(
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
