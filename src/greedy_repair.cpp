#include "greedy_repair.h"

#include <algorithm>
#include <cstddef>
#include <stdexcept>
#include <string>
#include <tuple>
#include <unordered_set>
#include <utility>
#include <vector>

#ifdef _OPENMP
#include <omp.h>
#endif

namespace {

struct RepairCandidate {
  std::vector<Constraint> constraints;
  std::vector<Robot> robots;
  int collision_count;
  int total_cost;
};

struct RepairTask {
  std::vector<Constraint> constraints;
  Constraint new_constraint;
};

int resolveTopKConflicts(int requested_top_k_conflicts) {
  if (requested_top_k_conflicts > 0) {
    return requested_top_k_conflicts;
  }

#ifdef _OPENMP
  return std::max(1, omp_get_max_threads());
#else
  return 1;
#endif
}

bool betterCandidate(const RepairCandidate& lhs,
                     const RepairCandidate& rhs) {
  return std::tie(lhs.collision_count, lhs.total_cost) <
      std::tie(rhs.collision_count, rhs.total_cost);
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

std::string constraintSignature(const std::vector<Constraint>& constraints) {
  std::vector<Constraint> sorted_constraints = constraints;
  std::sort(sorted_constraints.begin(),
            sorted_constraints.end(),
            [](const Constraint& lhs, const Constraint& rhs) {
              return std::tie(lhs.robot_id,
                              lhs.time_step,
                              lhs.type,
                              lhs.location.row,
                              lhs.location.col,
                              lhs.from.row,
                              lhs.from.col,
                              lhs.to.row,
                              lhs.to.col) <
                  std::tie(rhs.robot_id,
                           rhs.time_step,
                           rhs.type,
                           rhs.location.row,
                           rhs.location.col,
                           rhs.from.row,
                           rhs.from.col,
                           rhs.to.row,
                           rhs.to.col);
            });

  std::string signature;
  signature.reserve(sorted_constraints.size() * 32);
  for (const Constraint& constraint : sorted_constraints) {
    signature += std::to_string(static_cast<int>(constraint.type));
    signature += ':';
    signature += std::to_string(constraint.robot_id);
    signature += ':';
    signature += std::to_string(constraint.time_step);
    signature += ':';
    signature += std::to_string(constraint.location.row);
    signature += ',';
    signature += std::to_string(constraint.location.col);
    signature += ':';
    signature += std::to_string(constraint.from.row);
    signature += ',';
    signature += std::to_string(constraint.from.col);
    signature += ':';
    signature += std::to_string(constraint.to.row);
    signature += ',';
    signature += std::to_string(constraint.to.col);
    signature += ';';
  }
  return signature;
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
      top_k_conflicts_(resolveTopKConflicts(top_k_conflicts)),
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

// Greedily resolves collisions by repeatedly keeping the best single-robot replan from the first K conflicts.
std::optional<std::vector<Robot>> GreedyRepairPlanner::findPaths(
    const std::vector<Robot>& robots,
    GreedyRepairStats* stats) const {
  if (stats != nullptr) {
    *stats = {};
  }

  std::vector<Constraint> constraints;
  std::vector<Robot> current_robots = robots;
  std::unordered_set<std::string> seen_constraint_sets;
  seen_constraint_sets.insert(constraintSignature(constraints));
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
    const int conflict_limit = std::min(
        top_k_conflicts_, static_cast<int>(current_collisions.size()));
    if (stats != nullptr) {
      stats->conflicts_considered += conflict_limit;
    }

    std::vector<RepairTask> tasks;
    tasks.reserve(static_cast<std::size_t>(conflict_limit) * 2);

    for (int i = 0; i < conflict_limit; ++i) {
      for (const Constraint& new_constraint :
           repairConstraints(current_collisions[i])) {
        std::vector<Constraint> child_constraints = constraints;
        child_constraints.push_back(new_constraint);

        if (!seen_constraint_sets.insert(
                constraintSignature(child_constraints)).second) {
          continue;
        }

        tasks.push_back(
            {std::move(child_constraints), new_constraint});
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
      bool replanned = false;

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
          replanned = false;
          break;
        }

        robot.path = *path;
        replanned = true;
        break;
      }

      if (!replanned) {
        continue;
      }

      const int child_collision_count = static_cast<int>(
          collision_detector.detectCollisions(child_robots).size());
      const int child_total_cost = totalCost(child_robots);

      candidates[task_index] = RepairCandidate{
          task.constraints,
          std::move(child_robots),
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
    for (std::optional<RepairCandidate>& candidate : candidates) {
      if (!candidate.has_value()) {
        continue;
      }

      if (!best_candidate.has_value() ||
          betterCandidate(*candidate, *best_candidate)) {
        best_candidate = std::move(candidate);
      }
    }

    if (!best_candidate.has_value()) {
      return std::nullopt;
    }

    constraints = std::move(best_candidate->constraints);
    current_robots = std::move(best_candidate->robots);
    if (stats != nullptr) {
      ++stats->successful_repairs;
      if (best_candidate->collision_count >= current_collision_count) {
        ++stats->stagnant_repairs;
      }
    }
  }
}

// Returns the number of earliest conflicts evaluated per greedy repair round.
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
