#include "greedy_repair.h"

#include <algorithm>
#include <stdexcept>
#include <string>
#include <tuple>
#include <unordered_set>
#include <vector>

namespace {

struct RepairCandidate {
  std::vector<Constraint> constraints;
  std::vector<Robot> robots;
  int collision_count;
  int total_cost;
};

bool betterCandidate(const RepairCandidate& lhs,
                     const RepairCandidate& rhs) {
  return std::tie(lhs.collision_count, lhs.total_cost) <
      std::tie(rhs.collision_count, rhs.total_cost);
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
GreedyRepairPlanner::GreedyRepairPlanner(int rows, int cols, int max_repairs)
    : rows_(rows), cols_(cols), max_repairs_(max_repairs), a_star_(rows, cols) {
  if (rows_ <= 0 || cols_ <= 0) {
    throw std::invalid_argument("grid dimensions must be positive");
  }
  if (max_repairs_ <= 0) {
    throw std::invalid_argument("max repairs must be positive");
  }
}

// Greedily resolves collisions by repeatedly keeping the best single-robot replan for the current conflict.
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
    const std::vector<Collision> current_collisions =
        collision_detector_.detectCollisions(current_robots);
    if (current_collisions.empty()) {
      return current_robots;
    }

    const Collision& collision = current_collisions.front();
    ++repair_iterations;
    if (stats != nullptr) {
      ++stats->repair_iterations;
    }
    if (repair_iterations > max_repairs_) {
      return std::nullopt;
    }

    std::optional<RepairCandidate> best_candidate;

    for (const Constraint& new_constraint : repairConstraints(collision)) {
      std::vector<Constraint> child_constraints = constraints;
      child_constraints.push_back(new_constraint);

      if (!seen_constraint_sets.insert(constraintSignature(child_constraints)).second) {
        continue;
      }

      std::vector<Robot> child_robots = current_robots;

      bool replanned = false;
      for (Robot& robot : child_robots) {
        if (robot.id != new_constraint.robot_id) {
          continue;
        }

        AStarStats a_star_stats;
        const auto path =
            a_star_.findPath(robot, child_constraints, &a_star_stats);
        if (stats != nullptr) {
          ++stats->low_level_searches;
          stats->low_level_states_expanded += a_star_stats.states_expanded;
          stats->low_level_states_generated += a_star_stats.states_generated;
        }
        if (!path.has_value()) {
          break;
        }

        robot.path = *path;
        replanned = true;
        break;
      }

      if (!replanned) {
        if (stats != nullptr) {
          ++stats->failed_repairs;
        }
        continue;
      }

      const int child_collision_count = static_cast<int>(
          collision_detector_.detectCollisions(child_robots).size());
      const int child_total_cost = totalCost(child_robots);

      RepairCandidate candidate = {
          std::move(child_constraints),
          std::move(child_robots),
          child_collision_count,
          child_total_cost,
      };

      if (!best_candidate.has_value() ||
          betterCandidate(candidate, *best_candidate)) {
        best_candidate = std::move(candidate);
      }
    }

    if (!best_candidate.has_value()) {
      return std::nullopt;
    }

    const int current_collision_count =
        static_cast<int>(current_collisions.size());
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
