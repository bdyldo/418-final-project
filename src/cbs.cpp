#include "cbs.h"

#include <algorithm>
#include <queue>
#include <stdexcept>
#include <string>
#include <tuple>
#include <unordered_set>
#include <vector>

namespace {

struct NodeEntry {
  CBSNode node;
  int insertion_order;
};

struct NodeEntryCompare {
  bool operator()(const NodeEntry& lhs, const NodeEntry& rhs) const {
    if (lhs.node.cost != rhs.node.cost) {
      return lhs.node.cost > rhs.node.cost;
    }
    return lhs.insertion_order > rhs.insertion_order;
  }
};

// Builds a canonical signature for a set of constraints so duplicate CBS nodes can be skipped.
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

// Returns whether two collisions refer to the same underlying conflict, regardless of robot ordering.
bool sameCollision(const Collision& lhs, const Collision& rhs) {
  if (lhs.type != rhs.type || lhs.time_step != rhs.time_step) {
    return false;
  }

  const bool same_robots =
      (lhs.robot_a == rhs.robot_a && lhs.robot_b == rhs.robot_b) ||
      (lhs.robot_a == rhs.robot_b && lhs.robot_b == rhs.robot_a);
  if (!same_robots) {
    return false;
  }

  if (lhs.type == CollisionType::Vertex) {
    return lhs.location == rhs.location;
  }

  const bool same_order =
      lhs.from_a == rhs.from_a && lhs.to_a == rhs.to_a &&
      lhs.from_b == rhs.from_b && lhs.to_b == rhs.to_b;
  const bool swapped_order =
      lhs.from_a == rhs.from_b && lhs.to_a == rhs.to_b &&
      lhs.from_b == rhs.from_a && lhs.to_b == rhs.to_a;
  return same_order || swapped_order;
}

// Returns whether a collision list still contains the conflict currently being resolved.
bool containsCollision(const std::vector<Collision>& collisions,
                       const Collision& target) {
  for (const Collision& collision : collisions) {
    if (sameCollision(collision, target)) {
      return true;
    }
  }
  return false;
}

}  // namespace

// Creates a CBS planner that uses constrained A* on a grid of the given size.
CBSPlanner::CBSPlanner(int rows, int cols)
    : rows_(rows), cols_(cols), a_star_(rows, cols) {
  if (rows_ <= 0 || cols_ <= 0) {
    throw std::invalid_argument("grid dimensions must be positive");
  }
}

// Finds collision-free paths for all robots using sequential CBS.
std::optional<std::vector<Robot>> CBSPlanner::findPaths(
    const std::vector<Robot>& robots,
    CBSStats* stats) const {
  if (stats != nullptr) {
    *stats = {};
  }

  CBSNode root;
  root.robots = robots;

  for (Robot& robot : root.robots) {
    AStarStats a_star_stats;
    const auto path = a_star_.findPath(robot, root.constraints, &a_star_stats);
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
  root.cost = totalCost(root.robots);

  std::priority_queue<NodeEntry, std::vector<NodeEntry>, NodeEntryCompare> frontier;
  std::unordered_set<std::string> seen_constraint_sets;
  int insertion_order = 0;
  frontier.push({root, insertion_order++});
  seen_constraint_sets.insert(constraintSignature(root.constraints));

  while (!frontier.empty()) {
    CBSNode current = frontier.top().node;
    frontier.pop();
    while (true) {
      if (stats != nullptr) {
        ++stats->high_level_nodes_expanded;
      }

      const auto collision = collision_detector_.findFirstCollision(current.robots);
      if (!collision.has_value()) {
        return current.robots;
      }
      if (stats != nullptr) {
        ++stats->collisions_resolved;
      }

      const std::vector<Constraint> branches = branchConstraints(*collision);
      std::optional<CBSNode> bypass_node;
      int current_conflict_count = -1;
      int bypass_conflict_count = 0;

      for (const Constraint& new_constraint : branches) {
        CBSNode child = current;
        child.constraints.push_back(new_constraint);

        if (!seen_constraint_sets.insert(constraintSignature(child.constraints)).second) {
          if (stats != nullptr) {
            ++stats->duplicate_nodes_pruned;
          }
          continue;
        }

        bool replanned = false;
        for (Robot& robot : child.robots) {
          if (robot.id != new_constraint.robot_id) {
            continue;
          }

          AStarStats a_star_stats;
          const auto path = a_star_.findPath(robot, child.constraints, &a_star_stats);
          if (stats != nullptr) {
            ++stats->low_level_searches;
            stats->low_level_states_expanded += a_star_stats.states_expanded;
            stats->low_level_states_generated += a_star_stats.states_generated;
          }
          if (!path.has_value()) {
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

        child.cost = totalCost(child.robots);
        if (child.cost == current.cost) {
          if (current_conflict_count < 0) {
            current_conflict_count = static_cast<int>(
                collision_detector_.detectCollisions(current.robots).size());
          }

          const std::vector<Collision> child_collisions =
              collision_detector_.detectCollisions(child.robots);
          const int child_conflict_count =
              static_cast<int>(child_collisions.size());

          if (!containsCollision(child_collisions, *collision) &&
              child_conflict_count <= current_conflict_count &&
              (!bypass_node.has_value() ||
               child_conflict_count < bypass_conflict_count)) {
            bypass_node = std::move(child);
            bypass_conflict_count = child_conflict_count;
            continue;
          }
        }

        frontier.push({child, insertion_order++});
      }

      if (bypass_node.has_value()) {
        current = *bypass_node;
        if (stats != nullptr) {
          ++stats->bypasses_applied;
        }
        continue;
      }

      break;
    }
  }

  return std::nullopt;
}

// Returns the sum of all robot path lengths for CBS best-first ordering.
int CBSPlanner::totalCost(const std::vector<Robot>& robots) const {
  int cost = 0;
  for (const Robot& robot : robots) {
    cost += static_cast<int>(robot.path.size());
  }
  return cost;
}

// Converts one collision into the pair of CBS branch constraints.
std::vector<Constraint> CBSPlanner::branchConstraints(
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
