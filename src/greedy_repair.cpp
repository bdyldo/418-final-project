#include "greedy_repair.h"

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <deque>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#ifdef _OPENMP
#include <omp.h>
#endif

namespace {

struct SharedRobotState {
  explicit SharedRobotState(const Robot& initial_robot)
      : robot(initial_robot), version(0) {}

  Robot robot;
  std::vector<Constraint> constraints;
  std::mutex mutex;
  std::atomic<long long> version;
};

struct RobotSnapshot {
  Robot robot;
  std::vector<Constraint> constraints;
  long long version;
};

struct CandidateScore {
  int conflict_count;
  int path_cost;
};

struct PlateauCandidate {
  int target_robot_index;
  int partner_robot_index;
  CandidateScore score;
  std::vector<Constraint> constraints;
  std::vector<Point> path;
};

bool betterScore(const CandidateScore& lhs, const CandidateScore& rhs) {
  return std::tie(lhs.conflict_count, lhs.path_cost) <
      std::tie(rhs.conflict_count, rhs.path_cost);
}

bool nonWorseConflictCount(const CandidateScore& lhs,
                           const CandidateScore& rhs) {
  return lhs.conflict_count <= rhs.conflict_count;
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

Point positionAtTime(const Robot& robot, int time_step) {
  if (robot.path.empty()) {
    throw std::invalid_argument("robot path must contain at least one point");
  }

  if (time_step < static_cast<int>(robot.path.size())) {
    return robot.path[time_step];
  }

  return robot.path.back();
}

std::vector<RobotSnapshot> snapshotRobots(
    std::deque<SharedRobotState>& shared_states) {
  std::vector<RobotSnapshot> snapshots;
  snapshots.reserve(shared_states.size());

  for (SharedRobotState& state : shared_states) {
    std::lock_guard<std::mutex> lock(state.mutex);
    snapshots.push_back(
        {state.robot,
         state.constraints,
         state.version.load(std::memory_order_relaxed)});
  }

  return snapshots;
}

std::vector<Robot> extractRobots(const std::vector<RobotSnapshot>& snapshots) {
  std::vector<Robot> robots;
  robots.reserve(snapshots.size());
  for (const RobotSnapshot& snapshot : snapshots) {
    robots.push_back(snapshot.robot);
  }
  return robots;
}

std::vector<Collision> selectConflictWindow(
    const std::vector<Collision>& collisions,
    int start_index,
    int limit) {
  std::vector<Collision> selected;
  selected.reserve(limit);

  for (int i = 0; i < limit; ++i) {
    selected.push_back(
        collisions[(start_index + i) % static_cast<int>(collisions.size())]);
  }

  return selected;
}

std::vector<int> buildConflictCounts(
    const std::vector<Collision>& collisions,
    const std::unordered_map<int, int>& robot_index_by_id,
    int robot_count) {
  std::vector<int> counts(robot_count, 0);

  for (const Collision& collision : collisions) {
    ++counts[robot_index_by_id.at(collision.robot_a)];
    ++counts[robot_index_by_id.at(collision.robot_b)];
  }

  return counts;
}

int chooseRepairRobot(
    const Collision& conflict,
    const std::vector<int>& conflict_counts,
    const std::vector<Robot>& snapshot_robots,
    const std::unordered_map<int, int>& robot_index_by_id) {
  const int robot_a_index = robot_index_by_id.at(conflict.robot_a);
  const int robot_b_index = robot_index_by_id.at(conflict.robot_b);

  if (conflict_counts[robot_b_index] > conflict_counts[robot_a_index]) {
    return conflict.robot_b;
  }
  if (conflict_counts[robot_b_index] < conflict_counts[robot_a_index]) {
    return conflict.robot_a;
  }

  if (snapshot_robots[robot_b_index].path.size() >
      snapshot_robots[robot_a_index].path.size()) {
    return conflict.robot_b;
  }
  return conflict.robot_a;
}

std::vector<Constraint> repairConstraintsForRobot(const Collision& conflict,
                                                  int robot_id) {
  if (conflict.type == CollisionType::Vertex) {
    return {
        {ConstraintType::Vertex,
         robot_id,
         conflict.time_step,
         conflict.location,
         conflict.location,
         conflict.location},
    };
  }

  if (robot_id == conflict.robot_a) {
    return {
        {ConstraintType::Edge,
         robot_id,
         conflict.time_step,
         conflict.to_a,
         conflict.from_a,
         conflict.to_a},
        {ConstraintType::Vertex,
         robot_id,
         conflict.time_step + 1,
         conflict.from_a,
         conflict.from_a,
         conflict.from_a},
    };
  }

  return {
      {ConstraintType::Edge,
       robot_id,
       conflict.time_step,
       conflict.to_b,
       conflict.from_b,
       conflict.to_b},
      {ConstraintType::Vertex,
       robot_id,
       conflict.time_step + 1,
       conflict.from_b,
       conflict.from_b,
       conflict.from_b},
  };
}

int countConflictsForRobot(const Robot& target_robot,
                           const std::vector<Robot>& robots) {
  int conflict_count = 0;

  for (const Robot& robot : robots) {
    if (robot.id == target_robot.id) {
      continue;
    }

    const int time_steps = std::max(static_cast<int>(target_robot.path.size()),
                                    static_cast<int>(robot.path.size()));

    for (int time_step = 0; time_step < time_steps; ++time_step) {
      if (positionAtTime(target_robot, time_step) ==
          positionAtTime(robot, time_step)) {
        ++conflict_count;
      }
    }

    for (int time_step = 0; time_step + 1 < time_steps; ++time_step) {
      const Point target_from = positionAtTime(target_robot, time_step);
      const Point target_to = positionAtTime(target_robot, time_step + 1);
      const Point other_from = positionAtTime(robot, time_step);
      const Point other_to = positionAtTime(robot, time_step + 1);

      if (target_from == target_to || other_from == other_to) {
        continue;
      }

      if (target_from == other_to && target_to == other_from) {
        ++conflict_count;
      }
    }
  }

  return conflict_count;
}

}  // namespace

// Creates an asynchronous repair planner for a grid of the given size.
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

// Resolves conflicts with asynchronous single-robot repairs on shared paths.
std::optional<std::vector<Robot>> GreedyRepairPlanner::findPaths(
    const std::vector<Robot>& robots,
    GreedyRepairStats* stats) const {
  if (stats != nullptr) {
    *stats = {};
  }

  std::deque<SharedRobotState> shared_states;
  std::unordered_map<int, int> robot_index_by_id;
  for (const Robot& robot : robots) {
    robot_index_by_id.emplace(robot.id, static_cast<int>(shared_states.size()));
    shared_states.emplace_back(robot);
  }

  for (int robot_index = 0; robot_index < static_cast<int>(shared_states.size());
       ++robot_index) {
    AStarStats a_star_stats;
    const auto path =
        a_star_.findPath(shared_states[robot_index].robot, {}, &a_star_stats);
    if (stats != nullptr) {
      ++stats->low_level_searches;
      stats->low_level_states_expanded += a_star_stats.states_expanded;
      stats->low_level_states_generated += a_star_stats.states_generated;
    }
    if (!path.has_value()) {
      return std::nullopt;
    }
    shared_states[robot_index].robot.path = *path;
  }

  for (int repair_iteration = 1; repair_iteration <= max_repairs_;
       ++repair_iteration) {
    const std::vector<RobotSnapshot> snapshots = snapshotRobots(shared_states);
    const std::vector<Robot> snapshot_robots = extractRobots(snapshots);
    const std::vector<Collision> current_collisions =
        collision_detector_.detectCollisions(snapshot_robots);
    if (current_collisions.empty()) {
      return snapshot_robots;
    }

    if (stats != nullptr) {
      ++stats->repair_iterations;
    }

    const int conflict_limit = std::min(
        top_k_conflicts_, static_cast<int>(current_collisions.size()));
    const int conflict_start =
        ((repair_iteration - 1) * conflict_limit) %
        static_cast<int>(current_collisions.size());
    const std::vector<Collision> selected_conflicts =
        selectConflictWindow(current_collisions, conflict_start, conflict_limit);
    const std::vector<int> conflict_counts =
        buildConflictCounts(current_collisions,
                            robot_index_by_id,
                            static_cast<int>(snapshot_robots.size()));

    if (stats != nullptr) {
      stats->conflicts_considered +=
          static_cast<long long>(selected_conflicts.size());
    }

    long long local_searches = 0;
    long long local_candidates_evaluated = 0;
    long long local_states_expanded = 0;
    long long local_states_generated = 0;
    long long local_successful_repairs = 0;
    long long local_failed_repairs = 0;
    long long local_stagnant_repairs = 0;
    std::vector<std::optional<PlateauCandidate>> plateau_candidates(
        selected_conflicts.size());

#ifdef _OPENMP
#pragma omp parallel for schedule(dynamic) reduction(+ : local_searches, local_candidates_evaluated, local_states_expanded, local_states_generated, local_successful_repairs, local_failed_repairs, local_stagnant_repairs)
#endif
    for (int conflict_index = 0;
         conflict_index < static_cast<int>(selected_conflicts.size());
         ++conflict_index) {
      const Collision& conflict = selected_conflicts[conflict_index];
      const int preferred_robot_id = chooseRepairRobot(conflict,
                                                       conflict_counts,
                                                       snapshot_robots,
                                                       robot_index_by_id);
      const int alternate_robot_id =
          preferred_robot_id == conflict.robot_a ? conflict.robot_b : conflict.robot_a;
      const int attempt_robot_ids[2] = {preferred_robot_id, alternate_robot_id};

      bool found_any_path = false;
      bool found_improvement = false;
      std::optional<PlateauCandidate> best_plateau_candidate;
      int target_robot_index = -1;
      int partner_robot_index = -1;
      std::vector<Constraint> repaired_constraints;
      std::vector<Point> repaired_path;

      for (int attempt = 0; attempt < 2; ++attempt) {
        const int target_robot_id = attempt_robot_ids[attempt];
        const int partner_robot_id =
            target_robot_id == conflict.robot_a ? conflict.robot_b : conflict.robot_a;
        const int candidate_robot_index = robot_index_by_id.at(target_robot_id);
        const Robot& candidate_robot = snapshot_robots[candidate_robot_index];
        const CandidateScore baseline_score = {
            conflict_counts[candidate_robot_index],
            static_cast<int>(candidate_robot.path.size()),
        };

        std::vector<Constraint> candidate_constraints =
            snapshots[candidate_robot_index].constraints;
        const std::vector<Constraint> new_constraints =
            repairConstraintsForRobot(conflict, target_robot_id);
        candidate_constraints.reserve(candidate_constraints.size() +
                                      new_constraints.size());
        bool added_constraint = false;
        for (const Constraint& constraint : new_constraints) {
          if (containsConstraint(candidate_constraints, constraint)) {
            continue;
          }
          candidate_constraints.push_back(constraint);
          added_constraint = true;
        }
        if (!added_constraint) {
          found_any_path = true;
          continue;
        }

        AStarPlanner a_star(rows_, cols_);
        AStarStats a_star_stats;
        const auto path =
            a_star.findPath(candidate_robot, candidate_constraints, &a_star_stats);

        ++local_searches;
        ++local_candidates_evaluated;
        local_states_expanded += a_star_stats.states_expanded;
        local_states_generated += a_star_stats.states_generated;

        if (!path.has_value()) {
          continue;
        }
        found_any_path = true;

        Robot repaired_robot = candidate_robot;
        repaired_robot.path = *path;
        const CandidateScore repaired_score = {
            countConflictsForRobot(repaired_robot, snapshot_robots),
            static_cast<int>(repaired_robot.path.size()),
        };
        if (!nonWorseConflictCount(repaired_score, baseline_score)) {
          continue;
        }
        if (repaired_score.conflict_count == baseline_score.conflict_count) {
          PlateauCandidate candidate = {
              candidate_robot_index,
              robot_index_by_id.at(partner_robot_id),
              repaired_score,
              candidate_constraints,
              *path,
          };
          if (!best_plateau_candidate.has_value() ||
              betterScore(candidate.score, best_plateau_candidate->score)) {
            best_plateau_candidate = std::move(candidate);
          }
          continue;
        }

        found_improvement = true;
        target_robot_index = candidate_robot_index;
        partner_robot_index = robot_index_by_id.at(partner_robot_id);
        repaired_constraints = std::move(candidate_constraints);
        repaired_path = std::move(*path);
        break;
      }

      plateau_candidates[conflict_index] = std::move(best_plateau_candidate);
      if (!found_improvement) {
        if (found_any_path) {
          ++local_stagnant_repairs;
        } else {
          ++local_failed_repairs;
        }
        continue;
      }

      std::unique_lock<std::mutex> lock(shared_states[target_robot_index].mutex,
                                        std::try_to_lock);
      if (!lock.owns_lock()) {
        ++local_stagnant_repairs;
        continue;
      }

      if (shared_states[target_robot_index].version.load(std::memory_order_relaxed) !=
              snapshots[target_robot_index].version ||
          shared_states[partner_robot_index].version.load(std::memory_order_relaxed) !=
              snapshots[partner_robot_index].version) {
        ++local_stagnant_repairs;
        continue;
      }

      shared_states[target_robot_index].robot.path = std::move(repaired_path);
      shared_states[target_robot_index].constraints =
          std::move(repaired_constraints);
      shared_states[target_robot_index].version.fetch_add(1,
                                                          std::memory_order_relaxed);
      ++local_successful_repairs;
    }

    if (local_successful_repairs == 0) {
      std::optional<PlateauCandidate> best_plateau_candidate;
      for (std::optional<PlateauCandidate>& candidate : plateau_candidates) {
        if (!candidate.has_value()) {
          continue;
        }
        if (!best_plateau_candidate.has_value() ||
            betterScore(candidate->score, best_plateau_candidate->score)) {
          best_plateau_candidate = std::move(candidate);
        }
      }

      if (best_plateau_candidate.has_value()) {
        std::unique_lock<std::mutex> lock(
            shared_states[best_plateau_candidate->target_robot_index].mutex,
            std::try_to_lock);
        if (lock.owns_lock() &&
            shared_states[best_plateau_candidate->target_robot_index].version.load(
                std::memory_order_relaxed) ==
                snapshots[best_plateau_candidate->target_robot_index].version &&
            shared_states[best_plateau_candidate->partner_robot_index].version.load(
                std::memory_order_relaxed) ==
                snapshots[best_plateau_candidate->partner_robot_index].version) {
          shared_states[best_plateau_candidate->target_robot_index].robot.path =
              std::move(best_plateau_candidate->path);
          shared_states[best_plateau_candidate->target_robot_index].constraints =
              std::move(best_plateau_candidate->constraints);
          shared_states[best_plateau_candidate->target_robot_index].version.fetch_add(
              1,
              std::memory_order_relaxed);
          ++local_successful_repairs;
        }
      }
    }

    if (stats != nullptr) {
      stats->candidate_repairs_evaluated += local_candidates_evaluated;
      stats->low_level_searches += local_searches;
      stats->low_level_states_expanded += local_states_expanded;
      stats->low_level_states_generated += local_states_generated;
      stats->successful_repairs += local_successful_repairs;
      stats->failed_repairs += local_failed_repairs;
      stats->stagnant_repairs += local_stagnant_repairs;
    }
  }

  return std::nullopt;
}

// Returns the maximum number of conflicts processed in one async repair round.
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
