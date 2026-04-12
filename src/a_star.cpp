#include "a_star.h"

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstdlib>
#include <queue>
#include <stdexcept>
#include <unordered_set>
#include <unordered_map>
#include <utility>
#include <vector>

namespace {

struct PointHash {
  std::size_t operator()(const Point& point) const {
    const std::uint64_t row_bits =
        static_cast<std::uint32_t>(point.row);
    const std::uint64_t col_bits =
        static_cast<std::uint32_t>(point.col);
    return static_cast<std::size_t>((row_bits << 32) ^ col_bits);
  }
};

enum class MoveDirection : std::uint8_t {
  Start = 0,
  Up,
  Down,
  Left,
  Right,
};

struct SearchState {
  Point position;
  int time_step;
  MoveDirection direction;
};

bool operator==(const SearchState& lhs, const SearchState& rhs) {
  return lhs.position == rhs.position &&
      lhs.time_step == rhs.time_step &&
      lhs.direction == rhs.direction;
}

struct SearchStateHash {
  std::size_t operator()(const SearchState& state) const {
    const std::uint64_t row_bits =
        static_cast<std::uint32_t>(state.position.row);
    const std::uint64_t col_bits =
        static_cast<std::uint32_t>(state.position.col);
    const std::uint64_t time_bits =
        static_cast<std::uint32_t>(state.time_step);
    const std::uint64_t direction_bits =
        static_cast<std::uint8_t>(state.direction);
    return static_cast<std::size_t>(
        (row_bits << 42) ^ (col_bits << 21) ^ (time_bits << 3) ^ direction_bits);
  }
};

struct QueueEntry {
  SearchState state;
  int g_cost;
  int f_cost;
  int turn_count;
  std::uint64_t insertion_order;
};

struct SearchCost {
  int g_cost;
  int turn_count;
};

struct QueueEntryCompare {
  bool operator()(const QueueEntry& lhs, const QueueEntry& rhs) const {
    if (lhs.f_cost != rhs.f_cost) {
      return lhs.f_cost > rhs.f_cost;
    }
    if (lhs.turn_count != rhs.turn_count) {
      return lhs.turn_count > rhs.turn_count;
    }
    if (lhs.g_cost != rhs.g_cost) {
      return lhs.g_cost > rhs.g_cost;
    }
    return lhs.insertion_order > rhs.insertion_order;
  }
};

// Reconstructs a path by following parent links from the goal state to the start.
std::vector<Point> reconstructPath(
    const SearchState& goal_state,
    const std::unordered_map<SearchState, SearchState, SearchStateHash>& came_from) {
  std::vector<Point> path;
  SearchState current = goal_state;
  path.push_back(current.position);

  while (true) {
    const auto parent_it = came_from.find(current);
    if (parent_it == came_from.end()) {
      break;
    }
    current = parent_it->second;
    path.push_back(current.position);
  }

  std::reverse(path.begin(), path.end());
  return path;
}

struct MoveOption {
  Point delta;
  MoveDirection direction;
  bool is_wait;
  int base_priority;
};

int pointDistanceToGoal(const Point& point, const Point& goal) {
  return std::abs(point.row - goal.row) + std::abs(point.col - goal.col);
}

int turnPenalty(MoveDirection previous_direction, MoveDirection next_direction) {
  if (previous_direction == MoveDirection::Start ||
      previous_direction == next_direction) {
    return 0;
  }
  return 1;
}

std::array<MoveOption, 5> orderedMoves(const SearchState& state,
                                       const Point& goal) {
  std::array<MoveOption, 5> moves = {{
      {{0, 1}, MoveDirection::Right, false, 0},
      {{-1, 0}, MoveDirection::Up, false, 1},
      {{1, 0}, MoveDirection::Down, false, 2},
      {{0, -1}, MoveDirection::Left, false, 3},
      {{0, 0}, state.direction, true, 4},
  }};

  const int current_row_distance = std::abs(state.position.row - goal.row);
  const int current_col_distance = std::abs(state.position.col - goal.col);
  const bool prefer_horizontal = current_col_distance >= current_row_distance;

  std::sort(moves.begin(), moves.end(),
            [&](const MoveOption& lhs, const MoveOption& rhs) {
              auto key = [&](const MoveOption& move) {
                const Point next_position = {
                    state.position.row + move.delta.row,
                    state.position.col + move.delta.col,
                };
                const int next_distance = pointDistanceToGoal(next_position, goal);
                const bool reduces_row =
                    std::abs(next_position.row - goal.row) < current_row_distance;
                const bool reduces_col =
                    std::abs(next_position.col - goal.col) < current_col_distance;
                const int turn_rank = move.is_wait
                    ? 0
                    : turnPenalty(state.direction, move.direction);
                const int axis_rank = move.is_wait
                    ? 2
                    : prefer_horizontal
                        ? (reduces_col ? 0 : (reduces_row ? 1 : 3))
                        : (reduces_row ? 0 : (reduces_col ? 1 : 3));
                return std::make_tuple(
                    turn_rank,
                    axis_rank,
                    next_distance,
                    move.base_priority);
              };
              return key(lhs) < key(rhs);
            });

  return moves;
}

struct VertexConstraintKey {
  Point location;
  int time_step;
};

bool operator==(const VertexConstraintKey& lhs, const VertexConstraintKey& rhs) {
  return lhs.location == rhs.location && lhs.time_step == rhs.time_step;
}

struct VertexConstraintKeyHash {
  std::size_t operator()(const VertexConstraintKey& key) const {
    const PointHash point_hash;
    const std::size_t point_value = point_hash(key.location);
    const std::size_t time_value = static_cast<std::size_t>(
        static_cast<std::uint32_t>(key.time_step));
    return point_value ^ (time_value + 0x9e3779b9 + (point_value << 6) +
                          (point_value >> 2));
  }
};

struct EdgeConstraintKey {
  Point from;
  Point to;
  int time_step;
};

bool operator==(const EdgeConstraintKey& lhs, const EdgeConstraintKey& rhs) {
  return lhs.from == rhs.from && lhs.to == rhs.to &&
      lhs.time_step == rhs.time_step;
}

struct EdgeConstraintKeyHash {
  std::size_t operator()(const EdgeConstraintKey& key) const {
    const PointHash point_hash;
    const std::size_t from_hash = point_hash(key.from);
    const std::size_t to_hash = point_hash(key.to);
    const std::size_t time_hash = static_cast<std::size_t>(
        static_cast<std::uint32_t>(key.time_step));
    return from_hash ^ (to_hash + 0x9e3779b9 + (from_hash << 6) + (from_hash >> 2)) ^
        (time_hash + 0x9e3779b9 + (to_hash << 6) + (to_hash >> 2));
  }
};

}  // namespace

struct AStarPlanner::ConstraintIndex {
  std::unordered_set<VertexConstraintKey, VertexConstraintKeyHash>
      vertex_constraints;
  std::unordered_set<EdgeConstraintKey, EdgeConstraintKeyHash> edge_constraints;
  int latest_constraint_end_time = 0;
};

// Creates an A* planner for a grid with the given dimensions.
AStarPlanner::AStarPlanner(int rows, int cols) : rows_(rows), cols_(cols) {
  if (rows_ <= 0 || cols_ <= 0) {
    throw std::invalid_argument("grid dimensions must be positive");
  }
}

// Finds a path for one robot on the grid while respecting time-indexed constraints.
std::optional<std::vector<Point>> AStarPlanner::findPath(
    const Robot& robot,
    const std::vector<Constraint>& constraints,
    AStarStats* stats) const {
  if (!isInBounds(robot.start) || !isInBounds(robot.goal)) {
    throw std::invalid_argument("robot start and goal must lie inside the grid");
  }

  if (stats != nullptr) {
    *stats = {};
  }

  const ConstraintIndex constraint_index =
      buildConstraintIndex(robot.id, constraints);

  if (violatesVertexConstraint(robot.start, 0, constraint_index)) {
    return std::nullopt;
  }

  const int horizon = searchHorizon(robot, constraint_index);
  const SearchState start_state = {robot.start, 0, MoveDirection::Start};

  std::priority_queue<QueueEntry, std::vector<QueueEntry>, QueueEntryCompare> frontier;
  std::unordered_map<SearchState, SearchCost, SearchStateHash> best_cost;
  std::unordered_map<SearchState, SearchState, SearchStateHash> came_from;
  std::uint64_t next_insertion_order = 1;

  best_cost[start_state] = {0, 0};
  frontier.push(
      {start_state,
       0,
       manhattanDistance(robot.start, robot.goal),
       0,
       0});
  if (stats != nullptr) {
    ++stats->states_generated;
  }

  while (!frontier.empty()) {
    const QueueEntry current_entry = frontier.top();
    frontier.pop();

    const auto best_it = best_cost.find(current_entry.state);
    if (best_it == best_cost.end() ||
        current_entry.g_cost != best_it->second.g_cost ||
        current_entry.turn_count != best_it->second.turn_count) {
      continue;
    }

    if (stats != nullptr) {
      ++stats->states_expanded;
    }

    if (current_entry.state.position == robot.goal &&
        current_entry.state.time_step >=
            constraint_index.latest_constraint_end_time) {
      return reconstructPath(current_entry.state, came_from);
    }

    if (current_entry.state.time_step >= horizon) {
      continue;
    }

    const std::array<MoveOption, 5> moves =
        orderedMoves(current_entry.state, robot.goal);

    for (const MoveOption& move : moves) {
      const Point next_position = {
          current_entry.state.position.row + move.delta.row,
          current_entry.state.position.col + move.delta.col,
      };
      const int next_time_step = current_entry.state.time_step + 1;

      if (!isInBounds(next_position)) {
        continue;
      }
      if (violatesVertexConstraint(next_position, next_time_step, constraint_index)) {
        continue;
      }
      if (violatesEdgeConstraint(current_entry.state.position,
                                 next_position,
                                 current_entry.state.time_step,
                                 constraint_index)) {
        continue;
      }

      const SearchState next_state = {
          next_position,
          next_time_step,
          move.direction,
      };
      const int next_g_cost = current_entry.g_cost + 1;
      const int next_turn_count = current_entry.turn_count +
          (move.is_wait ? 0 : turnPenalty(current_entry.state.direction,
                                          move.direction));
      const auto existing_it = best_cost.find(next_state);
      if (existing_it != best_cost.end()) {
        const SearchCost& existing_cost = existing_it->second;
        if (existing_cost.g_cost < next_g_cost ||
            (existing_cost.g_cost == next_g_cost &&
             existing_cost.turn_count <= next_turn_count)) {
          continue;
        }
      }

      best_cost[next_state] = {next_g_cost, next_turn_count};
      came_from[next_state] = current_entry.state;

      const int next_f_cost =
          next_g_cost + manhattanDistance(next_position, robot.goal);
      frontier.push({next_state,
                     next_g_cost,
                     next_f_cost,
                     next_turn_count,
                     next_insertion_order++});
      if (stats != nullptr) {
        ++stats->states_generated;
      }
    }
  }

  return std::nullopt;
}

// Returns whether a point lies within the planner's grid bounds.
bool AStarPlanner::isInBounds(const Point& point) const {
  return point.row >= 0 && point.row < rows_ &&
      point.col >= 0 && point.col < cols_;
}

// Returns the Manhattan-distance heuristic between two grid points.
int AStarPlanner::manhattanDistance(const Point& lhs, const Point& rhs) const {
  return std::abs(lhs.row - rhs.row) + std::abs(lhs.col - rhs.col);
}

// Builds hash-based lookup tables for the subset of constraints relevant to one robot.
AStarPlanner::ConstraintIndex AStarPlanner::buildConstraintIndex(
    int robot_id,
    const std::vector<Constraint>& constraints) const {
  ConstraintIndex constraint_index;

  for (const Constraint& constraint : constraints) {
    if (constraint.robot_id != robot_id) {
      continue;
    }

    if (constraint.type == ConstraintType::Vertex) {
      constraint_index.vertex_constraints.insert(
          {constraint.location, constraint.time_step});
      constraint_index.latest_constraint_end_time =
          std::max(constraint_index.latest_constraint_end_time,
                   constraint.time_step);
      continue;
    }

    constraint_index.edge_constraints.insert(
        {constraint.from, constraint.to, constraint.time_step});
    constraint_index.latest_constraint_end_time =
        std::max(constraint_index.latest_constraint_end_time,
                 constraint.time_step + 1);
  }

  return constraint_index;
}

// Returns a conservative time horizon for the space-time A* search.
int AStarPlanner::searchHorizon(const Robot& robot,
                                const ConstraintIndex& constraint_index) const {
  const int base_distance = manhattanDistance(robot.start, robot.goal);
  const int detour_slack = rows_ + cols_ + 10;
  return std::max(base_distance, constraint_index.latest_constraint_end_time) +
      detour_slack;
}

// Checks whether a robot is forbidden from occupying a cell at a specific timestep.
bool AStarPlanner::violatesVertexConstraint(
    const Point& location,
    int time_step,
    const ConstraintIndex& constraint_index) const {
  return constraint_index.vertex_constraints.find({location, time_step}) !=
      constraint_index.vertex_constraints.end();
}

// Checks whether a robot is forbidden from traversing a directed edge at a timestep.
bool AStarPlanner::violatesEdgeConstraint(
    const Point& from,
    const Point& to,
    int time_step,
    const ConstraintIndex& constraint_index) const {
  return constraint_index.edge_constraints.find({from, to, time_step}) !=
      constraint_index.edge_constraints.end();
}
