#include "time_expanded_bfs.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <cstdlib>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <tuple>
#include <unordered_set>
#include <vector>

#ifdef _OPENMP
#include <omp.h>
#endif

namespace {

struct PointHash {
  std::size_t operator()(const Point& point) const {
    const std::uint64_t row_bits = static_cast<std::uint32_t>(point.row);
    const std::uint64_t col_bits = static_cast<std::uint32_t>(point.col);
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

struct MoveOption {
  Point delta;
  MoveDirection direction;
  bool is_wait;
  int base_priority;
};

struct VertexConstraintKey {
  Point location;
  int time_step;
};

struct EdgeConstraintKey {
  Point from;
  Point to;
  int time_step;
};

bool operator==(const VertexConstraintKey& lhs, const VertexConstraintKey& rhs) {
  return lhs.location == rhs.location && lhs.time_step == rhs.time_step;
}

bool operator==(const EdgeConstraintKey& lhs, const EdgeConstraintKey& rhs) {
  return lhs.from == rhs.from && lhs.to == rhs.to &&
      lhs.time_step == rhs.time_step;
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

constexpr int kDirectionCount = 5;
constexpr int kUnreachedTurns = std::numeric_limits<int>::max();

int workerCountForFrontier(int frontier_size) {
  if (frontier_size < 128) {
    return 1;
  }

#ifdef _OPENMP
  return std::max(1, std::min(omp_get_max_threads(), frontier_size / 64));
#else
  return 1;
#endif
}

int directionIndex(MoveDirection direction) {
  return static_cast<int>(direction);
}

MoveDirection directionFromIndex(int index) {
  switch (index) {
    case 0:
      return MoveDirection::Start;
    case 1:
      return MoveDirection::Up;
    case 2:
      return MoveDirection::Down;
    case 3:
      return MoveDirection::Left;
    case 4:
      return MoveDirection::Right;
    default:
      throw std::runtime_error("invalid direction index");
  }
}

}  // namespace

struct TimeExpandedBFSPlanner::ConstraintIndex {
  std::unordered_set<VertexConstraintKey, VertexConstraintKeyHash>
      vertex_constraints;
  std::unordered_set<EdgeConstraintKey, EdgeConstraintKeyHash> edge_constraints;
  int latest_constraint_end_time = 0;
};

TimeExpandedBFSPlanner::TimeExpandedBFSPlanner(int rows, int cols)
    : rows_(rows), cols_(cols) {
  if (rows_ <= 0 || cols_ <= 0) {
    throw std::invalid_argument("grid dimensions must be positive");
  }
}

std::optional<std::vector<Point>> TimeExpandedBFSPlanner::findPath(
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
  const int states_per_time = rows_ * cols_ * kDirectionCount;
  const int total_state_count = (horizon + 1) * states_per_time;
  std::vector<int> best_turn_count(total_state_count, kUnreachedTurns);
  std::vector<int> parent_indices(total_state_count, -1);
  std::vector<int> current_frontier;
  current_frontier.reserve(rows_ * cols_ * kDirectionCount);

  auto encodeStateIndex = [&](int time_step,
                              int row,
                              int col,
                              MoveDirection direction) {
    return ((time_step * rows_ + row) * cols_ + col) * kDirectionCount +
        directionIndex(direction);
  };

  auto decodeState = [&](int state_index) {
    const int direction_value = state_index % kDirectionCount;
    state_index /= kDirectionCount;
    const int col = state_index % cols_;
    state_index /= cols_;
    const int row = state_index % rows_;
    const int time_step = state_index / rows_;
    return SearchState{{row, col}, time_step, directionFromIndex(direction_value)};
  };

  auto reconstructPath = [&](int goal_state_index) {
    std::vector<Point> path;
    int current_index = goal_state_index;

    while (current_index != -1) {
      const SearchState state = decodeState(current_index);
      path.push_back(state.position);
      current_index = parent_indices[current_index];
    }

    std::reverse(path.begin(), path.end());
    return path;
  };

  const int start_index =
      encodeStateIndex(0, robot.start.row, robot.start.col, MoveDirection::Start);
  best_turn_count[start_index] = 0;
  current_frontier.push_back(start_index);
  if (stats != nullptr) {
    ++stats->states_generated;
  }

  for (int time_step = 0; time_step <= horizon; ++time_step) {
    int best_goal_state_index = -1;

    if (stats != nullptr) {
      stats->states_expanded += static_cast<long long>(current_frontier.size());
    }

    for (int state_index : current_frontier) {
      const SearchState state = decodeState(state_index);
      if (!(state.position == robot.goal) ||
          state.time_step < constraint_index.latest_constraint_end_time) {
        continue;
      }

      if (best_goal_state_index == -1 ||
          best_turn_count[state_index] < best_turn_count[best_goal_state_index] ||
          (best_turn_count[state_index] == best_turn_count[best_goal_state_index] &&
           state_index < best_goal_state_index)) {
        best_goal_state_index = state_index;
      }
    }

    if (best_goal_state_index != -1) {
      return reconstructPath(best_goal_state_index);
    }

    if (time_step == horizon || current_frontier.empty()) {
      break;
    }

    std::vector<int> next_frontier;
    next_frontier.reserve(current_frontier.size() * 2);
    const int next_time_step = time_step + 1;
    const int next_layer_offset = next_time_step * states_per_time;
    std::vector<std::atomic<int>> next_layer_turn_count(states_per_time);
    std::vector<std::atomic<int>> next_layer_parent_index(states_per_time);
    for (int layer_index = 0; layer_index < states_per_time; ++layer_index) {
      next_layer_turn_count[layer_index].store(kUnreachedTurns,
                                               std::memory_order_relaxed);
      next_layer_parent_index[layer_index].store(-1, std::memory_order_relaxed);
    }

    long long generated_updates = 0;
    const int frontier_size = static_cast<int>(current_frontier.size());
    const int frontier_worker_count = workerCountForFrontier(frontier_size);

#ifdef _OPENMP
#pragma omp parallel for if(frontier_worker_count > 1) num_threads(frontier_worker_count) schedule(static) reduction(+ : generated_updates)
#endif
    for (int frontier_index = 0; frontier_index < frontier_size; ++frontier_index) {
      const int state_index = current_frontier[frontier_index];
      const SearchState state = decodeState(state_index);
      const int current_turn_count = best_turn_count[state_index];
      const std::array<MoveOption, 5> moves = orderedMoves(state, robot.goal);

      for (const MoveOption& move : moves) {
        const Point next_position = {
            state.position.row + move.delta.row,
            state.position.col + move.delta.col,
        };
        if (!isInBounds(next_position)) {
          continue;
        }

        if (violatesVertexConstraint(next_position,
                                     next_time_step,
                                     constraint_index)) {
          continue;
        }
        if (violatesEdgeConstraint(state.position,
                                   next_position,
                                   state.time_step,
                                   constraint_index)) {
          continue;
        }

        const MoveDirection next_direction = move.direction;
        const int next_state_index = encodeStateIndex(next_time_step,
                                                      next_position.row,
                                                      next_position.col,
                                                      next_direction);
        const int next_turn_count = current_turn_count +
            (move.is_wait ? 0 : turnPenalty(state.direction, next_direction));
        const int next_layer_index = next_state_index - next_layer_offset;

        int observed_turn_count =
            next_layer_turn_count[next_layer_index].load(std::memory_order_relaxed);
        bool improved = false;
        while (next_turn_count < observed_turn_count) {
          if (next_layer_turn_count[next_layer_index].compare_exchange_weak(
                  observed_turn_count,
                  next_turn_count,
                  std::memory_order_relaxed,
                  std::memory_order_relaxed)) {
            improved = true;
            break;
          }
        }

        if (!improved) {
          continue;
        }

        next_layer_parent_index[next_layer_index].store(state_index,
                                                        std::memory_order_relaxed);
        ++generated_updates;
      }
    }

    for (int layer_index = 0; layer_index < states_per_time; ++layer_index) {
      const int layer_turn_count =
          next_layer_turn_count[layer_index].load(std::memory_order_relaxed);
      if (layer_turn_count == kUnreachedTurns) {
        continue;
      }

      const int next_state_index = next_layer_offset + layer_index;
      best_turn_count[next_state_index] = layer_turn_count;
      parent_indices[next_state_index] =
          next_layer_parent_index[layer_index].load(std::memory_order_relaxed);
      next_frontier.push_back(next_state_index);
    }

    if (stats != nullptr) {
      stats->states_generated += generated_updates;
    }

    current_frontier = std::move(next_frontier);
  }

  return std::nullopt;
}

bool TimeExpandedBFSPlanner::isInBounds(const Point& point) const {
  return point.row >= 0 && point.row < rows_ &&
      point.col >= 0 && point.col < cols_;
}

int TimeExpandedBFSPlanner::manhattanDistance(const Point& lhs,
                                              const Point& rhs) const {
  return std::abs(lhs.row - rhs.row) + std::abs(lhs.col - rhs.col);
}

TimeExpandedBFSPlanner::ConstraintIndex TimeExpandedBFSPlanner::buildConstraintIndex(
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

int TimeExpandedBFSPlanner::searchHorizon(
    const Robot& robot,
    const ConstraintIndex& constraint_index) const {
  const int base_distance = manhattanDistance(robot.start, robot.goal);
  const int detour_slack = rows_ + cols_ + 10;
  return std::max(base_distance, constraint_index.latest_constraint_end_time) +
      detour_slack;
}

bool TimeExpandedBFSPlanner::violatesVertexConstraint(
    const Point& location,
    int time_step,
    const ConstraintIndex& constraint_index) const {
  return constraint_index.vertex_constraints.find({location, time_step}) !=
      constraint_index.vertex_constraints.end();
}

bool TimeExpandedBFSPlanner::violatesEdgeConstraint(
    const Point& from,
    const Point& to,
    int time_step,
    const ConstraintIndex& constraint_index) const {
  return constraint_index.edge_constraints.find({from, to, time_step}) !=
      constraint_index.edge_constraints.end();
}
