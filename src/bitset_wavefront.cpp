/*
 * BitsetWavefrontPlanner is a low-level space-time planner for one robot under
 * CBS-style constraints. It packs each time layer of the grid into 64-bit row
 * chunks and performs a bit-parallel wavefront expansion over wait/right/left/
 * down/up moves. Vertex and edge constraints are pre-indexed into directional
 * bit masks so each expansion step can apply legality checks with fast bitwise
 * operations. The planner stores one parent-move label per reached cell-time
 * state, then reconstructs a concrete path once the goal is reachable at or
 * beyond the latest relevant constraint time. OpenMP row parallelism is used
 * for mask generation when it is beneficial.
 */
#include "bitset_wavefront.h"

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstdlib>
#include <stdexcept>
#include <vector>

#ifdef _OPENMP
#include <omp.h>
#endif

namespace {

// A numerical rep of sets of all possible moves from iteraation i - 1
enum ParentMove : std::int8_t {
  ParentUnset = -1,
  ParentStart = 0,
  ParentWait = 1,
  ParentFromLeft = 2,
  ParentFromRight = 3,
  ParentFromUp = 4,
  ParentFromDown = 5,
};

struct MoveDescriptor {
  ParentMove parent_move;
  const std::vector<std::uint64_t>* masks;
};

int workerCountForRows(int rows) {
  if (rows < 16) {
    return 1;
  }

#ifdef _OPENMP
  if (omp_get_active_level() > 0) {
    return 1;
  }
  return std::max(1, std::min(omp_get_max_threads(), rows));
#else
  return 1;
#endif
}

// Returns a per-chunk validity mask so bits beyond the grid width are zeroed
// in the last 64-bit chunk, while full chunks remain all-ones.
std::uint64_t validMaskForChunk(int chunk_index, int chunk_count, int cols) {
  if (chunk_index + 1 != chunk_count) {
    return ~std::uint64_t{0};
  }
  const int remainder = cols % 64;
  if (remainder == 0) {
    return ~std::uint64_t{0};
  }
  return (std::uint64_t{1} << remainder) - 1;
}

void shiftRowLeft(const std::uint64_t* source,
                  std::uint64_t* destination,
                  int chunk_count,
                  int cols) {
  std::uint64_t carry = 0;
  for (int chunk_index = 0; chunk_index < chunk_count; ++chunk_index) {
    const std::uint64_t value = source[chunk_index];
    const std::uint64_t next_carry = value >> 63;
    destination[chunk_index] =
        ((value << 1) | carry) & validMaskForChunk(chunk_index, chunk_count, cols);
    carry = next_carry;
  }
}

void shiftRowRight(const std::uint64_t* source,
                   std::uint64_t* destination,
                   int chunk_count) {
  std::uint64_t carry = 0;
  for (int chunk_index = chunk_count - 1; chunk_index >= 0; --chunk_index) {
    const std::uint64_t value = source[chunk_index];
    const std::uint64_t next_carry = (value & 1ULL) << 63;
    destination[chunk_index] = (value >> 1) | carry;
    carry = next_carry;
  }
}

// Builds an expansion order biased toward the goal direction (horizontal vs
// vertical first), with opposite-direction moves next and wait last.
std::vector<MoveDescriptor> movePriority(const Robot& robot,
                                         const std::vector<std::uint64_t>& wait_masks,
                                         const std::vector<std::uint64_t>& right_masks,
                                         const std::vector<std::uint64_t>& left_masks,
                                         const std::vector<std::uint64_t>& down_masks,
                                         const std::vector<std::uint64_t>& up_masks) {
  const int row_delta = robot.goal.row - robot.start.row;
  const int col_delta = robot.goal.col - robot.start.col;
  const bool prefer_horizontal = std::abs(col_delta) >= std::abs(row_delta);

  const MoveDescriptor positive_horizontal = {
      col_delta >= 0 ? ParentFromLeft : ParentFromRight,
      col_delta >= 0 ? &right_masks : &left_masks,
  };
  const MoveDescriptor negative_horizontal = {
      col_delta >= 0 ? ParentFromRight : ParentFromLeft,
      col_delta >= 0 ? &left_masks : &right_masks,
  };
  const MoveDescriptor positive_vertical = {
      row_delta >= 0 ? ParentFromUp : ParentFromDown,
      row_delta >= 0 ? &down_masks : &up_masks,
  };
  const MoveDescriptor negative_vertical = {
      row_delta >= 0 ? ParentFromDown : ParentFromUp,
      row_delta >= 0 ? &up_masks : &down_masks,
  };
  const MoveDescriptor wait_move = {
      ParentWait,
      &wait_masks,
  };

  if (prefer_horizontal) {
    return {
        positive_horizontal,
        positive_vertical,
        negative_vertical,
        negative_horizontal,
        wait_move,
    };
  }

  return {
      positive_vertical,
      positive_horizontal,
      negative_horizontal,
      negative_vertical,
      wait_move,
  };
}

}  // namespace

struct BitsetWavefrontPlanner::ConstraintIndex {
  std::vector<std::uint64_t> vertex_forbidden;
  std::vector<std::uint64_t> blocked_right;
  std::vector<std::uint64_t> blocked_left;
  std::vector<std::uint64_t> blocked_down;
  std::vector<std::uint64_t> blocked_up;
  int latest_constraint_end_time = 0;
};

BitsetWavefrontPlanner::BitsetWavefrontPlanner(int rows, int cols)
    : rows_(rows), cols_(cols) {
  if (rows_ <= 0 || cols_ <= 0) {
    throw std::invalid_argument("grid dimensions must be positive");
  }
}

// Runs bit-parallel space-time search for a single robot: build constraint
// masks, expand reachable cells layer-by-layer, and reconstruct a path from
// recorded parent moves once the goal is reached at a valid timestep.
std::optional<std::vector<Point>> BitsetWavefrontPlanner::findPath(
    const Robot& robot,
    const std::vector<Constraint>& constraints,
    AStarStats* stats) const {
  if (!isInBounds(robot.start) || !isInBounds(robot.goal)) {
    throw std::invalid_argument("robot start and goal must lie inside the grid");
  }

  if (stats != nullptr) {
    *stats = {};
  }

  const int chunk_count = (cols_ + 63) / 64;
  const int row_stride = chunk_count;
  const int layer_word_count = rows_ * row_stride;
  const int horizon_guess = rows_ + cols_ + std::abs(robot.goal.row - robot.start.row) +
      std::abs(robot.goal.col - robot.start.col) + 10;
  const ConstraintIndex preliminary_index =
      buildConstraintIndex(robot.id, constraints, horizon_guess);
  const int horizon = searchHorizon(robot, preliminary_index);
  const ConstraintIndex constraint_index =
      buildConstraintIndex(robot.id, constraints, horizon);

  auto wordIndex = [&](int time_step, int row, int chunk_index) {
    return (time_step * rows_ + row) * row_stride + chunk_index;
  };
  auto bitMask = [&](int col) {
    return std::uint64_t{1} << (col % 64);
  };
  auto cellIndex = [&](int time_step, int row, int col) {
    return (time_step * rows_ + row) * cols_ + col;
  };

  const int start_vertex_index = wordIndex(0,
                                           robot.start.row,
                                           robot.start.col / 64);
  if ((constraint_index.vertex_forbidden[start_vertex_index] &
       bitMask(robot.start.col)) != 0) {
    return std::nullopt;
  }

  std::vector<std::uint64_t> current(layer_word_count, 0);
  std::vector<std::uint64_t> next(layer_word_count, 0);
  std::vector<std::uint64_t> wait_masks(layer_word_count, 0);
  std::vector<std::uint64_t> right_masks(layer_word_count, 0);
  std::vector<std::uint64_t> left_masks(layer_word_count, 0);
  std::vector<std::uint64_t> down_masks(layer_word_count, 0);
  std::vector<std::uint64_t> up_masks(layer_word_count, 0);
  std::vector<std::int8_t> parent_move(
      (horizon + 1) * rows_ * cols_, ParentUnset);
  parent_move[cellIndex(0, robot.start.row, robot.start.col)] = ParentStart;
  current[robot.start.row * row_stride + robot.start.col / 64] |= bitMask(robot.start.col);
  if (stats != nullptr) {
    ++stats->states_generated;
  }

  const std::vector<MoveDescriptor> move_order =
      movePriority(robot, wait_masks, right_masks, left_masks, down_masks, up_masks);
  const int worker_count = workerCountForRows(rows_);

  for (int time_step = 0; time_step <= horizon; ++time_step) {
    const int goal_word_index =
        robot.goal.row * row_stride + robot.goal.col / 64;
    if ((current[goal_word_index] & bitMask(robot.goal.col)) != 0 &&
        time_step >= constraint_index.latest_constraint_end_time) {
      std::vector<Point> path(time_step + 1);
      int row = robot.goal.row;
      int col = robot.goal.col;
      for (int t = time_step; t >= 0; --t) {
        path[t] = {row, col};
        const ParentMove move =
            static_cast<ParentMove>(parent_move[cellIndex(t, row, col)]);
        if (move == ParentStart) {
          break;
        }
        if (move == ParentFromLeft) {
          --col;
        } else if (move == ParentFromRight) {
          ++col;
        } else if (move == ParentFromUp) {
          --row;
        } else if (move == ParentFromDown) {
          ++row;
        } else if (move != ParentWait) {
          throw std::runtime_error("invalid parent move during reconstruction");
        }
      }
      return path;
    }

    if (time_step == horizon) {
      break;
    }

    if (stats != nullptr) {
      for (std::uint64_t word : current) {
        stats->states_expanded += static_cast<long long>(__builtin_popcountll(word));
      }
    }

    std::fill(wait_masks.begin(), wait_masks.end(), 0);
    std::fill(right_masks.begin(), right_masks.end(), 0);
    std::fill(left_masks.begin(), left_masks.end(), 0);
    std::fill(down_masks.begin(), down_masks.end(), 0);
    std::fill(up_masks.begin(), up_masks.end(), 0);
    std::fill(next.begin(), next.end(), 0);

#ifdef _OPENMP
#pragma omp parallel for if(worker_count > 1) num_threads(worker_count) schedule(static)
#endif
    for (int row = 0; row < rows_; ++row) {
      const int row_offset = row * row_stride;
      const int next_time_offset = (time_step + 1) * rows_ * row_stride;
      const int current_time_offset = time_step * rows_ * row_stride;
      std::vector<std::uint64_t> filtered(row_stride, 0);

      for (int chunk_index = 0; chunk_index < row_stride; ++chunk_index) {
        const std::uint64_t current_word = current[row_offset + chunk_index];
        const std::uint64_t next_vertex_mask =
            constraint_index.vertex_forbidden[next_time_offset + row_offset + chunk_index];
        wait_masks[row_offset + chunk_index] = current_word & ~next_vertex_mask;
        filtered[chunk_index] =
            current_word &
            ~constraint_index.blocked_right[current_time_offset + row_offset + chunk_index];
      }
      shiftRowLeft(filtered.data(),
                   right_masks.data() + row_offset,
                   row_stride,
                   cols_);
      for (int chunk_index = 0; chunk_index < row_stride; ++chunk_index) {
        right_masks[row_offset + chunk_index] &=
            ~constraint_index.vertex_forbidden[next_time_offset + row_offset + chunk_index];
      }

      for (int chunk_index = 0; chunk_index < row_stride; ++chunk_index) {
        filtered[chunk_index] =
            current[row_offset + chunk_index] &
            ~constraint_index.blocked_left[current_time_offset + row_offset + chunk_index];
      }
      shiftRowRight(filtered.data(), left_masks.data() + row_offset, row_stride);
      for (int chunk_index = 0; chunk_index < row_stride; ++chunk_index) {
        left_masks[row_offset + chunk_index] &=
            ~constraint_index.vertex_forbidden[next_time_offset + row_offset + chunk_index];
      }

      if (row + 1 < rows_) {
        const int target_row_offset = (row + 1) * row_stride;
        for (int chunk_index = 0; chunk_index < row_stride; ++chunk_index) {
          down_masks[target_row_offset + chunk_index] =
              (current[row_offset + chunk_index] &
               ~constraint_index.blocked_down[current_time_offset + row_offset + chunk_index]) &
              ~constraint_index.vertex_forbidden[next_time_offset + target_row_offset + chunk_index];
        }
      }

      if (row - 1 >= 0) {
        const int target_row_offset = (row - 1) * row_stride;
        for (int chunk_index = 0; chunk_index < row_stride; ++chunk_index) {
          up_masks[target_row_offset + chunk_index] =
              (current[row_offset + chunk_index] &
               ~constraint_index.blocked_up[current_time_offset + row_offset + chunk_index]) &
              ~constraint_index.vertex_forbidden[next_time_offset + target_row_offset + chunk_index];
        }
      }
    }

    long long generated_this_layer = 0;
    for (const MoveDescriptor& move : move_order) {
      const std::vector<std::uint64_t>& candidate_masks = *move.masks;
      for (int row = 0; row < rows_; ++row) {
        const int row_offset = row * row_stride;
        for (int chunk_index = 0; chunk_index < row_stride; ++chunk_index) {
          std::uint64_t new_bits =
              candidate_masks[row_offset + chunk_index] &
              ~next[row_offset + chunk_index];
          if (new_bits == 0) {
            continue;
          }

          next[row_offset + chunk_index] |= new_bits;
          generated_this_layer += __builtin_popcountll(new_bits);
          while (new_bits != 0) {
            const int bit_index = __builtin_ctzll(new_bits);
            const int col = chunk_index * 64 + bit_index;
            if (col >= cols_) {
              break;
            }
            parent_move[cellIndex(time_step + 1, row, col)] = move.parent_move;
            new_bits &= (new_bits - 1);
          }
        }
      }
    }

    if (stats != nullptr) {
      stats->states_generated += generated_this_layer;
    }

    current.swap(next);
  }

  return std::nullopt;
}

bool BitsetWavefrontPlanner::isInBounds(const Point& point) const {
  return point.row >= 0 && point.row < rows_ &&
      point.col >= 0 && point.col < cols_;
}

int BitsetWavefrontPlanner::manhattanDistance(const Point& lhs,
                                              const Point& rhs) const {
  return std::abs(lhs.row - rhs.row) + std::abs(lhs.col - rhs.col);
}

// Converts one robot's vertex/edge constraints into dense time-layered bit
// masks (vertex forbidden + directional blocked moves) used during expansion.
BitsetWavefrontPlanner::ConstraintIndex BitsetWavefrontPlanner::buildConstraintIndex(
    int robot_id,
    const std::vector<Constraint>& constraints,
    int horizon) const {
  ConstraintIndex index;
  const int chunk_count = (cols_ + 63) / 64;
  const int word_count = (horizon + 1) * rows_ * chunk_count;
  index.vertex_forbidden.assign(word_count, 0);
  index.blocked_right.assign(word_count, 0);
  index.blocked_left.assign(word_count, 0);
  index.blocked_down.assign(word_count, 0);
  index.blocked_up.assign(word_count, 0);

  auto wordIndex = [&](int time_step, int row, int col) {
    return (time_step * rows_ + row) * chunk_count + (col / 64);
  };
  auto bitMask = [&](int col) {
    return std::uint64_t{1} << (col % 64);
  };

  for (const Constraint& constraint : constraints) {
    if (constraint.robot_id != robot_id) {
      continue;
    }

    if (constraint.type == ConstraintType::Vertex) {
      if (constraint.time_step <= horizon) {
        index.vertex_forbidden[wordIndex(constraint.time_step,
                                         constraint.location.row,
                                         constraint.location.col)] |=
            bitMask(constraint.location.col);
      }
      index.latest_constraint_end_time =
          std::max(index.latest_constraint_end_time, constraint.time_step);
      continue;
    }

    if (constraint.time_step > horizon) {
      index.latest_constraint_end_time =
          std::max(index.latest_constraint_end_time, constraint.time_step + 1);
      continue;
    }

    const int blocked_index =
        wordIndex(constraint.time_step, constraint.from.row, constraint.from.col);
    if (constraint.to.col == constraint.from.col + 1 &&
        constraint.to.row == constraint.from.row) {
      index.blocked_right[blocked_index] |= bitMask(constraint.from.col);
    } else if (constraint.to.col == constraint.from.col - 1 &&
               constraint.to.row == constraint.from.row) {
      index.blocked_left[blocked_index] |= bitMask(constraint.from.col);
    } else if (constraint.to.row == constraint.from.row + 1 &&
               constraint.to.col == constraint.from.col) {
      index.blocked_down[blocked_index] |= bitMask(constraint.from.col);
    } else if (constraint.to.row == constraint.from.row - 1 &&
               constraint.to.col == constraint.from.col) {
      index.blocked_up[blocked_index] |= bitMask(constraint.from.col);
    }
    index.latest_constraint_end_time =
        std::max(index.latest_constraint_end_time, constraint.time_step + 1);
  }

  return index;
}

int BitsetWavefrontPlanner::searchHorizon(
    const Robot& robot,
    const ConstraintIndex& constraint_index) const {
  const int base_distance = manhattanDistance(robot.start, robot.goal);
  const int detour_slack = rows_ + cols_ + 10;
  return std::max(base_distance, constraint_index.latest_constraint_end_time) +
      detour_slack;
}
