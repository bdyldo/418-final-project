#include "collision_detector.h"

#include <algorithm>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace {

//Hash function for Point struct
struct PointHash {
  std::size_t operator()(const Point& point) const {
    const std::uint64_t row_bits =
        static_cast<std::uint32_t>(point.row);
    const std::uint64_t col_bits =
        static_cast<std::uint32_t>(point.col);
    return static_cast<std::size_t>((row_bits << 32) ^ col_bits);
  }
};

struct EdgeKey {
  Point from;
  Point to;
};

bool operator==(const EdgeKey& lhs, const EdgeKey& rhs) {
  return lhs.from == rhs.from && lhs.to == rhs.to;
}

//Hash function for EdgeKey struct
struct EdgeKeyHash {
  std::size_t operator()(const EdgeKey& edge) const {
    const PointHash point_hash;
    const std::size_t from_hash = point_hash(edge.from);
    const std::size_t to_hash = point_hash(edge.to);
    return from_hash ^ (to_hash + 0x9e3779b9 + (from_hash << 6) + (from_hash >> 2));
  }
};

struct RobotMove {
  int robot_id;
  Point from;
  Point to;
};

// Formats a grid point for human-readable collision messages.
std::string pointString(const Point& point) {
  return "(" + std::to_string(point.row) + "," + std::to_string(point.col) + ")";
}

}  // namespace

// Returns the earliest collision in time order, which is the only collision CBS needs to branch on.
std::optional<Collision> CollisionDetector::findFirstCollision(
    const std::vector<Robot>& robots) const {
  const int time_steps = maxPathLength(robots);

  for (int time_step = 0; time_step < time_steps; ++time_step) {
    std::unordered_map<Point, std::vector<int>, PointHash> occupied_cells;
    occupied_cells.reserve(robots.size());

    for (const Robot& robot : robots) {
      occupied_cells[positionAtTime(robot, time_step)].push_back(robot.id);
    }

    for (const auto& entry : occupied_cells) {
      if (entry.second.size() < 2) {
        continue;
      }

      return Collision{
          CollisionType::Vertex,
          time_step,
          entry.second[0],
          entry.second[1],
          entry.first,
          entry.first,
          entry.first,
          entry.first,
          entry.first,
      };
    }

    if (time_step + 1 >= time_steps) {
      continue;
    }

    std::unordered_map<EdgeKey, std::vector<RobotMove>, EdgeKeyHash> moves_by_edge;
    moves_by_edge.reserve(robots.size());

    for (const Robot& robot : robots) {
      const Point from = positionAtTime(robot, time_step);
      const Point to = positionAtTime(robot, time_step + 1);

      if (from == to) {
        continue;
      }

      const EdgeKey reverse_edge = {to, from};
      const auto reverse_it = moves_by_edge.find(reverse_edge);
      if (reverse_it != moves_by_edge.end()) {
        const RobotMove& other_move = reverse_it->second.front();
        return Collision{
            CollisionType::Edge,
            time_step,
            other_move.robot_id,
            robot.id,
            to,
            other_move.from,
            other_move.to,
            from,
            to,
        };
      }

      moves_by_edge[{from, to}].push_back({robot.id, from, to});
    }
  }

  return std::nullopt;
}

// Scans all timesteps for vertex collisions and edge-swap collisions.
std::vector<Collision> CollisionDetector::detectCollisions(
    const std::vector<Robot>& robots) const {
  std::vector<Collision> collisions;
  const int time_steps = maxPathLength(robots);

  for (int time_step = 0; time_step < time_steps; ++time_step) {
    detectVertexCollisions(robots, time_step, collisions);
  }

  for (int time_step = 0; time_step + 1 < time_steps; ++time_step) {
    detectEdgeCollisions(robots, time_step, collisions);
  }

  return collisions;
}

// Prints a readable summary of all detected collisions.
void CollisionDetector::printCollisions(
    const std::vector<Collision>& collisions,
    std::ostream& out) const {
  if (collisions.empty()) {
    out << "\nNo collisions detected.\n";
    return;
  }

  out << "\nDetected " << collisions.size() << " collision(s):\n";
  for (const Collision& collision : collisions) {
    out << "t=" << collision.time_step << ": R" << collision.robot_a
        << " and R" << collision.robot_b << " ";

    if (collision.type == CollisionType::Vertex) {
      out << "share cell " << pointString(collision.location) << "\n";
    } else {
      out << "swap edges R" << collision.robot_a << " "
          << pointString(collision.from_a) << "->"
          << pointString(collision.to_a) << " and R" << collision.robot_b
          << " " << pointString(collision.from_b) << "->"
          << pointString(collision.to_b) << "\n";
    }
  }
}

// Returns the longest robot path length, which determines the time horizon.
int CollisionDetector::maxPathLength(const std::vector<Robot>& robots) const {
  int max_length = 0;
  for (const Robot& robot : robots) {
    max_length = std::max(max_length, static_cast<int>(robot.path.size()));
  }
  return max_length;
}

// Returns a robot's position at a timestep, treating completed paths as waiting at the goal.
Point CollisionDetector::positionAtTime(const Robot& robot, int time_step) const {
  if (robot.path.empty()) {
    throw std::invalid_argument("robot path must contain at least one point");
  }

  if (time_step < static_cast<int>(robot.path.size())) {
    return robot.path[time_step];
  }

  return robot.path.back();
}

// Uses a hash map from cell position to robots occupying that cell at this timestep.
void CollisionDetector::detectVertexCollisions(
    const std::vector<Robot>& robots,
    int time_step,
    std::vector<Collision>& collisions) const {
  std::unordered_map<Point, std::vector<int>, PointHash> occupied_cells;
  occupied_cells.reserve(robots.size());

  for (const Robot& robot : robots) {
    occupied_cells[positionAtTime(robot, time_step)].push_back(robot.id);
  }

  for (const auto& entry : occupied_cells) {
    const Point& location = entry.first;
    const std::vector<int>& robot_ids = entry.second;

    if (robot_ids.size() < 2) {
      continue;
    }

    for (int i = 0; i < static_cast<int>(robot_ids.size()); ++i) {
      for (int j = i + 1; j < static_cast<int>(robot_ids.size()); ++j) {
        collisions.push_back({
            CollisionType::Vertex,
            time_step,
            robot_ids[i],
            robot_ids[j],
            location,
            location,
            location,
            location,
            location,
        });
      }
    }
  }
}

// Uses a hash map from directed edges to robot moves, then checks for reverse-edge swaps.
void CollisionDetector::detectEdgeCollisions(
    const std::vector<Robot>& robots,
    int time_step,
    std::vector<Collision>& collisions)     
const {
  std::unordered_map<EdgeKey, std::vector<RobotMove>, EdgeKeyHash> moves_by_edge;
  moves_by_edge.reserve(robots.size());

  for (const Robot& robot : robots) {
    const Point from = positionAtTime(robot, time_step);
    const Point to = positionAtTime(robot, time_step + 1);

    if (from == to) { //Waiting state
      continue;
    }

    const EdgeKey reverse_edge = {to, from};
    const auto reverse_it = moves_by_edge.find(reverse_edge);
    if (reverse_it != moves_by_edge.end()) {
      for (const RobotMove& other_move : reverse_it->second) {
        collisions.push_back({
            CollisionType::Edge,
            time_step,
            other_move.robot_id,
            robot.id,
            to,
            other_move.from,
            other_move.to,
            from,
            to,
        });
      }
    }

    moves_by_edge[{from, to}].push_back({robot.id, from, to});
  }
}
