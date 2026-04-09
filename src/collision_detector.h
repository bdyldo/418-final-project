#ifndef COLLISION_DETECTOR_H
#define COLLISION_DETECTOR_H

#include "grid_planner.h"

#include <optional>
#include <ostream>
#include <vector>

enum class CollisionType {
  Vertex,
  Edge,
};

struct Collision {
  CollisionType type;
  int time_step;
  int robot_a;
  int robot_b;
  Point location;
  Point from_a;
  Point to_a;
  Point from_b;
  Point to_b;
};

class CollisionDetector {
 public:
  std::optional<Collision> findFirstCollision(
      const std::vector<Robot>& robots) const;
  std::vector<Collision> detectCollisions(
      const std::vector<Robot>& robots) const;
  void printCollisions(const std::vector<Collision>& collisions,
                       std::ostream& out) const;

 private:
  int maxPathLength(const std::vector<Robot>& robots) const;
  Point positionAtTime(const Robot& robot, int time_step) const;
  void detectVertexCollisions(const std::vector<Robot>& robots,
                              int time_step,
                              std::vector<Collision>& collisions) const;
  void detectEdgeCollisions(const std::vector<Robot>& robots,
                            int time_step,
                            std::vector<Collision>& collisions) const;
};

#endif
