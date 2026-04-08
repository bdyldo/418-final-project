#include "collision_detector.h"

#include <algorithm>
#include <stdexcept>
#include <string>

namespace{
//Formats a grid point for readable collision messages.
  std::string pointString(const Point& point){
    return "(" + std::to_string(point.row)+","+std::to_string(point.col)+")";
  }
}

//Scans all timesteps for vertex collisions and edge-swap collisions.
std::vector<Collision> CollisionDetector::detectCollisions(
    const std::vector<Robot>& robots) const {

  std::vector<Collision> collisions;
  const int time_steps = maxPathLength(robots);
  
  for (int time_step = 0; time_step < time_steps; time_step++){
    detectVertexCollisions(robots, time_step,  collisions);
  }

  for (int time_step = 0; time_step < time_steps; time_step++){
    detectEdgeCollisions(robots, time_step,  collisions);
  }

  return collisions;
}

//Prints a summary of all detected collisions.
void CollisionDetector::printCollisions(
  const std::vector<Collision>& collisions, std::ostream& out) const{

  if(collisions.empty()){
    out << "\nNo collisions detected.\n";
    return;
  }

  out << "\nDetected " << collisions.size() << " collision(s):\n";
  for(const Collision& collision : collisions){
    out << "t=" << collision.time_step << ": R" << collision.robot_a
        << " and R" << collision.robot_b << " ";
    
    if (collision.type == CollisionType::Vertex){
      out << "share cell " << pointString(collision.location) << "\n";
    }
    else{
      out << "swap edges R" << collision.robot_a << " "
      << pointString(collision.from_a) << "->"
      << pointString(collision.to_a) << " and R" << collision.robot_b
      << " " << pointString(collision.from_b) << "->"
      << pointString(collision.to_b) << "\n";
    }

  }
}

// Returns the longest robot path length
int CollisionDetector::maxPathLength(const std::vector<Robot>& robots) const{
  int max_length = 0;
  for(const Robot& robot:robots){
    max_length = std::max(max_length, static_cast<int>(robot.path.size()));
  }
  return max_length;
}

//Returns a robot's position at time t
Point CollisionDetector::positionAtTime(const Robot& robot, int time_step) const{
  if(robot.path.empty()){
    throw std::invalid_argument("Robot path must contain at least one point\n");
  }
  
  if(time_step<static_cast<int>(robot.path.size())){
    return robot.path[time_step];
  }
  return robot.path.back();
}

//Find Pairs of Robots that collide at one point at the same time
void CollisionDetector::detectVertexCollisions(
  const std::vector<Robot>& robots, int time_step,
  std::vector<Collision>& collisions) 
  const{
  for(int i=0; i < static_cast<int>(robots.size()); i++){
    const Point position_i = positionAtTime(robots[i], time_step);
    for(int j=i+1; j<static_cast<int>(robots.size()); j++){
      const Point position_j = positionAtTime(robots[j], time_step);
      if(position_i == position_j){
        collisions.push_back({
          CollisionType::Vertex,
          time_step,
          robots[i].id,
          robots[j].id,
          position_i,
          position_i,
          position_i,
          position_j,
          position_j,
        });
      }
    }
  }
}

//Find Pairs of Robots that swap cells across the same edge between two timesteps
void CollisionDetector::detectVertexCollisions(
  const std::vector<Robot>& robots, int time_step,
  std::vector<Collision>& collisions) 
  const{
  for(int i=0; i < static_cast<int>(robots.size()); i++){
    const Point from_i = positionAtTime(robots[i], time_step);
    const Point to_i = positionAtTime(robots[i], time_step+1);
    for(int j=i+1; j<static_cast<int>(robots.size()); j++){
      const Point from_j = positionAtTime(robots[j], time_step);
      const Point to_j = positionAtTime(robots[j], time_step+1);
      if (from_i == to_j && to_i == from_j){
        collisions.push_back({
          CollisionType::Edge,
          time_step,
          robots[i].id,
          robots[j].id,
          to_i,
          from_i,
          to_i,
          from_j,
          to_j,
        });
      }
    }
  }
}

