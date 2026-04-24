#include "a_star.h"
#include "bitset_wavefront.h"
#include "collision_detector.h"
#include "greedy_repair.h"
#include "grid_planner.h"
#include "parallel_greedy_repair.h"
#include "time_expanded_bfs.h"

#include <exception>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

void require(bool condition, const std::string& message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

void expectNoCollisions(const std::vector<Robot>& robots,
                        const std::string& test_name) {
  CollisionDetector detector;
  const std::vector<Collision> collisions = detector.detectCollisions(robots);
  require(collisions.empty(), test_name + ": expected no collisions");
}

void expectOneVertexCollision(const std::vector<Robot>& robots,
                              int expected_time,
                              const Point& expected_location,
                              const std::string& test_name) {
  CollisionDetector detector;
  const std::vector<Collision> collisions = detector.detectCollisions(robots);

  require(collisions.size() == 1,
          test_name + ": expected exactly one collision");
  require(collisions[0].type == CollisionType::Vertex,
          test_name + ": expected a vertex collision");
  require(collisions[0].time_step == expected_time,
          test_name + ": wrong vertex collision time");
  require(collisions[0].location == expected_location,
          test_name + ": wrong vertex collision location");
}

void expectOneEdgeCollision(const std::vector<Robot>& robots,
                            int expected_time,
                            const Point& expected_from_a,
                            const Point& expected_to_a,
                            const std::string& test_name) {
  CollisionDetector detector;
  const std::vector<Collision> collisions = detector.detectCollisions(robots);

  require(collisions.size() == 1,
          test_name + ": expected exactly one collision");
  require(collisions[0].type == CollisionType::Edge,
          test_name + ": expected an edge collision");
  require(collisions[0].time_step == expected_time,
          test_name + ": wrong edge collision time");
  require(collisions[0].from_a == expected_from_a &&
              collisions[0].to_a == expected_to_a,
          test_name + ": wrong edge collision move");
}

void testPlannerCreatesRowRobots() {
  GridPlanner planner(3, 4);
  planner.createRowRobots();

  const std::vector<Robot>& robots = planner.robots();
  require(robots.size() == 3, "planner: expected one robot per row");

  require(robots[0].start == Point{0, 0}, "planner: wrong start for robot 1");
  require(robots[0].goal == Point{0, 3}, "planner: wrong goal for robot 1");
  require(robots[1].start == Point{1, 0}, "planner: wrong start for robot 2");
  require(robots[1].goal == Point{1, 3}, "planner: wrong goal for robot 2");
  require(robots[2].path.size() == 4, "planner: wrong path length");
  require(robots[2].path.front() == Point{2, 0},
          "planner: wrong first path point");
  require(robots[2].path.back() == Point{2, 3},
          "planner: wrong last path point");
}

void testNoCollisionCase() {
  const std::vector<Robot> robots = {
      {1, {0, 0}, {0, 2}, {{0, 0}, {0, 1}, {0, 2}}},
      {2, {1, 0}, {1, 2}, {{1, 0}, {1, 1}, {1, 2}}},
  };

  expectNoCollisions(robots, "no_collision");
}

void testVertexCollisionCase() {
  const std::vector<Robot> robots = {
      {1, {0, 0}, {0, 2}, {{0, 0}, {0, 1}, {0, 2}}},
      {2, {0, 2}, {0, 0}, {{0, 2}, {0, 1}, {0, 0}}},
  };

  expectOneVertexCollision(robots, 1, {0, 1}, "vertex_collision");
}

void testEdgeCollisionCase() {
  const std::vector<Robot> robots = {
      {1, {1, 0}, {1, 1}, {{1, 0}, {1, 1}}},
      {2, {1, 1}, {1, 0}, {{1, 1}, {1, 0}}},
  };

  expectOneEdgeCollision(robots, 0, {1, 0}, {1, 1}, "edge_collision");
}

void testWaitAtGoalCollisionCase() {
  const std::vector<Robot> robots = {
      {1, {0, 0}, {0, 1}, {{0, 0}, {0, 1}}},
      {2, {0, 2}, {0, 1}, {{0, 2}, {0, 2}, {0, 1}}},
  };

  expectOneVertexCollision(robots, 2, {0, 1}, "wait_at_goal_collision");
}

void testAStarFindsShortestPathWithoutConstraints() {
  const Robot robot = {1, {0, 0}, {2, 2}, {}};
  AStarPlanner planner(3, 3);
  const auto path = planner.findPath(robot, {});

  require(path.has_value(), "a_star_shortest_path: expected a path");
  require(path->front() == Point{0, 0},
          "a_star_shortest_path: wrong start");
  require(path->back() == Point{2, 2},
          "a_star_shortest_path: wrong goal");
  require(path->size() == 5,
          "a_star_shortest_path: wrong shortest path length");
}

void testAStarRespectsVertexConstraint() {
  const Robot robot = {1, {0, 0}, {0, 2}, {}};
  const std::vector<Constraint> constraints = {
      {ConstraintType::Vertex, 1, 1, {0, 1}, {0, 1}, {0, 1}},
  };

  AStarPlanner planner(2, 3);
  const auto path = planner.findPath(robot, constraints);

  require(path.has_value(), "a_star_constraint: expected a path");
  require(path->size() == 4,
          "a_star_constraint: expected a delayed path");
  require((*path)[0] == Point{0, 0}, "a_star_constraint: wrong start");
  require((*path)[1] == Point{0, 0},
          "a_star_constraint: expected wait to avoid constraint");
  require(path->back() == Point{0, 2}, "a_star_constraint: wrong goal");
}

void testTimeExpandedBFSFindsShortestPathWithoutConstraints() {
  const Robot robot = {1, {0, 0}, {2, 2}, {}};
  TimeExpandedBFSPlanner planner(3, 3);
  const auto path = planner.findPath(robot, {});

  require(path.has_value(), "time_expanded_bfs_shortest_path: expected a path");
  require(path->front() == Point{0, 0},
          "time_expanded_bfs_shortest_path: wrong start");
  require(path->back() == Point{2, 2},
          "time_expanded_bfs_shortest_path: wrong goal");
  require(path->size() == 5,
          "time_expanded_bfs_shortest_path: wrong shortest path length");
}

void testTimeExpandedBFSRespectsVertexConstraint() {
  const Robot robot = {1, {0, 0}, {0, 2}, {}};
  const std::vector<Constraint> constraints = {
      {ConstraintType::Vertex, 1, 1, {0, 1}, {0, 1}, {0, 1}},
  };

  TimeExpandedBFSPlanner planner(2, 3);
  const auto path = planner.findPath(robot, constraints);

  require(path.has_value(), "time_expanded_bfs_constraint: expected a path");
  require(path->size() == 4,
          "time_expanded_bfs_constraint: expected a delayed path");
  require((*path)[0] == Point{0, 0},
          "time_expanded_bfs_constraint: wrong start");
  require((*path)[1] == Point{0, 0},
          "time_expanded_bfs_constraint: expected wait to avoid constraint");
  require(path->back() == Point{0, 2},
          "time_expanded_bfs_constraint: wrong goal");
}

void testBitsetWavefrontFindsShortestPathWithoutConstraints() {
  const Robot robot = {1, {0, 0}, {2, 2}, {}};
  BitsetWavefrontPlanner planner(3, 3);
  const auto path = planner.findPath(robot, {});

  require(path.has_value(), "bitset_wavefront_shortest_path: expected a path");
  require(path->front() == Point{0, 0},
          "bitset_wavefront_shortest_path: wrong start");
  require(path->back() == Point{2, 2},
          "bitset_wavefront_shortest_path: wrong goal");
  require(path->size() == 5,
          "bitset_wavefront_shortest_path: wrong shortest path length");
}

void testBitsetWavefrontRespectsVertexConstraint() {
  const Robot robot = {1, {0, 0}, {0, 2}, {}};
  const std::vector<Constraint> constraints = {
      {ConstraintType::Vertex, 1, 1, {0, 1}, {0, 1}, {0, 1}},
  };

  BitsetWavefrontPlanner planner(2, 3);
  const auto path = planner.findPath(robot, constraints);

  require(path.has_value(), "bitset_wavefront_constraint: expected a path");
  require(path->size() == 4,
          "bitset_wavefront_constraint: expected a delayed path");
  require((*path)[0] == Point{0, 0},
          "bitset_wavefront_constraint: wrong start");
  require((*path)[1] == Point{0, 0},
          "bitset_wavefront_constraint: expected wait to avoid constraint");
  require(path->back() == Point{0, 2},
          "bitset_wavefront_constraint: wrong goal");
}

void testGreedyRepairResolvesSimpleSwap() {
  std::vector<Robot> robots = {
      {1, {0, 0}, {0, 1}, {}},
      {2, {0, 1}, {0, 0}, {}},
  };

  GreedyRepairPlanner planner(2, 2, 10000, 4);
  const auto solution = planner.findPaths(robots);

  require(solution.has_value(), "greedy_simple_swap: expected a solution");

  CollisionDetector detector;
  const std::vector<Collision> collisions =
      detector.detectCollisions(*solution);
  require(collisions.empty(),
          "greedy_simple_swap: solution still has collisions");
  require((*solution)[0].path.front() == Point{0, 0},
          "greedy_simple_swap: wrong robot 1 start");
  require((*solution)[0].path.back() == Point{0, 1},
          "greedy_simple_swap: wrong robot 1 goal");
  require((*solution)[1].path.front() == Point{0, 1},
          "greedy_simple_swap: wrong robot 2 start");
  require((*solution)[1].path.back() == Point{0, 0},
          "greedy_simple_swap: wrong robot 2 goal");
}

void testGreedyRepairBatchesDisjointConflicts() {
  std::vector<Robot> robots = {
      {1, {2, 0}, {1, 0}, {}},
      {2, {1, 1}, {2, 2}, {}},
      {3, {0, 2}, {1, 2}, {}},
      {4, {0, 0}, {2, 1}, {}},
  };

  GreedyRepairPlanner planner(3, 3, 10000, 4);
  GreedyRepairStats stats;
  const auto solution = planner.findPaths(robots, &stats);

  require(solution.has_value(), "greedy_batch: expected a solution");

  CollisionDetector detector;
  const std::vector<Collision> collisions =
      detector.detectCollisions(*solution);
  require(collisions.empty(),
          "greedy_async: solution still has collisions");
  require(stats.successful_repairs > 0,
          "greedy_async: expected at least one committed repair");
}

void testParallelGreedyRepairResolvesSimpleSwap() {
  std::vector<Robot> robots = {
      {1, {0, 0}, {0, 1}, {}},
      {2, {0, 1}, {0, 0}, {}},
  };

  ParallelGreedyRepairPlanner planner(2, 2, 10000, 8, 8, 1);
  GreedyRepairStats stats;
  const auto solution = planner.findPaths(robots, &stats);

  require(solution.has_value(), "parallel_greedy_simple_swap: expected a solution");

  CollisionDetector detector;
  const std::vector<Collision> collisions =
      detector.detectCollisions(*solution);
  require(collisions.empty(),
          "parallel_greedy_simple_swap: solution still has collisions");
  require(stats.candidate_repairs_evaluated > 0,
          "parallel_greedy_simple_swap: expected candidate repair work");
}

void runTest(void (*test_fn)(), const std::string& test_name) {
  test_fn();
  std::cout << "[PASS] " << test_name << "\n";
}

}  // namespace

int main() {
  try {
    runTest(testPlannerCreatesRowRobots, "planner_creates_row_robots");
    runTest(testNoCollisionCase, "no_collision");
    runTest(testVertexCollisionCase, "vertex_collision");
    runTest(testEdgeCollisionCase, "edge_collision");
    runTest(testWaitAtGoalCollisionCase, "wait_at_goal_collision");
    runTest(testAStarFindsShortestPathWithoutConstraints,
            "a_star_shortest_path");
    runTest(testAStarRespectsVertexConstraint,
            "a_star_constraint");
    runTest(testTimeExpandedBFSFindsShortestPathWithoutConstraints,
            "time_expanded_bfs_shortest_path");
    runTest(testTimeExpandedBFSRespectsVertexConstraint,
            "time_expanded_bfs_constraint");
    runTest(testBitsetWavefrontFindsShortestPathWithoutConstraints,
            "bitset_wavefront_shortest_path");
    runTest(testBitsetWavefrontRespectsVertexConstraint,
            "bitset_wavefront_constraint");
    runTest(testGreedyRepairResolvesSimpleSwap, "greedy_simple_swap");
    runTest(testGreedyRepairBatchesDisjointConflicts,
            "greedy_async_multi_conflict");
    runTest(testParallelGreedyRepairResolvesSimpleSwap,
            "parallel_greedy_simple_swap");
  } catch (const std::exception& error) {
    std::cerr << "[FAIL] " << error.what() << "\n";
    return 1;
  }

  std::cout << "All project tests passed.\n";
  return 0;
}
