#ifndef SOLUTION_SNAPSHOT_H
#define SOLUTION_SNAPSHOT_H

#include "grid_planner.h"

#include <string>
#include <vector>

struct StoredSolution {
  std::string case_name;
  int rows;
  int cols;
  std::vector<Robot> robots;
};

std::string defaultSolutionSnapshotPath(const std::string& case_name);
void writeSolutionSnapshot(const StoredSolution& solution,
                           const std::string& path);
StoredSolution readSolutionSnapshot(const std::string& path);

#endif
