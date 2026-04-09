#ifndef BENCHMARK_CASES_H
#define BENCHMARK_CASES_H

#include "grid_planner.h"

#include <cstdint>
#include <string>
#include <vector>

struct BenchmarkCaseDefinition {
  std::string name;
  int rows;
  int cols;
  int robot_count;
  std::uint32_t seed;
};

struct BenchmarkCase {
  std::string name;
  int rows;
  int cols;
  int robot_count;
  std::vector<Robot> robots;
};

std::string benchmarkCaseStorePath();
std::vector<BenchmarkCaseDefinition> loadBenchmarkCaseDefinitions();
void saveBenchmarkCaseDefinitions(
    const std::vector<BenchmarkCaseDefinition>& definitions);
std::vector<BenchmarkCaseDefinition> selectBenchmarkCaseDefinitions(
    const std::string& target);
BenchmarkCase buildBenchmarkCase(const BenchmarkCaseDefinition& definition);
void randomizeBenchmarkCase(const std::string& case_name);

#endif
