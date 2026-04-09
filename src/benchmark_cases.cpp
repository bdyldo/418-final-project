#include "benchmark_cases.h"

#include <algorithm>
#include <cstdint>
#include <fstream>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <vector>

namespace {

// Returns the default benchmark-case metadata written when no store file exists yet.
std::vector<BenchmarkCaseDefinition> defaultBenchmarkCaseDefinitions() {
  return {
      {"few_8", 32, 32, 8, 1008},
      {"few_16", 32, 32, 16, 1016},
      {"medium_16", 128, 128, 16, 2016},
      {"medium_32", 128, 128, 32, 2032},
      {"abundant_64", 512, 512, 64, 3064},
      {"abundant_128", 512, 512, 128, 3128},
  };
}

// Hashes a grid point into one integer for uniqueness checks in the scenario generator.
int pointKey(const Point& point, int cols) {
  return point.row * cols + point.col;
}

// Samples random unique points inside a column range using a deterministic seed.
std::vector<Point> sampleUniquePoints(int rows,
                                      int cols,
                                      int robot_count,
                                      int min_col,
                                      int max_col,
                                      std::mt19937& rng,
                                      const std::vector<Point>& existing_points = {}) {
  std::uniform_int_distribution<int> row_dist(0, rows - 1);
  std::uniform_int_distribution<int> col_dist(min_col, max_col);
  std::unordered_set<int> used_points;
  for (const Point& point : existing_points) {
    used_points.insert(pointKey(point, cols));
  }

  std::vector<Point> points;
  points.reserve(robot_count);

  while (static_cast<int>(points.size()) < robot_count) {
    const Point point = {row_dist(rng), col_dist(rng)};
    const int key = pointKey(point, cols);
    if (used_points.insert(key).second) {
      points.push_back(point);
    }
  }

  return points;
}

// Builds deterministic random start and goal points that still create crossing demand.
std::vector<Robot> makeCrossingScenarioRobots(int rows,
                                              int cols,
                                              int robot_count,
                                              std::uint32_t seed) {
  if (robot_count <= 0 || robot_count % 2 != 0) {
    throw std::invalid_argument("robot count must be a positive even number");
  }

  if (robot_count >= rows * cols / 2) {
    throw std::invalid_argument("robot count is too high for the current benchmark grid");
  }

  std::mt19937 rng(seed);
  const int left_max_col = std::max(0, cols / 4);
  const int right_min_col = std::min(cols - 1, (3 * cols) / 4);

  std::vector<Point> starts =
      sampleUniquePoints(rows, cols, robot_count, 0, left_max_col, rng);
  std::vector<Point> goals =
      sampleUniquePoints(rows, cols, robot_count, right_min_col, cols - 1, rng);

  std::sort(starts.begin(), starts.end(), [](const Point& lhs, const Point& rhs) {
    if (lhs.row != rhs.row) {
      return lhs.row < rhs.row;
    }
    return lhs.col < rhs.col;
  });
  std::sort(goals.begin(), goals.end(), [](const Point& lhs, const Point& rhs) {
    if (lhs.row != rhs.row) {
      return lhs.row > rhs.row;
    }
    return lhs.col > rhs.col;
  });

  std::vector<Robot> robots;
  robots.reserve(robot_count);
  for (int i = 0; i < robot_count; ++i) {
    robots.push_back({i + 1, starts[i], goals[i], {}});
  }

  return robots;
}

// Returns whether a benchmark name belongs to a given benchmark group prefix.
bool belongsToGroup(const std::string& name, const std::string& prefix) {
  return name.rfind(prefix, 0) == 0;
}

// Finds one benchmark definition by name and throws if it does not exist.
BenchmarkCaseDefinition findDefinitionByName(
    const std::vector<BenchmarkCaseDefinition>& definitions,
    const std::string& case_name) {
  for (const BenchmarkCaseDefinition& definition : definitions) {
    if (definition.name == case_name) {
      return definition;
    }
  }

  throw std::invalid_argument(
      "benchmark target must be one of "
      "'few', 'medium', 'abundant', "
      "'few_8', 'few_16', 'medium_16', 'medium_32', "
      "'abundant_64', or 'abundant_128'");
}

}  // namespace

// Returns the on-disk location of the benchmark-case metadata store.
std::string benchmarkCaseStorePath() {
  return "benchmarks/benchmark_cases.txt";
}

// Loads the persisted benchmark-case definitions, creating the store with defaults if needed.
std::vector<BenchmarkCaseDefinition> loadBenchmarkCaseDefinitions() {
  std::ifstream input(benchmarkCaseStorePath());
  if (!input) {
    const std::vector<BenchmarkCaseDefinition> defaults =
        defaultBenchmarkCaseDefinitions();
    saveBenchmarkCaseDefinitions(defaults);
    return defaults;
  }

  std::vector<BenchmarkCaseDefinition> definitions;
  std::string line;
  while (std::getline(input, line)) {
    if (line.empty() || line[0] == '#') {
      continue;
    }

    std::istringstream line_stream(line);
    BenchmarkCaseDefinition definition;
    line_stream >> definition.name
                >> definition.rows
                >> definition.cols
                >> definition.robot_count
                >> definition.seed;

    if (!line_stream || definition.name.empty()) {
      throw std::runtime_error("failed to parse benchmark case line: " + line);
    }

    definitions.push_back(definition);
  }

  if (definitions.empty()) {
    throw std::runtime_error("benchmark case store is empty");
  }

  return definitions;
}

// Saves the benchmark-case definitions back to the persistent metadata store.
void saveBenchmarkCaseDefinitions(
    const std::vector<BenchmarkCaseDefinition>& definitions) {
  std::ofstream output(benchmarkCaseStorePath());
  if (!output) {
    throw std::runtime_error(
        "could not open benchmark case store for writing: " + benchmarkCaseStorePath());
  }

  output << "# name rows cols robot_count seed\n";
  for (const BenchmarkCaseDefinition& definition : definitions) {
    output << definition.name << " "
           << definition.rows << " "
           << definition.cols << " "
           << definition.robot_count << " "
           << definition.seed << "\n";
  }
}

// Selects either one benchmark case or all cases inside a named benchmark group.
std::vector<BenchmarkCaseDefinition> selectBenchmarkCaseDefinitions(
    const std::string& target) {
  const std::vector<BenchmarkCaseDefinition> definitions =
      loadBenchmarkCaseDefinitions();

  if (target == "few" || target == "medium" || target == "abundant") {
    std::vector<BenchmarkCaseDefinition> selected;
    const std::string prefix = target + "_";

    for (const BenchmarkCaseDefinition& definition : definitions) {
      if (belongsToGroup(definition.name, prefix)) {
        selected.push_back(definition);
      }
    }
    return selected;
  }

  return {findDefinitionByName(definitions, target)};
}

// Builds one benchmark case, including deterministic robot start and goal points.
BenchmarkCase buildBenchmarkCase(const BenchmarkCaseDefinition& definition) {
  return {
      definition.name,
      definition.rows,
      definition.cols,
      definition.robot_count,
      makeCrossingScenarioRobots(definition.rows,
                                 definition.cols,
                                 definition.robot_count,
                                 definition.seed),
  };
}

// Replaces one persisted benchmark case seed with a new random seed.
void randomizeBenchmarkCase(const std::string& case_name) {
  std::vector<BenchmarkCaseDefinition> definitions =
      loadBenchmarkCaseDefinitions();

  std::random_device random_device;
  const std::uint32_t new_seed = random_device();

  bool updated = false;
  for (BenchmarkCaseDefinition& definition : definitions) {
    if (definition.name == case_name) {
      definition.seed = new_seed;
      updated = true;
      break;
    }
  }

  if (!updated) {
    findDefinitionByName(definitions, case_name);
  }

  saveBenchmarkCaseDefinitions(definitions);
}
