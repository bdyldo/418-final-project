#include "benchmark_cases.h"

#include <exception>
#include <iostream>
#include <string>
#include <vector>

namespace {

void printUsage(const char* program_name) {
  std::cerr << "Usage: " << program_name << " randomize <case_name>\n";
  std::cerr << "Benchmark case store: " << benchmarkCaseStorePath() << "\n";
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 3 || std::string(argv[1]) != "randomize") {
    printUsage(argv[0]);
    return 1;
  }

  try {
    const std::string case_name = argv[2];
    randomizeBenchmarkCase(case_name);

    const std::vector<BenchmarkCaseDefinition> updated_definitions =
        selectBenchmarkCaseDefinitions(case_name);
    const BenchmarkCaseDefinition& updated_definition = updated_definitions.front();

    std::cout << "Randomized " << updated_definition.name
              << " with new seed " << updated_definition.seed
              << " in " << benchmarkCaseStorePath() << "\n";
  } catch (const std::exception& error) {
    std::cerr << "randomize error: " << error.what() << "\n";
    return 1;
  }

  return 0;
}
