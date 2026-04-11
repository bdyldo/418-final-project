CXX := g++
CXXFLAGS := -std=c++17 -Wall -Wextra -pedantic -O2

TARGET := grid_demo
SRCS := src/main.cpp src/grid_planner.cpp src/grid_renderer.cpp src/collision_detector.cpp src/a_star.cpp src/greedy_repair.cpp
TEST_TARGET := correctness_tests
TEST_SRCS := tests/correctness_tests.cpp src/grid_planner.cpp src/collision_detector.cpp src/a_star.cpp src/greedy_repair.cpp
BENCHMARK_TARGET := benchmark_runner
BENCHMARK_COMMON_SRCS := src/benchmark_cases.cpp src/grid_planner.cpp src/grid_renderer.cpp src/collision_detector.cpp src/a_star.cpp src/greedy_repair.cpp
BENCHMARK_SRCS := benchmarks/benchmark_runner.cpp $(BENCHMARK_COMMON_SRCS)
BENCHMARK_CASE_TOOL_TARGET := benchmark_case_tool
BENCHMARK_CASE_TOOL_SRCS := benchmarks/benchmark_case_tool.cpp src/benchmark_cases.cpp
BENCHMARK_SVG_DIR := benchmark_svgs
BENCHMARK_RESULTS_DIR := benchmark_results

.PHONY: all clean run test test_correctness test_few test_medium test_abundant \
	test_few_8 test_few_16 test_medium_16 test_medium_32 test_abundant_64 test_abundant_128 \
	svg_few svg_medium svg_abundant svg_few_8 svg_few_16 svg_medium_16 svg_medium_32 \
	svg_abundant_64 svg_abundant_128 benchmark_case_tool

all: $(TARGET)

$(TARGET): $(SRCS) src/grid_planner.h src/grid_renderer.h src/collision_detector.h src/constraints.h src/a_star.h src/greedy_repair.h
	$(CXX) $(CXXFLAGS) $(SRCS) -o $(TARGET)

$(TEST_TARGET): $(TEST_SRCS) src/grid_planner.h src/collision_detector.h src/constraints.h src/a_star.h src/greedy_repair.h
	$(CXX) $(CXXFLAGS) -I src $(TEST_SRCS) -o $(TEST_TARGET)

$(BENCHMARK_TARGET): $(BENCHMARK_SRCS) src/benchmark_cases.h src/grid_planner.h src/grid_renderer.h src/collision_detector.h src/constraints.h src/a_star.h src/greedy_repair.h
	$(CXX) $(CXXFLAGS) -I src $(BENCHMARK_SRCS) -o $(BENCHMARK_TARGET)

$(BENCHMARK_CASE_TOOL_TARGET): $(BENCHMARK_CASE_TOOL_SRCS) src/benchmark_cases.h src/grid_planner.h
	$(CXX) $(CXXFLAGS) -I src $(BENCHMARK_CASE_TOOL_SRCS) -o $(BENCHMARK_CASE_TOOL_TARGET)

run: $(TARGET)
	./$(TARGET)

test: test_correctness

test_correctness: $(TEST_TARGET)
	./$(TEST_TARGET)

test_few: $(BENCHMARK_TARGET)
	mkdir -p $(BENCHMARK_RESULTS_DIR)
	./$(BENCHMARK_TARGET) few | tee $(BENCHMARK_RESULTS_DIR)/few.txt

test_few_8: $(BENCHMARK_TARGET)
	mkdir -p $(BENCHMARK_RESULTS_DIR)
	./$(BENCHMARK_TARGET) few_8 | tee $(BENCHMARK_RESULTS_DIR)/few_8.txt

test_few_16: $(BENCHMARK_TARGET)
	mkdir -p $(BENCHMARK_RESULTS_DIR)
	./$(BENCHMARK_TARGET) few_16 | tee $(BENCHMARK_RESULTS_DIR)/few_16.txt

test_medium: $(BENCHMARK_TARGET)
	mkdir -p $(BENCHMARK_RESULTS_DIR)
	./$(BENCHMARK_TARGET) medium | tee $(BENCHMARK_RESULTS_DIR)/medium.txt

test_medium_16: $(BENCHMARK_TARGET)
	mkdir -p $(BENCHMARK_RESULTS_DIR)
	./$(BENCHMARK_TARGET) medium_16 | tee $(BENCHMARK_RESULTS_DIR)/medium_16.txt

test_medium_32: $(BENCHMARK_TARGET)
	mkdir -p $(BENCHMARK_RESULTS_DIR)
	./$(BENCHMARK_TARGET) medium_32 | tee $(BENCHMARK_RESULTS_DIR)/medium_32.txt

test_abundant: $(BENCHMARK_TARGET)
	mkdir -p $(BENCHMARK_RESULTS_DIR)
	./$(BENCHMARK_TARGET) abundant | tee $(BENCHMARK_RESULTS_DIR)/abundant.txt

test_abundant_64: $(BENCHMARK_TARGET)
	mkdir -p $(BENCHMARK_RESULTS_DIR)
	./$(BENCHMARK_TARGET) abundant_64 | tee $(BENCHMARK_RESULTS_DIR)/abundant_64.txt

test_abundant_128: $(BENCHMARK_TARGET)
	mkdir -p $(BENCHMARK_RESULTS_DIR)
	./$(BENCHMARK_TARGET) abundant_128 | tee $(BENCHMARK_RESULTS_DIR)/abundant_128.txt

svg_few: $(BENCHMARK_TARGET)
	mkdir -p $(BENCHMARK_SVG_DIR)
	mkdir -p $(BENCHMARK_RESULTS_DIR)
	./$(BENCHMARK_TARGET) few $(BENCHMARK_SVG_DIR) | tee $(BENCHMARK_RESULTS_DIR)/few.txt

svg_few_8: $(BENCHMARK_TARGET)
	mkdir -p $(BENCHMARK_SVG_DIR)
	mkdir -p $(BENCHMARK_RESULTS_DIR)
	./$(BENCHMARK_TARGET) few_8 $(BENCHMARK_SVG_DIR) | tee $(BENCHMARK_RESULTS_DIR)/few_8.txt

svg_few_16: $(BENCHMARK_TARGET)
	mkdir -p $(BENCHMARK_SVG_DIR)
	mkdir -p $(BENCHMARK_RESULTS_DIR)
	./$(BENCHMARK_TARGET) few_16 $(BENCHMARK_SVG_DIR) | tee $(BENCHMARK_RESULTS_DIR)/few_16.txt

svg_medium: $(BENCHMARK_TARGET)
	mkdir -p $(BENCHMARK_SVG_DIR)
	mkdir -p $(BENCHMARK_RESULTS_DIR)
	./$(BENCHMARK_TARGET) medium $(BENCHMARK_SVG_DIR) | tee $(BENCHMARK_RESULTS_DIR)/medium.txt

svg_medium_16: $(BENCHMARK_TARGET)
	mkdir -p $(BENCHMARK_SVG_DIR)
	mkdir -p $(BENCHMARK_RESULTS_DIR)
	./$(BENCHMARK_TARGET) medium_16 $(BENCHMARK_SVG_DIR) | tee $(BENCHMARK_RESULTS_DIR)/medium_16.txt

svg_medium_32: $(BENCHMARK_TARGET)
	mkdir -p $(BENCHMARK_SVG_DIR)
	mkdir -p $(BENCHMARK_RESULTS_DIR)
	./$(BENCHMARK_TARGET) medium_32 $(BENCHMARK_SVG_DIR) | tee $(BENCHMARK_RESULTS_DIR)/medium_32.txt

svg_abundant: $(BENCHMARK_TARGET)
	mkdir -p $(BENCHMARK_SVG_DIR)
	mkdir -p $(BENCHMARK_RESULTS_DIR)
	./$(BENCHMARK_TARGET) abundant $(BENCHMARK_SVG_DIR) | tee $(BENCHMARK_RESULTS_DIR)/abundant.txt

svg_abundant_64: $(BENCHMARK_TARGET)
	mkdir -p $(BENCHMARK_SVG_DIR)
	mkdir -p $(BENCHMARK_RESULTS_DIR)
	./$(BENCHMARK_TARGET) abundant_64 $(BENCHMARK_SVG_DIR) | tee $(BENCHMARK_RESULTS_DIR)/abundant_64.txt

svg_abundant_128: $(BENCHMARK_TARGET)
	mkdir -p $(BENCHMARK_SVG_DIR)
	mkdir -p $(BENCHMARK_RESULTS_DIR)
	./$(BENCHMARK_TARGET) abundant_128 $(BENCHMARK_SVG_DIR) | tee $(BENCHMARK_RESULTS_DIR)/abundant_128.txt

clean:
	rm -f $(TARGET) $(TEST_TARGET) $(BENCHMARK_TARGET) $(BENCHMARK_CASE_TOOL_TARGET)
	rm -rf $(BENCHMARK_SVG_DIR) $(BENCHMARK_RESULTS_DIR)
