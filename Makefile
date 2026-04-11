CXX := g++
BASE_CXXFLAGS := -std=c++17 -Wall -Wextra -pedantic -O2

UNAME_S := $(shell uname -s)
OPENMP_CXXFLAGS :=
OPENMP_LDFLAGS :=
FILESYSTEM_LDFLAGS :=

ifeq ($(UNAME_S),Darwin)
LIBOMP_PREFIX := $(shell brew --prefix libomp 2>/dev/null)
ifneq ($(LIBOMP_PREFIX),)
OPENMP_CXXFLAGS += -Xpreprocessor -fopenmp -I$(LIBOMP_PREFIX)/include
OPENMP_LDFLAGS += -L$(LIBOMP_PREFIX)/lib -lomp
endif
else
OPENMP_CXXFLAGS += -fopenmp
OPENMP_LDFLAGS += -fopenmp
FILESYSTEM_LDFLAGS += -lstdc++fs
endif

CXXFLAGS := $(BASE_CXXFLAGS) $(OPENMP_CXXFLAGS)
LDFLAGS := $(OPENMP_LDFLAGS) $(FILESYSTEM_LDFLAGS)

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
THREAD_ARGS := $(if $(N),-N $(N),)

.PHONY: all clean run test test_correctness test_few test_medium test_abundant \
	test_few_8 test_few_16 test_medium_16 test_medium_32 test_abundant_64 test_abundant_128 \
	svg_few svg_medium svg_abundant svg_few_8 svg_few_16 svg_medium_16 svg_medium_32 \
	svg_abundant_64 svg_abundant_128 benchmark_case_tool

all: $(TARGET)

$(TARGET): $(SRCS) src/grid_planner.h src/grid_renderer.h src/collision_detector.h src/constraints.h src/a_star.h src/greedy_repair.h
	$(CXX) $(CXXFLAGS) $(SRCS) -o $(TARGET) $(LDFLAGS)

$(TEST_TARGET): $(TEST_SRCS) src/grid_planner.h src/collision_detector.h src/constraints.h src/a_star.h src/greedy_repair.h
	$(CXX) $(CXXFLAGS) -I src $(TEST_SRCS) -o $(TEST_TARGET) $(LDFLAGS)

$(BENCHMARK_TARGET): $(BENCHMARK_SRCS) src/benchmark_cases.h src/grid_planner.h src/grid_renderer.h src/collision_detector.h src/constraints.h src/a_star.h src/greedy_repair.h
	$(CXX) $(CXXFLAGS) -I src $(BENCHMARK_SRCS) -o $(BENCHMARK_TARGET) $(LDFLAGS)

$(BENCHMARK_CASE_TOOL_TARGET): $(BENCHMARK_CASE_TOOL_SRCS) src/benchmark_cases.h src/grid_planner.h
	$(CXX) $(CXXFLAGS) -I src $(BENCHMARK_CASE_TOOL_SRCS) -o $(BENCHMARK_CASE_TOOL_TARGET) $(LDFLAGS)

run: $(TARGET)
	./$(TARGET)

test: test_correctness

test_correctness: $(TEST_TARGET)
	./$(TEST_TARGET)

test_few: $(BENCHMARK_TARGET)
	mkdir -p $(BENCHMARK_RESULTS_DIR)
	./$(BENCHMARK_TARGET) few $(THREAD_ARGS) | tee $(BENCHMARK_RESULTS_DIR)/few.txt

test_few_8: $(BENCHMARK_TARGET)
	mkdir -p $(BENCHMARK_RESULTS_DIR)
	./$(BENCHMARK_TARGET) few_8 $(THREAD_ARGS) | tee $(BENCHMARK_RESULTS_DIR)/few_8.txt

test_few_16: $(BENCHMARK_TARGET)
	mkdir -p $(BENCHMARK_RESULTS_DIR)
	./$(BENCHMARK_TARGET) few_16 $(THREAD_ARGS) | tee $(BENCHMARK_RESULTS_DIR)/few_16.txt

test_medium: $(BENCHMARK_TARGET)
	mkdir -p $(BENCHMARK_RESULTS_DIR)
	./$(BENCHMARK_TARGET) medium $(THREAD_ARGS) | tee $(BENCHMARK_RESULTS_DIR)/medium.txt

test_medium_16: $(BENCHMARK_TARGET)
	mkdir -p $(BENCHMARK_RESULTS_DIR)
	./$(BENCHMARK_TARGET) medium_16 $(THREAD_ARGS) | tee $(BENCHMARK_RESULTS_DIR)/medium_16.txt

test_medium_32: $(BENCHMARK_TARGET)
	mkdir -p $(BENCHMARK_RESULTS_DIR)
	./$(BENCHMARK_TARGET) medium_32 $(THREAD_ARGS) | tee $(BENCHMARK_RESULTS_DIR)/medium_32.txt

test_abundant: $(BENCHMARK_TARGET)
	mkdir -p $(BENCHMARK_RESULTS_DIR)
	./$(BENCHMARK_TARGET) abundant $(THREAD_ARGS) | tee $(BENCHMARK_RESULTS_DIR)/abundant.txt

test_abundant_64: $(BENCHMARK_TARGET)
	mkdir -p $(BENCHMARK_RESULTS_DIR)
	./$(BENCHMARK_TARGET) abundant_64 $(THREAD_ARGS) | tee $(BENCHMARK_RESULTS_DIR)/abundant_64.txt

test_abundant_128: $(BENCHMARK_TARGET)
	mkdir -p $(BENCHMARK_RESULTS_DIR)
	./$(BENCHMARK_TARGET) abundant_128 $(THREAD_ARGS) | tee $(BENCHMARK_RESULTS_DIR)/abundant_128.txt

svg_few: $(BENCHMARK_TARGET)
	mkdir -p $(BENCHMARK_SVG_DIR)
	mkdir -p $(BENCHMARK_RESULTS_DIR)
	./$(BENCHMARK_TARGET) few $(BENCHMARK_SVG_DIR) $(THREAD_ARGS) | tee $(BENCHMARK_RESULTS_DIR)/few.txt

svg_few_8: $(BENCHMARK_TARGET)
	mkdir -p $(BENCHMARK_SVG_DIR)
	mkdir -p $(BENCHMARK_RESULTS_DIR)
	./$(BENCHMARK_TARGET) few_8 $(BENCHMARK_SVG_DIR) $(THREAD_ARGS) | tee $(BENCHMARK_RESULTS_DIR)/few_8.txt

svg_few_16: $(BENCHMARK_TARGET)
	mkdir -p $(BENCHMARK_SVG_DIR)
	mkdir -p $(BENCHMARK_RESULTS_DIR)
	./$(BENCHMARK_TARGET) few_16 $(BENCHMARK_SVG_DIR) $(THREAD_ARGS) | tee $(BENCHMARK_RESULTS_DIR)/few_16.txt

svg_medium: $(BENCHMARK_TARGET)
	mkdir -p $(BENCHMARK_SVG_DIR)
	mkdir -p $(BENCHMARK_RESULTS_DIR)
	./$(BENCHMARK_TARGET) medium $(BENCHMARK_SVG_DIR) $(THREAD_ARGS) | tee $(BENCHMARK_RESULTS_DIR)/medium.txt

svg_medium_16: $(BENCHMARK_TARGET)
	mkdir -p $(BENCHMARK_SVG_DIR)
	mkdir -p $(BENCHMARK_RESULTS_DIR)
	./$(BENCHMARK_TARGET) medium_16 $(BENCHMARK_SVG_DIR) $(THREAD_ARGS) | tee $(BENCHMARK_RESULTS_DIR)/medium_16.txt

svg_medium_32: $(BENCHMARK_TARGET)
	mkdir -p $(BENCHMARK_SVG_DIR)
	mkdir -p $(BENCHMARK_RESULTS_DIR)
	./$(BENCHMARK_TARGET) medium_32 $(BENCHMARK_SVG_DIR) $(THREAD_ARGS) | tee $(BENCHMARK_RESULTS_DIR)/medium_32.txt

svg_abundant: $(BENCHMARK_TARGET)
	mkdir -p $(BENCHMARK_SVG_DIR)
	mkdir -p $(BENCHMARK_RESULTS_DIR)
	./$(BENCHMARK_TARGET) abundant $(BENCHMARK_SVG_DIR) $(THREAD_ARGS) | tee $(BENCHMARK_RESULTS_DIR)/abundant.txt

svg_abundant_64: $(BENCHMARK_TARGET)
	mkdir -p $(BENCHMARK_SVG_DIR)
	mkdir -p $(BENCHMARK_RESULTS_DIR)
	./$(BENCHMARK_TARGET) abundant_64 $(BENCHMARK_SVG_DIR) $(THREAD_ARGS) | tee $(BENCHMARK_RESULTS_DIR)/abundant_64.txt

svg_abundant_128: $(BENCHMARK_TARGET)
	mkdir -p $(BENCHMARK_SVG_DIR)
	mkdir -p $(BENCHMARK_RESULTS_DIR)
	./$(BENCHMARK_TARGET) abundant_128 $(BENCHMARK_SVG_DIR) $(THREAD_ARGS) | tee $(BENCHMARK_RESULTS_DIR)/abundant_128.txt

clean:
	rm -f $(TARGET) $(TEST_TARGET) $(BENCHMARK_TARGET) $(BENCHMARK_CASE_TOOL_TARGET)
	rm -rf $(BENCHMARK_SVG_DIR) $(BENCHMARK_RESULTS_DIR)
