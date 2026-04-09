CXX := g++
CXXFLAGS := -std=c++17 -Wall -Wextra -pedantic -O2

TARGET := grid_demo
SRCS := src/main.cpp src/grid_planner.cpp src/grid_renderer.cpp src/collision_detector.cpp
TEST_TARGET := project_tests
TEST_SRCS := tests/project_tests.cpp src/grid_planner.cpp src/collision_detector.cpp

.PHONY: all clean run test

all: $(TARGET)

$(TARGET): $(SRCS) src/grid_planner.h src/grid_renderer.h src/collision_detector.h
	$(CXX) $(CXXFLAGS) $(SRCS) -o $(TARGET)

$(TEST_TARGET): $(TEST_SRCS) src/grid_planner.h src/collision_detector.h
	$(CXX) $(CXXFLAGS) -I src $(TEST_SRCS) -o $(TEST_TARGET)

run: $(TARGET)
	./$(TARGET)

test: $(TEST_TARGET)
	./$(TEST_TARGET)

clean:
	rm -f $(TARGET) $(TEST_TARGET)
