CXX := g++
CXXFLAGS := -std=c++17 -Wall -Wextra -pedantic -O2

TARGET := grid_demo
SRCS := src/main.cpp src/grid_planner.cpp src/grid_renderer.cpp

.PHONY: all clean run

all: $(TARGET)

$(TARGET): $(SRCS) src/grid_planner.h src/grid_renderer.h
	$(CXX) $(CXXFLAGS) $(SRCS) -o $(TARGET)

run: $(TARGET)
	./$(TARGET)

clean:
	rm -f $(TARGET)
