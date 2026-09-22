CXX := g++
CXXFLAGS := -std=c++20 -O2 -Wall -Wextra
CPPFLAGS := -Ilib -Ilib/core -Ilib/core/actions -Ilib/core/maze -Ilib/core/planner -Ilib/core/robot -Ilib/sim-io -Ilib/io
BUILD_DIR := build
TARGET := $(BUILD_DIR)/micromouse_sim

SOURCES := $(shell find lib -name '*.cpp' -not -path 'lib/io-teensy/*' | sort) sim-entry/main.cpp
OBJECTS := $(patsubst %.cpp,$(BUILD_DIR)/%.o,$(SOURCES))

.PHONY: all clean run

all: $(TARGET)

$(TARGET): $(OBJECTS)
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) $(OBJECTS) -o $@

$(BUILD_DIR)/%.o: %.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -c $< -o $@

run: $(TARGET)
	./$(TARGET)

clean:
	rm -rf $(BUILD_DIR)
	rm -f micromouse_sim
	rm -f lib/**/*.o sim-entry/*.o 2>/dev/null || true
