
# Host-side test Makefile
# Usage:
#   make -C test
#   make -C test run TARGET=test_foothold
#   make -C test run TARGET=test_osqp_solver
#   make -C test clean
#   make -C test rebuild TARGET=test_osqp_solver

TEST_DIR  := $(dir $(abspath $(lastword $(MAKEFILE_LIST))))
BUILD_DIR := $(abspath $(TEST_DIR)/../build_test)

# Default executable; override from the command line.
TARGET ?= test_foothold

.PHONY: all build run clean rebuild list

all: run

build:
	cmake -S $(TEST_DIR) -B $(BUILD_DIR)
	cmake --build $(BUILD_DIR)

run: build
	$(BUILD_DIR)/$(TARGET)

clean:
	rm -rf $(BUILD_DIR)

rebuild: clean run

# List available executables in the build directory.
list: build
	@find $(BUILD_DIR) -type f -perm -111 \
		! -path '*/CMakeFiles/*' \
		! -name '*.sh' \
		! -name '*.cmake' \
		-print
