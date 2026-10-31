# Host-side test Makefile (place in test/Makefile)
# Usage: `make -C test` from the project root, or `make` from inside test/

TEST_DIR  := $(dir $(abspath $(lastword $(MAKEFILE_LIST))))
BUILD_DIR := $(abspath $(TEST_DIR)/../build_test)
TARGET    := foothold_test

.PHONY: all build run clean rebuild

all: run

build:
	cmake -S $(TEST_DIR) -B $(BUILD_DIR)
	cmake --build $(BUILD_DIR)

run: build
	$(BUILD_DIR)/$(TARGET)

clean:
	rm -rf $(BUILD_DIR)

rebuild: clean run