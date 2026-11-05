# Host-side test Makefile
# Usage:
#   make -C testbench build

TEST_DIR  := $(abspath $(dir $(lastword $(MAKEFILE_LIST))))
BUILD_DIR := $(TEST_DIR)/build

TARGET ?= testbench_ik

.PHONY: all build run clean rebuild list

all: run

build:
	cmake -S "$(TEST_DIR)" -B "$(BUILD_DIR)" -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
	cmake --build "$(BUILD_DIR)"
	@test -f "$(BUILD_DIR)/compile_commands.json"
	@echo "Compilation database: $(BUILD_DIR)/compile_commands.json"

run: build
	"$(BUILD_DIR)/$(TARGET)"

clean:
	rm -rf "$(BUILD_DIR)"

rebuild: clean run

list: build
	@find "$(BUILD_DIR)" -type f -perm -111 
		! -path '*/CMakeFiles/*' 
		! -name '*.sh' 
		! -name '*.cmake' 
		-print