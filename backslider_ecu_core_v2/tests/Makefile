# Folder-Based Test Suite Makefile
CXX = g++
CXXFLAGS = -std=c++11 -Wall -I.. -I. -DTESTING

# Source files from parent directory
ECU_SOURCES = ../main_application.cpp

# Test module directories
TEST_MODULES = main_application message_bus fuel_module ignition_module sensors

# Find all test files in module directories
TEST_FILES = $(wildcard */test_*.cpp)

# Generate executable names from test files
TEST_TARGETS = $(TEST_FILES:%.cpp=%)

# Default target - build all tests
all: $(TEST_TARGETS)

# Pattern rule to build test executables from module folders
%/test_%: %/test_%.cpp get_ecu_sources_for_module
	@echo "Building test: $@"
	$(CXX) $(CXXFLAGS) -o $@ $< $(call get_ecu_sources,$*)

# Function to get ECU sources needed for each module
define get_ecu_sources
$(ECU_SOURCES) $(wildcard ../*$(1)*.cpp)
endef

# Specific rules for modules that need additional sources
main_application/test_%: main_application/test_%.cpp
	$(CXX) $(CXXFLAGS) -o $@ $< $(ECU_SOURCES)

fuel_module/test_%: fuel_module/test_%.cpp
	$(CXX) $(CXXFLAGS) -o $@ $< ../mod_fuel.cpp $(ECU_SOURCES)

ignition_module/test_%: ignition_module/test_%.cpp
	$(CXX) $(CXXFLAGS) -o $@ $< ../mod_ignition.cpp $(ECU_SOURCES)

sensors/test_%: sensors/test_%.cpp
	$(CXX) $(CXXFLAGS) -o $@ $< ../mod_sensors.cpp $(ECU_SOURCES)

message_bus/test_message_bus: message_bus/test_message_bus.cpp ../msg_bus.cpp
	$(CXX) $(CXXFLAGS) -o $@ $< ../msg_bus.cpp

# Run all tests in a specific module
run-main: $(wildcard main_application/test_*)
	@echo "=== Running Main Application Tests ==="
	@for test in $^; do echo "Running $$test..."; ./$$test || exit 1; done

run-msg-bus: $(wildcard message_bus/test_*)
	@echo "=== Running Message Bus Tests ==="
	@for test in $^; do echo "Running $$test..."; ./$$test || exit 1; done

run-fuel: $(wildcard fuel_module/test_*)
	@echo "=== Running Fuel Module Tests ==="
	@for test in $^; do echo "Running $$test..."; ./$$test || exit 1; done

run-ignition: $(wildcard ignition_module/test_*)
	@echo "=== Running Ignition Module Tests ==="
	@for test in $^; do echo "Running $$test..."; ./$$test || exit 1; done

run-sensors: $(wildcard sensors/test_*)
	@echo "=== Running Sensor Tests ==="
	@for test in $^; do echo "Running $$test..."; ./$$test || exit 1; done

# Run all tests
test: $(TEST_TARGETS)
	@echo "=== Running All ECU Tests ==="
	@for module in $(TEST_MODULES); do \
		if ls $$module/test_* 1> /dev/null 2>&1; then \
			echo ""; \
			echo "🧪 Testing $$module module..."; \
			for test in $$module/test_*; do \
				if [ -x "$$test" ]; then \
					echo "  Running $$test..."; \
					./$$test || exit 1; \
				fi \
			done \
		fi \
	done
	@echo ""
	@echo "🎉 ALL MODULE TESTS PASSED!"

# Clean up
clean:
	find . -name 'test_*' -type f ! -name '*.cpp' ! -name '*.h' -delete

# Create module directory structure
setup-dirs:
	@for module in $(TEST_MODULES); do \
		mkdir -p $$module; \
	done

# Help target
help:
	@echo "Available targets:"
	@echo "  all           - Build all test executables"
	@echo "  test          - Run all test suites"
	@echo "  run-main      - Run main application tests only"
	@echo "  run-msg-bus   - Run message bus tests only"
	@echo "  run-fuel      - Run fuel module tests only"
	@echo "  run-ignition  - Run ignition module tests only"
	@echo "  run-sensors   - Run sensor tests only"
	@echo "  setup-dirs    - Create module directory structure"
	@echo "  clean         - Remove all test executables"

.PHONY: all test run-main run-msg-bus run-fuel run-ignition run-sensors setup-dirs clean help