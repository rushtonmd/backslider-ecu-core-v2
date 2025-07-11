# Folder-Based Test Suite Makefile
CXX = g++
CXXFLAGS = -std=c++11 -Wall -I.. -I. -DTESTING

# Source files from parent directory
ECU_SOURCES = ../main_application.cpp

# Mock Arduino sources for desktop testing
MOCK_SOURCES = mock_arduino.cpp

# Test module directories
TEST_MODULES = main_application message_bus fuel_module ignition_module sensors input_manager transmission_module output_manager

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

# Main application now includes transmission module dependencies
main_application/test_main_application: main_application/test_main_application.cpp ../main_application.cpp ../msg_bus.cpp ../input_manager.cpp ../sensor_calibration.cpp ../thermistor_table_generator.cpp ../transmission_module.cpp $(MOCK_SOURCES)
	$(CXX) $(CXXFLAGS) -o $@ $< ../main_application.cpp ../msg_bus.cpp ../input_manager.cpp ../sensor_calibration.cpp ../thermistor_table_generator.cpp ../transmission_module.cpp $(MOCK_SOURCES)

fuel_module/test_%: fuel_module/test_%.cpp
	$(CXX) $(CXXFLAGS) -o $@ $< ../mod_fuel.cpp $(ECU_SOURCES)

ignition_module/test_%: ignition_module/test_%.cpp
	$(CXX) $(CXXFLAGS) -o $@ $< ../mod_ignition.cpp $(ECU_SOURCES)

sensors/test_%: sensors/test_%.cpp
	$(CXX) $(CXXFLAGS) -o $@ $< ../mod_sensors.cpp $(ECU_SOURCES)

message_bus/test_message_bus: message_bus/test_message_bus.cpp ../msg_bus.cpp
	$(CXX) $(CXXFLAGS) -o $@ $< ../msg_bus.cpp

# Input manager tests need msg_bus, input_manager, sensor_calibration, and mock_arduino
input_manager/test_input_manager: input_manager/test_input_manager.cpp ../msg_bus.cpp ../input_manager.cpp ../sensor_calibration.cpp $(MOCK_SOURCES)
	$(CXX) $(CXXFLAGS) -o $@ $< ../msg_bus.cpp ../input_manager.cpp ../sensor_calibration.cpp $(MOCK_SOURCES)

# Thermistor table generator test needs sensor_calibration, thermistor_table_generator, and mock_arduino
input_manager/test_thermistor_table_generator: input_manager/test_thermistor_table_generator.cpp ../sensor_calibration.cpp ../thermistor_table_generator.cpp $(MOCK_SOURCES)
	$(CXX) $(CXXFLAGS) -o $@ $< ../sensor_calibration.cpp ../thermistor_table_generator.cpp $(MOCK_SOURCES)

# Digital sensor test needs msg_bus, input_manager, sensor_calibration, and mock_arduino
input_manager/test_input_manager_digital_sensors: input_manager/test_input_manager_digital_sensors.cpp ../msg_bus.cpp ../input_manager.cpp ../sensor_calibration.cpp $(MOCK_SOURCES)
	$(CXX) $(CXXFLAGS) -o $@ $< ../msg_bus.cpp ../input_manager.cpp ../sensor_calibration.cpp $(MOCK_SOURCES)

# Analog linear sensor test needs msg_bus, input_manager, sensor_calibration, and mock_arduino
input_manager/test_input_manager_analog_linear_sensors: input_manager/test_input_manager_analog_linear_sensors.cpp ../msg_bus.cpp ../input_manager.cpp ../sensor_calibration.cpp $(MOCK_SOURCES)
	$(CXX) $(CXXFLAGS) -o $@ $< ../msg_bus.cpp ../input_manager.cpp ../sensor_calibration.cpp $(MOCK_SOURCES)

# Thermistor sensor test needs msg_bus, input_manager, sensor_calibration, and mock_arduino
input_manager/test_input_manager_thermistors: input_manager/test_input_manager_thermistors.cpp ../msg_bus.cpp ../input_manager.cpp ../sensor_calibration.cpp $(MOCK_SOURCES)
	$(CXX) $(CXXFLAGS) -o $@ $< ../msg_bus.cpp ../input_manager.cpp ../sensor_calibration.cpp $(MOCK_SOURCES)

# Transmission module tests need all sensor components plus transmission module
transmission_module/test_%: transmission_module/test_%.cpp ../msg_bus.cpp ../input_manager.cpp ../sensor_calibration.cpp ../thermistor_table_generator.cpp ../transmission_module.cpp mock_arduino.cpp
	$(CXX) $(CXXFLAGS) -o $@ $< ../msg_bus.cpp ../input_manager.cpp ../sensor_calibration.cpp ../thermistor_table_generator.cpp ../transmission_module.cpp mock_arduino.cpp

# Output manager tests need msg_bus, output_manager, and mock_arduino
output_manager/test_output_manager: output_manager/test_output_manager.cpp ../msg_bus.cpp ../output_manager.cpp $(MOCK_SOURCES)
	$(CXX) $(CXXFLAGS) -o $@ $< ../msg_bus.cpp ../output_manager.cpp $(MOCK_SOURCES)

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

run-input-manager: $(wildcard input_manager/test_*)
	@echo "=== Running Input Manager Tests ==="
	@for test in $^; do echo "Running $test..."; ./$test || exit 1; done

run-transmission: $(wildcard transmission_module/test_*)
	@echo "=== Running Transmission Module Tests ==="
	@for test in $^; do echo "Running $test..."; ./$test || exit 1; done

run-output-manager: $(wildcard output_manager/test_*)
	@echo "=== Running Output Manager Tests ==="
	@for test in $^; do echo "Running $test..."; ./$test || exit 1; done

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
	@echo "  all              - Build all test executables"
	@echo "  test             - Run all test suites"
	@echo "  run-main         - Run main application tests only"
	@echo "  run-msg-bus      - Run message bus tests only"
	@echo "  run-fuel         - Run fuel module tests only"
	@echo "  run-ignition     - Run ignition module tests only"
	@echo "  run-sensors      - Run sensor tests only"
	@echo "  run-input-manager    - Run input manager tests only"
	@echo "  run-transmission     - Run transmission module tests only"
	@echo "  run-output-manager   - Run output manager tests only"
	@echo "  setup-dirs       - Create module directory structure"
	@echo "  clean            - Remove all test executables"

.PHONY: all test run-main run-msg-bus run-fuel run-ignition run-sensors run-input-manager run-transmission run-output-manager setup-dirs clean help