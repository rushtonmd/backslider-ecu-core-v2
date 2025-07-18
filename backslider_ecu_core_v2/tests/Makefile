# Folder-Based Test Suite Makefile
CXX = g++
CXXFLAGS = -std=c++11 -Wall -I.. -I. -DTESTING

# Source files from parent directory
ECU_SOURCES = ../main_application.cpp

# Mock Arduino sources for desktop testing
MOCK_SOURCES = mock_arduino.cpp

# Test module directories
TEST_MODULES = main_application message_bus fuel_module ignition_module sensors input_manager transmission_module output_manager external_serial external_canbus storage_manager config_manager

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

# Main application now includes transmission module dependencies, external communications, storage manager, and configuration manager
main_application/test_main_application: main_application/test_main_application.cpp ../main_application.cpp ../msg_bus.cpp ../input_manager.cpp ../sensor_calibration.cpp ../thermistor_table_generator.cpp ../transmission_module.cpp ../output_manager.cpp ../external_serial.cpp ../external_canbus.cpp ../external_canbus_cache.cpp ../custom_message_handler.cpp ../obdii_handler.cpp ../custom_canbus_manager.cpp ../storage_manager.cpp ../spi_flash_storage_backend.cpp ../config_manager.cpp ../ecu_config.cpp $(MOCK_SOURCES)
	$(CXX) $(CXXFLAGS) -o $@ $< ../main_application.cpp ../msg_bus.cpp ../input_manager.cpp ../sensor_calibration.cpp ../thermistor_table_generator.cpp ../transmission_module.cpp ../output_manager.cpp ../external_serial.cpp ../external_canbus.cpp ../external_canbus_cache.cpp ../custom_message_handler.cpp ../obdii_handler.cpp ../custom_canbus_manager.cpp ../storage_manager.cpp ../spi_flash_storage_backend.cpp ../config_manager.cpp ../ecu_config.cpp $(MOCK_SOURCES)

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

# Frequency counter sensor test needs msg_bus, input_manager, sensor_calibration, and mock_arduino
input_manager/test_input_manager_frequency_counter: input_manager/test_input_manager_frequency_counter.cpp ../msg_bus.cpp ../input_manager.cpp ../sensor_calibration.cpp $(MOCK_SOURCES)
	$(CXX) $(CXXFLAGS) -o $@ $< ../msg_bus.cpp ../input_manager.cpp ../sensor_calibration.cpp $(MOCK_SOURCES)

# Transmission module tests need all sensor components plus transmission module and custom_canbus_manager
transmission_module/test_%: transmission_module/test_%.cpp ../msg_bus.cpp ../input_manager.cpp ../sensor_calibration.cpp ../thermistor_table_generator.cpp ../transmission_module.cpp ../custom_canbus_manager.cpp ../external_canbus.cpp ../external_canbus_cache.cpp ../custom_message_handler.cpp ../obdii_handler.cpp ../storage_manager.cpp ../spi_flash_storage_backend.cpp mock_arduino.cpp
	$(CXX) $(CXXFLAGS) -o $@ $< ../msg_bus.cpp ../input_manager.cpp ../sensor_calibration.cpp ../thermistor_table_generator.cpp ../transmission_module.cpp ../custom_canbus_manager.cpp ../external_canbus.cpp ../external_canbus_cache.cpp ../custom_message_handler.cpp ../obdii_handler.cpp ../storage_manager.cpp ../spi_flash_storage_backend.cpp mock_arduino.cpp

# Output manager tests need msg_bus, output_manager, and mock_arduino
output_manager/test_output_manager: output_manager/test_output_manager.cpp ../msg_bus.cpp ../output_manager.cpp $(MOCK_SOURCES)
	$(CXX) $(CXXFLAGS) -o $@ $< ../msg_bus.cpp ../output_manager.cpp $(MOCK_SOURCES)

# External serial tests need external_serial, msg_bus, and mock_arduino
external_serial/test_external_serial: external_serial/test_external_serial.cpp ../external_serial.cpp ../msg_bus.cpp $(MOCK_SOURCES)
	$(CXX) $(CXXFLAGS) -o $@ $< ../external_serial.cpp ../msg_bus.cpp $(MOCK_SOURCES)

# External CAN bus tests need external_canbus, cache, handlers, custom_canbus_manager, storage_manager, msg_bus, and mock_arduino
external_canbus/test_%: external_canbus/test_%.cpp ../external_canbus.cpp ../external_canbus_cache.cpp ../custom_message_handler.cpp ../obdii_handler.cpp ../custom_canbus_manager.cpp ../storage_manager.cpp ../spi_flash_storage_backend.cpp ../msg_bus.cpp $(MOCK_SOURCES)
	$(CXX) $(CXXFLAGS) -o $@ $< ../external_canbus.cpp ../external_canbus_cache.cpp ../custom_message_handler.cpp ../obdii_handler.cpp ../custom_canbus_manager.cpp ../storage_manager.cpp ../spi_flash_storage_backend.cpp ../msg_bus.cpp $(MOCK_SOURCES)

# Storage manager tests need storage_manager, spi_flash_storage_backend, msg_bus, and mock_arduino
storage_manager/test_storage_manager: storage_manager/test_storage_manager.cpp ../storage_manager.cpp ../spi_flash_storage_backend.cpp ../msg_bus.cpp $(MOCK_SOURCES)
	$(CXX) $(CXXFLAGS) -o $@ $< ../storage_manager.cpp ../spi_flash_storage_backend.cpp ../msg_bus.cpp $(MOCK_SOURCES)

# SPI Flash storage tests need spi_flash_storage_backend and mock_arduino
storage_manager/test_spi_flash_storage: storage_manager/test_spi_flash_storage.cpp ../spi_flash_storage_backend.cpp $(MOCK_SOURCES)
	$(CXX) $(CXXFLAGS) -o $@ $< ../spi_flash_storage_backend.cpp $(MOCK_SOURCES)

# Config manager tests need config_manager, storage_manager, spi_flash_storage_backend, ecu_config, msg_bus, and mock_arduino
config_manager/test_%: config_manager/test_%.cpp ../config_manager.cpp ../storage_manager.cpp ../spi_flash_storage_backend.cpp ../ecu_config.cpp ../msg_bus.cpp $(MOCK_SOURCES)
	$(CXX) $(CXXFLAGS) -o $@ $< ../config_manager.cpp ../storage_manager.cpp ../spi_flash_storage_backend.cpp ../ecu_config.cpp ../msg_bus.cpp $(MOCK_SOURCES)

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

run-external-serial: $(wildcard external_serial/test_*)
	@echo "=== Running External Serial Tests ==="
	@for test in $^; do echo "Running $test..."; ./$test || exit 1; done

run-external-canbus: $(wildcard external_canbus/test_*)
	@echo "=== Running External CAN Bus Tests ==="
	@for test in $^; do \
		if [ -x "$$test" ] && [ "$${test##*.}" != "cpp" ]; then \
			echo "Running $$test..."; \
			./$$test || exit 1; \
		fi; \
	done

run-storage-manager: $(wildcard storage_manager/test_*)
	@echo "=== Running Storage Manager Tests ==="
	@for test in $^; do \
		if [ -x "$$test" ] && [ "$${test##*.}" != "cpp" ]; then \
			echo "Running $$test..."; \
			./$$test || exit 1; \
		fi; \
	done

run-config-manager: $(wildcard config_manager/test_*)
	@echo "=== Running Config Manager Tests ==="
	@for test in $^; do \
		if [ -x "$$test" ] && [ "$${test##*.}" != "cpp" ]; then \
			echo "Running $$test..."; \
			./$$test || exit 1; \
		fi; \
	done

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
	@echo "  run-external-serial  - Run external serial tests only"
	@echo "  run-external-canbus  - Run external CAN bus tests only"
	@echo "  run-storage-manager  - Run storage manager tests only"
	@echo "  run-config-manager   - Run config manager tests only"
	@echo "  setup-dirs       - Create module directory structure"
	@echo "  clean            - Remove all test executables"

.PHONY: all test run-main run-msg-bus run-fuel run-ignition run-sensors run-input-manager run-transmission run-output-manager run-external-serial run-external-canbus run-storage-manager run-config-manager setup-dirs clean help