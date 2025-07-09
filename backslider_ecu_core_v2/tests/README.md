# ECU Test Runner

This folder contains a minimal test runner system for testing ECU business logic on desktop computers without Arduino hardware.

## 🚀 Quick Start

### Run Tests (Recommended)
```bash
cd tests/
./run_tests.sh
```

### Alternative Methods
```bash
# Using make directly
cd tests/
make test

# Manual build and run
cd tests/
make clean
make
./ecu_tests
```

## 📋 Expected Output

When tests run successfully, you should see output like this:

```
Building and running ECU tests...
rm -f ecu_tests
g++ -std=c++11 -Wall -I.. -DTESTING -o ecu_tests test_runner.cpp ../main_application.cpp
./ecu_tests
=== Backslider ECU Test Runner ===

Running test: main_application_creation... MainApplication initialized
PASSED
Running test: main_application_run... MainApplication initialized
digitalWrite(pin=13, value=1)
LED: ON
PASSED

Tests run: 2
Tests passed: 2
ALL TESTS PASSED!
```

### Output Explanation

- **Mock Arduino Functions**: You'll see `digitalWrite(pin=13, value=1)` - this is the mock hardware function
- **Serial Output**: `MainApplication initialized` and `LED: ON` are Serial.println() calls from your ECU code
- **Test Results**: Each test shows `PASSED` when successful
- **Summary**: Final count of tests run and passed

## 🔧 How It Works

### Mock Arduino System
The test system uses `mock_arduino.h` to simulate Arduino functions on desktop:

- `millis()` - Returns real system time in milliseconds
- `digitalWrite()` - Prints pin state changes
- `pinMode()` - No-op for testing
- `Serial` - Prints to console (cout)

### Conditional Compilation
Your ECU code uses `#ifdef TESTING` to choose between real Arduino and mock functions:

```cpp
#ifdef TESTING
#include "tests/mock_arduino.h"
#else
#include <Arduino.h>
#endif
```

## ➕ Adding New Tests

### 1. Create a Test Function
Add new tests to `test_runner.cpp`:

```cpp
TEST(my_new_test) {
    // Your test code here
    MainApplication app;
    app.init();
    
    // Test something specific
    assert(some_condition);
}
```

### 2. Register the Test
Add your test to the main() function:

```cpp
int main() {
    std::cout << "=== Backslider ECU Test Runner ===" << std::endl;
    std::cout << std::endl;
    
    // Run all tests
    run_test_main_application_creation();
    run_test_main_application_run();
    run_test_my_new_test();  // Add your test here
    
    // ... rest of main function
}
```

### 3. Test Your New Code
```bash
./run_tests.sh
```

## 📝 Test Framework

### Simple Macro-Based Framework
```cpp
#define TEST(name) \
    void test_##name(); \
    void run_test_##name() { \
        std::cout << "Running test: " #name "... "; \
        tests_run++; \
        test_##name(); \
        tests_passed++; \
        std::cout << "PASSED" << std::endl; \
    } \
    void test_##name()
```

### Test Guidelines
- Use `assert()` for test conditions
- If an assertion fails, the test will crash and show the failure
- Each test should be independent
- Initialize objects fresh in each test

## 🛠️ Build System

### Makefile Targets
- `make` or `make all` - Build the test executable
- `make test` - Build and run tests
- `make clean` - Remove build artifacts

### Compiler Flags
- `-std=c++11` - C++11 standard
- `-Wall` - All warnings
- `-I..` - Include parent directory for ECU headers
- `-DTESTING` - Define TESTING macro for conditional compilation

## 📁 File Structure

```
tests/
├── README.md              # This file
├── mock_arduino.h          # Arduino function mocks
├── test_runner.cpp         # Main test runner and tests
├── Makefile               # Build configuration
├── run_tests.sh           # Convenience script
└── ecu_tests              # Generated executable (after build)
```

## 🐛 Troubleshooting

### Compilation Errors
```
error: use of undeclared identifier 'SomeArduinoFunction'
```
**Solution**: Add the missing Arduino function to `mock_arduino.h`

### Linking Errors
```
undefined reference to 'SomeFunction'
```
**Solution**: Make sure your ECU source files are listed in the Makefile's `ECU_SOURCES`

### Test Failures
```
Assertion failed: (condition), function test_name, file test_runner.cpp, line 42.
```
**Solution**: Check the failing assertion and fix the test or the code being tested

## 🔄 Continuous Integration

This test system is designed to run on any system with g++ and make:

```bash
# CI/CD pipeline example
cd tests/
make clean
make test
echo "Exit code: $?"
```

## 📚 Best Practices

1. **Keep Tests Simple**: Each test should verify one specific behavior
2. **Test Edge Cases**: Include boundary conditions and error cases
3. **Mock External Dependencies**: Use the mock system for hardware interactions
4. **Fast Execution**: Tests should run quickly for rapid feedback
5. **Clear Assertions**: Make test failures easy to understand

## 🎯 Next Steps

1. Add more comprehensive tests for your ECU modules
2. Create separate test files for different components
3. Add performance benchmarks
4. Set up automated testing in your development workflow

---

**Note**: This test system runs your ECU logic on desktop computers. For hardware-specific testing, you'll still need to test on actual Teensy 4.1 hardware. 