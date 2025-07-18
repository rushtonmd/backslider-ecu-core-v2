#include "mock_arduino.h"
#include <iostream>

int main() {
    std::cout << "Testing mock I2C functions..." << std::endl;
    
    // Test initial state
    std::cout << "Initial MCP23017 pin 0: " << mock_mcp23017_read_pin(0) << std::endl;
    
    // Test setting pin
    mock_set_mcp23017_pin(0, false);
    std::cout << "After setting MCP23017 pin 0 to false: " << mock_mcp23017_read_pin(0) << std::endl;
    
    // Test setting pin to true
    mock_set_mcp23017_pin(0, true);
    std::cout << "After setting MCP23017 pin 0 to true: " << mock_mcp23017_read_pin(0) << std::endl;
    
    // Test reset
    mock_reset_all();
    std::cout << "After mock_reset_all(): MCP23017 pin 0: " << mock_mcp23017_read_pin(0) << std::endl;
    
    std::cout << "Mock I2C test complete!" << std::endl;
    return 0;
} 