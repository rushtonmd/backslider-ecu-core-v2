#!/usr/bin/env python3
"""
Simple CANbus Parameter Test using Arduino CLI Monitor
"""

import subprocess
import time
import sys

def test_canbus_parameters():
    """Test CANbus parameter request/response using Arduino CLI monitor"""
    
    print("🚀 Testing CANbus Parameter Request/Response")
    print("=" * 50)
    
    # Commands to send to the Teensy
    test_commands = [
        "HELP",                    # Show available commands
        "TEST_PARAM",              # Run internal parameter test
        "STATUS",                  # Show ECU status
        "CAN_SEND:10300002:8:01 00 00 00 04 7B 00",  # Vehicle Speed request
        "CAN_SEND:10300001:8:01 00 00 00 04 7C 00",  # Engine RPM request
    ]
    
    for command in test_commands:
        print(f"\n📤 Sending: {command}")
        
        try:
            # Use Arduino CLI monitor to send command
            result = subprocess.run([
                "arduino-cli", "monitor", "--port", "usb:5100000"
            ], input=f"{command}\n", text=True, capture_output=True, timeout=10)
            
            if result.stdout:
                print("📥 Response:")
                print(result.stdout)
            
            if result.stderr:
                print("⚠️  Errors:")
                print(result.stderr)
                
        except subprocess.TimeoutExpired:
            print("⏰ Command timed out")
        except Exception as e:
            print(f"❌ Error: {e}")
        
        time.sleep(1)  # Small delay between commands
    
    print("\n🏁 Test completed")

if __name__ == "__main__":
    test_canbus_parameters() 