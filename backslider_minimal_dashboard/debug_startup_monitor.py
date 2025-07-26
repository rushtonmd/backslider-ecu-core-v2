#!/usr/bin/env python3

import serial
import time
import sys

def monitor_startup():
    """Monitor all serial output to see startup debug messages"""
    
    port = '/dev/cu.usbmodem160544701'
    baud_rate = 115200
    
    print("🔍 Startup Debug Monitor - Capturing ALL serial output...")
    
    try:
        # Open serial connection
        ser = serial.Serial(port, baud_rate, timeout=1)
        print(f"✅ Connected to Teensy at {baud_rate} baud")
        
        print("📡 Waiting for startup messages (reset your Teensy now if needed)...")
        print("📡 Reading ALL output for 30 seconds...\n")
        
        start_time = time.time()
        while time.time() - start_time < 30:
            if ser.in_waiting > 0:
                try:
                    line = ser.readline().decode('utf-8', errors='replace').strip()
                    if line:
                        print(f"📝 {line}")
                except Exception as e:
                    print(f"❌ Error reading line: {e}")
            
            time.sleep(0.01)  # Small delay to prevent excessive CPU usage
            
        print("\n✅ Monitoring complete")
        ser.close()
        
    except serial.SerialException as e:
        print(f"❌ Serial connection error: {e}")
        return False
    except KeyboardInterrupt:
        print("\n🛑 Monitoring stopped by user")
        return True
    except Exception as e:
        print(f"❌ Unexpected error: {e}")
        return False

if __name__ == '__main__':
    monitor_startup() 