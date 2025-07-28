#!/usr/bin/env python3
"""
Simple script to monitor serial output and see debug messages
"""

import serial
import time

def monitor_serial():
    try:
        ser = serial.Serial('/dev/cu.usbmodem160544701', 115200, timeout=1)
        print("Monitoring serial output... Press Ctrl+C to stop")
        
        while True:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    print(f"SERIAL: {line}")
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        print("\nMonitoring stopped")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'ser' in locals():
            ser.close()

if __name__ == "__main__":
    monitor_serial() 