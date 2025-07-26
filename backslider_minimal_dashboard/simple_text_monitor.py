#!/usr/bin/env python3
"""
Simple Text Monitor - Read all text output from Teensy
"""
import serial
import time

def simple_text_monitor():
    print("🔍 Simple Text Monitor - Reading all output from Teensy...")
    
    # Connect to Teensy
    s = serial.Serial('/dev/cu.usbmodem160544701', 115200, timeout=1)
    print("✅ Connected to Teensy at 115200 baud")
    
    print("📡 Reading all output for 15 seconds...")
    start_time = time.time()
    
    while time.time() - start_time < 15:
        if s.in_waiting > 0:
            try:
                line = s.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    print(f"📝 {line}")
                    
                    # Look for CAN messages
                    if "CAN_ID=" in line:
                        print(f"🎯 FOUND CAN MESSAGE: {line}")
                        
            except Exception as e:
                print(f"⚠️  Error reading line: {e}")
        
        time.sleep(0.01)  # Faster polling
    
    s.close()
    print("✅ Monitoring complete")

if __name__ == '__main__':
    simple_text_monitor() 