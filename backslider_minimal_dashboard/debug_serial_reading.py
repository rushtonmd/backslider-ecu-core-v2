#!/usr/bin/env python3
"""
Debug Serial Reading - See what's actually being read from the port
"""
import serial
import time

def debug_serial_reading():
    print("🔍 Debug Serial Reading - Raw byte analysis...")
    
    # Connect to Teensy
    s = serial.Serial('/dev/cu.usbmodem160544701', 115200, timeout=1)
    print("✅ Connected to Teensy at 115200 baud")
    
    print("📡 Reading raw bytes for 10 seconds...")
    start_time = time.time()
    total_bytes = 0
    
    while time.time() - start_time < 10:
        if s.in_waiting > 0:
            # Read all available bytes
            data = s.read(s.in_waiting)
            total_bytes += len(data)
            
            if data:
                print(f"📦 Received {len(data)} bytes: {data}")
                
                # Try to decode as text
                try:
                    text = data.decode('utf-8', errors='ignore')
                    print(f"📝 As text: {repr(text)}")
                    
                    # Look for CAN messages
                    if "CAN_ID=" in text:
                        print(f"🎯 FOUND CAN MESSAGE IN TEXT!")
                        
                except Exception as e:
                    print(f"❌ Decode error: {e}")
        
        time.sleep(0.1)
    
    s.close()
    print(f"📊 Total bytes received: {total_bytes}")
    
    if total_bytes == 0:
        print("❌ NO DATA RECEIVED!")
        print("This suggests:")
        print("  1. The Teensy is not sending data")
        print("  2. The port is wrong")
        print("  3. The baud rate is wrong")
        print("  4. There's a connection issue")
    else:
        print("✅ Data is being received!")

if __name__ == '__main__':
    debug_serial_reading() 