#!/usr/bin/env python3
"""
Debug Serial Monitor - Read and display serial output from Teensy
"""

import serial
import time

def debug_serial_monitor():
    print("🔍 Debug Serial Monitor - Reading Teensy output...")
    print("Looking for debug messages about fluid temperature sensor...")
    
    # Try different baud rates
    baud_rates = [115200, 1000000, 2000000]
    
    for baud_rate in baud_rates:
        print(f"\n🔍 Testing baud rate: {baud_rate}")
        
        try:
            # Connect to serial port
            s = serial.Serial('/dev/cu.usbmodem160544701', baud_rate, timeout=1)
            print(f"✅ Connected at {baud_rate} baud")
            
            # Read data for 10 seconds
            start_time = time.time()
            total_bytes = 0
            
            while time.time() - start_time < 10:
                if s.in_waiting > 0:
                    try:
                        line = s.readline().decode('utf-8', errors='ignore').strip()
                        if line:
                            total_bytes += len(line)
                            print(f"📝 {line}")
                            
                            # Look for specific debug messages
                            if "fluid temp" in line.lower() or "trans fluid" in line.lower():
                                print(f"🎯 FOUND FLUID TEMP DEBUG: {line}")
                            if "publishing" in line.lower() and "temp" in line.lower():
                                print(f"🎯 FOUND TEMP PUBLISH: {line}")
                            if "reading fluid temp" in line.lower():
                                print(f"🎯 FOUND FLUID TEMP READ: {line}")
                    except UnicodeDecodeError:
                        # Skip binary data
                        pass
                
                time.sleep(0.1)
            
            s.close()
            
            print(f"📊 Total bytes received: {total_bytes}")
            
            if total_bytes > 0:
                print(f"✅ SUCCESS: Getting data at {baud_rate} baud!")
                return baud_rate
            else:
                print(f"❌ No data received at {baud_rate} baud")
                
        except Exception as e:
            print(f"❌ Error at {baud_rate} baud: {e}")
    
    print("\n❌ No working baud rate found!")
    return None

if __name__ == '__main__':
    working_baud = debug_serial_monitor()
    if working_baud:
        print(f"\n🎯 Use baud rate: {working_baud}")
    else:
        print("\n💡 Try resetting the Teensy or checking the USB connection") 