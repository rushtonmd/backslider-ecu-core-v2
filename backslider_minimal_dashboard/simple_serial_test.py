#!/usr/bin/env python3
"""
Simple Serial Test - Just read raw bytes to see if we're getting any data
"""

import serial
import time

def simple_serial_test():
    # Try different baud rates
    baud_rates = [115200, 1000000, 2000000]
    
    for baud_rate in baud_rates:
        print(f"\n🔍 Testing baud rate: {baud_rate}")
        
        try:
            # Connect to serial port
            s = serial.Serial('/dev/cu.usbmodem160544701', baud_rate, timeout=1)
            print(f"✅ Connected at {baud_rate} baud")
            
            # Read data for 5 seconds
            start_time = time.time()
            total_bytes = 0
            messages = []
            
            while time.time() - start_time < 5:
                if s.in_waiting > 0:
                    data = s.read(s.in_waiting)
                    if data:
                        total_bytes += len(data)
                        messages.append(data)
                        print(f"📦 Received {len(data)} bytes: {data.hex().upper()}")
                
                time.sleep(0.1)
            
            s.close()
            
            print(f"📊 Total bytes received: {total_bytes}")
            print(f"📊 Total messages: {len(messages)}")
            
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
    working_baud = simple_serial_test()
    if working_baud:
        print(f"\n🎯 Use baud rate: {working_baud}")
    else:
        print("\n💡 Try resetting the Teensy or checking the USB connection") 