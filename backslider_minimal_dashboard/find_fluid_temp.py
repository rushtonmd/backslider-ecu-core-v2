#!/usr/bin/env python3
"""
Find Fluid Temperature Messages - Look for exact CAN ID pattern
"""

import serial
import struct
import time

def find_fluid_temp_messages():
    # Connect to your serial port
    s = serial.Serial('/dev/cu.usbmodem160544701', 115200, timeout=1)
    print("🔍 Looking for fluid temperature messages (0x10500001)...")
    
    buffer = bytearray()
    fluid_temp_count = 0
    gear_count = 0
    
    # Look for the exact CAN ID patterns
    FLUID_TEMP_PATTERN = b'\x01\x00P\x10'  # 0x10500001 in little-endian
    GEAR_PATTERN = b'\x01\x01P\x10'        # 0x10500101 in little-endian
    
    for i in range(100):  # Run for ~10 seconds
        data = s.read(100)
        if data:
            buffer.extend(data)
            
            # Look for patterns in the buffer
            while len(buffer) >= 4:
                # Check for fluid temp pattern
                if buffer[0:4] == FLUID_TEMP_PATTERN:
                    fluid_temp_count += 1
                    print(f"🎯 FOUND FLUID TEMP MESSAGE #{fluid_temp_count}")
                    print(f"   Raw bytes: {buffer[0:18].hex().upper()}")
                    
                    # Try to parse the message
                    if len(buffer) >= 18:
                        try:
                            can_id = struct.unpack('<I', buffer[0:4])[0]
                            length = buffer[4]
                            data_bytes = buffer[5:13]
                            timestamp = struct.unpack('<I', buffer[13:17])[0]
                            flags = buffer[17]
                            
                            print(f"   CAN_ID: 0x{can_id:08X}")
                            print(f"   Length: {length}")
                            print(f"   Data: {data_bytes.hex().upper()}")
                            print(f"   Timestamp: {timestamp}")
                            print(f"   Flags: 0x{flags:02X}")
                            
                            # Try to parse as float
                            try:
                                float_value = struct.unpack('<f', data_bytes[0:4])[0]
                                print(f"   Float Value: {float_value:.2f}°C")
                            except:
                                print(f"   Float Value: Invalid")
                            print()
                            
                            # Remove the message from buffer
                            buffer = buffer[18:]
                        except:
                            print(f"   Parse Error")
                            buffer = buffer[1:]
                    else:
                        # Not enough data, wait for more
                        break
                        
                # Check for gear pattern
                elif buffer[0:4] == GEAR_PATTERN:
                    gear_count += 1
                    if gear_count <= 5:  # Only show first 5 gear messages
                        print(f"⚙️  Found gear message #{gear_count}")
                    buffer = buffer[1:]  # Move forward
                    
                else:
                    # No pattern found, move forward
                    buffer = buffer[1:]
            
            # Keep buffer manageable
            if len(buffer) > 2000:
                buffer = buffer[-1000:]
        
        time.sleep(0.1)
    
    s.close()
    
    print(f"\n📊 RESULTS:")
    print(f"Fluid Temperature Messages: {fluid_temp_count}")
    print(f"Gear Messages: {gear_count}")
    
    if fluid_temp_count == 0:
        print("\n❌ NO FLUID TEMPERATURE MESSAGES FOUND!")
        print("This means:")
        print("   1. The fluid temperature sensor is not being read")
        print("   2. The messages are not being published to the message bus")
        print("   3. The messages are not being registered for broadcasting")
        print("   4. The messages are not being sent via USB serial")
    else:
        print(f"\n✅ SUCCESS: Found {fluid_temp_count} fluid temperature messages!")

if __name__ == '__main__':
    find_fluid_temp_messages() 