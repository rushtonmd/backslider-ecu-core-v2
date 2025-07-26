#!/usr/bin/env python3
"""
Detailed CAN Message Analyzer - Parse and analyze the binary stream
"""

import serial
import struct
import time
from collections import defaultdict

def analyze_can_stream():
    # Connect to your serial port
    s = serial.Serial('/dev/cu.usbmodem160544701', 1000000, timeout=1)
    print("🔍 Detailed CAN Message Analyzer started...")
    print("Looking for CAN message format: [CAN_ID(4)][LEN(1)][DATA(8)][TIMESTAMP(4)][FLAGS(1)] = 18 bytes")
    
    buffer = bytearray()
    message_count = 0
    
    # Expected message size for our CANMessage structure
    MESSAGE_SIZE = 18  # CAN_ID(4) + LEN(1) + DATA(8) + TIMESTAMP(4) + FLAGS(1)
    
    for i in range(100):  # Run for ~10 seconds
        data = s.read(100)
        if data:
            buffer.extend(data)
            
            # Try to parse complete messages
            while len(buffer) >= MESSAGE_SIZE:
                # Parse the CANMessage structure
                try:
                    can_id = struct.unpack('<I', buffer[0:4])[0]  # Little-endian CAN ID
                    length = buffer[4]  # Data length
                    data_bytes = buffer[5:13]  # 8 bytes of data
                    timestamp = struct.unpack('<I', buffer[13:17])[0]  # Little-endian timestamp
                    flags = buffer[17]  # Flags byte
                    
                    # Check if this looks like a valid CAN ID
                    if 0x10000000 <= can_id <= 0x20000000:
                        # Try to parse the data as float (first 4 bytes)
                        float_value = None
                        try:
                            float_value = struct.unpack('<f', data_bytes[0:4])[0]
                        except:
                            pass
                        
                        print(f"📦 Message {message_count}: CAN_ID=0x{can_id:08X}")
                        print(f"   Length: {length}, Data: {data_bytes.hex().upper()}")
                        if float_value is not None:
                            print(f"   Float Value: {float_value:.2f}")
                        print(f"   Timestamp: {timestamp}")
                        print(f"   Flags: 0x{flags:02X}")
                        print(f"   Raw bytes: {buffer[0:MESSAGE_SIZE].hex().upper()}")
                        print()
                        
                        message_count += 1
                        
                        # Remove the parsed message from buffer
                        buffer = buffer[MESSAGE_SIZE:]
                    else:
                        # Not a valid CAN ID, shift buffer by 1 byte
                        buffer = buffer[1:]
                        
                except struct.error:
                    # Invalid data, shift buffer by 1 byte
                    buffer = buffer[1:]
            
            # Keep buffer manageable
            if len(buffer) > 2000:
                buffer = buffer[-1000:]
        
        time.sleep(0.1)
    
    s.close()
    
    print(f"\n📊 ANALYSIS COMPLETE:")
    print(f"Total messages parsed: {message_count}")
    
    if message_count == 0:
        print("\n❌ No valid CAN messages found!")
        print("This suggests:")
        print("   1. The binary format is different than expected")
        print("   2. The messages are being corrupted during transmission")
        print("   3. The byte ordering is different")
        print("   4. There are extra bytes between messages")

if __name__ == '__main__':
    analyze_can_stream()