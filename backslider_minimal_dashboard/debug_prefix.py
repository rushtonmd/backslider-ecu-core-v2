#!/usr/bin/env python3
"""
Debug script to see what bytes are being received
"""

import serial
import time

def debug_bytes():
    """Debug what bytes are being received"""
    
    port = '/dev/cu.usbmodem160544701'
    baud_rate = 115200
    
    print("🔍 Debug: What bytes are being received?")
    print("=" * 60)
    
    try:
        ser = serial.Serial(port, baud_rate, timeout=2)
        print(f"✅ Connected to Teensy at {baud_rate} baud\n")
        
        # Clear any existing data
        ser.reset_input_buffer()
        time.sleep(0.1)
        
        # Send a single parameter request
        can_id = 0x10500001  # Fluid Temperature
        param_data = struct.pack('<Bfbbb', 0x01, 0.0, 0x01, 42, 0)  # READ_REQUEST
        
        message = bytearray(24)
        message[0:4] = struct.pack('<I', can_id)
        message[11] = 8  # len
        message[12:20] = param_data
        
        print(f"📤 Sending request for Fluid Temperature...")
        
        ser.write(bytes(message))
        ser.flush()
        
        # Listen for response and show all bytes
        print("\n📡 Listening for response (showing all bytes)...")
        start_time = time.time()
        
        buffer = bytearray()
        
        while time.time() - start_time < 3:  # Wait 3 seconds
            if ser.in_waiting > 0:
                data = ser.read(ser.in_waiting)
                buffer.extend(data)
                
                # Show the raw bytes
                print(f"📥 Received {len(data)} bytes: {data.hex()}")
                
                # Look for 0xFF 0xFF in the entire buffer
                for i in range(len(buffer) - 1):
                    if buffer[i] == 0xFF and buffer[i + 1] == 0xFF:
                        print(f"🎯 Found 0xFF 0xFF at position {i} in buffer!")
                        
                        # Show context around the prefix
                        start_pos = max(0, i - 5)
                        end_pos = min(len(buffer), i + 30)
                        context = buffer[start_pos:end_pos]
                        print(f"   Context: {context.hex()}")
            
            time.sleep(0.01)
        
        # Show final buffer
        print(f"\n📊 Final buffer ({len(buffer)} bytes): {buffer.hex()}")
        
        # Count 0xFF bytes
        ff_count = buffer.count(0xFF)
        print(f"📊 Found {ff_count} 0xFF bytes in total")
        
        ser.close()
        
    except Exception as e:
        print(f"❌ Error: {e}")

if __name__ == "__main__":
    import struct
    debug_bytes() 