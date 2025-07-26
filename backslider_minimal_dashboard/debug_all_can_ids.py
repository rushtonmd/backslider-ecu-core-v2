#!/usr/bin/env python3

import serial
import time
import struct
import re

def debug_all_can_ids():
    """Capture and decode ALL CAN IDs to see what's actually being broadcast"""
    
    port = '/dev/cu.usbmodem160544701'
    baud_rate = 115200
    
    print("🔍 Debug All CAN IDs - Capturing EVERYTHING...")
    
    try:
        ser = serial.Serial(port, baud_rate, timeout=1)
        print(f"✅ Connected to Teensy at {baud_rate} baud")
        print("📡 Reading ALL CAN messages for 15 seconds...\n")
        
        can_ids_seen = {}
        can_pattern = re.compile(r'CAN_ID=0x([0-9A-F]+),LEN=(\d+),DATA=(.+?),TIMESTAMP=(\d+),FLAGS=0x([0-9A-F]+)')
        
        start_time = time.time()
        while time.time() - start_time < 15:
            if ser.in_waiting > 0:
                try:
                    line = ser.readline().decode('utf-8', errors='replace').strip()
                    match = can_pattern.search(line)
                    if match:
                        can_id_hex = match.group(1)
                        can_id = int(can_id_hex, 16)
                        length = int(match.group(2))
                        data_str = match.group(3)
                        timestamp = int(match.group(4))
                        
                        # Parse data bytes
                        data_parts = data_str.split(',')
                        data_bytes = []
                        for part in data_parts:
                            if part.startswith('0x'):
                                data_bytes.append(int(part, 16))
                        
                        # Try to decode as float if length >= 4
                        value_str = "N/A"
                        if length >= 4 and len(data_bytes) >= 4:
                            try:
                                value = struct.unpack('<f', bytes(data_bytes[:4]))[0]
                                value_str = f"{value:.2f}"
                            except:
                                value_str = "Invalid"
                        
                        # Track this CAN ID
                        if can_id not in can_ids_seen:
                            can_ids_seen[can_id] = {'count': 0, 'last_value': value_str}
                        
                        can_ids_seen[can_id]['count'] += 1
                        can_ids_seen[can_id]['last_value'] = value_str
                        
                        # Print real-time
                        print(f"📝 0x{can_id:08X} = {value_str} (count: {can_ids_seen[can_id]['count']})")
                        
                except Exception as e:
                    pass  # Skip invalid lines
            
            time.sleep(0.01)
        
        print(f"\n📊 SUMMARY - Found {len(can_ids_seen)} unique CAN IDs:")
        print("=" * 60)
        
        # Sort by CAN ID for easy reading
        for can_id in sorted(can_ids_seen.keys()):
            info = can_ids_seen[can_id]
            print(f"🆔 0x{can_id:08X}: {info['count']:3d} messages, last value = {info['last_value']}")
        
        # Check for expected IDs
        expected_ids = {
            0x10300002: "Vehicle Speed (EXPECTED)",
            0x10500001: "Fluid Temperature",
            0x10500101: "Gear Selector", 
            0x10500104: "Drive Gear"
        }
        
        print(f"\n🎯 EXPECTED ID CHECK:")
        print("=" * 40)
        for expected_id, description in expected_ids.items():
            if expected_id in can_ids_seen:
                print(f"✅ 0x{expected_id:08X} ({description}): FOUND")
            else:
                print(f"❌ 0x{expected_id:08X} ({description}): MISSING")
                
        ser.close()
        
    except Exception as e:
        print(f"❌ Error: {e}")

if __name__ == '__main__':
    debug_all_can_ids() 