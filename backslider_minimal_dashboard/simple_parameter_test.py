#!/usr/bin/env python3

import serial
import time
import struct

# Parameter operation types
PARAM_OP_READ_REQUEST = 0x01
CHANNEL_CAN_BUS = 0x02

def create_parameter_request(can_id, operation=PARAM_OP_READ_REQUEST, value=0.0, source_channel=CHANNEL_CAN_BUS, request_id=1):
    """Create a parameter_msg_t structure (8 bytes)"""
    return struct.pack('<Bfbbb', operation, value, source_channel, request_id, 0)

def create_binary_can_message(can_id, data_bytes):
    """Create a binary CANMessage struct (CAN_message_t from FlexCAN_T4)"""
    # CANMessage structure:
    # id (4 bytes) + len (1 byte) + buf[8] (8 bytes) + timestamp (4 bytes) + flags (1 byte) + reserved (6 bytes) = 24 bytes
    
    # Create 24-byte message with exact field placement
    message = bytearray(24)  # Initialize to all zeros
    
    # CAN ID at offset 0 (4 bytes, little-endian)
    message[0:4] = struct.pack('<I', can_id)
    
    # Length at offset 4 (1 byte)
    message[4] = len(data_bytes)
    
    # Data buffer at offset 5 (8 bytes)
    for i, byte in enumerate(data_bytes):
        if i < 8:  # Ensure we don't exceed buffer
            message[5 + i] = byte
    
    # Timestamp at offset 13 (4 bytes, little-endian) - set to 0 for now
    message[13:17] = struct.pack('<I', 0)
    
    # Flags at offset 17 (1 byte) - set extended flag (bit 0)
    message[17] = 0x01  # Set extended flag
    
    # Reserved bytes at offset 18 (6 bytes) - already 0
    
    return bytes(message)

def test_single_parameter():
    """Test a single parameter request and monitor debug output"""
    print("🧪 Testing Single Parameter Request...")
    print("=" * 50)
    
    # Try different ports
    ports = ['/dev/cu.usbmodem160544701', '/dev/cu.usbmodem160544702', '/dev/cu.usbmodem160544703']
    
    ser = None
    for port in ports:
        try:
            ser = serial.Serial(port, 115200, timeout=1)
            print(f"✅ Connected to Teensy at {port}")
            break
        except Exception as e:
            print(f"❌ Failed to connect to {port}: {e}")
            continue
    
    if ser is None:
        print("❌ Could not connect to any port")
        return
    
    time.sleep(2)  # Wait for Teensy to initialize
    
    # Test vehicle speed parameter
    can_id = 0x10300002  # MSG_VEHICLE_SPEED
    param_data = create_parameter_request(can_id, PARAM_OP_READ_REQUEST, 0.0, CHANNEL_CAN_BUS, 1)
    can_message = create_binary_can_message(can_id, param_data)
    
    print(f"📤 Sending parameter request for Vehicle Speed (CAN ID: 0x{can_id:08X})")
    print(f"   Parameter data: {[f'0x{b:02X}' for b in param_data]}")
    print(f"   CAN message length: {len(can_message)} bytes")
    
    # Send with prefix
    message_with_prefix = b'\xFF\xFF' + can_message
    ser.write(message_with_prefix)
    
    print("\n📡 Monitoring serial output for 10 seconds...")
    print("=" * 50)
    
    # Monitor serial output
    start_time = time.time()
    while time.time() - start_time < 10:
        if ser.in_waiting > 0:
            try:
                line = ser.readline().decode('utf-8', errors='replace').strip()
                if line:
                    print(f"📝 {line}")
            except Exception as e:
                pass
        time.sleep(0.01)
    
    print("\n✅ Test complete")
    ser.close()

if __name__ == "__main__":
    test_single_parameter() 