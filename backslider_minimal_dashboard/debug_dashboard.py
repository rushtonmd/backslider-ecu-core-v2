#!/usr/bin/env python3
"""
Debug Dashboard - Shows all raw data and parsing attempts
"""

import serial
import time
import struct
import threading

class DebugECUClient:
    """Debug ECU client with detailed logging"""
    
    PARAMETERS = {
        0x10500001: "Fluid Temperature",
        0x10500101: "Current Gear", 
        0x10300002: "Vehicle Speed"
    }
    
    READ_REQUEST = 0x01
    READ_RESPONSE = 0x03
    
    def __init__(self):
        self.serial_conn = None
        self.is_connected = False
        self.is_running = False
        self.buffer = bytearray()
        self.stats = {
            'total_requests': 0,
            'total_responses': 0,
            'successful_responses': 0,
            'parse_errors': 0,
            'raw_bytes_received': 0
        }
    
    def connect(self, port: str, baudrate: int = 115200) -> bool:
        """Connect to ECU"""
        try:
            print(f"🔌 Connecting to {port} at {baudrate} baud...")
            self.serial_conn = serial.Serial(port, baudrate, timeout=0.1)
            
            if not self.serial_conn.is_open:
                raise serial.SerialException("Failed to open serial port")
            
            self.is_connected = True
            self.is_running = True
            print("✅ Connected successfully!")
            return True
            
        except Exception as e:
            print(f"❌ Failed to connect: {e}")
            return False
    
    def disconnect(self):
        """Disconnect from ECU"""
        self.is_running = False
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
        print("🔌 Disconnected")
    
    def start_monitoring(self):
        """Start monitoring thread"""
        self.monitor_thread = threading.Thread(target=self._monitor_serial, daemon=True)
        self.monitor_thread.start()
        print("📡 Started serial monitoring")
    
    def _monitor_serial(self):
        """Monitor serial port with detailed logging"""
        print("🔍 Starting serial monitor...")
        
        while self.is_running and self.is_connected:
            try:
                if self.serial_conn.in_waiting > 0:
                    data = self.serial_conn.read(self.serial_conn.in_waiting)
                    if data:
                        self.stats['raw_bytes_received'] += len(data)
                        print(f"📥 Received {len(data)} bytes: {data.hex()}")
                        
                        # Add to buffer
                        self.buffer.extend(data)
                        
                        # Process buffer
                        self._process_buffer()
                
                time.sleep(0.001)
                
            except Exception as e:
                print(f"❌ Monitor error: {e}")
                break
        
        print("🔍 Serial monitor stopped")
    
    def _process_buffer(self):
        """Process buffer with detailed logging"""
        if not self.buffer:
            return
        
        print(f"🔍 Processing buffer ({len(self.buffer)} bytes): {self.buffer.hex()}")
        
        # Look for text messages first
        text_end = -1
        for i, byte in enumerate(self.buffer):
            if byte == ord('\n'):
                text_end = i + 1
                break
        
        if text_end > 0:
            text_data = self.buffer[:text_end]
            try:
                text = text_data.decode('utf-8', errors='ignore')
                print(f"📝 Text message: {repr(text)}")
            except:
                print(f"📝 Raw text bytes: {text_data.hex()}")
            
            self.buffer = self.buffer[text_end:]
            print(f"🔍 Buffer after text: {len(self.buffer)} bytes")
        
        # Look for CAN messages
        while len(self.buffer) >= 24:
            print(f"🔍 Checking for CAN message at start of buffer...")
            
            if self._is_valid_can_message(self.buffer[:24]):
                print(f"✅ Valid CAN message found!")
                if self._try_parse_can_message(self.buffer[:24]):
                    print(f"✅ CAN message parsed successfully!")
                    self.buffer = self.buffer[24:]
                else:
                    print(f"❌ CAN message parsing failed")
                    self.buffer = self.buffer[1:]
            else:
                print(f"❌ Not a valid CAN message")
                self.buffer = self.buffer[1:]
    
    def _is_valid_can_message(self, data: bytes) -> bool:
        """Check if data looks like a valid CAN message"""
        if len(data) != 24:
            print(f"❌ Invalid length: {len(data)} (expected 24)")
            return False
        
        try:
            can_id = struct.unpack('<I', data[0:4])[0]
            length = data[11]
            
            print(f"🔍 CAN ID: 0x{can_id:08X}, Length: {length}")
            
            if can_id > 0x1FFFFFFF or can_id == 0:
                print(f"❌ Invalid CAN ID: 0x{can_id:08X}")
                return False
            
            if length > 8:
                print(f"❌ Invalid length: {length}")
                return False
            
            if can_id in self.PARAMETERS:
                print(f"✅ Known parameter: {self.PARAMETERS[can_id]}")
                return True
            
            print(f"⚠️ Unknown CAN ID: 0x{can_id:08X}")
            return True
            
        except Exception as e:
            print(f"❌ CAN validation error: {e}")
            return False
    
    def _try_parse_can_message(self, buffer: bytearray) -> bool:
        """Try to parse a CAN message"""
        try:
            can_id = struct.unpack('<I', buffer[0:4])[0]
            timestamp = struct.unpack('<I', buffer[4:8])[0]
            length = buffer[11]
            flags = buffer[10]
            data = buffer[12:20]
            reserved = buffer[20:24]
            
            print(f"🔍 Parsed: ID=0x{can_id:08X}, len={length}, flags={flags}")
            print(f"🔍 Data: {data.hex()}")
            
            if can_id in self.PARAMETERS and length == 8:
                return self._process_parameter_response(can_id, data)
            
            return True
            
        except Exception as e:
            print(f"❌ Parse error: {e}")
            return False
    
    def _process_parameter_response(self, can_id: int, data: bytes) -> bool:
        """Process parameter response"""
        try:
            operation = data[0]
            value = struct.unpack('<f', data[1:5])[0]
            source_channel = data[5]
            request_id = data[6]
            reserved = data[7]
            
            print(f"🔍 Parameter: op={operation}, value={value:.2f}, channel={source_channel}, req_id={request_id}")
            
            if operation != self.READ_RESPONSE:
                print(f"❌ Not a READ_RESPONSE: {operation}")
                return False
            
            param_name = self.PARAMETERS.get(can_id, "Unknown")
            print(f"✅ {param_name}: {value:.2f}")
            
            self.stats['total_responses'] += 1
            self.stats['successful_responses'] += 1
            
            return True
            
        except Exception as e:
            print(f"❌ Parameter processing error: {e}")
            self.stats['parse_errors'] += 1
            return False
    
    def send_parameter_request(self, can_id: int):
        """Send a parameter request"""
        try:
            param_data = struct.pack('<Bfbbb', 
                self.READ_REQUEST,  # Operation
                0.0,                # Value
                1,                  # Source channel
                42,                 # Request ID
                0                   # Reserved
            )
            
            message = bytearray(24)
            message[0:4] = struct.pack('<I', can_id)
            message[11] = 8
            message[12:20] = param_data
            
            self.serial_conn.write(bytes(message))
            self.serial_conn.flush()
            
            param_name = self.PARAMETERS.get(can_id, "Unknown")
            print(f"📤 Sent request for {param_name} (0x{can_id:08X})")
            self.stats['total_requests'] += 1
            
        except Exception as e:
            print(f"❌ Send error: {e}")
    
    def get_stats(self):
        """Get current statistics"""
        return self.stats.copy()

def main():
    """Main function"""
    print("🔍 Debug ECU Dashboard")
    print("=" * 50)
    
    client = DebugECUClient()
    
    if not client.connect('/dev/cu.usbmodem160544701'):
        return
    
    client.start_monitoring()
    
    try:
        # Send requests for each parameter
        for can_id, param_name in client.PARAMETERS.items():
            print(f"\n📤 Testing {param_name}...")
            client.send_parameter_request(can_id)
            time.sleep(2)  # Wait 2 seconds for response
        
        # Keep monitoring for a bit
        print("\n📡 Monitoring for 10 seconds...")
        time.sleep(10)
        
        # Show final stats
        stats = client.get_stats()
        print(f"\n📊 Final Statistics:")
        print(f"   Requests sent: {stats['total_requests']}")
        print(f"   Responses received: {stats['total_responses']}")
        print(f"   Successful responses: {stats['successful_responses']}")
        print(f"   Parse errors: {stats['parse_errors']}")
        print(f"   Raw bytes received: {stats['raw_bytes_received']}")
        
    except KeyboardInterrupt:
        print("\n🛑 Interrupted by user")
    finally:
        client.disconnect()

if __name__ == "__main__":
    main() 