#!/usr/bin/env python3
"""
Robust Dashboard - Better handling of mixed text/binary data
"""

import serial
import time
import struct
import threading
import logging
from typing import Dict, Any

# Configure logging
logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

class RobustECUClient:
    """Robust ECU client with better mixed data parsing"""
    
    PARAMETERS = {
        0x10500001: {"name": "Fluid Temperature", "unit": "°C"},
        0x10500101: {"name": "Current Gear", "unit": ""}, 
        0x10300002: {"name": "Vehicle Speed", "unit": "mph"}
    }
    
    READ_REQUEST = 0x01
    READ_RESPONSE = 0x03
    
    def __init__(self):
        self.serial_conn = None
        self.is_connected = False
        self.is_running = False
        self.buffer = bytearray()
        self.parameter_values = {}
        self.stats = {
            'total_requests': 0,
            'total_responses': 0,
            'successful_responses': 0,
            'parse_errors': 0,
            'raw_bytes_received': 0,
            'text_messages': 0,
            'binary_messages': 0
        }
        self.request_interval = 1.0  # seconds
        self.next_request_id = 1
    
    def connect(self, port: str, baudrate: int = 115200) -> bool:
        """Connect to ECU"""
        try:
            logger.info(f"🔌 Connecting to {port} at {baudrate} baud...")
            self.serial_conn = serial.Serial(port, baudrate, timeout=0.1)
            
            if not self.serial_conn.is_open:
                raise serial.SerialException("Failed to open serial port")
            
            self.is_connected = True
            self.is_running = True
            logger.info("✅ Connected successfully!")
            return True
            
        except Exception as e:
            logger.error(f"❌ Failed to connect: {e}")
            return False
    
    def disconnect(self):
        """Disconnect from ECU"""
        self.is_running = False
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
        logger.info("🔌 Disconnected")
    
    def start_monitoring(self):
        """Start monitoring thread"""
        self.monitor_thread = threading.Thread(target=self._monitor_serial, daemon=True)
        self.monitor_thread.start()
        logger.info("📡 Started serial monitoring")
    
    def _monitor_serial(self):
        """Monitor serial port with robust parsing"""
        logger.info("🔍 Starting robust serial monitor...")
        
        while self.is_running and self.is_connected:
            try:
                if self.serial_conn.in_waiting > 0:
                    data = self.serial_conn.read(self.serial_conn.in_waiting)
                    if data:
                        self.stats['raw_bytes_received'] += len(data)
                        
                        # Add to buffer
                        self.buffer.extend(data)
                        
                        # Process buffer
                        self._process_buffer_robust()
                
                time.sleep(0.001)
                
            except Exception as e:
                logger.error(f"❌ Monitor error: {e}")
                break
        
        logger.info("🔍 Serial monitor stopped")
    
    def _process_buffer_robust(self):
        """Robust buffer processing for mixed text/binary data"""
        if not self.buffer:
            return
        
        # Step 1: Extract complete text lines
        self._extract_text_lines()
        
        # Step 2: Search for embedded binary messages in remaining data
        self._extract_binary_messages()
    
    def _extract_text_lines(self):
        """Extract and process complete text lines"""
        while True:
            # Find next newline
            newline_pos = -1
            for i, byte in enumerate(self.buffer):
                if byte == ord('\n'):
                    newline_pos = i
                    break
            
            if newline_pos == -1:
                break  # No complete line found
            
            # Extract the line (including the newline)
            line_data = self.buffer[:newline_pos + 1]
            self.buffer = self.buffer[newline_pos + 1:]
            
            # Process the text line
            try:
                text = line_data.decode('utf-8', errors='ignore')
                self._process_text_line(text)
                self.stats['text_messages'] += 1
            except Exception as e:
                logger.debug(f"Text decode error: {e}")
    
    def _extract_binary_messages(self):
        """Extract binary CAN messages from remaining buffer"""
        if len(self.buffer) < 24:
            return
            
        # Look for patterns that might be CAN messages
        i = 0
        while i <= len(self.buffer) - 24:
            # Check if this could be a CAN message
            if self._looks_like_can_message(self.buffer[i:i+24]):
                logger.debug(f"🔍 Potential CAN message at position {i}: {self.buffer[i:i+24].hex()}")
                if self._try_parse_can_message(self.buffer[i:i+24]):
                    # Successfully parsed, remove the message and everything before it
                    self.buffer = self.buffer[i+24:]
                    self.stats['binary_messages'] += 1
                    i = 0  # Reset search
                else:
                    i += 1
            else:
                i += 1
        
        # If buffer is getting too large, trim it
        if len(self.buffer) > 1024:
            logger.warning(f"Buffer too large ({len(self.buffer)} bytes), trimming...")
            self.buffer = self.buffer[-512:]  # Keep last 512 bytes
    
    def _looks_like_can_message(self, data: bytes) -> bool:
        """Quick check if data might be a CAN message"""
        if len(data) != 24:
            return False
        
        try:
            # Check CAN ID range (should be reasonable)
            can_id = struct.unpack('<I', data[0:4])[0]
            if can_id == 0 or can_id > 0x1FFFFFFF:
                return False
            
            # Check length field
            length = data[11]
            if length > 8:
                return False
            
            # Check if it's one of our known parameters
            if can_id in self.PARAMETERS:
                return True
            
            # For unknown IDs, be more lenient but still check basic structure
            return True
            
        except:
            return False
    
    def _process_text_line(self, text: str):
        """Process a single text line"""
        # Look for specific patterns that indicate binary data
        if 'Sending binary response' in text:
            logger.debug(f"🔍 Binary response indicator: {text.strip()}")
        
        # Look for other interesting debug messages
        if any(keyword in text for keyword in ['ParameterRegistry:', 'SerialBridge:', 'MessageBus:']):
            logger.debug(f"📝 Debug: {text.strip()}")
    
    def _try_parse_can_message(self, data: bytes) -> bool:
        """Try to parse a CAN message"""
        try:
            can_id = struct.unpack('<I', data[0:4])[0]
            timestamp = struct.unpack('<I', data[4:8])[0]
            length = data[11]
            flags = data[10]
            param_data = data[12:20]
            reserved = data[20:24]
            
            # Only process if it's one of our parameters and has correct length
            if can_id in self.PARAMETERS and length == 8:
                return self._process_parameter_response(can_id, param_data)
            
            return False
            
        except Exception as e:
            logger.debug(f"❌ CAN parse error: {e}")
            return False
    
    def _process_parameter_response(self, can_id: int, data: bytes) -> bool:
        """Process parameter response"""
        try:
            operation = data[0]
            value = struct.unpack('<f', data[1:5])[0]
            source_channel = data[5]
            request_id = data[6]
            reserved = data[7]
            
            if operation != self.READ_RESPONSE:
                return False
            
            param_info = self.PARAMETERS.get(can_id, {"name": "Unknown", "unit": ""})
            param_name = param_info["name"]
            unit = param_info["unit"]
            
            # Store the value
            self.parameter_values[can_id] = {
                'value': value,
                'timestamp': time.time(),
                'name': param_name,
                'unit': unit
            }
            
            logger.info(f"✅ {param_name}: {value:.2f} {unit} (CAN ID 0x{can_id:08X})")
            
            self.stats['total_responses'] += 1
            self.stats['successful_responses'] += 1
            
            return True
            
        except Exception as e:
            logger.error(f"❌ Parameter processing error: {e}")
            self.stats['parse_errors'] += 1
            return False
    
    def send_parameter_request(self, can_id: int):
        """Send a parameter request"""
        try:
            param_data = struct.pack('<Bfbbb', 
                self.READ_REQUEST,  # Operation
                0.0,                # Value
                1,                  # Source channel
                self.next_request_id,  # Request ID
                0                   # Reserved
            )
            
            message = bytearray(24)
            message[0:4] = struct.pack('<I', can_id)
            message[11] = 8
            message[12:20] = param_data
            
            self.serial_conn.write(bytes(message))
            self.serial_conn.flush()
            
            param_name = self.PARAMETERS.get(can_id, {}).get("name", "Unknown")
            logger.info(f"📤 Sent request for {param_name} (CAN ID 0x{can_id:08X})")
            self.stats['total_requests'] += 1
            self.next_request_id = (self.next_request_id % 255) + 1
            
        except Exception as e:
            logger.error(f"❌ Send error: {e}")
    
    def request_all_parameters(self):
        """Request all parameters"""
        for can_id in self.PARAMETERS.keys():
            self.send_parameter_request(can_id)
            time.sleep(0.2)  # Small delay between requests
    
    def get_status(self) -> Dict[str, Any]:
        """Get current status"""
        return {
            'connected': self.is_connected,
            'running': self.is_running,
            'buffer_size': len(self.buffer),
            'parameter_values': self.parameter_values.copy(),
            'stats': self.stats.copy()
        }

def main():
    """Main function"""
    print("🚀 Robust ECU Dashboard")
    print("=" * 50)
    
    client = RobustECUClient()
    
    if not client.connect('/dev/cu.usbmodem160544701'):
        return
    
    client.start_monitoring()
    
    try:
        print("\n📊 Starting parameter monitoring...")
        print("Press Ctrl+C to stop\n")
        
        while True:
            # Request all parameters
            client.request_all_parameters()
            
            # Display current values
            print("\n" + "="*50)
            print("📊 CURRENT VALUES:")
            for can_id, param_info in client.PARAMETERS.items():
                if can_id in client.parameter_values:
                    value_info = client.parameter_values[can_id]
                    print(f"  {value_info['name']}: {value_info['value']:.2f} {value_info['unit']}")
                else:
                    print(f"  {param_info['name']}: No data")
            
            # Display stats
            status = client.get_status()
            stats = status['stats']
            print(f"\n📈 STATS:")
            print(f"  Requests sent: {stats['total_requests']}")
            print(f"  Responses received: {stats['total_responses']}")
            print(f"  Successful: {stats['successful_responses']}")
            print(f"  Parse errors: {stats['parse_errors']}")
            print(f"  Text messages: {stats['text_messages']}")
            print(f"  Binary messages: {stats['binary_messages']}")
            print(f"  Raw bytes: {stats['raw_bytes_received']}")
            print(f"  Buffer size: {status['buffer_size']}")
            
            time.sleep(client.request_interval)
            
    except KeyboardInterrupt:
        print("\n🛑 Stopping...")
    finally:
        client.disconnect()

if __name__ == "__main__":
    main() 