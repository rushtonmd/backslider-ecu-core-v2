#!/usr/bin/env python3
"""
Simple ECU Parameter Dashboard
Real-time monitoring of Teensy 4.1 ECU parameters
"""

import os
import sys
import struct
import time
import datetime
import threading
import json
import logging
from collections import defaultdict, deque
from typing import Optional, Dict, List, Any
import serial
import serial.tools.list_ports

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

class ECUClient:
    """Simple ECU parameter client"""
    
    # Parameter definitions
    PARAMETERS = {
        0x10500001: {
            'name': 'Fluid Temperature',
            'unit': '°C',
            'min_value': -40,
            'max_value': 150
        },
        0x10500101: {
            'name': 'Current Gear',
            'unit': 'gear',
            'min_value': 0,
            'max_value': 8
        },
        0x10300002: {
            'name': 'Vehicle Speed',
            'unit': 'mph',
            'min_value': 0,
            'max_value': 200
        }
    }
    
    READ_REQUEST = 0x01
    READ_RESPONSE = 0x03
    
    def __init__(self):
        self.serial_conn: Optional[serial.Serial] = None
        self.is_connected: bool = False
        self.is_running: bool = False
        self.request_interval: float = 1.0  # 1Hz
        self.next_request_id: int = 1
        
        # Parameter values and history
        self.parameter_values: Dict[int, float] = {}
        self.parameter_history: Dict[int, deque] = {
            param_id: deque(maxlen=50) for param_id in self.PARAMETERS.keys()
        }
        
        # Statistics
        self.stats = {
            'total_requests': 0,
            'total_responses': 0,
            'successful_responses': 0,
            'timeouts': 0,
            'parse_errors': 0,
            'start_time': None,
            'last_activity': None
        }
        
        # Thread management
        self.monitor_thread: Optional[threading.Thread] = None
        self.request_thread: Optional[threading.Thread] = None
        self.lock = threading.RLock()
    
    def get_available_ports(self) -> List[Dict[str, str]]:
        """Get list of available serial ports"""
        ports = []
        for port in serial.tools.list_ports.comports():
            ports.append({
                'device': port.device,
                'description': port.description or 'Unknown device',
                'manufacturer': port.manufacturer or 'Unknown'
            })
        return sorted(ports, key=lambda x: x['device'])
    
    def connect(self, port: str, baudrate: int = 115200) -> bool:
        """Connect to ECU via serial port"""
        try:
            if self.is_connected:
                self.disconnect()
            
            logger.info(f"Connecting to {port} at {baudrate} baud")
            self.serial_conn = serial.Serial(
                port=port,
                baudrate=baudrate,
                timeout=0.1,
                write_timeout=1.0
            )
            
            if not self.serial_conn.is_open:
                raise serial.SerialException("Failed to open serial port")
            
            self.is_connected = True
            self.is_running = True
            self.stats['start_time'] = time.time()
            
            # Start monitoring and request threads
            self.start_threads()
            
            logger.info(f"Successfully connected to {port}")
            return True
            
        except Exception as e:
            logger.error(f"Failed to connect to {port}: {e}")
            self.cleanup_connection()
            return False
    
    def disconnect(self):
        """Disconnect from ECU"""
        logger.info("Disconnecting from ECU")
        self.is_running = False
        self.is_connected = False
        
        # Wait for threads to finish
        if self.monitor_thread and self.monitor_thread.is_alive():
            self.monitor_thread.join(timeout=2.0)
        if self.request_thread and self.request_thread.is_alive():
            self.request_thread.join(timeout=2.0)
        
        self.cleanup_connection()
    
    def cleanup_connection(self):
        """Clean up serial connection"""
        if self.serial_conn and self.serial_conn.is_open:
            try:
                self.serial_conn.close()
            except Exception as e:
                logger.error(f"Error closing serial connection: {e}")
        
        self.serial_conn = None
        self.is_connected = False
    
    def start_threads(self):
        """Start monitoring and request threads"""
        self.monitor_thread = threading.Thread(target=self._monitor_serial, daemon=True)
        self.request_thread = threading.Thread(target=self._request_parameters, daemon=True)
        
        self.monitor_thread.start()
        self.request_thread.start()
    
    def _monitor_serial(self):
        """Monitor serial port for incoming messages"""
        logger.info("Starting serial monitor thread")
        buffer = bytearray()
        
        while self.is_running and self.is_connected and self.serial_conn:
            try:
                # Read available data
                if self.serial_conn.in_waiting > 0:
                    data = self.serial_conn.read(self.serial_conn.in_waiting)
                    if data:
                        buffer.extend(data)
                        self.stats['last_activity'] = time.time()
                
                # Process buffer for both text and binary messages
                buffer = self._process_buffer(buffer)
                
                # Keep buffer manageable
                if len(buffer) > 1000:
                    buffer = buffer[-500:]
                
                time.sleep(0.001)  # Small delay to prevent busy waiting
                
            except Exception as e:
                logger.error(f"Error in serial monitor: {e}")
                if not self.is_connected:
                    break
                time.sleep(0.1)
        
        logger.info("Serial monitor thread stopped")
    
    def _process_buffer(self, buffer: bytearray) -> bytearray:
        """Process buffer for mixed text and binary messages"""
        if not buffer:
            return buffer
        
        # First, try to find text messages (lines ending with \n)
        text_end = -1
        for i, byte in enumerate(buffer):
            if byte == ord('\n'):
                text_end = i + 1
                break
        
        # Process text messages if found
        if text_end > 0:
            text_data = buffer[:text_end]
            self._process_text_messages(text_data)
            buffer = buffer[text_end:]
        
        # Now look for binary CAN messages (exactly 24 bytes)
        # Search through the entire buffer for valid CAN messages
        i = 0
        while i <= len(buffer) - 24:
            # Check if this looks like a valid CAN message
            if self._is_valid_can_message(buffer[i:i+24]):
                if self._try_parse_can_message(buffer[i:i+24]):
                    # Remove the processed message and everything before it
                    buffer = buffer[i+24:]
                    i = 0  # Reset search position
                else:
                    i += 1  # Move to next position
            else:
                i += 1  # Move to next position
        
        return buffer
    
    def _is_valid_can_message(self, data: bytes) -> bool:
        """Check if data looks like a valid CAN message"""
        if len(data) != 24:
            return False
        
        try:
            # Parse CAN ID (first 4 bytes)
            can_id = struct.unpack('<I', data[0:4])[0]
            
            # Check if CAN ID is reasonable (not text data)
            if can_id > 0x1FFFFFFF or can_id == 0:  # Extended CAN IDs are 29 bits max
                return False
            
            # Check length field (offset 11)
            length = data[11]
            if length > 8:  # Parameter messages are 8 bytes max
                return False
            
            # Check if this is one of our known parameter IDs
            if can_id in self.PARAMETERS:
                return True
            
            # Could be other valid CAN messages
            return True
            
        except Exception:
            return False
    
    def _try_parse_can_message(self, buffer: bytearray) -> bool:
        """Try to parse a CAN message from buffer"""
        try:
            if len(buffer) < 24:
                return False
            
            # Parse CAN message structure (correct Teensy 4.1 offsets)
            can_id = struct.unpack('<I', buffer[0:4])[0]      # CAN ID at offset 0
            timestamp = struct.unpack('<I', buffer[4:8])[0]   # Timestamp at offset 4
            length = buffer[11]                               # Length at offset 11
            flags = buffer[10]                                # Flags at offset 10
            data = buffer[12:20]                              # Data buffer at offset 12
            reserved = buffer[20:24]                          # Reserved at offset 20
            
            # Validate message
            if length > 8:
                return False
            
            # Check if this is a parameter response
            if can_id in self.PARAMETERS and length == 8:
                return self._process_parameter_response(can_id, data)
            
            return True  # Valid CAN message but not a parameter response
            
        except Exception as e:
            logger.debug(f"Failed to parse CAN message: {e}")
            return False
    
    def _process_parameter_response(self, can_id: int, data: bytes) -> bool:
        """Process parameter response message"""
        try:
            if len(data) < 8:
                return False
            
            # Parse parameter message
            operation = data[0]
            value = struct.unpack('<f', data[1:5])[0]
            source_channel = data[5]
            request_id = data[6]
            reserved = data[7]
            
            # Validate response
            if operation != self.READ_RESPONSE:
                return False
            
            param_info = self.PARAMETERS.get(can_id)
            if not param_info:
                return False
            
            # Update statistics
            with self.lock:
                self.stats['total_responses'] += 1
                self.stats['successful_responses'] += 1
                
                # Store parameter value
                self.parameter_values[can_id] = value
                self.parameter_history[can_id].append({
                    'timestamp': time.time(),
                    'value': value
                })
            
            logger.info(f"✅ {param_info['name']}: {value:.2f} {param_info['unit']} (CAN ID 0x{can_id:08X})")
            return True
            
        except Exception as e:
            logger.error(f"Error processing parameter response: {e}")
            self.stats['parse_errors'] += 1
            return False
    
    def _process_text_messages(self, buffer: bytearray):
        """Process any text debug messages in buffer"""
        try:
            # Decode text and look for debug patterns
            text = buffer.decode('utf-8', errors='ignore')
            if text.strip():
                lines = text.split('\n')
                for line in lines:
                    line = line.strip()
                    if line:
                        # Look for ECU debug messages
                        if any(pattern in line for pattern in ['DEBUG:', 'ERROR:', 'SerialBridge:', 'ParameterRegistry:']):
                            logger.debug(f"ECU: {line}")
                        # Look for binary response indicators
                        elif 'Sending binary response' in line:
                            logger.info(f"ECU: {line}")
        except Exception as e:
            logger.debug(f"Text processing error: {e}")
    
    def _request_parameters(self):
        """Periodically request parameter values"""
        logger.info("Starting parameter request thread")
        
        while self.is_running and self.is_connected:
            try:
                # Send requests for all parameters with delays between them
                for can_id, param_info in self.PARAMETERS.items():
                    self._send_parameter_request(can_id)
                    time.sleep(0.2)  # 200ms delay between requests
                
                time.sleep(self.request_interval)
                
            except Exception as e:
                logger.error(f"Error in parameter request thread: {e}")
                if not self.is_connected:
                    break
                time.sleep(1.0)
        
        logger.info("Parameter request thread stopped")
    
    def _send_parameter_request(self, can_id: int):
        """Send a parameter request"""
        try:
            if not self.serial_conn or not self.serial_conn.is_open:
                return
            
            # Create parameter request data
            param_data = struct.pack('<Bfbbb', 
                self.READ_REQUEST,  # Operation
                0.0,                # Value (ignored for requests)
                1,                  # Source channel
                self.next_request_id, # Request ID
                0                   # Reserved
            )
            
            # Create binary CAN message
            message = bytearray(24)
            message[0:4] = struct.pack('<I', can_id)  # CAN ID
            message[11] = 8                           # Length
            message[12:20] = param_data               # Parameter data
            
            # Send message
            self.serial_conn.write(bytes(message))
            self.serial_conn.flush()
            
            with self.lock:
                self.stats['total_requests'] += 1
                self.next_request_id = (self.next_request_id % 255) + 1
            
            logger.info(f"📤 Sent request for {self.PARAMETERS[can_id]['name']} (CAN ID 0x{can_id:08X})")
            
        except Exception as e:
            logger.error(f"Error sending parameter request: {e}")
    
    def get_status(self) -> Dict[str, Any]:
        """Get current status"""
        with self.lock:
            uptime = time.time() - self.stats['start_time'] if self.stats['start_time'] else 0
            
            return {
                'connected': self.is_connected,
                'port': self.serial_conn.port if self.serial_conn else None,
                'parameter_values': self.parameter_values.copy(),
                'stats': {
                    'total_requests': self.stats['total_requests'],
                    'total_responses': self.stats['total_responses'],
                    'successful_responses': self.stats['successful_responses'],
                    'timeouts': self.stats['timeouts'],
                    'parse_errors': self.stats['parse_errors'],
                    'uptime': uptime,
                    'last_activity': self.stats['last_activity']
                }
            }

def main():
    """Main function for simple dashboard"""
    print("🚗 Simple ECU Parameter Dashboard")
    print("=" * 50)
    
    # Create ECU client
    client = ECUClient()
    
    # Show available ports
    ports = client.get_available_ports()
    print(f"\n📡 Available ports ({len(ports)}):")
    for i, port in enumerate(ports):
        print(f"  {i+1}. {port['device']} - {port['description']} ({port['manufacturer']})")
    
    # Find Teensy port
    teensy_port = None
    for port in ports:
        if 'Teensyduino' in port['manufacturer'] or 'usbmodem' in port['device']:
            teensy_port = port['device']
            break
    
    if not teensy_port:
        print("\n❌ No Teensy port found!")
        return
    
    print(f"\n🎯 Found Teensy port: {teensy_port}")
    
    # Connect to ECU
    print(f"\n🔌 Connecting to {teensy_port}...")
    if not client.connect(teensy_port):
        print("❌ Failed to connect!")
        return
    
    print("✅ Connected! Starting parameter monitoring...")
    print("\n📊 Parameter Values:")
    print("-" * 50)
    
    try:
        # Main monitoring loop
        while True:
            status = client.get_status()
            
            # Clear screen (simple approach)
            print("\033[2J\033[H")  # Clear screen and move cursor to top
            
            print("🚗 Simple ECU Parameter Dashboard")
            print("=" * 50)
            print(f"Status: {'🟢 Connected' if status['connected'] else '🔴 Disconnected'}")
            print(f"Port: {status['port']}")
            print(f"Uptime: {status['stats']['uptime']:.1f}s")
            print(f"Requests: {status['stats']['total_requests']}")
            print(f"Responses: {status['stats']['successful_responses']}")
            print(f"Errors: {status['stats']['parse_errors']}")
            print()
            
            print("📊 Parameter Values:")
            print("-" * 50)
            
            for can_id, param_info in client.PARAMETERS.items():
                value = status['parameter_values'].get(can_id, 'N/A')
                if value != 'N/A':
                    print(f"  {param_info['name']}: {value:.2f} {param_info['unit']}")
                else:
                    print(f"  {param_info['name']}: {value}")
            
            print("\n" + "-" * 50)
            print("Press Ctrl+C to exit")
            
            time.sleep(1.0)  # Update every second
            
    except KeyboardInterrupt:
        print("\n\n🛑 Shutting down...")
    finally:
        client.disconnect()
        print("✅ Disconnected")

if __name__ == "__main__":
    main() 