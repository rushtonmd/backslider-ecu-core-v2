#!/usr/bin/env python3
"""
Prefix-based Web Dashboard - Uses 0xFF 0xFF prefix for reliable binary message parsing
Based on your working implementation with web interface added
"""

import serial
import time
import struct
import threading
import logging
from typing import Dict, Any, List, Optional
from flask import Flask, render_template_string, request, jsonify
from flask_socketio import SocketIO, emit
import serial.tools.list_ports

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

class PrefixECUClient:
    """ECU client using 0xFF 0xFF prefix for binary message framing"""
    
    PARAMETERS = {
        0x10500001: {"name": "Fluid Temperature", "unit": "°C"},
        0x10500101: {"name": "Current Gear", "unit": ""}, 
        0x10300002: {"name": "Vehicle Speed", "unit": "mph"}
    }
    
    READ_REQUEST = 0x01
    READ_RESPONSE = 0x03
    
    # Binary message prefix
    BINARY_PREFIX = bytes([0xFF, 0xFF])
    CAN_MESSAGE_SIZE = 24
    TOTAL_BINARY_SIZE = 2 + CAN_MESSAGE_SIZE  # prefix + CAN message
    
    def __init__(self):
        self.serial_conn = None
        self.current_port = None
        self.is_connected = False
        self.is_running = False
        self.buffer = bytearray()
        self.parameter_values = {}
        self.connected_clients = 0
        
        self.stats = {
            'total_requests': 0,
            'total_responses': 0,
            'successful_responses': 0,
            'parse_errors': 0,
            'raw_bytes_received': 0,
            'text_messages': 0,
            'binary_messages': 0,
            'prefixes_found': 0,
            'start_time': None
        }
        
        self.request_interval = 1.0  # seconds
        self.next_request_id = 1
        self.monitor_thread = None
        self.request_thread = None
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
        """Connect to ECU"""
        try:
            if self.is_connected:
                self.disconnect()
            
            logger.info(f"🔌 Connecting to {port} at {baudrate} baud...")
            self.serial_conn = serial.Serial(
                port=port,
                baudrate=baudrate,
                timeout=0.1,
                write_timeout=1.0
            )
            
            if not self.serial_conn.is_open:
                raise serial.SerialException("Failed to open serial port")
            
            self.current_port = port
            self.is_connected = True
            self.is_running = True
            self.stats['start_time'] = time.time()
            
            # Start monitoring and request threads
            self.start_threads()
            
            logger.info("✅ Connected successfully!")
            return True
            
        except Exception as e:
            logger.error(f"❌ Failed to connect: {e}")
            self.cleanup_connection()
            return False
    
    def disconnect(self):
        """Disconnect from ECU"""
        logger.info("🔌 Disconnecting...")
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
        self.current_port = None
        self.buffer.clear()
        self.parameter_values.clear()
    
    def start_threads(self):
        """Start monitoring and request threads"""
        # Start serial monitor thread
        self.monitor_thread = threading.Thread(
            target=self._monitor_serial,
            name="SerialMonitor",
            daemon=True
        )
        self.monitor_thread.start()
        
        # Start parameter request thread
        self.request_thread = threading.Thread(
            target=self._request_parameters,
            name="ParameterRequester", 
            daemon=True
        )
        self.request_thread.start()
        
        logger.info("📡 Started monitoring and request threads")
    
    def _monitor_serial(self):
        """Monitor serial port with prefix-based parsing"""
        logger.info("🔍 Starting prefix-based serial monitor...")
        
        while self.is_running and self.is_connected and self.serial_conn:
            try:
                if self.serial_conn.in_waiting > 0:
                    data = self.serial_conn.read(self.serial_conn.in_waiting)
                    if data:
                        with self.lock:
                            self.stats['raw_bytes_received'] += len(data)
                            # Add to buffer
                            self.buffer.extend(data)
                            # Process buffer
                            self._process_buffer_with_prefix()
                
                time.sleep(0.001)
                
            except Exception as e:
                logger.error(f"❌ Monitor error: {e}")
                if not self.is_connected:
                    break
                time.sleep(0.1)
        
        logger.info("🔍 Serial monitor stopped")
    
    def _request_parameters(self):
        """Periodically request parameter values"""
        logger.info("📤 Starting parameter request thread")
        
        while self.is_running and self.is_connected:
            try:
                # Request each parameter
                for can_id in self.PARAMETERS.keys():
                    if not self.is_running:
                        break
                    self.send_parameter_request(can_id)
                    time.sleep(0.2)  # Small delay between requests
                
                # Wait for next cycle
                time.sleep(self.request_interval)
                
            except Exception as e:
                logger.error(f"❌ Request thread error: {e}")
                time.sleep(1.0)
        
        logger.info("📤 Parameter request thread stopped")
    
    def _process_buffer_with_prefix(self):
        """Process buffer using 0xFF 0xFF prefix for binary message framing"""
        if len(self.buffer) < 2:
            return
        
        # Step 1: Look for binary messages with prefix FIRST
        self._extract_binary_messages_with_prefix()
        
        # Step 2: Extract complete text lines (after binary processing)
        self._extract_text_lines()
    
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
    
    def _extract_binary_messages_with_prefix(self):
        """Extract binary messages using 0xFF 0xFF prefix"""
        if len(self.buffer) < self.TOTAL_BINARY_SIZE:
            return
        
        # Look for 0xFF 0xFF prefix
        i = 0
        while i <= len(self.buffer) - self.TOTAL_BINARY_SIZE:
            # Check for prefix
            if (self.buffer[i] == 0xFF and self.buffer[i + 1] == 0xFF):
                self.stats['prefixes_found'] += 1
                logger.debug(f"🔍 Found binary prefix at position {i}")
                
                # Extract the CAN message (skip the 2-byte prefix)
                can_message_data = self.buffer[i + 2:i + 2 + self.CAN_MESSAGE_SIZE]
                
                # Try to parse the CAN message
                if self._try_parse_can_message(can_message_data):
                    # Successfully parsed, remove the entire message including prefix
                    self.buffer = self.buffer[i + self.TOTAL_BINARY_SIZE:]
                    self.stats['binary_messages'] += 1
                    i = 0  # Reset search position
                else:
                    # Parsing failed, move to next position
                    i += 1
            else:
                # Not a prefix, move to next position
                i += 1
        
        # If buffer is getting too large, trim it
        if len(self.buffer) > 1024:
            logger.warning(f"Buffer too large ({len(self.buffer)} bytes), trimming...")
            self.buffer = self.buffer[-512:]  # Keep last 512 bytes
    
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
            if len(data) < 24:
                return False
            
            can_id = struct.unpack('<I', data[0:4])[0]
            timestamp = struct.unpack('<I', data[4:8])[0]
            length = data[11]
            flags = data[10]
            param_data = data[12:20]
            reserved = data[20:24]
            
            logger.debug(f"🔍 Parsing CAN message: ID=0x{can_id:08X}, len={length}, data={param_data.hex()}")
            
            # Only process if it's one of our parameters and has correct length
            if can_id in self.PARAMETERS and length == 8:
                logger.debug(f"✅ Valid parameter message found for {self.PARAMETERS[can_id]['name']}")
                return self._process_parameter_response(can_id, param_data)
            else:
                logger.debug(f"⚠️ Not a valid parameter message: can_id=0x{can_id:08X}, length={length}")
            
            return False
            
        except Exception as e:
            logger.error(f"❌ CAN parse error: {e}")
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
            with self.lock:
                self.parameter_values[can_id] = {
                    'value': value,
                    'timestamp': time.time(),
                    'name': param_name,
                    'unit': unit
                }
                
                self.stats['total_responses'] += 1
                self.stats['successful_responses'] += 1
            
            logger.info(f"✅ {param_name}: {value:.2f} {unit} (CAN ID 0x{can_id:08X})")
            
            # Emit update to web clients
            if self.connected_clients > 0:
                self._emit_parameter_update(can_id, value, param_name, unit)
            
            return True
            
        except Exception as e:
            logger.error(f"❌ Parameter processing error: {e}")
            self.stats['parse_errors'] += 1
            return False
    
    def _emit_parameter_update(self, can_id: int, value: float, name: str, unit: str):
        """Emit parameter update to web clients"""
        update_data = {
            'can_id': f"0x{can_id:08X}",
            'name': name,
            'value': value,
            'unit': unit,
            'timestamp': time.time()
        }
        socketio.emit('parameter_update', update_data)
    
    def send_parameter_request(self, can_id: int):
        """Send a parameter request"""
        try:
            if not self.serial_conn or not self.serial_conn.is_open:
                return
            
            param_data = struct.pack('<BfBBb', 
                self.READ_REQUEST,  # Operation
                0.0,                # Value
                1,                  # Source channel
                self.next_request_id,  # Request ID (unsigned byte)
                0                   # Reserved
            )
            
            message = bytearray(24)
            message[0:4] = struct.pack('<I', can_id)
            message[11] = 8
            message[12:20] = param_data
            
            # Send 0xFF 0xFF prefix first
            self.serial_conn.write(self.BINARY_PREFIX)
            # Send the CAN message
            self.serial_conn.write(bytes(message))
            self.serial_conn.flush()
            
            with self.lock:
                self.stats['total_requests'] += 1
                self.next_request_id = (self.next_request_id % 255) + 1
            
            param_name = self.PARAMETERS.get(can_id, {}).get("name", "Unknown")
            logger.debug(f"📤 Sent request for {param_name} (CAN ID 0x{can_id:08X})")
            
        except Exception as e:
            logger.error(f"❌ Send error: {e}")
    
    def get_status(self) -> Dict[str, Any]:
        """Get current status"""
        with self.lock:
            uptime = 0
            if self.stats['start_time']:
                uptime = time.time() - self.stats['start_time']
            
            return {
                'connected': self.is_connected,
                'port': self.current_port,
                'running': self.is_running,
                'uptime': uptime,
                'buffer_size': len(self.buffer),
                'parameter_values': self.parameter_values.copy(),
                'stats': self.stats.copy()
            }

# Global ECU client instance
ecu_client = PrefixECUClient()

# Flask application
app = Flask(__name__)
app.config['SECRET_KEY'] = 'prefix-dashboard-secret'
socketio = SocketIO(app, cors_allowed_origins="*")

# HTML Template
HTML_TEMPLATE = """
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Prefix-based ECU Dashboard</title>
    <script src="https://cdnjs.cloudflare.com/ajax/libs/socket.io/4.0.1/socket.io.js"></script>
    <style>
        * { margin: 0; padding: 0; box-sizing: border-box; }
        
        body {
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            background: linear-gradient(135deg, #1e3c72 0%, #2a5298 100%);
            color: white;
            min-height: 100vh;
            padding: 20px;
        }
        
        .container { max-width: 1400px; margin: 0 auto; }
        
        .header {
            background: rgba(255,255,255,0.1);
            padding: 20px;
            border-radius: 15px;
            margin-bottom: 30px;
            backdrop-filter: blur(10px);
            text-align: center;
        }
        
        .header h1 {
            font-size: 2.5em;
            margin-bottom: 10px;
            text-shadow: 2px 2px 4px rgba(0,0,0,0.3);
        }
        
        .connection-status {
            display: flex;
            align-items: center;
            justify-content: center;
            gap: 10px;
            margin-top: 15px;
        }
        
        .status-dot {
            width: 12px;
            height: 12px;
            border-radius: 50%;
            background: #ff4444;
            animation: pulse 2s infinite;
        }
        
        .status-dot.connected { background: #44ff44; }
        
        @keyframes pulse {
            0%, 100% { opacity: 1; }
            50% { opacity: 0.5; }
        }
        
        .controls {
            background: rgba(255,255,255,0.1);
            padding: 20px;
            border-radius: 15px;
            margin-bottom: 30px;
            backdrop-filter: blur(10px);
        }
        
        .control-row {
            display: grid;
            grid-template-columns: 2fr 1fr 1fr 1fr;
            gap: 15px;
            align-items: end;
            margin-bottom: 15px;
        }
        
        .form-group {
            display: flex;
            flex-direction: column;
        }
        
        label {
            font-size: 0.9em;
            margin-bottom: 5px;
            opacity: 0.9;
        }
        
        select, input, button {
            padding: 12px;
            border: none;
            border-radius: 8px;
            font-size: 14px;
        }
        
        select, input {
            background: rgba(255,255,255,0.2);
            color: white;
            border: 1px solid rgba(255,255,255,0.3);
        }
        
        select option {
            background: #2a5298;
            color: white;
        }
        
        button {
            background: #4CAF50;
            color: white;
            cursor: pointer;
            transition: all 0.3s ease;
            font-weight: bold;
        }
        
        button:hover { background: #45a049; transform: translateY(-1px); }
        button:disabled { background: #666; cursor: not-allowed; transform: none; }
        
        .disconnect-btn { background: #f44336; }
        .disconnect-btn:hover { background: #da190b; }
        
        .parameters {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(350px, 1fr));
            gap: 20px;
            margin-bottom: 30px;
        }
        
        .parameter-card {
            background: rgba(255,255,255,0.1);
            padding: 25px;
            border-radius: 15px;
            backdrop-filter: blur(10px);
            border: 1px solid rgba(255,255,255,0.2);
            text-align: center;
            position: relative;
            overflow: hidden;
        }
        
        .parameter-card::before {
            content: '';
            position: absolute;
            top: 0;
            left: 0;
            right: 0;
            height: 3px;
            background: linear-gradient(90deg, #ff6b6b, #4ecdc4, #45b7d1);
        }
        
        .parameter-name {
            font-size: 1.2em;
            margin-bottom: 15px;
            opacity: 0.9;
            font-weight: 600;
        }
        
        .parameter-value {
            font-size: 3.5em;
            font-weight: bold;
            margin: 15px 0;
            text-shadow: 2px 2px 4px rgba(0,0,0,0.3);
        }
        
        .parameter-unit {
            font-size: 0.4em;
            opacity: 0.7;
            margin-left: 10px;
        }
        
        .parameter-status {
            font-size: 0.9em;
            opacity: 0.7;
            margin-top: 10px;
        }
        
        .parameter-card.stale {
            opacity: 0.6;
            border-color: rgba(255,255,255,0.1);
        }
        
        .stats {
            background: rgba(255,255,255,0.1);
            padding: 20px;
            border-radius: 15px;
            backdrop-filter: blur(10px);
        }
        
        .stats h3 {
            margin-bottom: 15px;
            text-align: center;
        }
        
        .stats-grid {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(150px, 1fr));
            gap: 15px;
        }
        
        .stat-item {
            background: rgba(255,255,255,0.1);
            padding: 15px;
            border-radius: 10px;
            text-align: center;
        }
        
        .stat-value {
            font-size: 1.8em;
            font-weight: bold;
            margin-bottom: 5px;
        }
        
        .stat-label {
            font-size: 0.9em;
            opacity: 0.8;
        }
        
        .error-message {
            background: rgba(244, 67, 54, 0.9);
            color: white;
            padding: 15px;
            border-radius: 10px;
            margin: 15px 0;
            display: none;
        }
        
        .protocol-info {
            background: rgba(76, 175, 80, 0.2);
            padding: 15px;
            border-radius: 10px;
            margin-top: 15px;
            border: 1px solid rgba(76, 175, 80, 0.3);
        }
        
        @media (max-width: 768px) {
            .control-row { grid-template-columns: 1fr; }
            .parameters { grid-template-columns: 1fr; }
            .stats-grid { grid-template-columns: repeat(2, 1fr); }
        }
    </style>
</head>
<body>
    <div class="container">
        <div class="header">
            <h1>🚗 Prefix-based ECU Dashboard</h1>
            <div class="connection-status">
                <div class="status-dot" id="connectionDot"></div>
                <span id="connectionText">Disconnected</span>
                <span id="uptimeText"></span>
            </div>
        </div>
        
        <div class="controls">
            <div class="control-row">
                <div class="form-group">
                    <label for="portSelect">Serial Port</label>
                    <select id="portSelect">
                        <option value="">Select Port...</option>
                    </select>
                </div>
                <div class="form-group">
                    <label for="baudrateSelect">Baud Rate</label>
                    <select id="baudrateSelect">
                        <option value="9600">9600</option>
                        <option value="115200" selected>115200</option>
                        <option value="230400">230400</option>
                        <option value="460800">460800</option>
                    </select>
                </div>
                <div class="form-group">
                    <button id="connectBtn" onclick="connect()">Connect</button>
                </div>
                <div class="form-group">
                    <button id="disconnectBtn" onclick="disconnect()" class="disconnect-btn" disabled>Disconnect</button>
                </div>
            </div>
            
            <div class="protocol-info">
                <strong>📡 Protocol:</strong> 0xFF 0xFF prefix + 24-byte CAN message | 
                <strong>📤 Requests:</strong> 1Hz automatic | 
                <strong>📥 Responses:</strong> Binary parameter messages
            </div>
            
            <div class="error-message" id="errorMessage"></div>
        </div>
        
        <div class="parameters">
            <div class="parameter-card" id="param-0x10500001">
                <div class="parameter-name">Fluid Temperature</div>
                <div class="parameter-value">--<span class="parameter-unit">°C</span></div>
                <div class="parameter-status">No data</div>
            </div>
            
            <div class="parameter-card" id="param-0x10500101">
                <div class="parameter-name">Current Gear</div>
                <div class="parameter-value">--<span class="parameter-unit"></span></div>
                <div class="parameter-status">No data</div>
            </div>
            
            <div class="parameter-card" id="param-0x10300002">
                <div class="parameter-name">Vehicle Speed</div>
                <div class="parameter-value">--<span class="parameter-unit">mph</span></div>
                <div class="parameter-status">No data</div>
            </div>
        </div>
        
        <div class="stats">
            <h3>📊 Communication Statistics</h3>
            <div class="stats-grid">
                <div class="stat-item">
                    <div class="stat-value" id="totalRequests">0</div>
                    <div class="stat-label">Requests Sent</div>
                </div>
                <div class="stat-item">
                    <div class="stat-value" id="totalResponses">0</div>
                    <div class="stat-label">Responses</div>
                </div>
                <div class="stat-item">
                    <div class="stat-value" id="successRate">0%</div>
                    <div class="stat-label">Success Rate</div>
                </div>
                <div class="stat-item">
                    <div class="stat-value" id="binaryMessages">0</div>
                    <div class="stat-label">Binary Messages</div>
                </div>
                <div class="stat-item">
                    <div class="stat-value" id="prefixesFound">0</div>
                    <div class="stat-label">Prefixes Found</div>
                </div>
                <div class="stat-item">
                    <div class="stat-value" id="parseErrors">0</div>
                    <div class="stat-label">Parse Errors</div>
                </div>
                <div class="stat-item">
                    <div class="stat-value" id="textMessages">0</div>
                    <div class="stat-label">Text Messages</div>
                </div>
                <div class="stat-item">
                    <div class="stat-value" id="bufferSize">0</div>
                    <div class="stat-label">Buffer Size</div>
                </div>
            </div>
        </div>
    </div>

    <script>
        const socket = io();
        let isConnected = false;
        let lastUpdateTimes = {};
        
        window.onload = function() {
            loadPorts();
            startStatusUpdates();
        };
        
        function loadPorts() {
            fetch('/api/ports')
                .then(response => response.json())
                .then(data => {
                    const select = document.getElementById('portSelect');
                    select.innerHTML = '<option value="">Select Port...</option>';
                    
                    data.ports.forEach(port => {
                        const option = document.createElement('option');
                        option.value = port.device;
                        option.textContent = `${port.device} - ${port.description}`;
                        select.appendChild(option);
                    });
                })
                .catch(error => {
                    console.error('Error loading ports:', error);
                    showError('Failed to load serial ports');
                });
        }
        
        function connect() {
            const port = document.getElementById('portSelect').value;
            const baudrate = parseInt(document.getElementById('baudrateSelect').value);
            
            if (!port) {
                showError('Please select a serial port');
                return;
            }
            
            hideError();
            setConnecting(true);
            
            fetch('/api/connect', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({port: port, baudrate: baudrate})
            })
            .then(response => response.json())
            .then(data => {
                if (data.success) {
                    updateConnectionStatus(true, port);
                } else {
                    showError(data.error || 'Connection failed');
                    setConnecting(false);
                }
            })
            .catch(error => {
                console.error('Connection error:', error);
                showError('Connection request failed');
                setConnecting(false);
            });
        }
        
        function disconnect() {
            fetch('/api/disconnect', {method: 'POST'})
                .then(response => response.json())
                .then(data => {
                    updateConnectionStatus(false, '');
                })
                .catch(error => {
                    console.error('Disconnect error:', error);
                });
        }
        
        function setConnecting(connecting) {
            const connectBtn = document.getElementById('connectBtn');
            const disconnectBtn = document.getElementById('disconnectBtn');
            
            if (connecting) {
                connectBtn.textContent = 'Connecting...';
                connectBtn.disabled = true;
            } else {
                connectBtn.textContent = 'Connect';
                connectBtn.disabled = false;
                disconnectBtn.disabled = true;
            }
        }
        
        function updateConnectionStatus(connected, port) {
            isConnected = connected;
            const dot = document.getElementById('connectionDot');
            const text = document.getElementById('connectionText');
            const connectBtn = document.getElementById('connectBtn');
            const disconnectBtn = document.getElementById('disconnectBtn');
            
            if (connected) {
                dot.classList.add('connected');
                text.textContent = `Connected to ${port}`;
                connectBtn.textContent = 'Connect';
                connectBtn.disabled = true;
                disconnectBtn.disabled = false;
                hideError();
            } else {
                dot.classList.remove('connected');
                text.textContent = 'Disconnected';
                connectBtn.textContent = 'Connect';
                connectBtn.disabled = false;
                disconnectBtn.disabled = true;
                resetParameterDisplays();
            }
        }
        
        function resetParameterDisplays() {
            const paramCards = document.querySelectorAll('.parameter-card');
            paramCards.forEach(card => {
                const valueEl = card.querySelector('.parameter-value');
                const statusEl = card.querySelector('.parameter-status');
                const unitEl = card.querySelector('.parameter-unit');
                
                if (valueEl && unitEl) {
                    valueEl.innerHTML = `--<span class="parameter-unit">${unitEl.textContent}</span>`;
                }
                if (statusEl) {
                    statusEl.textContent = 'No data';
                }
                card.classList.remove('stale');
            });
            
            lastUpdateTimes = {};
        }
        
        function showError(message) {
            const errorEl = document.getElementById('errorMessage');
            errorEl.textContent = message;
            errorEl.style.display = 'block';
        }
        
        function hideError() {
            document.getElementById('errorMessage').style.display = 'none';
        }
        
        function startStatusUpdates() {
            setInterval(updateStatus, 1000);
            setInterval(checkStaleData, 5000);
        }
        
        function updateStatus() {
            if (!isConnected) return;
            
            fetch('/api/status')
                .then(response => response.json())
                .then(data => {
                    updateStats(data);
                    updateUptime(data.uptime);
                })
                .catch(error => {
                    console.error('Status update error:', error);
                });
        }
        
        function updateStats(status) {
            const stats = status.stats || {};
            
            document.getElementById('totalRequests').textContent = stats.total_requests || 0;
            document.getElementById('totalResponses').textContent = stats.total_responses || 0;
            document.getElementById('binaryMessages').textContent = stats.binary_messages || 0;
            document.getElementById('prefixesFound').textContent = stats.prefixes_found || 0;
            document.getElementById('parseErrors').textContent = stats.parse_errors || 0;
            document.getElementById('textMessages').textContent = stats.text_messages || 0;
            document.getElementById('bufferSize').textContent = status.buffer_size || 0;
            
            // Calculate success rate
            const totalRequests = stats.total_requests || 0;
            const successfulResponses = stats.successful_responses || 0;
            const successRate = totalRequests > 0 ? Math.round((successfulResponses / totalRequests) * 100) : 0;
            document.getElementById('successRate').textContent = `${successRate}%`;
        }
        
        function updateUptime(uptime) {
            if (uptime > 0) {
                const hours = Math.floor(uptime / 3600);
                const minutes = Math.floor((uptime % 3600) / 60);
                const seconds = Math.floor(uptime % 60);
                
                let uptimeText = '';
                if (hours > 0) {
                    uptimeText = `${hours}h ${minutes}m ${seconds}s`;
                } else if (minutes > 0) {
                    uptimeText = `${minutes}m ${seconds}s`;
                } else {
                    uptimeText = `${seconds}s`;
                }
                
                document.getElementById('uptimeText').textContent = `(${uptimeText})`;
            } else {
                document.getElementById('uptimeText').textContent = '';
            }
        }
        
        function checkStaleData() {
            const now = Date.now();
            const staleThreshold = 10000; // 10 seconds
            
            Object.keys(lastUpdateTimes).forEach(canId => {
                const lastUpdate = lastUpdateTimes[canId];
                const card = document.getElementById(`param-${canId}`);
                
                if (card) {
                    if (now - lastUpdate > staleThreshold) {
                        card.classList.add('stale');
                        const statusEl = card.querySelector('.parameter-status');
                        if (statusEl) {
                            statusEl.textContent = 'Data stale';
                        }
                    }
                }
            });
        }
        
        function updateParameter(data) {
            const canId = data.can_id;
            const card = document.getElementById(`param-${canId}`);
            
            if (!card) return;
            
            const valueEl = card.querySelector('.parameter-value');
            const statusEl = card.querySelector('.parameter-status');
            
            if (valueEl) {
                // Format value based on parameter type
                let formattedValue;
                if (data.name.includes('Temperature')) {
                    formattedValue = data.value.toFixed(1);
                } else if (data.name.includes('Gear')) {
                    formattedValue = Math.round(data.value).toString();
                } else if (data.name.includes('Speed')) {
                    formattedValue = Math.round(data.value).toString();
                } else {
                    formattedValue = data.value.toFixed(2);
                }
                
                valueEl.innerHTML = `${formattedValue}<span class="parameter-unit">${data.unit}</span>`;
            }
            
            if (statusEl) {
                const updateTime = new Date(data.timestamp * 1000);
                statusEl.textContent = `Updated ${updateTime.toLocaleTimeString()}`;
            }
            
            // Remove stale class and update last update time
            card.classList.remove('stale');
            lastUpdateTimes[canId] = Date.now();
        }
        
        // Socket event handlers
        socket.on('connect', function() {
            console.log('WebSocket connected');
        });
        
        socket.on('disconnect', function() {
            console.log('WebSocket disconnected');
        });
        
        socket.on('parameter_update', function(data) {
            updateParameter(data);
        });
        
        socket.on('connection_status', function(data) {
            updateConnectionStatus(data.connected, data.port || '');
        });
        
        // Refresh ports when dropdown is focused
        document.getElementById('portSelect').addEventListener('focus', loadPorts);
    </script>
</body>
</html>
"""

# API Routes
@app.route('/')
def index():
    return render_template_string(HTML_TEMPLATE)

@app.route('/api/ports')
def get_ports():
    """Get available serial ports"""
    try:
        ports = ecu_client.get_available_ports()
        return jsonify({'success': True, 'ports': ports})
    except Exception as e:
        logger.error(f"Error getting ports: {e}")
        return jsonify({'success': False, 'error': str(e)})

@app.route('/api/connect', methods=['POST'])
def connect():
    """Connect to ECU"""
    try:
        data = request.get_json()
        port = data.get('port')
        baudrate = data.get('baudrate', 115200)
        
        if not port:
            return jsonify({'success': False, 'error': 'Port not specified'})
        
        success = ecu_client.connect(port, baudrate)
        
        if success:
            # Emit connection status to all clients
            socketio.emit('connection_status', {
                'connected': True,
                'port': port
            })
            return jsonify({'success': True})
        else:
            return jsonify({'success': False, 'error': 'Failed to connect to ECU'})
            
    except Exception as e:
        logger.error(f"Connection error: {e}")
        return jsonify({'success': False, 'error': str(e)})

@app.route('/api/disconnect', methods=['POST'])
def disconnect():
    """Disconnect from ECU"""
    try:
        ecu_client.disconnect()
        
        # Emit disconnection status to all clients
        socketio.emit('connection_status', {
            'connected': False,
            'port': None
        })
        
        return jsonify({'success': True})
        
    except Exception as e:
        logger.error(f"Disconnection error: {e}")
        return jsonify({'success': False, 'error': str(e)})

@app.route('/api/status')
def get_status():
    """Get current ECU status"""
    try:
        status = ecu_client.get_status()
        return jsonify(status)
    except Exception as e:
        logger.error(f"Status error: {e}")
        return jsonify({'error': str(e)})

# WebSocket event handlers
@socketio.on('connect')
def handle_connect():
    ecu_client.connected_clients += 1
    logger.info(f"Client connected to WebSocket (total: {ecu_client.connected_clients})")
    
    # Send current connection status
    emit('connection_status', {
        'connected': ecu_client.is_connected,
        'port': ecu_client.current_port
    })

@socketio.on('disconnect')
def handle_disconnect():
    ecu_client.connected_clients -= 1
    logger.info(f"Client disconnected from WebSocket (total: {ecu_client.connected_clients})")

if __name__ == '__main__':
    import argparse
    
    parser = argparse.ArgumentParser(description='Prefix-based ECU Dashboard')
    parser.add_argument('--host', default='0.0.0.0', help='Host address')
    parser.add_argument('--port', type=int, default=5000, help='Web server port')
    parser.add_argument('--debug', action='store_true', help='Enable debug mode')
    
    args = parser.parse_args()
    
    logger.info("🚀 Starting Prefix-based ECU Dashboard")
    logger.info(f"📊 Dashboard: http://{args.host}:{args.port}")
    logger.info("🔧 Protocol Features:")
    logger.info("   - 0xFF 0xFF binary message prefix")
    logger.info("   - 24-byte CAN message structure")
    logger.info("   - Automatic 1Hz parameter requests")
    logger.info("   - Real-time WebSocket updates")
    logger.info("   - Mixed text/binary stream handling")
    
    try:
        socketio.run(
            app,
            host=args.host,
            port=args.port,
            debug=args.debug,
            allow_unsafe_werkzeug=True
        )
    except KeyboardInterrupt:
        logger.info("\n🛑 Shutting down dashboard...")
    finally:
        ecu_client.disconnect()
        logger.info("Dashboard stopped")