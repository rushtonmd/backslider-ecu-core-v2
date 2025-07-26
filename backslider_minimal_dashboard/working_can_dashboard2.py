#!/usr/bin/env python3
"""
Fresh Working CAN Dashboard - Based on successful minimal test
"""

print("🔥 FRESH VERSION 3.0 LOADED 🔥")

from flask import Flask, render_template_string, request, jsonify
from flask_socketio import SocketIO, emit
import serial
import serial.tools.list_ports
import struct
import time
import datetime
import threading
import re

app = Flask(__name__)
app.config['SECRET_KEY'] = 'can-dashboard-secret'
socketio = SocketIO(app, cors_allowed_origins="*")

class FreshCANDashboard:
    def __init__(self):
        self.serial_conn = None
        self.current_port = None
        self.running = False
        self.connected_clients = 0
        
        # Statistics
        self.stats = {
            'total_can_messages': 0,
            'total_debug_messages': 0,
            'start_time': None,
            'message_rate': 0
        }
        
        # Known CAN message types
        self.message_types = {
            0x10500101: {"name": "Transmission Current Gear", "unit": "gear"},
            0x10300002: {"name": "Vehicle Speed", "unit": "mph"},
            0x10500001: {"name": "Transmission Fluid Temperature", "unit": "°F"}
        }
        
        print("✅ Fresh CAN Dashboard initialized")
    
    def get_available_ports(self):
        """Get available serial ports"""
        ports = []
        for port in serial.tools.list_ports.comports():
            ports.append({
                'device': port.device,
                'description': port.description or 'Unknown device'
            })
        return sorted(ports, key=lambda x: x['device'])
    
    def connect_serial(self, port, baudrate=115200):
        """Connect to serial port"""
        try:
            if self.serial_conn and self.serial_conn.is_open:
                self.disconnect_serial()
            
            self.serial_conn = serial.Serial(
                port=port,
                baudrate=baudrate,
                timeout=1.0
            )
            
            self.current_port = port
            self.stats['start_time'] = time.time()
            self.stats['total_can_messages'] = 0
            self.stats['total_debug_messages'] = 0
            
            print(f"✅ Connected to {port} at {baudrate} baud")
            return True
            
        except Exception as e:
            print(f"❌ Failed to connect to {port}: {e}")
            return False
    
    def disconnect_serial(self):
        """Disconnect from serial port"""
        self.running = False
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
        self.current_port = None
        print("🔌 Serial disconnected")
    
    def start_monitoring(self):
        """Start monitoring - SIMPLE VERSION BASED ON WORKING TEST"""
        if not self.serial_conn or not self.serial_conn.is_open:
            print("❌ Cannot start monitoring - serial not ready")
            return False
        
        self.running = True
        print("🚀 Starting FRESH monitoring (like the working test)...")
        
        def read_loop():
            print("📖 FRESH monitor started successfully!")
            loop_count = 0
            buffer = bytearray()
            
            while self.running and self.serial_conn and self.serial_conn.is_open:
                loop_count += 1
                
                try:
                    # Use same approach as working minimal test
                    data = self.serial_conn.read(100)
                    
                    if data:
                        buffer.extend(data)
                        self.stats['total_can_messages'] += 1
                        print(f"📡 Loop {loop_count}: Got {len(data)} bytes (total messages: {self.stats['total_can_messages']})")
                        
                        # Check for debug messages
                        if b'DEBUG:' in data:
                            self.stats['total_debug_messages'] += 1
                            print(f"📝 Found debug message (total: {self.stats['total_debug_messages']})")
                        
                        # Check for CAN patterns
                        if b'\x01\x01P\x10' in buffer or b'\x01\x00P\x10' in buffer or b'\x02\x00\x30\x10' in buffer:
                            print(f"🎯 Found CAN pattern! Parsing...")
                            self.parse_and_send_messages(buffer)
                        
                        # Keep buffer manageable
                        if len(buffer) > 2000:
                            buffer = buffer[-1000:]
                        
                        # Send stats update
                        if self.connected_clients > 0:
                            socketio.emit('stats_update', self.get_stats())
                    
                    time.sleep(0.1)
                    
                except Exception as e:
                    print(f"❌ Read error: {e}")
                    import traceback
                    traceback.print_exc()
                    break
            
            print(f"🛑 FRESH monitor stopped after {loop_count} loops")
        
        thread = threading.Thread(target=read_loop, daemon=True)
        thread.start()
        print("🧵 FRESH monitor thread started")
        return True
    
    def parse_and_send_messages(self, buffer):
        """Parse CAN messages and send to dashboard"""
        # Look for multiple CAN ID patterns
        can_patterns = {
            b'\x01\x01P\x10': 0x10500101,  # Transmission Current Gear
            b'\x01\x00P\x10': 0x10500001,  # Transmission Fluid Temperature  
            b'\x02\x00\x30\x10': 0x10300002,  # Vehicle Speed
        }
        
        i = 0
        messages_found = 0
        
        while i < len(buffer) - 25:
            # Check for any of our known CAN ID patterns
            found_pattern = None
            found_can_id = None
            
            for pattern, can_id in can_patterns.items():
                if i + len(pattern) <= len(buffer) and buffer[i:i+len(pattern)] == pattern:
                    found_pattern = pattern
                    found_can_id = can_id
                    break
            
            if found_pattern:
                try:
                    print(f"   🎯 Found CAN ID 0x{found_can_id:08X} at position {i}")
                    
                    # Based on your pattern analysis:
                    # CAN ID (4) + variable + 60 00 01 00 00 00 + length (1) + 00 00 + data (4)
                    
                    # Look for length at position i+11
                    length_pos = i + 11
                    if length_pos < len(buffer):
                        msg_length = buffer[length_pos]
                        print(f"   📏 Length at pos {length_pos}: {msg_length}")
                        
                        if msg_length == 4:  # Expecting 4-byte float
                            data_pos = length_pos + 3  # Skip length + 2 padding bytes
                            if data_pos + 4 <= len(buffer):
                                data_bytes = buffer[data_pos:data_pos+4]
                                
                                # Show raw data
                                print(f"   🔢 Raw data: {data_bytes.hex().upper()}")
                                
                                # Try to parse as float (both endianness)
                                try:
                                    float_le = struct.unpack('<f', data_bytes)[0]
                                    float_be = struct.unpack('>f', data_bytes)[0]
                                    
                                    print(f"   📊 Float LE: {float_le:.3f}, BE: {float_be:.3f}")
                                    
                                    # Choose the most reasonable value based on the message type
                                    if found_can_id == 0x10500101:  # Gear (0-10 range)
                                        if 0 <= float_le <= 10:
                                            float_val = float_le
                                        elif 0 <= float_be <= 10:
                                            float_val = float_be
                                        else:
                                            float_val = float_le  # Default to LE
                                        msg_name = "Transmission Current Gear"
                                        msg_unit = "gear"
                                    elif found_can_id == 0x10500001:  # Temperature (50-400 range)
                                        if 50 <= float_le <= 400:
                                            float_val = float_le
                                        elif 50 <= float_be <= 400:
                                            float_val = float_be
                                        else:
                                            float_val = float_le  # Default to LE
                                        msg_name = "Transmission Fluid Temperature"
                                        msg_unit = "°F"
                                    elif found_can_id == 0x10300002:  # Speed (0-200 range)
                                        if 0 <= float_le <= 200:
                                            float_val = float_le
                                        elif 0 <= float_be <= 200:
                                            float_val = float_be
                                        else:
                                            float_val = float_le  # Default to LE
                                        msg_name = "Vehicle Speed"
                                        msg_unit = "mph"
                                    else:
                                        float_val = float_le
                                        msg_name = f"Unknown (0x{found_can_id:08X})"
                                        msg_unit = ""
                                    
                                    print(f"   ✅ Final value: {float_val:.2f} {msg_unit}")
                                    
                                    # Send to dashboard
                                    msg = {
                                        'can_id': f"0x{found_can_id:08X}",
                                        'can_id_int': found_can_id,
                                        'length': msg_length,
                                        'data': list(data_bytes),
                                        'hex_data': ' '.join(f'{b:02X}' for b in data_bytes),
                                        'timestamp': datetime.datetime.now().strftime('%H:%M:%S.%f')[:-3],
                                        'message_name': msg_name,
                                        'message_unit': msg_unit,
                                        'interpreted_value': float_val
                                    }
                                    
                                    if self.connected_clients > 0:
                                        socketio.emit('can_message', msg)
                                        print(f"   📤 Sent to dashboard: {msg_name} = {float_val:.2f}")
                                    
                                    messages_found += 1
                                    i += 25  # Skip past this message
                                    continue
                                    
                                except Exception as e:
                                    print(f"   ❌ Float parse error: {e}")
                        else:
                            print(f"   ⚠️  Unexpected length: {msg_length}")
                    else:
                        print(f"   ⚠️  Length position out of bounds")
                
                except Exception as e:
                    print(f"   ❌ Message parse error: {e}")
                
                i += 1
            else:
                i += 1
        
        if messages_found > 0:
            print(f"🎉 Successfully parsed {messages_found} CAN messages!")
    
    def get_stats(self):
        """Get current statistics"""
        uptime = 0
        if self.stats['start_time']:
            uptime = time.time() - self.stats['start_time']
        
        return {
            'total_can_messages': self.stats['total_can_messages'],
            'total_debug_messages': self.stats['total_debug_messages'],
            'message_rate': self.stats['message_rate'],
            'uptime': int(uptime),
            'connected_port': self.current_port,
            'is_connected': self.serial_conn and self.serial_conn.is_open if self.serial_conn else False
        }

# Global dashboard instance
dashboard = FreshCANDashboard()

# Simple HTML Template
HTML_TEMPLATE = """
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Fresh CAN Dashboard</title>
    <script src="https://cdnjs.cloudflare.com/ajax/libs/socket.io/4.0.1/socket.io.js"></script>
    <style>
        * { margin: 0; padding: 0; box-sizing: border-box; }
        
        body {
            font-family: 'Segoe UI', sans-serif;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            color: white;
            min-height: 100vh;
            padding: 20px;
        }
        
        .container { max-width: 1200px; margin: 0 auto; }
        
        .header {
            text-align: center;
            margin-bottom: 30px;
            background: rgba(255,255,255,0.1);
            padding: 20px;
            border-radius: 15px;
            backdrop-filter: blur(10px);
        }
        
        .controls {
            display: grid;
            grid-template-columns: 1fr;
            gap: 20px;
            margin-bottom: 30px;
        }
        
        .control-panel {
            background: rgba(255,255,255,0.1);
            padding: 20px;
            border-radius: 15px;
            backdrop-filter: blur(10px);
        }
        
        select, input, button {
            width: 100%;
            padding: 10px;
            margin: 5px 0;
            border: none;
            border-radius: 8px;
            font-size: 14px;
        }
        
        button {
            background: #4CAF50;
            color: white;
            cursor: pointer;
            transition: background 0.3s;
        }
        
        button:hover { background: #45a049; }
        button:disabled { background: #666; cursor: not-allowed; }
        
        .disconnect-btn { background: #f44336; }
        .disconnect-btn:hover { background: #da190b; }
        
        .status {
            display: flex;
            align-items: center;
            gap: 10px;
            margin: 10px 0;
        }
        
        .status-dot {
            width: 12px;
            height: 12px;
            border-radius: 50%;
            background: #ff4444;
        }
        
        .status-dot.connected { background: #44ff44; }
        
        .dashboard {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(300px, 1fr));
            gap: 20px;
            margin-bottom: 30px;
        }
        
        .gauge {
            background: rgba(255,255,255,0.1);
            padding: 20px;
            border-radius: 15px;
            text-align: center;
            backdrop-filter: blur(10px);
        }
        
        .gauge-title {
            font-size: 1.2em;
            margin-bottom: 10px;
            opacity: 0.9;
        }
        
        .gauge-value {
            font-size: 3em;
            font-weight: bold;
            margin: 10px 0;
        }
        
        .gauge-unit {
            font-size: 0.4em;
            opacity: 0.7;
        }
        
        .stats-grid {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(150px, 1fr));
            gap: 15px;
            margin-bottom: 30px;
        }
        
        .stat-card {
            background: rgba(255,255,255,0.1);
            padding: 15px;
            border-radius: 10px;
            text-align: center;
        }
        
        .stat-value {
            font-size: 1.8em;
            font-weight: bold;
        }
        
        .stat-label {
            font-size: 0.9em;
            opacity: 0.8;
            margin-top: 5px;
        }
        
        .message-log {
            background: rgba(0,0,0,0.3);
            border-radius: 15px;
            padding: 20px;
            max-height: 400px;
            overflow-y: auto;
        }
        
        .message-entry {
            margin: 5px 0;
            padding: 8px;
            background: rgba(255,255,255,0.1);
            border-radius: 5px;
            font-family: monospace;
            font-size: 0.9em;
            border-left: 3px solid #4CAF50;
        }
    </style>
</head>
<body>
    <div class="container">
        <div class="header">
            <h1>🚗 Fresh CAN Dashboard v3.0</h1>
            <div class="status">
                <div class="status-dot" id="connectionStatus"></div>
                <span id="connectionText">Disconnected</span>
            </div>
        </div>
        
        <div class="controls">
            <div class="control-panel">
                <h3>📡 Serial Connection</h3>
                <select id="portSelect">
                    <option value="">Select Serial Port...</option>
                </select>
                <input type="number" id="baudrateInput" value="115200" placeholder="Baud Rate">
                <button id="connectBtn" onclick="connectSerial()">Connect</button>
                <button id="disconnectBtn" onclick="disconnectSerial()" class="disconnect-btn" disabled>Disconnect</button>
            </div>
        </div>
        
        <div class="dashboard">
            <div class="gauge">
                <div class="gauge-title">Transmission Gear</div>
                <div class="gauge-value" id="gearValue">--<span class="gauge-unit">GEAR</span></div>
            </div>
            
            <div class="gauge">
                <div class="gauge-title">Vehicle Speed</div>
                <div class="gauge-value" id="speedValue">--<span class="gauge-unit">MPH</span></div>
            </div>
            
            <div class="gauge">
                <div class="gauge-title">Fluid Temperature</div>
                <div class="gauge-value" id="tempValue">--<span class="gauge-unit">°F</span></div>
            </div>
        </div>
        
        <div class="stats-grid">
            <div class="stat-card">
                <div class="stat-value" id="totalCANMessages">0</div>
                <div class="stat-label">CAN Messages</div>
            </div>
            <div class="stat-card">
                <div class="stat-value" id="totalDebugMessages">0</div>
                <div class="stat-label">Debug Messages</div>
            </div>
            <div class="stat-card">
                <div class="stat-value" id="uptime">0</div>
                <div class="stat-label">Uptime (sec)</div>
            </div>
            <div class="stat-card">
                <div class="stat-value" id="portStatus">None</div>
                <div class="stat-label">Connected Port</div>
            </div>
        </div>
        
        <div class="message-log">
            <h3>📋 Recent CAN Messages</h3>
            <div id="messageEntries"></div>
        </div>
    </div>

    <script>
        const socket = io();
        let isConnected = false;
        
        window.onload = function() {
            loadAvailablePorts();
        };
        
        function loadAvailablePorts() {
            fetch('/api/ports')
                .then(response => response.json())
                .then(data => {
                    const select = document.getElementById('portSelect');
                    select.innerHTML = '<option value="">Select Serial Port...</option>';
                    
                    data.ports.forEach(port => {
                        const option = document.createElement('option');
                        option.value = port.device;
                        option.textContent = `${port.device} - ${port.description}`;
                        select.appendChild(option);
                    });
                })
                .catch(error => console.error('Error loading ports:', error));
        }
        
        function connectSerial() {
            const port = document.getElementById('portSelect').value;
            const baudrate = parseInt(document.getElementById('baudrateInput').value);
            
            if (!port) {
                alert('Please select a serial port');
                return;
            }
            
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
                    alert('Connection failed: ' + data.error);
                }
            })
            .catch(error => {
                console.error('Connection error:', error);
                alert('Connection failed');
            });
        }
        
        function disconnectSerial() {
            fetch('/api/disconnect', {method: 'POST'})
            .then(response => response.json())
            .then(data => {
                updateConnectionStatus(false, '');
            });
        }
        
        function updateConnectionStatus(connected, port) {
            isConnected = connected;
            const statusDot = document.getElementById('connectionStatus');
            const statusText = document.getElementById('connectionText');
            const connectBtn = document.getElementById('connectBtn');
            const disconnectBtn = document.getElementById('disconnectBtn');
            
            if (connected) {
                statusDot.classList.add('connected');
                statusText.textContent = `Connected to ${port}`;
                connectBtn.disabled = true;
                disconnectBtn.disabled = false;
            } else {
                statusDot.classList.remove('connected');
                statusText.textContent = 'Disconnected';
                connectBtn.disabled = false;
                disconnectBtn.disabled = true;
            }
        }
        
        socket.on('can_message', function(msg) {
            updateGauges(msg);
            addMessageToLog(msg);
        });
        
        socket.on('stats_update', function(stats) {
            document.getElementById('totalCANMessages').textContent = stats.total_can_messages;
            document.getElementById('totalDebugMessages').textContent = stats.total_debug_messages;
            document.getElementById('uptime').textContent = stats.uptime;
            document.getElementById('portStatus').textContent = stats.connected_port || 'None';
        });
        
        function updateGauges(msg) {
            const canId = msg.can_id_int;
            const value = msg.interpreted_value;
            
            if (value === null) return;
            
            switch(canId) {
                case 0x10500101:
                    document.getElementById('gearValue').innerHTML = 
                        `${value.toFixed(1)}<span class="gauge-unit">GEAR</span>`;
                    break;
                case 0x10300002:
                    document.getElementById('speedValue').innerHTML = 
                        `${value.toFixed(0)}<span class="gauge-unit">MPH</span>`;
                    break;
                case 0x10500001:
                    document.getElementById('tempValue').innerHTML = 
                        `${value.toFixed(0)}<span class="gauge-unit">°F</span>`;
                    break;
            }
        }
        
        function addMessageToLog(msg) {
            const logEntries = document.getElementById('messageEntries');
            const entry = document.createElement('div');
            entry.className = 'message-entry';
            
            let valueStr = '';
            if (msg.interpreted_value !== null) {
                valueStr = ` → ${msg.interpreted_value.toFixed(2)} ${msg.message_unit}`;
            }
            
            entry.innerHTML = `
                <strong>[${msg.timestamp}]</strong> ${msg.message_name}<br>
                ID: ${msg.can_id} | Len: ${msg.length} | Data: ${msg.hex_data}${valueStr}
            `;
            
            logEntries.insertBefore(entry, logEntries.firstChild);
            
            while (logEntries.children.length > 20) {
                logEntries.removeChild(logEntries.lastChild);
            }
        }
    </script>
</body>
</html>
"""

@app.route('/')
def index():
    return render_template_string(HTML_TEMPLATE)

@app.route('/api/ports')
def get_ports():
    """Get available serial ports"""
    ports = dashboard.get_available_ports()
    return jsonify({'ports': ports})

@app.route('/api/connect', methods=['POST'])
def connect():
    """Connect to serial port"""
    data = request.get_json()
    port = data.get('port')
    baudrate = data.get('baudrate', 115200)
    
    if dashboard.connect_serial(port, baudrate):
        if dashboard.start_monitoring():
            return jsonify({'success': True})
        else:
            return jsonify({'success': False, 'error': 'Failed to start monitoring'})
    else:
        return jsonify({'success': False, 'error': 'Failed to connect to serial port'})

@app.route('/api/disconnect', methods=['POST'])
def disconnect():
    """Disconnect from serial port"""
    dashboard.disconnect_serial()
    return jsonify({'success': True})

@socketio.on('connect')
def handle_connect():
    dashboard.connected_clients += 1
    print(f"🔌 Client connected. Total: {dashboard.connected_clients}")
    emit('stats_update', dashboard.get_stats())

@socketio.on('disconnect')
def handle_disconnect():
    dashboard.connected_clients -= 1
    print(f"🔌 Client disconnected. Total: {dashboard.connected_clients}")

if __name__ == '__main__':
    print("🚀 Starting Fresh CAN Dashboard Server...")
    print("📊 Dashboard: http://localhost:5000")
    print("🔧 This version uses the WORKING approach from minimal test")
    
    try:
        socketio.run(app, host='0.0.0.0', port=5000, debug=False)
    except KeyboardInterrupt:
        print("\n🛑 Shutting down...")
    finally:
        dashboard.disconnect_serial()