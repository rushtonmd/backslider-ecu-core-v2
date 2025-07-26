#!/usr/bin/env python3
"""
Complete CAN Dashboard - Single File Solution
Handles serial port selection, real-time CAN message display, and message sending
"""

from flask import Flask, render_template_string, request, jsonify
from flask_socketio import SocketIO, emit
import serial
import serial.tools.list_ports
import struct
import time
import datetime
import threading
import json
from collections import defaultdict
import traceback

app = Flask(__name__)
app.config['SECRET_KEY'] = 'can-dashboard-secret'
socketio = SocketIO(app, cors_allowed_origins="*")

class CANDashboard:
    def __init__(self):
        self.serial_conn = None
        self.current_port = None
        self.baudrate = 2000000
        self.running = False
        self.connected_clients = 0
        
        # Statistics
        self.stats = {
            'total_messages': 0,
            'messages_per_id': defaultdict(int),
            'start_time': None,
            'message_rate': 0
        }
        
        # Known CAN message types
        self.message_types = {
            0x10500101: {"name": "Transmission Current Gear", "unit": "gear", "type": "float"},
            0x10300002: {"name": "Vehicle Speed", "unit": "mph", "type": "float"},
            0x10500001: {"name": "Transmission Fluid Temperature", "unit": "°F", "type": "float"}
        }
        
        # Recent messages for dashboard
        self.recent_messages = []
        self.max_recent = 50
    
    def get_available_ports(self):
        """Get list of available serial ports"""
        ports = []
        for port in serial.tools.list_ports.comports():
            ports.append({
                'device': port.device,
                'description': port.description,
                'manufacturer': port.manufacturer or 'Unknown'
            })
        return sorted(ports, key=lambda x: x['device'])
    
    def connect_serial(self, port, baudrate=2000000):
        """Connect to specified serial port"""
        try:
            if self.serial_conn and self.serial_conn.is_open:
                self.disconnect_serial()
            
            self.serial_conn = serial.Serial(
                port=port,
                baudrate=baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.1
            )
            
            self.current_port = port
            self.baudrate = baudrate
            self.stats['start_time'] = time.time()
            self.stats['total_messages'] = 0
            self.stats['messages_per_id'].clear()
            
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
    
    def parse_can_message(self, data):
        """Parse binary CAN message"""
        if len(data) < 9:  # Minimum message size
            return None
        
        try:
            # Parse CAN ID (4 bytes, little-endian)
            can_id = struct.unpack('<I', data[0:4])[0]
            
            # Parse message length (1 byte)
            msg_length = data[4]
            if msg_length > 8:
                return None
            
            # Parse data payload
            payload = data[5:5+msg_length]
            
            # Parse timestamp (4 bytes after data)
            if len(data) < 5 + msg_length + 4:
                return None
            
            timestamp_raw = struct.unpack('<I', data[5+msg_length:9+msg_length])[0]
            
            return {
                'can_id': can_id,
                'length': msg_length,
                'data': list(payload),
                'timestamp_raw': timestamp_raw,
                'timestamp': datetime.datetime.now(),
                'raw_size': 9 + msg_length
            }
            
        except (struct.error, IndexError) as e:
            return None
    
    def interpret_message(self, msg):
        """Interpret CAN message data"""
        can_id = msg['can_id']
        data = bytes(msg['data'])
        
        interpretation = {
            'hex_data': ' '.join(f'{b:02X}' for b in data),
            'interpreted_value': None,
            'message_info': self.message_types.get(can_id, {
                'name': f'Unknown Message (0x{can_id:08X})',
                'unit': '',
                'type': 'raw'
            })
        }
        
        # Try to interpret as float for known message types
        if can_id in self.message_types and len(data) >= 4:
            try:
                # Try little-endian first
                float_val = struct.unpack('<f', data[:4])[0]
                
                # Validate ranges and switch endianness if needed
                if can_id == 0x10500101:  # Gear
                    if not (0 <= float_val <= 10):
                        float_val = struct.unpack('>f', data[:4])[0]
                elif can_id == 0x10300002:  # Speed
                    if not (0 <= float_val <= 200):
                        float_val = struct.unpack('>f', data[:4])[0]
                elif can_id == 0x10500001:  # Temperature
                    if not (50 <= float_val <= 400):
                        float_val = struct.unpack('>f', data[:4])[0]
                
                interpretation['interpreted_value'] = float_val
                
            except struct.error:
                pass
        
        return interpretation
    
    def send_can_message(self, can_id, data_bytes):
        """Send CAN message in binary format"""
        if not self.serial_conn or not self.serial_conn.is_open:
            return False, "Serial port not connected"
        
        try:
            # Create binary message
            message = bytearray()
            
            # CAN ID (4 bytes, little-endian)
            message.extend(struct.pack('<I', can_id))
            
            # Data length (1 byte)
            message.append(len(data_bytes))
            
            # Data payload
            message.extend(data_bytes)
            
            # Timestamp (4 bytes, current time in milliseconds)
            timestamp = int(time.time() * 1000) & 0xFFFFFFFF
            message.extend(struct.pack('<I', timestamp))
            
            # Send message
            self.serial_conn.write(message)
            print(f"📤 Sent CAN message: ID=0x{can_id:08X}, Data={data_bytes.hex().upper()}")
            
            return True, "Message sent successfully"
            
        except Exception as e:
            print(f"❌ Failed to send message: {e}")
            return False, str(e)
    
    def start_monitoring(self):
        """Start monitoring serial port for CAN messages"""
        if not self.serial_conn or not self.serial_conn.is_open:
            return False
        
        self.running = True
        
        def monitor_thread():
            buffer = bytearray()
            last_rate_calc = time.time()
            last_msg_count = 0
            
            while self.running and self.serial_conn and self.serial_conn.is_open:
                try:
                    # Read available data
                    if self.serial_conn.in_waiting > 0:
                        new_data = self.serial_conn.read(self.serial_conn.in_waiting)
                        buffer.extend(new_data)
                    
                    # Try to parse messages from buffer
                    while len(buffer) >= 9:
                        msg = self.parse_can_message(buffer)
                        
                        if msg:
                            # Valid message found
                            self.process_message(msg)
                            buffer = buffer[msg['raw_size']:]
                        else:
                            # Remove first byte and try again
                            buffer = buffer[1:]
                    
                    # Calculate message rate every second
                    now = time.time()
                    if now - last_rate_calc >= 1.0:
                        current_count = self.stats['total_messages']
                        self.stats['message_rate'] = current_count - last_msg_count
                        last_msg_count = current_count
                        last_rate_calc = now
                        
                        # Send stats update to clients
                        if self.connected_clients > 0:
                            socketio.emit('stats_update', self.get_stats())
                    
                    time.sleep(0.001)  # Small delay
                    
                except Exception as e:
                    print(f"❌ Monitor error: {e}")
                    traceback.print_exc()
                    break
            
            self.running = False
            print("🛑 Monitoring stopped")
        
        thread = threading.Thread(target=monitor_thread, daemon=True)
        thread.start()
        return True
    
    def process_message(self, msg):
        """Process parsed CAN message"""
        # Update statistics
        self.stats['total_messages'] += 1
        self.stats['messages_per_id'][msg['can_id']] += 1
        
        # Interpret message
        interpretation = self.interpret_message(msg)
        
        # Create dashboard message
        dashboard_msg = {
            'can_id': f"0x{msg['can_id']:08X}",
            'can_id_int': msg['can_id'],
            'length': msg['length'],
            'data': msg['data'],
            'hex_data': interpretation['hex_data'],
            'timestamp': msg['timestamp'].strftime('%H:%M:%S.%f')[:-3],
            'message_name': interpretation['message_info']['name'],
            'message_unit': interpretation['message_info']['unit'],
            'interpreted_value': interpretation['interpreted_value']
        }
        
        # Add to recent messages
        self.recent_messages.insert(0, dashboard_msg)
        if len(self.recent_messages) > self.max_recent:
            self.recent_messages.pop()
        
        # Send to connected clients
        if self.connected_clients > 0:
            socketio.emit('can_message', dashboard_msg)
    
    def get_stats(self):
        """Get current statistics"""
        uptime = 0
        if self.stats['start_time']:
            uptime = time.time() - self.stats['start_time']
        
        return {
            'total_messages': self.stats['total_messages'],
            'message_rate': self.stats['message_rate'],
            'uptime': int(uptime),
            'connected_port': self.current_port,
            'is_connected': self.serial_conn and self.serial_conn.is_open if self.serial_conn else False
        }

# Global dashboard instance
dashboard = CANDashboard()

# HTML Template
HTML_TEMPLATE = """
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>CAN Bus Dashboard</title>
    <script src="https://cdnjs.cloudflare.com/ajax/libs/socket.io/4.0.1/socket.io.js"></script>
    <style>
        * { margin: 0; padding: 0; box-sizing: border-box; }
        
        body {
            font-family: 'Segoe UI', Tango, Geneva, Verdana, sans-serif;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            color: white;
            min-height: 100vh;
            padding: 20px;
        }
        
        .container { max-width: 1400px; margin: 0 auto; }
        
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
            grid-template-columns: 1fr 1fr;
            gap: 20px;
            margin-bottom: 30px;
        }
        
        .control-panel {
            background: rgba(255,255,255,0.1);
            padding: 20px;
            border-radius: 15px;
            backdrop-filter: blur(10px);
        }
        
        .control-panel h3 {
            margin-bottom: 15px;
            color: #fff;
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
        
        .send-message {
            background: rgba(255,255,255,0.1);
            padding: 20px;
            border-radius: 15px;
            margin-bottom: 20px;
        }
        
        .send-form {
            display: grid;
            grid-template-columns: 2fr 1fr 3fr 1fr;
            gap: 10px;
            align-items: end;
        }
        
        @media (max-width: 768px) {
            .controls { grid-template-columns: 1fr; }
            .send-form { grid-template-columns: 1fr; }
        }
    </style>
</head>
<body>
    <div class="container">
        <div class="header">
            <h1>🚗 CAN Bus Dashboard</h1>
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
                <input type="number" id="baudrateInput" value="2000000" placeholder="Baud Rate">
                <button id="connectBtn" onclick="connectSerial()">Connect</button>
                <button id="disconnectBtn" onclick="disconnectSerial()" class="disconnect-btn" disabled>Disconnect</button>
            </div>
            
            <div class="control-panel">
                <h3>📤 Send CAN Message</h3>
                <div class="send-form">
                    <input type="text" id="canIdInput" placeholder="CAN ID (hex)" value="0x10500101">
                    <input type="number" id="dataLengthInput" placeholder="Length" value="4" min="0" max="8">
                    <input type="text" id="dataInput" placeholder="Data (hex bytes)" value="41 20 00 00">
                    <button onclick="sendMessage()">Send</button>
                </div>
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
                <div class="stat-value" id="totalMessages">0</div>
                <div class="stat-label">Total Messages</div>
            </div>
            <div class="stat-card">
                <div class="stat-value" id="messageRate">0</div>
                <div class="stat-label">Messages/Sec</div>
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
        
        // Load available ports on page load
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
        
        function sendMessage() {
            if (!isConnected) {
                alert('Please connect to a serial port first');
                return;
            }
            
            const canIdStr = document.getElementById('canIdInput').value;
            const dataStr = document.getElementById('dataInput').value;
            
            // Parse CAN ID
            let canId;
            try {
                canId = parseInt(canIdStr, 16);
            } catch (e) {
                alert('Invalid CAN ID format');
                return;
            }
            
            // Parse data bytes
            const dataBytes = dataStr.split(' ').map(b => parseInt(b, 16));
            
            fetch('/api/send', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({
                    can_id: canId,
                    data: dataBytes
                })
            })
            .then(response => response.json())
            .then(data => {
                if (!data.success) {
                    alert('Send failed: ' + data.error);
                }
            })
            .catch(error => {
                console.error('Send error:', error);
                alert('Send failed');
            });
        }
        
        // Socket event handlers
        socket.on('can_message', function(msg) {
            updateGauges(msg);
            addMessageToLog(msg);
        });
        
        socket.on('stats_update', function(stats) {
            document.getElementById('totalMessages').textContent = stats.total_messages;
            document.getElementById('messageRate').textContent = stats.message_rate;
            document.getElementById('uptime').textContent = stats.uptime;
            document.getElementById('portStatus').textContent = stats.connected_port || 'None';
        });
        
        function updateGauges(msg) {
            const canId = msg.can_id_int;
            const value = msg.interpreted_value;
            
            if (value === null) return;
            
            switch(canId) {
                case 0x10500101: // Transmission Gear
                    document.getElementById('gearValue').innerHTML = 
                        `${value.toFixed(1)}<span class="gauge-unit">GEAR</span>`;
                    break;
                case 0x10300002: // Vehicle Speed
                    document.getElementById('speedValue').innerHTML = 
                        `${value.toFixed(0)}<span class="gauge-unit">MPH</span>`;
                    break;
                case 0x10500001: // Transmission Fluid Temperature
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
            
            // Keep only last 20 entries
            while (logEntries.children.length > 20) {
                logEntries.removeChild(logEntries.lastChild);
            }
        }
        
        // Refresh ports button
        document.getElementById('portSelect').addEventListener('focus', loadAvailablePorts);
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
    baudrate = data.get('baudrate', 2000000)
    
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

@app.route('/api/send', methods=['POST'])
def send_message():
    """Send CAN message"""
    data = request.get_json()
    can_id = data.get('can_id')
    data_bytes = bytes(data.get('data', []))
    
    success, message = dashboard.send_can_message(can_id, data_bytes)
    return jsonify({'success': success, 'message': message})

@socketio.on('connect')
def handle_connect():
    dashboard.connected_clients += 1
    print(f"🔌 Client connected. Total: {dashboard.connected_clients}")
    
    # Send current stats
    emit('stats_update', dashboard.get_stats())

@socketio.on('disconnect')
def handle_disconnect():
    dashboard.connected_clients -= 1
    print(f"🔌 Client disconnected. Total: {dashboard.connected_clients}")

if __name__ == '__main__':
    import argparse
    
    parser = argparse.ArgumentParser(description='CAN Bus Dashboard')
    parser.add_argument('--host', default='0.0.0.0', help='Host address')
    parser.add_argument('--port', type=int, default=5000, help='Web server port')
    parser.add_argument('--debug', action='store_true', help='Debug mode')
    
    args = parser.parse_args()
    
    print(f"🚀 Starting CAN Dashboard Server...")
    print(f"📊 Dashboard: http://{args.host}:{args.port}")
    print(f"🔧 Available endpoints:")
    print(f"   GET  / - Dashboard interface")
    print(f"   GET  /api/ports - List serial ports")
    print(f"   POST /api/connect - Connect to serial port")
    print(f"   POST /api/disconnect - Disconnect from serial port")
    print(f"   POST /api/send - Send CAN message")
    
    try:
        socketio.run(app, host=args.host, port=args.port, debug=args.debug)
    except KeyboardInterrupt:
        print("\n🛑 Shutting down...")
    finally:
        dashboard.disconnect_serial()