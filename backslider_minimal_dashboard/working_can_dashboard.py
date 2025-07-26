#!/usr/bin/env python3
"""
Working CAN Dashboard - handles mixed text/binary format
Based on the actual data format: debug text + binary CAN messages
"""

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

class WorkingCANDashboard:
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
        
        print("✅ Working CAN Dashboard initialized")
    
    def get_available_ports(self):
        """Get available cu.* ports (macOS style)"""
        ports = []
        for port in serial.tools.list_ports.comports():
            if port.device.startswith('/dev/cu.'):
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
                timeout=1.0,  # Increased timeout
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE
            )
            
            # Give it a moment to stabilize
            time.sleep(0.5)
            
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
    
    def parse_can_from_mixed_data(self, data_chunk):
        """Parse CAN messages from mixed text/binary data"""
        messages = []
        
        # First, let's see if we can find the pattern at all
        pattern_count = data_chunk.count(b'\x01\x01P\x10')
        if pattern_count > 0:
            print(f"🔍 Found {pattern_count} CAN ID patterns in chunk of {len(data_chunk)} bytes")
        
        # Look for the CAN ID pattern (0x10500101 = \x01\x01P\x10 in little-endian)
        i = 0
        while i < len(data_chunk) - 25:  # Need at least 25 bytes for full message
            
            if (i + 24 < len(data_chunk) and 
                data_chunk[i:i+4] == b'\x01\x01P\x10'):  # CAN ID 0x10500101
                
                print(f"🎯 Found CAN ID at position {i}")
                
                try:
                    can_id = struct.unpack('<I', data_chunk[i:i+4])[0]
                    print(f"   CAN ID: 0x{can_id:08X}")
                    
                    # Show the next 25 bytes for debugging
                    debug_bytes = data_chunk[i:i+25]
                    print(f"   Next 25 bytes: {debug_bytes.hex().upper()}")
                    
                    # Based on your pattern analysis:
                    # CAN ID (4) + variable byte + 60 00 01 00 00 00 + length (1) + 00 00 + data (4) + padding (8)
                    # Position: i+4 = start of pattern after CAN ID
                    
                    # Look for the length byte at position i+11 (after the 7-byte pattern)
                    length_pos = i + 11
                    if length_pos < len(data_chunk):
                        msg_length = data_chunk[length_pos]
                        print(f"   Length byte at pos {length_pos}: {msg_length}")
                        
                        if msg_length == 4:  # We expect 4-byte float data
                            # Data starts at length_pos + 3 (skip length + 2 padding bytes)
                            data_start = length_pos + 3
                            data_end = data_start + msg_length
                            
                            print(f"   Data should be at positions {data_start}-{data_end}")
                            
                            if data_end <= len(data_chunk):
                                data_bytes = data_chunk[data_start:data_end]
                                print(f"   Raw data bytes: {data_bytes.hex().upper()}")
                                
                                # Try to interpret as float
                                interpreted_value = None
                                if len(data_bytes) == 4:
                                    try:
                                        # Try little-endian first
                                        float_val = struct.unpack('<f', data_bytes)[0]
                                        print(f"   Float (little-endian): {float_val}")
                                        
                                        # Try big-endian too
                                        float_val_be = struct.unpack('>f', data_bytes)[0]
                                        print(f"   Float (big-endian): {float_val_be}")
                                        
                                        # Use little-endian for now
                                        interpreted_value = float_val
                                    except Exception as e:
                                        print(f"   Float conversion error: {e}")
                                
                                messages.append({
                                    'can_id': can_id,
                                    'length': msg_length,
                                    'data': list(data_bytes),
                                    'interpreted_value': interpreted_value,
                                    'timestamp': datetime.datetime.now(),
                                    'raw_hex': ' '.join(f'{b:02X}' for b in data_bytes)
                                })
                                
                                print(f"✅ Successfully parsed CAN message!")
                                
                                # Move past this entire message (about 25 bytes total)
                                i = data_end + 8  # Skip the padding too
                                continue
                            else:
                                print(f"   Not enough data remaining")
                        else:
                            print(f"   Length {msg_length} != 4, skipping")
                    else:
                        print(f"   Length position {length_pos} out of bounds")
                    
                    i += 1
                        
                except Exception as e:
                    print(f"❌ Parse error: {e}")
                    i += 1
            else:
                i += 1
        
        if messages:
            print(f"🎉 Returning {len(messages)} parsed messages")
        
        return messages
    
    def start_monitoring(self):
        """Start monitoring serial port"""
        print(f"🔧 start_monitoring called")
        print(f"   self.serial_conn: {self.serial_conn}")
        print(f"   self.serial_conn.is_open: {self.serial_conn.is_open if self.serial_conn else 'N/A'}")
        
        if not self.serial_conn or not self.serial_conn.is_open:
            print(f"❌ Cannot start monitoring - serial not ready")
            return False
        
        self.running = True
        print(f"✅ Setting self.running = True")
        
        def monitor_thread():
            print("🎯 Starting mixed format monitoring...")
            buffer = bytearray()
            last_rate_calc = time.time()
            last_msg_count = 0
            
            while self.running and self.serial_conn and self.serial_conn.is_open:
                try:
                    if self.serial_conn.in_waiting > 0:
                        new_data = self.serial_conn.read(self.serial_conn.in_waiting)
                        buffer.extend(new_data)
                        
                        print(f"📡 Read {len(new_data)} bytes, buffer now {len(buffer)} bytes")
                        
                        # Show a sample of what we're receiving
                        if len(new_data) > 0:
                            sample = new_data[:50]  # First 50 bytes
                            print(f"   Sample: {sample.hex().upper()}")
                            try:
                                ascii_sample = ''.join(chr(b) if 32 <= b <= 126 else '.' for b in sample)
                                print(f"   ASCII:  {ascii_sample}")
                            except:
                                pass
                        
                        # Process debug messages (text)
                        try:
                            buffer_str = buffer.decode('latin-1', errors='ignore')
                            debug_matches = re.findall(r'DEBUG: Sending to external_serial: (0x[0-9A-Fa-f]+)', buffer_str)
                            if debug_matches:
                                self.stats['total_debug_messages'] += len(debug_matches)
                                for match in debug_matches:
                                    print(f"📝 Debug: {match}")
                        except:
                            pass
                        
                        # Look for CAN messages in binary data
                        print(f"🔍 Looking for CAN patterns in buffer of {len(buffer)} bytes...")
                        can_messages = self.parse_can_from_mixed_data(buffer)
                        
                        for msg in can_messages:
                            self.process_can_message(msg)
                        
                        # Keep buffer manageable
                        if len(buffer) > 2000:
                            print(f"🗑️  Trimming buffer from {len(buffer)} to 1000 bytes")
                            buffer = buffer[-1000:]
                    
                    # Calculate message rate
                    now = time.time()
                    if now - last_rate_calc >= 1.0:
                        current_count = self.stats['total_can_messages']
                        self.stats['message_rate'] = current_count - last_msg_count
                        last_msg_count = current_count
                        last_rate_calc = now
                        
                        if self.connected_clients > 0:
                            socketio.emit('stats_update', self.get_stats())
                    
                    time.sleep(0.01)
                    
                except Exception as e:
                    print(f"❌ Monitor error: {e}")
                    import traceback
                    traceback.print_exc()
                    break
            
            self.running = False
            print("🛑 Monitoring stopped")
        
        thread = threading.Thread(target=monitor_thread, daemon=True)
        thread.start()
        return True
    
    def process_can_message(self, msg):
        """Process a parsed CAN message"""
        self.stats['total_can_messages'] += 1
        
        can_id = msg['can_id']
        msg_info = self.message_types.get(can_id, {'name': f'Unknown (0x{can_id:08X})', 'unit': ''})
        
        # Create dashboard message
        dashboard_msg = {
            'can_id': f"0x{can_id:08X}",
            'can_id_int': can_id,
            'length': msg['length'],
            'data': msg['data'],
            'hex_data': msg['raw_hex'],
            'timestamp': msg['timestamp'].strftime('%H:%M:%S.%f')[:-3],
            'message_name': msg_info['name'],
            'message_unit': msg_info['unit'],
            'interpreted_value': msg['interpreted_value']
        }
        
        if msg['interpreted_value'] is not None:
            print(f"📡 CAN: {dashboard_msg['message_name']} = {msg['interpreted_value']:.2f} {msg_info['unit']}")
        else:
            print(f"📡 CAN: {dashboard_msg['message_name']} - Raw: {msg['raw_hex']}")
        
        # Send to connected clients
        if self.connected_clients > 0:
            socketio.emit('can_message', dashboard_msg)
    
    def send_can_message(self, can_id, data_bytes):
        """Send CAN message (placeholder for now)"""
        if not self.serial_conn or not self.serial_conn.is_open:
            return False, "Serial port not connected"
        
        # For now, just log what we would send
        print(f"📤 Would send CAN: ID=0x{can_id:08X}, Data={' '.join(f'{b:02X}' for b in data_bytes)}")
        return True, "Message logged (sending not implemented yet)"
    
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
dashboard = WorkingCANDashboard()

# HTML Template
HTML_TEMPLATE = """
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Working CAN Dashboard</title>
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
            <h1>🚗 Working CAN Dashboard</h1>
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
                <div class="stat-value" id="totalCANMessages">0</div>
                <div class="stat-label">CAN Messages</div>
            </div>
            <div class="stat-card">
                <div class="stat-value" id="totalDebugMessages">0</div>
                <div class="stat-label">Debug Messages</div>
            </div>
            <div class="stat-card">
                <div class="stat-value" id="messageRate">0</div>
                <div class="stat-label">Messages/Sec</div>
            </div>
            <div class="stat-card">
                <div class="stat-value" id="uptime">0</div>
                <div class="stat-label">Uptime (sec)</div>
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
        
        function sendMessage() {
            if (!isConnected) {
                alert('Please connect to a serial port first');
                return;
            }
            
            const canIdStr = document.getElementById('canIdInput').value;
            const dataStr = document.getElementById('dataInput').value;
            
            let canId;
            try {
                canId = parseInt(canIdStr, 16);
            } catch (e) {
                alert('Invalid CAN ID format');
                return;
            }
            
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
        
        socket.on('can_message', function(msg) {
            updateGauges(msg);
            addMessageToLog(msg);
        });
        
        socket.on('stats_update', function(stats) {
            document.getElementById('totalCANMessages').textContent = stats.total_can_messages;
            document.getElementById('totalDebugMessages').textContent = stats.total_debug_messages;
            document.getElementById('messageRate').textContent = stats.message_rate;
            document.getElementById('uptime').textContent = stats.uptime;
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
    emit('stats_update', dashboard.get_stats())

@socketio.on('disconnect')
def handle_disconnect():
    dashboard.connected_clients -= 1
    print(f"🔌 Client disconnected. Total: {dashboard.connected_clients}")

if __name__ == '__main__':
    print("🚀 Starting Working CAN Dashboard Server...")
    print("📊 Dashboard: http://localhost:5000")
    print("🔧 This version handles mixed text/binary format")
    print("📡 Baud rate default: 115200")
    print("🖥️  Looking for cu.* ports on macOS")
    
    try:
        socketio.run(app, host='0.0.0.0', port=5000, debug=False)
    except KeyboardInterrupt:
        print("\n🛑 Shutting down...")
    finally:
        dashboard.disconnect_serial()