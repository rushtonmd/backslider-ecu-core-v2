# Quick Python test
python3 -c "
import serial
import time
s = serial.Serial('/dev/cu.usbmodem160544701', 115200, timeout=1)  # Replace with your actual port
print('Connected, reading for 5 seconds...')
for i in range(50):
    data = s.read(100)
    if data:
        print(f'Got {len(data)} bytes:', data[:50])
    time.sleep(0.1)
s.close()
"