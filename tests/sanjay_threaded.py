"""
tests/sanjay_threaded.py
------------------------
Proves that standard Python threads running the exact sanjay.py 
busy-wait GPIO logic can run two motors perfectly in sync without 
relying on the tmc_driver library's built-in loops.
"""

import time
import threading
import json
import sys
import socket
import struct
import RPi.GPIO as GPIO


print("---")
print("SCRIPT START")
print("---")

# -----------------------------------------------------------------------

# -----------------------------------------------------------------------

# -----------------------------------------------------------------------
# 1. Listen for Trajectory JSON over TCP
# -----------------------------------------------------------------------
TCP_PORT = 5010

print(f"Waiting for Mac to send a trajectory on TCP port {TCP_PORT}...")
server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server.bind(('0.0.0.0', TCP_PORT))
server.listen(1)

conn, addr = server.accept()
print(f"Connected from {addr[0]}:{addr[1]}")

try:
    # Read 4-byte big-endian length header
    length_bytes = b''
    while len(length_bytes) < 4:
        chunk = conn.recv(4 - len(length_bytes))
        if not chunk:
            raise ConnectionError("Connection closed while reading length header")
        length_bytes += chunk

    payload_length = struct.unpack('>I', length_bytes)[0]
    
    # Read full JSON payload
    payload = b''
    while len(payload) < payload_length:
        chunk = conn.recv(min(4096, payload_length - len(payload)))
        if not chunk:
            raise ConnectionError("Connection closed while reading payload")
        payload += chunk

    data = json.loads(payload.decode('utf-8'))
    
    a_direction_forward = data["a_fwd"]
    a_step_times = data["a_times"]
    
    b_direction_forward = data["b_fwd"]
    b_step_times = data["b_times"]
    
    print(f"Loaded Motor A: {len(a_step_times)} steps | Motor B: {len(b_step_times)} steps.")
    
    # Send ACK back to Mac
    ack = json.dumps({"status": "ok"}).encode('utf-8')
    conn.sendall(ack)
    
except Exception as e:
    print(f"Failed to receive JSON: {e}")
    conn.close()
    server.close()
    sys.exit(1)
    
conn.close()
server.close()

# -----------------------------------------------------------------------
# 2. Setup GPIO for Direct High-Speed Control

# -----------------------------------------------------------------------
PAN_STEP = 16
PAN_DIR = 20
PAN_EN = 21

TILT_STEP = 13
TILT_DIR = 19
TILT_EN = 26

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup([PAN_STEP, PAN_DIR, PAN_EN, TILT_STEP, TILT_DIR, TILT_EN], GPIO.OUT)

# Enable the motors (LOW = Enabled for TMC2209)
GPIO.output(PAN_EN, GPIO.LOW)
GPIO.output(TILT_EN, GPIO.LOW)

# -----------------------------------------------------------------------
# 3. Thread Execution Engine
# -----------------------------------------------------------------------
def motor_pulse_loop(step_pin, times_array, start_time_real):

    """The exact sanjay.py logic running inside a dedicated thread"""
    for target_t in times_array:
        # Busy-wait loop for microsecond precision against the global clock
        while time.time() - start_time_real < target_t:
            pass 
            
        # Fire pulse
        GPIO.output(step_pin, GPIO.HIGH)
        time.sleep(0.0000001) # 1 microsecond hold
        GPIO.output(step_pin, GPIO.LOW)

print("Starting infinite playback loop. Press Ctrl+C to stop.")

try:
    while True:
        # --- FORWARD PASS ---
        print("\nMoving Forward...")
        GPIO.output(PAN_DIR, GPIO.HIGH if a_direction_forward else GPIO.LOW)
        GPIO.output(TILT_DIR, GPIO.LOW if b_direction_forward else GPIO.HIGH) # Inverted
        
        start_time_real = time.time() + 0.1  # 100ms sync buffer
        
        pan_thread = threading.Thread(target=motor_pulse_loop, args=(PAN_STEP, a_step_times, start_time_real))
        tilt_thread = threading.Thread(target=motor_pulse_loop, args=(TILT_STEP, b_step_times, start_time_real))
        
        pan_thread.start()
        tilt_thread.start()
        pan_thread.join()
        tilt_thread.join()
        
        print("Forward move complete. Waiting 1 second...")
        time.sleep(1.0)
        
        # --- BACKWARD PASS (Return to Start) ---
        print("Moving Backward...")
        # Flip the direction logic
        GPIO.output(PAN_DIR, GPIO.LOW if a_direction_forward else GPIO.HIGH)
        GPIO.output(TILT_DIR, GPIO.HIGH if b_direction_forward else GPIO.LOW) # Inverted
        
        start_time_real = time.time() + 0.1  # 100ms sync buffer
        
        pan_thread = threading.Thread(target=motor_pulse_loop, args=(PAN_STEP, a_step_times, start_time_real))
        tilt_thread = threading.Thread(target=motor_pulse_loop, args=(TILT_STEP, b_step_times, start_time_real))
        
        pan_thread.start()
        tilt_thread.start()
        pan_thread.join()
        tilt_thread.join()

        print("Backward move complete. Waiting 1 second...")
        time.sleep(1.0)

except KeyboardInterrupt:
    print("\n[STOP] Loop interrupted by user.")

# -----------------------------------------------------------------------
# Clean up
# -----------------------------------------------------------------------
# Disable motors (HIGH = Disabled)
GPIO.output(PAN_EN, GPIO.HIGH)
GPIO.output(TILT_EN, GPIO.HIGH)

print("---")
print("SCRIPT FINISHED")
print("---")
