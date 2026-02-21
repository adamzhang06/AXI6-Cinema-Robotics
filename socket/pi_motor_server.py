import socket
import json
import time
import threading
from tmc_driver import Tmc2209, Loglevel, MovementAbsRel, TmcEnableControlPin, TmcMotionControlStepDir

# --- NETWORK SETUP ---
UDP_IP = "0.0.0.0" 
UDP_PORT = 5005

class MotorState:
    def __init__(self):
        self.axes = {
            "slide": {"speed": 0, "dir": 0, "accel": 400},
            "pan":   {"speed": 0, "dir": 0, "accel": 400},
            "tilt":  {"speed": 0, "dir": 0, "accel": 400}
        }
        self.last_update = time.time()
        self.running = True

state = MotorState()

# --- 1. NETWORK LISTENER THREAD ---
def udp_listener():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))
    sock.settimeout(1.0)
    print(f"[NETWORK] Listening for Mac Brain on port {UDP_PORT}...")
    
    while state.running:
        try:
            data, addr = sock.recvfrom(1024)
            command = json.loads(data.decode('utf-8'))
            
            # Using .get() with defaults ensures it won't crash if a packet drops a key
            if "pan" in command: 
                state.axes["pan"] = command.get("pan", {"speed": 0, "dir": 0, "accel": 400})
            if "slide" in command: 
                state.axes["slide"] = command.get("slide", {"speed": 0, "dir": 0, "accel": 400})
            if "tilt" in command: 
                state.axes["tilt"] = command.get("tilt", {"speed": 0, "dir": 0, "accel": 400})
                
            state.last_update = time.time()
        except socket.timeout:
            pass 
        except Exception:
            pass

# --- 2. MOTOR CONTROL THREAD (Velocity Mode Fix) ---
def motor_loop():
    pan_motor = Tmc2209(TmcEnableControlPin(21), TmcMotionControlStepDir(16, 20), loglevel=Loglevel.INFO)
    pan_motor.set_motor_enabled(True)
    
    print("[MOTOR] Continuous Velocity Drive Active.")
    
    current_dir = 0
    print_counter = 0
    
    while state.running:
        start_time = time.time()
        
        # FAILSAFE: Mac disconnected
        if time.time() - state.last_update > 0.5:
            pan_motor.max_speed_fullstep = 0
            if current_dir != 0:
                pan_motor.run_to_position_steps(0, MovementAbsRel.RELATIVE) # Clear the queue
                current_dir = 0
        else:
            p_speed = state.axes["pan"]["speed"]
            p_dir   = state.axes["pan"]["dir"]
            p_accel = state.axes["pan"]["accel"]
            
            # 1. Update Acceleration
            if pan_motor.acceleration_fullstep != p_accel:
                pan_motor.acceleration_fullstep = p_accel
            
            # 2. Update Speed and Direction
            if p_speed > 0:
                pan_motor.max_speed_fullstep = p_speed
                
                # If we are changing direction or starting from a dead stop
                if p_dir != current_dir:
                    pan_motor.run_to_position_steps(p_dir * 1000000, MovementAbsRel.RELATIVE)
                    current_dir = p_dir
            else:
                # --- THE FIX: ACTIVELY BRAKE AND CLEAR THE QUEUE ---
                if current_dir != 0:
                    pan_motor.max_speed_fullstep = 0
                    # Sending a 0 relative move overwrites the 1,000,000 step target
                    pan_motor.run_to_position_steps(0, MovementAbsRel.RELATIVE) 
                    
                    # Optional: Some TMC wrapper versions require explicitly calling stop()
                    try:
                        pan_motor.stop()
                    except AttributeError:
                        pass # Ignore if this specific library version doesn't use .stop()
                        
                    current_dir = 0

        print_counter += 1
        
        # 500Hz precision timing
        sleep_time = 0.002 - (time.time() - start_time)
        if sleep_time > 0:
            time.sleep(sleep_time)
            
    pan_motor.set_motor_enabled(False)
    print("\n[SHUTDOWN] Motors safely disabled.")