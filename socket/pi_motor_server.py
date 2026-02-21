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
            "slide": {"speed": 0, "dir": 0},
            "pan":   {"speed": 0, "dir": 0},
            "tilt":  {"speed": 0, "dir": 0}
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
            
            # Map incoming data
            if "pan" in command: state.axes["pan"] = command["pan"]
            if "slide" in command: state.axes["slide"] = command["slide"]
            if "tilt" in command: state.axes["tilt"] = command["tilt"]
            
            state.last_update = time.time()
        except socket.timeout:
            pass 
        except Exception:
            pass

# --- 2. 500Hz MOTOR CONTROL THREAD ---
def motor_loop():
    # Setup PAN motor
    pan_motor = Tmc2209(TmcEnableControlPin(21), TmcMotionControlStepDir(16, 20), loglevel=Loglevel.INFO)
    pan_motor.acceleration_fullstep = 400 
    pan_motor.set_motor_enabled(True)
    
    print("[MOTOR] 500Hz Hardware Drive Active.")
    print_counter = 0
    
    while state.running:
        start_time = time.time()
        
        # FAILSAFE: If the Mac disconnects, stop immediately
        if time.time() - state.last_update > 0.5:
            pan_motor.max_speed_fullstep = 0
            if print_counter % 200 == 0:
                print("⚠️ [FAILSAFE] No network data. Motors frozen.")
        else:
            p_speed = state.axes["pan"]["speed"]
            p_dir = state.axes["pan"]["dir"]
            
            pan_motor.max_speed_fullstep = p_speed
            
            # Only command relative steps if the speed is > 0 and we have a direction
            if p_speed > 0 and p_dir != 0:
                pan_motor.run_to_position_steps(p_dir, MovementAbsRel.RELATIVE)
                
            if print_counter % 200 == 0 and p_speed > 0:
                print(f"⚙️ [RUNNING] Pan Speed: {p_speed} | Dir: {p_dir}")

        print_counter += 1
        
        # 500Hz precision timing
        sleep_time = 0.002 - (time.time() - start_time)
        if sleep_time > 0:
            time.sleep(sleep_time)
            
    pan_motor.set_motor_enabled(False)
    print("\n[SHUTDOWN] Motors safely disabled.")

if __name__ == "__main__":
    net_thread = threading.Thread(target=udp_listener, daemon=True)
    net_thread.start()
    
    try:
        motor_loop()
    except KeyboardInterrupt:
        state.running = False