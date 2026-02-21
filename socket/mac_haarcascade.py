import cv2
import time
import socket
import json

# --- NETWORK SETUP ---
PI_IP = "100.69.176.89"  # Ensure this is your Pi's current IP
PI_PORT = 5005
udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# --- CONFIGURATION & PID ---
WIDTH, HEIGHT = 320, 240
CENTER_X = WIDTH // 2
DEADZONE = 25 

kp, ki, kd = 0.075, 0.01, 3.5
GLOBAL_SPEED_SCALE = 1

# --- STATE VARIABLES ---
prev_error = 0
integral = 0
last_time = time.time() 

history_cx = []
FILTER_SIZE = 4 

# Mac Webcams: 0 is usually built-in, 1 is usually external (Logitech Brio)
cap = cv2.VideoCapture(1) 
face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

print("\n[SYSTEM] Mac Vision & PID Brain Online.")
print("Press 'q' in the video window to quit.\n")

while True:
    success, frame = cap.read()
    if not success or frame is None: 
        time.sleep(0.01)
        continue

    frame = cv2.resize(frame, (WIDTH, HEIGHT))
    frame = cv2.flip(frame, 1)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    current_time = time.time()
    dt = current_time - last_time
    if dt <= 0: dt = 0.001

    faces = face_cascade.detectMultiScale(gray, 1.3, 5)
    
    # Initialize default motor command
    pan_speed = 0
    pan_dir = 0

    if len(faces) > 0:
        (x, y, w, h) = faces[0]
        raw_cx = x + (w // 2)
        
        # 1. APPLY FILTER
        history_cx.append(raw_cx)
        if len(history_cx) > FILTER_SIZE:
            history_cx.pop(0)
        smooth_cx = sum(history_cx) / len(history_cx)
        
        # 2. CALCULATE PID
        motor_error = smooth_cx - CENTER_X
        
        if abs(motor_error) > DEADZONE:
            integral += motor_error * dt
            integral = max(min(integral, 1000), -1000) # Anti-windup
            
            derivative = (motor_error - prev_error) / dt
            raw_velocity = (kp * motor_error) + (ki * integral) + (kd * derivative)
            
            scaled_velocity = abs(raw_velocity) * GLOBAL_SPEED_SCALE
            pan_speed = min(int(scaled_velocity), 200) # Capped at 200 as per your script
            pan_dir = 10 if motor_error > 0 else -10
            
            prev_error = motor_error
        else:
            integral = 0
            
        # Draw targeting visuals
        cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
        cv2.circle(frame, (int(smooth_cx), y + (h // 2)), 6, (0, 0, 255), -1) 
    else:
        history_cx.clear()
        integral = 0

    last_time = current_time

    # 3. SEND UDP PACKET TO PI
    # Keeping slide and tilt ready for your future upgrades
    payload = {
        "pan": {"speed": pan_speed, "dir": pan_dir},
        "slide": {"speed": 0, "dir": 0},
        "tilt": {"speed": 0, "dir": 0}
    }
    udp_sock.sendto(json.dumps(payload).encode('utf-8'), (PI_IP, PI_PORT))

    # 4. LOCAL MAC VISUALS
    cv2.line(frame, (CENTER_X - DEADZONE, 0), (CENTER_X - DEADZONE, HEIGHT), (0, 255, 255), 1)
    cv2.line(frame, (CENTER_X + DEADZONE, 0), (CENTER_X + DEADZONE, HEIGHT), (0, 255, 255), 1)
    
    cv2.putText(frame, f"SPD: {pan_speed} | DIR: {pan_dir}", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    cv2.imshow("Mac Brain View", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Send safe shutdown packet
udp_sock.sendto(json.dumps({"pan": {"speed": 0, "dir": 0}, "slide": {"speed": 0, "dir": 0}, "tilt": {"speed": 0, "dir": 0}}).encode('utf-8'), (PI_IP, PI_PORT))
cap.release()
cv2.destroyAllWindows()