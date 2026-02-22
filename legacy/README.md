# Legacy Code Reference

This folder is a **read-only snapshot** of the project before the differential pan/tilt refactor.
Do not develop here — use it to reference working camera, tracking, and motor logic.

## Contents

| Folder | What it contains |
|---|---|
| `legacy/pi/` | Pi-side tracking scripts (haarcascade, mediapipe, YOLO, PID), cameral flask test, and demo scripts |
| `legacy/mac/` | Mac-side orbit/YOLO/mediapipe tracking test scripts |
| `legacy/socket/` | Original Mac→Pi socket servers (`mac_spline_server.py`, `pi_motor_server.py`, `pi_spline_motor.py`, `mac_yolo.py`, etc.) |
| `legacy/tests/` | Single-motor experiments (`vactual.py`, `motor_velocity.py`, `absolute.py`, `spline_test.py`, `sanjay.py`) and the original `spline_editor.html` |

## Key References

- **VACTUAL / UART register logic** → `legacy/tests/vactual.py`  
- **Original spline server** → `legacy/socket/mac_spline_server.py`  
- **Pi motor server** → `legacy/socket/pi_motor_server.py`, `pi_spline_motor.py`  
- **Haarcascade PID** → `legacy/pi/haarcascade_PID.py`  
- **YOLO tracking** → `legacy/socket/mac_yolo.py`, `legacy/mac/YOLO_nano_test.py`  
- **Orbit mode** → `legacy/socket/mac_orbit.py`, `legacy/mac/orbit_test.py`  
