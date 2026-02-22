# AXI6 Cinema Robotics

Autonomous camera head with differential pan/tilt drive and AI tracking.

## Project Structure

```
AXI6 Cinema Robotics/
│
├── core/                    # Shared logic — pure Python, hardware-agnostic
│   ├── motor/
│   │   ├── tmc2209.py       # TMC2209 UART register driver
│   │   ├── axis.py          # Single-axis wrapper (velocity ramping, position)
│   │   └── differential.py  # Pan/tilt differential drive (2-motor)
│   ├── motion/
│   │   ├── spline.py        # Cubic Bézier spline interpolation
│   │   └── trajectory.py    # Real-time spline playback → DifferentialDrive
│   └── comms/
│       └── protocol.py      # Shared Mac↔Pi socket message protocol
│
├── pi/                      # Runs on Raspberry Pi
│   ├── server.py            # Main Pi socket server → DifferentialDrive
│   ├── tracking/            # Computer vision tracking algorithms
│   │   ├── haarcascade.py
│   │   ├── haarcascade_pid.py
│   │   ├── mediapipe_stream.py
│   │   └── yolo_stream.py
│   └── demos/               # Standalone Pi test/demo scripts
│
├── mac/                     # Runs on Mac (operator side)
│   ├── spline_server.py     # Spline editor server + Pi relay
│   ├── tracking/            # Mac-side tracking (YOLO, mediapipe, orbit)
│   │   ├── yolo.py
│   │   ├── yolo_kf.py
│   │   ├── mediapipe.py
│   │   └── orbit.py
│   └── ui/
│       └── spline_editor.html  # Pan/tilt spline editor UI
│
├── models/                  # YOLO weights and model files
├── legacy/                  # Pre-refactor single-motor reference code (read-only)
│   └── README.md            # Index of what's in legacy/
│
├── requirements.txt
└── README.md
```

## Quick Start

### Mac (Spline Editor)
```bash
python mac/spline_server.py
# Open http://localhost:8080/spline_editor.html
```

### Pi (Motor Server)
```bash
python pi/server.py
```

## Architecture

The system uses a **differential drive** for pan and tilt:

```
Pan  velocity = (motor_A + motor_B) / 2
Tilt velocity = (motor_A - motor_B) / 2
```

Both motors are TMC2209 steppers controlled via VACTUAL register over UART,
allowing the chip to generate step pulses internally (no step/dir wiring needed).

## Legacy Reference

See `legacy/README.md` for the original single-motor scripts and camera tracking logic.
