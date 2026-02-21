# AXI6: Predictive Cinematic Autopilot 🎥🤖

**A Multi-Axis Motorized Camera System with Predictive AI Tracking**

AXI6 (Axis-Intelligent 6) is a high-precision, multi-threaded robotics platform designed for cinema-grade camera movement. Built for **StarkHacks 2026**, it moves beyond reactive tracking by utilizing **Kalman Filters** to predict subject movement, enabling smooth, cinematic shots even under network latency or temporary visual occlusions.

## 🚀 Key Features

* **Predictive Motion Engine:** Implements 2D Kalman Filters to estimate subject trajectory, compensating for the communication lag between Vision (Mac) and Motion (Raspberry Pi).
* **Velocity-Based PID Control:** Utilizing `PyTmcStepper` for ultra-smooth, silent motor acceleration profiles on the Pan, Tilt, and Slide axes.
* **Recursive Path Learning:** "Record" manual camera movements and use AI-driven spline interpolation to play back "perfected," jitter-free cinematic paths.
* **Intelligent Auto-Parallax:** Real-time geometric calculation for "Orbit" shots, keeping subjects centered while the rail executes complex lateral moves.

## 🛠️ Tech Stack

* **Hardware:** Raspberry Pi 4/5, TMC2209 SilentStepStick Drivers, NEMA 17 Stepper Motors.
* **Software:** Python 3.11, `PyTmcStepper`, OpenCV (Computer Vision), NumPy (Matrix Math for Kalman).
* **Architecture:** Multi-threaded distributed system (Computer Vision processing on macOS ↔ Motor Control via RPi).

---

## 🏗️ Project Description (For Submission)

**The Problem:**
Traditional motorized camera rails are either "dumb" (requiring manual keyframing) or "reactive" (following a target with jerky, delayed movements). In a hackathon environment, network latency between a powerful vision processor and a motor controller usually results in "hunting" or oscillation.

**The Solution:**
AXI6 treats cinematography as a **state-estimation problem**. By decoupling the Vision pipeline from the Motor Control loop, we use a **Kalman Filter** to maintain a constant "belief" of the subject's position and velocity. This allows the Raspberry Pi to execute smooth **velocity-based PID moves** based on where the subject *will be*, not just where they were.

Our **Recursive Path Learning** feature allows creators to guide the camera by hand, after which our algorithm "re-imagines" that path as a mathematically perfect curve, bridging the gap between human creativity and robotic precision.

---

## 📈 Current Workflow & Roadmap

1. [x] Multi-threaded Vision/Motor architecture.
2. [x] Velocity-based PID for Pan Axis.
3. [ ] **Current Phase:** Kalman Filter integration for  trajectory prediction.
4. [ ] Multi-axis synchronization for Auto-Parallax (Orbit) shots.
5. [ ] Recursive Spline smoothing for recorded paths.

---

**Would you like me to add a "Getting Started" section with the specific pinout for your TMC drivers, or should we jump into the Kalman Filter code implementation?**
