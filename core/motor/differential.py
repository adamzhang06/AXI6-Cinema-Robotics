"""
core/motor/differential.py
--------------------------
Differential pan/tilt drive executor.

Converts pan/tilt spline trajectories into absolute step arrays for both 
co-axial motors, then executes them simultaneously in parallel threads
using busy-wait precision.

Also supports continuous velocity control for live tracking.
"""

import time
import threading
import queue
from typing import List
import RPi.GPIO as GPIO
from .axis import Axis


class DifferentialDrive:
    """
    Two-DOF differential drive executor.

    Motor A = Pan + Tilt
    Motor B = Pan - Tilt
    """


    def __init__(
        self,
        axis_a: Axis,
        axis_b: Axis,
    ):
        self.axis_a = axis_a
        self.axis_b = axis_b

        self._running = True
        
        # Thread-safe queues for exact step timings
        self.queue_a = queue.Queue()
        self.queue_b = queue.Queue()

        self._thread_a = threading.Thread(target=self._worker_a, daemon=True)
        self._thread_b = threading.Thread(target=self._worker_b, daemon=True)
        
        self._thread_a.start()
        self._thread_b.start()



    # --- Seamless Queued Execution ---

    def _worker_a(self):
        """Infinite loop consuming precise step times for Motor A."""
        p = self.axis_a.step_pin
        # start_time anchors the relative times in the queue to a real world clock
        start_time = 0 
        
        while self._running:
            try:
                # Block until a timing packet arrives 
                # Packet: (is_forward, list_of_times, anchor_time)
                direction_fwd, times, anchor_time = self.queue_a.get(timeout=1.0)
                
                # If the queue was empty, we establish the new anchor time.
                # If we are rapidly pulling from a backlog, we keep the old anchor time 
                # so the segments mathematically connect perfectly!
                if self.queue_a.qsize() == 0 or start_time == 0:
                     start_time = anchor_time
                     
                self.axis_a.set_direction(direction_fwd)
                
                for t in times:
                    # Busy wait for microsecond precision
                    while time.time() - start_time < t:
                        pass
                    GPIO.output(p, GPIO.HIGH)
                    time.sleep(0.0000001) # 0.1us
                    GPIO.output(p, GPIO.LOW)
                    
                self.queue_a.task_done()
            except queue.Empty:
                # Queue is empty, just loop and wait. Reset anchor so next segment is fresh.
                start_time = 0

    def _worker_b(self):
        """Infinite loop consuming precise step times for Motor B."""
        p = self.axis_b.step_pin
        start_time = 0 
        
        while self._running:
            try:
                direction_fwd, times, anchor_time = self.queue_b.get(timeout=1.0)
                
                if self.queue_b.qsize() == 0 or start_time == 0:
                     start_time = anchor_time
                     
                self.axis_b.set_direction(direction_fwd)
                
                for t in times:
                    while time.time() - start_time < t:
                        pass
                    GPIO.output(p, GPIO.HIGH)
                    time.sleep(0.0000001)
                    GPIO.output(p, GPIO.LOW)
                    
                self.queue_b.task_done()
            except queue.Empty:
                start_time = 0

    def enqueue_timing(
        self, 
        motor_a_forward: bool,
        motor_a_times: List[float],
        motor_b_forward: bool,
        motor_b_times: List[float]
    ) -> None:
        """
        Pushes step arrays into the background queues and returns instantly.
        If the motors were idle, establishes a synchronization anchor in the near future.
        """
        # Give a tiny buffer for threads to sync up if they were idle
        anchor = time.time() + 0.05 
        
        self.queue_a.put((motor_a_forward, motor_a_times, anchor))
        self.queue_b.put((motor_b_forward, motor_b_times, anchor))



    def close(self) -> None:
        self._running = False
        self.axis_a.close()
        self.axis_b.close()

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()
