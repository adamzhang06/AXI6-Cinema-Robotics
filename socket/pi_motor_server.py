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