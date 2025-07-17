# In __init__
# ...
self.camera_socket = None
self.localization_thread = None
self.camera_is_connected = tk.BooleanVar(master, value=False)
self.use_camera_for_gcode_control = tk.BooleanVar(master, value=False) # New control flag
self.toggle_camera_gcode_control_mode

# Add to create_widgets()
ttk.Checkbutton(
    control_frame, # Or a new frame for advanced control
    text="Use Camera for G-code Control",
    variable=self.use_camera_for_gcode_control,
    command=self.toggle_camera_gcode_control_mode
).pack(pady=5)

# ...

def start_localization_client(self):
    if self.localization_thread and self.localization_thread.is_alive():
        return # Already running
    self.localization_thread = threading.Thread(target=self._localization_listener_thread, daemon=True)
    self.localization_thread.start()
    print("Localization client thread started.")

def _localization_listener_thread(self):
    # This is a simplified example, needs robust error handling and reconnection logic
    HOST = '127.0.0.1' # Or the IP where AprilTagTest14 is running
    PORT = 65432       # Make sure this matches AprilTagTest14's server port
    try:
        self.camera_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.camera_socket.connect((HOST, PORT))
        self.camera_is_connected.set(True)
        print(f"Connected to localization server at {HOST}:{PORT}")

        buffer = ""
        while self.running: # Assuming self.running controls main app loop
            data = self.camera_socket.recv(1024).decode('utf-8')
            if not data:
                break
            buffer += data
            while '\\n' in buffer:
                line, buffer = buffer.split('\\n', 1)
                try:
                    loc_data = json.loads(line)
                    cam_x = loc_data['x']
                    cam_y = loc_data['y']
                    cam_rot = loc_data['rotation']

                    # --- IMPORTANT: Apply origin compensation here ---
                    # Example: If camera (0,0) is center, and G-code (0,0) is bottom-left
                    # You'll need to know your workspace dimensions for this.
                    # e.g., If workspace is 400x300mm, camera center is (200, 150)
                    # G-code X = cam_x + (workspace_width / 2)
                    # G-code Y = cam_y + (workspace_height / 2)
                    # Or more simply, apply fixed offsets:
                    gcode_x = cam_x + self.camera_x_offset_to_gcode_origin
                    gcode_y = cam_y + self.camera_y_offset_to_gcode_origin

                    self.master.after_idle(
                        lambda x=gcode_x, y=gcode_y, r=cam_rot: self._update_gui_localization(x, y, r)
                    )
                except json.JSONDecodeError as e:
                    print(f"JSON decode error: {e}, Data: {line}")
                except Exception as e:
                    print(f"Error processing localization data: {e}")
        
    except ConnectionRefusedError:
        print(f"Connection refused by localization server at {HOST}:{PORT}")
    except Exception as e:
        print(f"Localization client error: {e}")
    finally:
        if self.camera_socket:
            self.camera_socket.close()
            self.camera_socket = None
        self.camera_is_connected.set(False)
        print("Localization client stopped.")

# In _process_next_gcode_command(self):
# ...
if cmd_type == "G0" or cmd_type == "G1":
    # ... (calculate target_x, target_y, distance_mm, effective_feed_rate_mm_per_sec as before) ...

    if self.use_camera_for_gcode_control.get() and self.camera_is_connected.get():
        print(f"  [GCODE-CAM] Executing {cmd_type} with camera feedback.")
        
        # This is your new iterative control loop
        self._execute_gcode_segment_with_camera_feedback(target_x, target_y, effective_feed_rate_mm_per_sec, self.gcode_current_laser_power if self.gcode_current_laser_on else 0.0)
        
        # After the segment is done (in _execute_gcode_segment_with_camera_feedback),
        # update gcode_current_x/y to match the *actual* final position measured by camera
        self.gcode_current_x = self.x_pos.get()
        self.gcode_current_y = self.y_pos.get()

        return # The new method will handle scheduling the next G-code command

    else: # Fallback to existing G-code execution if camera control is off or not connected
        # ... (Your existing motion sending logic for G0/G1) ...
        # This part of the code remains largely as it is now.
        # It's what happens if camera control isn't active.
        # ...
        self.master.after(delay_ms, self._stop_robot_and_continue_gcode)
        return
# ... (rest of _process_next_gcode_command for M3, M5, etc.) ...

def _update_gui_localization(self, x, y, r):
    # This method is called from the main Tkinter thread
    self.x_pos.set(x)
    self.y_pos.set(y)
    self.rotation_val.set(r)
    self.current_localization_method.set("AprilTag Camera")
    self.update_localization_display() # Ensure this method uses the Tk.DoubleVars

def toggle_camera_gcode_control_mode(self):
    if self.use_camera_for_gcode_control.get():
        print("Camera-driven G-code control enabled.")
        self.start_localization_client() # Ensure client is running
    else:
        print("Camera-driven G-code control disabled. Reverting to internal G-code tracking.")
        self.current_localization_method.set("Dead Reckoning (G-code)") # Or whatever default
        # You might want to stop the localization thread gracefully here if it's not needed for other displays

def _execute_gcode_segment_with_camera_feedback(self, target_x_abs, target_y_abs, target_rotation_deg, speed_mm_per_sec, laser_power_value):
    # Get current actual position and rotation from camera (already updated by the localization thread)
    current_robot_x = self.x_pos.get()
    current_robot_y = self.y_pos.get()
    current_robot_rotation = self.rotation_val.get() # Camera provides yaw

    # --- Positional Error Calculation and Correction ---
    dx_error = target_x_abs - current_robot_x
    dy_error = target_y_abs - current_robot_y
    position_error_magnitude = math.sqrt(dx_error**2 + dy_error**2)

    # If already at target within tolerance, stop and move to next G-code command
    if position_error_magnitude < self.motion_tolerance_mm:
        # Also check rotation if it's crucial for the *end* of a segment
        rot_error_normalized = (target_rotation_deg - current_robot_rotation + 180) % 360 - 180 # Normalize to -180 to 180
        if abs(rot_error_normalized) < self.rotation_tolerance_deg: # New: rotation_tolerance_deg
            print(f"  [GCODE-CAM] Reached target X/Y/Rot: ({current_robot_x:.2f}, {current_robot_y:.2f}, {current_robot_rotation:.2f})")
            self._stop_robot_and_continue_gcode() # This method should just ensure robot stops and schedules next G-code
            return

    # Calculate desired positional movement command based on proportional error
    # This directly calculates the *command* to send, not a fixed step size.
    move_x_cmd = dx_error * self.Kp_pos
    move_y_cmd = dy_error * self.Kp_pos

    # Clamp the movement commands to prevent overshooting or too fast corrections
    move_distance_cmd = math.sqrt(move_x_cmd**2 + move_y_cmd**2)
    if move_distance_cmd > self.max_xy_step_per_iteration_mm:
        scale_factor = self.max_xy_step_per_iteration_mm / move_distance_cmd
        move_x_cmd *= scale_factor
        move_y_cmd *= scale_factor
    
    # --- Rotational Error Calculation and Correction ---
    # Normalize error to be between -180 and 180 degrees
    # Example: target 10, current 350. Error = 10 - 350 = -340. (-340 + 180)%360 - 180 = 20. (Should turn 20 deg CW)
    # Example: target 350, current 10. Error = 350 - 10 = 340. (340 + 180)%360 - 180 = -20. (Should turn 20 deg CCW)
    rot_error_normalized = (target_rotation_deg - current_robot_rotation + 180) % 360 - 180

    # Calculate desired rotational movement command based on proportional error
    move_rotation_cmd = rot_error_normalized * self.Kp_rot
    
    # Clamp rotation command
    move_rotation_cmd = max(-self.max_rot_step_per_iteration_deg, min(self.max_rot_step_per_iteration_deg, move_rotation_cmd))

    # --- Send Combined Command ---
    self.motion_command["x"] = move_x_cmd
    self.motion_command["y"] = move_y_cmd
    self.motion_command["rotation"] = move_rotation_cmd # Send the rotation correction!
    self.motion_command["elevation"] = laser_power_value
    self.send_control_command()

# --- Send Combined Command ---
self.motion_command["x"] = move_x_cmd
self.motion_command["y"] = move_y_cmd
self.motion_command["rotation"] = move_rotation_cmd # Send the rotation correction!

# Correctly map the laser_power_value (which is laser power)
# and also set the laser_on state based on your G-code's M3/M5 commands.
# You'll need to store the current M3/M5 state in a class variable (e.g., self.gcode_current_laser_on)
# and the S value in self.gcode_current_laser_power, as you're likely doing already.
self.motion_command["laser_on"] = self.gcode_current_laser_on # This comes from M3/M5
self.motion_command["laser_power"] = laser_power_value # This comes from S value (0-255)

    # --- Send Combined Command ---
self.motion_command["x"] = move_x_cmd
self.motion_command["y"] = move_y_cmd
self.motion_command["rotation"] = move_rotation_cmd # Send the rotation correction!

# Correctly set the laser status and power
self.motion_command["laser_on"] = self.gcode_current_laser_on # True/False based on M3/M5
self.motion_command["laser_power"] = laser_power_value          # The S value (0-255)

self.send_control_command()

self.send_control_command()

    # --- Determine Next Iteration Delay ---
    # The delay should be short enough for responsive control, but long enough
    # for the robot to execute the command and for the camera to provide a new reading.
    # This might be a fixed small delay (e.g., 50-100 ms) or dynamically calculated.
    # If dynamically calculated, base it on the *sent* move_distance_cmd and speed_mm_per_sec.
    # However, for a closed-loop system, a fixed, frequent update rate is often simpler
    # initially if the robot and camera can keep up.
    
    # Example: Fixed short delay, allowing camera to update
    iteration_delay_ms = 50 # Tune this based on your robot's responsiveness and camera update rate

    # Schedule the next check/step
    self.master.after(iteration_delay_ms, lambda: self._execute_gcode_segment_with_camera_feedback(
        target_x_abs, target_y_abs, target_rotation_deg, speed_mm_per_sec, laser_power_value
    ))

# --- And in _stop_robot_and_continue_gcode, ensure it correctly schedules the next G-code ---
# (Your current _stop_robot_and_continue_gcode already does this with self.master.after)
