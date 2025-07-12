import tkinter as tk
from tkinter import ttk
from tkinter import filedialog # Import filedialog
import serial
import time
import re
import math # Import math for sqrt
import socket
import json
from tkinter import messagebox
import struct
import threading

class robotDirector:

    def __init__(self, master): 
        self.master = master
        master.title("Lyttle ReSearch Robot Director")
        master.config(bg="lightgreen")
        CE_PIN = 10 # GPIO17 (Physical pin 11)
        CSN_PIN = 9 # GPIO8 (Physical pin 24)

        self.port = tk.StringVar(value="/dev/ttyUSB0") # <--- Define self.port FIRST!
        self.baud_rate = 115200 # <--- Define self.baud_rate FIRST!

        self.gcode_file_path = tk.StringVar(master) # Add this line
        self.gcode_status_label = None # Add this line as well, if you haven't already

        self.current_laser_power = tk.IntVar(master, value=0)
        self.laser_on = tk.BooleanVar(master, value=False)
        self.radio_status = tk.StringVar(master, value="Idle")
        self.serial_port = None  # <--- ADD THIS LINE (or ensure it's here)
        self.arduino_connected = False
        self.running = False # Flag for controlling the serial read thread
        self.motion_command = {"x": 0.0, "y": 0.0, "rotation": 0.0, "laser_on": False, "laser_power": 0}
        self.north_angle = 0.0
        self.control_source = "keyboard"
        self.spacebar_pressed = False
        self.arduino_connected = False

        #self.current_control_method = tk.StringVar(value="Direct X/Y/R Buttons") # If using Tkinter StringVar
        # OR if just a regular string:
        self.current_control_method = "Direct X/Y/R Buttons"

        self.speed_var = tk.DoubleVar(master, value=0.5)
        self.motion_update_job = None
        self.is_moving = {
            "forward": False,
            "backward": False,
            "left": False,
            "right": False,
            "CCW": False,
            "CW": False,}

         # --- Localization variables ---
        self.x_pos = tk.DoubleVar(master, value=0.0)
        self.y_pos = tk.DoubleVar(master, value=0.0)
        self.rotation_val = tk.DoubleVar(master, value=0.0)
        self.elevation_val = tk.DoubleVar(master, value=0.0)

        self.create_widgets()
        self.is_sending_command = False
        self.master.bind('<KeyPress>', self.read_keyboard)
        self.master.bind('<KeyRelease>', self.read_keyrelease)
        self.master.bind('<FocusIn>', self.focus_change_handler, add='+')
        self.master.bind('<FocusOut>', self.focus_change_handler, add='+')
        self.update_radio_status("Disconnected")
        self.port = tk.StringVar(value="/dev/ttyUSB0")
        self.baud_rate = 115200
        self.connect_arduino_serial()
        # --- G-code Specific State Variables ---
        self.gcode_queue = [] # Queue to hold parsed G-code commands
        self.gcode_current_x = 0.0 # Robot's current X position in G-code coordinates (e.g., mm)
        self.gcode_current_y = 0.0 # Robot's current Y position in G-code coordinates (e.g., mm)
        self.gcode_current_laser_on = False
        self.gcode_current_laser_power = 0 # 0-255 scale
        self.gcode_current_feed_rate = 1.0 # Default feed rate in G-code units (e.g., mm/min)
        self.gcode_absolute_mode = True # True for G90 (absolute), False for G91 (relative)
        self.gcode_processing_active = False # Flag to indicate if G-code is currently being processed

        # --- New Joystick Control Variables ---
        self.joystick_socket = None
        self.client_connection = None
        self.joystick_listening = False
        self.joystick_port = 52345
        self.joystick_host = '127.0.0.1'
        self.joystick_buffer_size = 1024 # Buffer size for receiving data
        self.joystick_data_buffer = b'' # Buffer for accumulating partial messages
        # ...
        self.ROBOT_MAX_LINEAR_VELOCITY_MM_PER_SEC = 10000 # Adjust this to your robot's actual max linear speed in mm/sec
        self.gcode_current_feed_rate = self.speed_var.get() * self.ROBOT_MAX_LINEAR_VELOCITY_MM_PER_SEC

    def send_loop_callback(self): # <--- This is now a regular method
        # This callback runs repeatedly. It sends the current motion_command.
        # The calculation of self.motion_command based on pressed keys happens in read_keyboard/read_keyrelease.
        self.send_control_command()
        # Schedule the next call after a short delay (e.g., 50-100ms)
        # Adjust this delay for smoother or more responsive control.
        self.motion_update_job = self.master.after(150, self.send_loop_callback) # <--- Call itself with self.
        self.gcode_current_x = 0
        self.gcode_current_y = 0

        self.is_sending_command = False

    def _start_motion_sending_loop(self):
        # Cancel any existing loop to prevent duplicates
        if self.motion_update_job:
            self.master.after_cancel(self.motion_update_job)
            self.motion_update_job = None # Clear the old job ID

        # Start the first iteration of the loop by calling the method
        # This is the line that was missing!
        self.send_loop_callback()
        print("DEBUG: _start_motion_sending_loop - first callback initiated.") # Added this print back for verification

    def _setup_joystick_listener(self):
        """Sets up the non-blocking socket to listen for joystick data."""
        if self.joystick_listening:
            print("Joystick listener already running.")
            return

        try:
            self.joystick_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.joystick_socket.setblocking(False) # Non-blocking socket
            self.joystick_socket.bind((self.joystick_host, self.joystick_port))
            self.joystick_socket.listen(1) # Listen for one incoming connection
            self.joystick_listening = True
            print(f"Listening for joystick connection on {self.joystick_host}:{self.joystick_port}")
            # Schedule the check for new connections
            self.master.after(100, self._accept_joystick_connection) # Check every 100ms
        except socket.error as e:
            print(f"Could not set up joystick listener socket: {e}. Check if port {self.joystick_port} is in use.")
            self.joystick_listening = False
            if self.joystick_socket:
                self.joystick_socket.close()
            self.joystick_socket = None

    def _accept_joystick_connection(self):
        """Attempts to accept a new joystick connection."""
        if not self.joystick_listening or not self.joystick_socket:
            # Listener stopped or socket not initialized
            return

        try:
            conn, addr = self.joystick_socket.accept()
            self.client_connection = conn
            print(f"Accepted joystick connection from {addr}")
            self.client_connection.setblocking(False) # Make client connection non-blocking too
            self.joystick_data_buffer = b'' # Clear buffer for new connection
            # Start receiving data from this connection
            self.master.after(10, self._receive_joystick_data) # Start receiving data quickly
        except BlockingIOError:
            # No incoming connection yet, try again later
            pass
        except socket.error as e:
            print(f"Error accepting joystick connection: {e}")
            self._close_joystick_connection() # Attempt to close and restart listening if an error occurred during accept
        finally:
            # Always reschedule to keep checking for connections even if none accepted this time
            if self.joystick_listening and self.joystick_socket and not self.client_connection:
                self.master.after(100, self._accept_joystick_connection) # Keep looking for new clients if none connected

    def _receive_joystick_data(self):
        """Receives and processes JSON data from the joystick connection."""
        if not self.client_connection:
            print("No joystick client connected, stopping data reception.")
            return

        try:
            # Receive data in chunks.
            chunk = self.client_connection.recv(self.joystick_buffer_size)
            if not chunk:
                # Connection closed by client
                print("Joystick client disconnected.")
                self._close_joystick_connection()
                return

            self.joystick_data_buffer += chunk

            # Process all complete messages in the buffer
            # The sender adds '\n' (newline character), which needs to be escaped in the string literal b'\\n'
            while b'\n' in self.joystick_data_buffer:
                message_bytes, self.joystick_data_buffer = self.joystick_data_buffer.split(b'\n', 1)
                message_string = message_bytes.decode('utf-8')

                try:
                    joystick_data = json.loads(message_string)
                    # print(f"Received joystick data: {joystick_data}") # Uncomment for debugging

                    new_speed_multiplier = float(joystick_data.get("speed", self.speed_var.get()))
                    if self.speed_var.get() != new_speed_multiplier:
                        self.speed_var.set(new_speed_multiplier)

                    current_speed_factor = self.speed_var.get()
                    self.motion_command["x"] = float(joystick_data.get("x", 0.0)) * current_speed_factor
                    self.motion_command["y"] = float(joystick_data.get("y", 0.0)) * current_speed_factor
                    self.motion_command["rotation"] = float(joystick_data.get("r", 0.0)) * current_speed_factor

                    # --- START OF MODIFICATION FOR LASER CONTROL ---
                    # Ensure laser state and power from joystick are applied to motion_command
                    laser_on_value = bool(joystick_data.get("laser", 0))
                    # Use joystick power if provided, otherwise use the current slider power as default
                    laser_power_value = int(joystick_data.get("power", self.current_laser_power.get()))
                    
                    self.laser_on.set(laser_on_value) # Update Tkinter variable for GUI
                    self.motion_command["laser_on"] = laser_on_value # Update the command dictionary
                    self.motion_command["laser_power"] = laser_power_value # Update the command dictionary
                    # --- END OF MODIFICATION FOR LASER CONTROL ---

                    # Send the combined control command to the robot
                    self.send_control_command() # Send command on every joystick data receipt

                except json.JSONDecodeError as e:
                    print(f"Received invalid JSON data: '{message_string}' - Error: {e}")
                except ValueError as e:
                    print(f"Data conversion error in joystick data: {e} for message: '{message_string}'")
                except Exception as e:
                    print(f"Error processing joystick data: {e} for message: '{message_string}'")

        except BlockingIOError:
            # No data currently available on the socket, try again later
            pass
        except socket.error as e:
            print(f"Socket error during joystick data reception: {e}")
            self._close_joystick_connection()
            return
        finally:
            # Reschedule itself to keep receiving data as long as a client is connected
            if self.client_connection:
                self.master.after(10, self._receive_joystick_data)

    def _close_joystick_connection(self):
        """Closes the joystick client connection and prepares for a new one."""
        if self.client_connection:
            self.client_connection.close()
            self.client_connection = None
            print("Joystick client connection closed.")
        # Restart listening for a new client if the listener is active
        if self.joystick_listening and self.joystick_socket:
            print("Listening for a new joystick connection...")
            self.master.after(100, self._accept_joystick_connection)
        elif self.joystick_socket: # If client disconnected but listener was manually stopped
            self.joystick_socket.close()
            self.joystick_socket = None


    def focus_change_handler(self,event):
        print(f"Focus changed to: {event.widget}")

    def deactivate_laser_button(self):
        """Turns the laser OFF via button press."""
        self.laser_on.set(False) # Update Tkinter variable for GUI
        # --- START OF MODIFICATION FOR DEACTIVATE BUTTON ---
        self.motion_command["laser_on"] = False # Update motion_command for send_control_command
        self.motion_command["laser_power"] = 0 # Turn off power
        # --- END OF MODIFICATION FOR DEACTIVATE BUTTON ---
        self.send_control_command()
        print("Laser OFF")

    def read_keyrelease(self, event):
        keysym = event.keysym.lower()
        print(f"read_keyrelease: event.keysym = {keysym!r}")

        if keysym == 'space':
            self.laser_on.set(False) # Update Tkinter variable for GUI
            # --- START OF MODIFICATION FOR SPACEBAR LASER CONTROL ---
            self.motion_command["laser_on"] = False
            self.motion_command["laser_power"] = 0 # Turn off laser power on release
            # --- END OF MODIFICATION FOR SPACEBAR LASER CONTROL ---
            self.send_control_command()
            self.spacebar_pressed = False
            print("Spacebar released: Laser OFF")
            return

        # Ensure current_control_method is handled consistently (e.g., if it's a StringVar, use .get())
        if hasattr(self, 'current_control_method') and isinstance(self.current_control_method, str):
            control_method_value = self.current_control_method
        elif hasattr(self, 'current_control_method') and hasattr(self.current_control_method, 'get'):
            control_method_value = self.current_control_method.get()
        else:
            control_method_value = None # Fallback if not set correctly or is unexpected type

        if control_method_value == "Direct X/Y/R Buttons":
            if keysym == 'w':
                self.is_moving['forward'] = False
            elif keysym == 's':
                self.is_moving['backward'] = False
            elif keysym == 'a':
                self.is_moving['left'] = False
            elif keysym == 'd':
                self.is_moving['right'] = False
            elif keysym == 'q':
                self.is_moving['CCW'] = False
            elif keysym == 'e':
                self.is_moving['CW'] = False

            # Check if *any* movement key is still pressed. If not, *then* stop and cancel the loop.
            if not any(self.is_moving.values()):
                self.motion_command["x"] = 0.0
                self.motion_command["y"] = 0.0
                self.motion_command["rotation"] = 0.0
                self.send_control_command() # Send one final stop command
                self._stop_motion_sending_loop() # <-- NEW: Stop the repeating loop!

    def _stop_motion_sending_loop(self):
        if self.motion_update_job:
            self.master.after_cancel(self.motion_update_job)
            self.motion_update_job = None
            print("DEBUG: _stop_motion_sending_loop - Motion sending loop stopped.")

    def stop_movement(self, axis):
        print(f"Stop movement on axis: {axis}")
        if axis == "X":
            self.motion_command["x"] = 0.0
        elif axis == "Y":
            self.motion_command["y"] = 0.0
        elif axis == "R":
            self.motion_command["rotation"] = 0.0
        self.send_control_command()

    def stop_movement_wrapper(self, axis, direction, stop_event=None):
        print(f"Stop button released for {direction} on axis {axis}")
        self.is_moving[direction] = False
        self.stop_movement(axis)  # Call stop_movement, NOT _stop_movement

    def connect_arduino_serial(self):
        """Attempts to connect to the Arduino via serial."""
        # Close existing serial port if open
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()

        try:
            # Use self.port.get() to get the current port string
            self.serial_port = serial.Serial(self.port.get(), self.baud_rate, timeout=1)
            print(f"Connected to Arduino on {self.port.get()} at {self.baud_rate} baud.")
            self.arduino_connected = True
            self.update_radio_status("Connected")  # Update status to indicate connection

            # Start the serial reading thread ONLY after a successful connection
            self.running = True
            self.serial_read_thread = threading.Thread(target=self._read_from_serial_port, daemon=True)
            self.serial_read_thread.start()

        except serial.SerialException as e:
            print(f"Error connecting to Arduino on {self.port.get()}: {e}")
            self.serial_port = None
            self.arduino_connected = False
            self.update_radio_status("Disconnected")

    def _read_from_serial_port(self):
        while self.running:
            if self.serial_port and self.serial_port.is_open: # Check if port is open
                try:
                    if self.serial_port.in_waiting > 0:
                        line = self.serial_port.readline().decode('utf-8').strip()
                        if line:
                            print("[SERIAL READ] Received:", line) # Keep this temporary print for now
                            # Process the line here, e.g., for "Radio Success"
                            if "Radio Success:" in line:
                                status = line.split(":")[-1].strip()
                                if status == "1":
                                    self.update_radio_status("Radio OK")
                                else:
                                    self.update_radio_status("Radio Error")
                            # Add other parsing logic here (like position updates etc.)
                except Exception as e:
                    print(f"Error reading from serial: {e}")
                    # Consider setting self.arduino_connected = False and stopping the thread here
                    # if the error is critical, or just sleep and retry.
                    time.sleep(0.1) # Small delay to prevent busy-waiting on error
            else:
                time.sleep(0.5) # Sleep longer if not connected to avoid CPU hogging

    def emergency_stop(self):
        print("Emergency Stop Activated!")
        self.update_radio_status("Error")
        self.motion_command = {"x": 0.0, "y": 0.0, "rotation": 0.0}
        self.laser_on.set(False)
        self.current_laser_power.set(0)
        self.send_control_command()

    def move_robot(self, direction, speed, start_event=None):
        print(f"Start moving robot {direction} at {speed} speed.")
        self.is_moving[direction] = True  # Track which direction is active
        self._send_repeated_command(direction, speed)

    def _send_repeated_command(self, direction, speed):
        if self.is_moving.get(direction, False):
            vx, vy, omega = 0.0, 0.0, 0.0
            speed_factor = speed  # Use the direct speed value now

            if direction == "left":
                vx = -speed_factor
            elif direction == "right":
                vx = speed_factor
            elif direction == "forward":
                vy = speed_factor
            elif direction == "backward":
                vy = -speed_factor
            elif direction == "CCW":
                omega = speed_factor
            elif direction == "CW":
                omega = -speed_factor

            self.motion_command["x"] = vx
            self.motion_command["y"] = vy
            self.motion_command["rotation"] = omega
            self.send_control_command()
            self.master.after(150, self._send_repeated_command, direction, speed)  # Repeat every 100ms

    def _update_gcode_feed_rate_from_slider(self, value):
        """
        Updates gcode_current_feed_rate when the speed slider is moved.
        Sends the command immediately if in G-code control mode.
        """
        new_speed_multiplier = float(value)
        self.gcode_current_feed_rate = new_speed_multiplier * self.ROBOT_MAX_LINEAR_VELOCITY_MM_PER_SEC
        
        # If in G-code control mode, send the command immediately to reflect the new feed rate.
        # This allows the slider to act as a real-time override for the feed rate.
        if self.current_control_method == "G-code Control":
            self.send_control_command()
            # print(f"Sent command with feed rate: {self.gcode_current_feed_rate:.2f} (from slider in G-code mode)")

    def send_control_command(self):
        print(f"  [DEBUG] Entering send_control_command. is_sending_command: {self.is_sending_command}")

        # Ensure a serial port is connected (this is the serial connection to the desktop Nano bridge)
        if not self.arduino_connected or not self.serial_port or not self.serial_port.is_open:
            print("  [DEBUG] send_control_command: Serial port to desktop Nano not connected or open. Cannot send command.")
            self.radio_status.set("Bridge Not Connected") # Update status to reflect bridge
            return

        # Add this check back for safety against rapid calls if one is already in progress
        if self.is_sending_command:
            print("  [DEBUG] send_control_command: Command already in progress, skipping send.")
            return
        
        self.is_sending_command = True # Set flag to prevent re-entry
        print("  [DEBUG] is_sending_command set to True.")

        try:
            motion_x = self.motion_command["x"]
            motion_y = self.motion_command["y"]
            rotation = self.motion_command["rotation"]
            
            # --- THIS IS THE CRUCIAL CORRECTION FOR LASER CONTROL SOURCE ---
            # Get laser state and power from self.motion_command.
            # This dictionary is updated by both G-code commands (M3/M5) and manual controls.
            # Use .get() with a default value for robustness, in case a key isn't always present.
            laser = int(self.motion_command.get("laser_on", False)) # Get boolean, convert to 0 or 1
            laser_power = self.motion_command.get("laser_power", 0) # Get integer power, default to 0
            # --- END OF CORRECTION ---

            # The speed slider's value (0.0-1.0) is the `target_speed`
            # This should still come from the speed_var as it's a UI slider value
            target_speed = self.speed_var.get()

            # Construct the command string in the ORIGINAL WORKING FORMAT (that the Arduino expects)
            # Corrected: Removed extra commas after 'laser' and 'laser_power'
            command_string = (
                f"MX:{motion_x: .8f},"
                f"MY:{motion_y: .8f},"
                f"R:{rotation: .8f},"
                f"L:{laser},"       # Corrected: Removed trailing comma
                f"P:{laser_power}," # Corrected: Removed trailing comma
                f"S:{target_speed: .8f}\n" # Add newline for string termination
            )

            print(f"  [DEBUG] Attempting to send (via serial bridge): {command_string.strip()}")
            self.serial_port.write(command_string.encode('utf-8')) # Encode and send
            self.radio_status.set(f"Bridge Sent: {command_string.strip()}") # Update status
            print(f"  [SENT VIA BRIDGE] {command_string.strip()}") # CONFIRMATION PRINT

        except serial.SerialException as e:
            print(f"  [ERROR] Serial communication error during send to bridge: {e}")
            self.radio_status.set(f"Bridge Serial Error: {e}")
        except Exception as e:
            print(f"  [ERROR] send_control_command failed unexpectedly: {e}")
            self.radio_status.set(f"Command Error: {e}")
        finally:
            # Ensure the flag is reset in all cases (success or failure)
            self.is_sending_command = False
            print("  [DEBUG] is_sending_command set to False (finally block).")

    def _reset_is_sending_command(self):
        """Helper to reset the is_sending_command flag."""
        self.is_sending_command = False
        print("  [DEBUG] is_sending_command flag reset to False.") # Debug print for reset

    def create_widgets(self):
        # Configure the master grid for responsiveness
        # Row 0: Top-level control, gcode, localization displays
        # Row 1: Dynamic control area
        # Row 2: General sliders (Speed)
        # Row 3: Laser Control
        # Row 4: Robot Location Initialization
        # Row 5: Status Bar (New!)
        self.master.grid_rowconfigure(0, weight=1)
        self.master.grid_rowconfigure(1, weight=1)
        self.master.grid_rowconfigure(2, weight=0)
        self.master.grid_rowconfigure(3, weight=0)
        self.master.grid_rowconfigure(4, weight=0)
        self.master.grid_rowconfigure(5, weight=0) # Configure new status row

        self.master.grid_columnconfigure(0, weight=1)
        self.master.grid_columnconfigure(1, weight=1)
        self.master.grid_columnconfigure(2, weight=1)

        # --- Control Frame ---
        control_frame = ttk.LabelFrame(self.master, text="Robot Control", padding="10")
        control_frame.grid(row=0, column=0, padx=10, pady=10, sticky="nsew")
        control_frame.grid_columnconfigure(0, weight=1)

        # Modify self.control_styles list:
        self.control_styles = ["Direct X/Y/R Buttons", "Circle Click Control", "Python Script Director",
                               "Tarantino as Director", "Frickin Shoot Everyone Director", ".dxf Director",
                               ".jpg (Python Contour) Director", ".ngc/.nc/.gcode Director", "SVG/BMP Director",
                               "Joystick Control", "Placeholder Style"]

        # Modify self.control_styles_dict dictionary:
        self.control_styles_dict = {
            "Direct X/Y/R Buttons": self.create_keyboard_control_area,
            "Circle Click Control": self.create_circle_control,
            "Python Script Director": self.create_python_script_director,
            "Tarantino as Director": self.create_tarantino_director,
            "Frickin Shoot Everyone Director": self.create_frickin_shoot_everyone_director,
            ".dxf Director": self.create_dxf_director,
            ".jpg (Python Contour) Director": self.create_jpg_director,
            ".ngc/.nc/.gcode Director": self.create_gcode_director,
            "SVG/BMP Director": self.create_svg_bmp_director,
            "Joystick Control": self.create_joystick_control_area,
            "Placeholder Style": self.create_placeholder_style,
        }
        self.control_style_dropdown = ttk.Combobox(control_frame, values=self.control_styles, state="readonly")
        self.control_style_dropdown.set("Direct X/Y/R Buttons")
        self.control_style_dropdown.grid(row=0, column=0, pady=5, padx=5, sticky="ew")
        self.control_style_dropdown.bind("<<ComboboxSelected>>", self.control_style_changed)

        # --- Current Position Display Frame ---
        current_position_display_frame = ttk.LabelFrame(self.master, text="Current Position", padding="10")
        current_position_display_frame.grid(row=0, column=2, padx=10, pady=10, sticky="nsew")
        current_position_display_frame.grid_columnconfigure(0, weight=1)

        self.x_label = ttk.Label(current_position_display_frame, text=f"X: {self.x_pos.get():.2f}")
        self.x_label.grid(row=0, column=0, sticky="w", pady=2)
        self.y_label = ttk.Label(current_position_display_frame, text=f"Y: {self.y_pos.get():.2f}")
        self.y_label.grid(row=1, column=0, sticky="w", pady=2)
        self.r_label = ttk.Label(current_position_display_frame, text=f"Rotation (R): {self.rotation_val.get():.2f} deg")
        self.r_label.grid(row=2, column=0, sticky="w", pady=2)
        self.e_label = ttk.Label(current_position_display_frame, text=f"Elevation (E): {self.elevation_val.get():.2f}")
        self.e_label.grid(row=3, column=0, sticky="w", pady=2)


        # --- Dynamic Control Area ---
        # CHANGE 'parent_frame' to 'self.dynamic_control_area' here:
        self.dynamic_control_area = ttk.Frame(self.master, style="TFrame") # <--- MAKE THIS CHANGE
        self.dynamic_control_area.grid(row=1, column=0, columnspan=3, padx=10, pady=10, sticky="nsew")
        self.dynamic_control_area.grid_columnconfigure(0, weight=1)
        self.dynamic_control_area.grid_rowconfigure(0, weight=1)

        # Keep self.control_style_changed() immediately after the creation of self.dynamic_control_area
        self.control_style_changed()


        # --- General Controls Frame (Speed Control) ---
        general_sliders_frame = ttk.LabelFrame(self.master, text="Speed Control", style="TLabelframe")
        general_sliders_frame.grid(row=2, column=0, columnspan=3, padx=10, pady=5, sticky="ew")
        general_sliders_frame.grid_columnconfigure(1, weight=1)

        speed_label = ttk.Label(general_sliders_frame, text="Robot Speed (Feed Rate):", style="TLabel")
        speed_label.grid(row=0, column=0, padx=(5, 0), pady=5, sticky="w")

        self.speed_slider = ttk.Scale(general_sliders_frame, from_=0.0, to=1.0,
                                         orient=tk.HORIZONTAL, variable=self.speed_var,
                                         command=self._update_gcode_feed_rate_from_slider)
        self.speed_slider.grid(row=0, column=1, padx=5, pady=5, sticky="ew")

        self.speed_value_label = ttk.Label(general_sliders_frame, textvariable=self.speed_var, style="TLabel")
        self.speed_value_label.grid(row=0, column=2, padx=(0, 5), pady=5, sticky="e")


        # --- Laser Control Frame ---
        laser_frame = ttk.LabelFrame(self.master, text="Laser Control", style="TLabelframe")
        laser_frame.grid(row=3, column=0, columnspan=3, padx=10, pady=10, sticky="ew")
        laser_frame.grid_columnconfigure(2, weight=1)

        ttk.Button(laser_frame, text="Activate Laser", command=self.activate_laser_button).grid(row=0, column=0, padx=5, pady=5, sticky="w")
        ttk.Button(laser_frame, text="Deactivate Laser", command=self.deactivate_laser_button).grid(row=0, column=1, padx=5, pady=5, sticky="w")

        ttk.Label(laser_frame, text="Power:", background="lightgreen").grid(row=1, column=0, padx=5, pady=5, sticky="w")
        self.power_label = ttk.Label(laser_frame,
                                      text=f"{self.current_laser_power.get()}",
                                      background="lightgreen")
        self.power_label.grid(row=1, column=1, padx=0, pady=5, sticky="w")

        self.laser_power_slider = tk.Scale(laser_frame, from_=0, to=255, orient=tk.HORIZONTAL,
                                             command=self.set_laser_power)
        self.laser_power_slider.set(0)
        self.laser_power_slider.grid(row=1, column=2, padx=5, pady=5, sticky="ew")


        # --- Robot Location Initialization Frame ---
        robot_location_init_frame = ttk.LabelFrame(self.master, text="Robot Location Initialization", style="TLabelframe")
        robot_location_init_frame.grid(row=4, column=0, columnspan=3, padx=10, pady=10, sticky="ew")
        robot_location_init_frame.grid_columnconfigure(0, weight=1)

        ttk.Button(robot_location_init_frame, text="Initialize Location", command=self.initiate_localization).grid(row=0, column=0, padx=5, pady=5)


        # --- Status Frame (NEW!) ---
        status_frame = ttk.Frame(self.master, style="TFrame")
        status_frame.grid(row=5, column=0, columnspan=3, padx=10, pady=5, sticky="ew")
        status_frame.grid_columnconfigure(0, weight=1) # Allow status label to expand

        # G-code Status Label - Place inside status_frame, at row 0, column 0
        self.gcode_status_label = tk.Label(status_frame, text="G-code Status: Idle", bg="lightgreen", font=("Arial", 10))
        self.gcode_status_label.grid(row=0, column=0, padx=5, pady=2, sticky="w") # Corrected grid placement and indentation

        # Radio Status Label - Place inside status_frame, at row 0, column 1 (or 0, 0 if you want it below gcode status)
        # Assuming you want these side-by-side or stacked within the status frame
        self.radio_status_label = ttk.Label(status_frame, textvariable=self.radio_status, style="TLabel")
        self.radio_status_label.grid(row=0, column=1, padx=5, pady=2, sticky="w") # Placed next to G-code status

        # Removed the problematic 'row_counter += 1' as it doesn't apply here.
        # The 'row' numbers are relative to the status_frame grid now.

        # Now, you can safely call update_radio_status in __init__ after create_widgets()
        # because self.radio_status_label will now exist.

    def activate_laser_button(self):
        # ... (existing code) ...
        self.laser_on.set(True) # Update Tkinter variable for GUI
        self.motion_command["laser_on"] = True # Update motion_command for send_control_command
        self.motion_command["laser_power"] = self.current_laser_power.get() # Ensure power is also set
        self.send_control_command()

    def set_laser_power(self, value): # 'value' is passed by the slider
        power = int(float(value))
        self.current_laser_power.set(power) # Update Tkinter variable for GUI
        self.motion_command["laser_power"] = power # Update motion_command
        # Only send command if laser is currently on, or if it's an M3/M5 G-code command
        if self.laser_on.get() or self.gcode_processing_active: # Only send if laser is on or G-code is active
         self.send_control_command()

    def update_localization_display(self):
        """Updates the labels in the GUI with current localization values."""
        self.x_label.config(text=f"X: {self.x_pos.get():.2f}")
        self.y_label.config(text=f"Y: {self.y_pos.get():.2f}")
        self.r_label.config(text=f"Rotation (R): {self.rotation_val.get():.2f} deg")
        self.e_label.config(text=f"Elevation (E): {self.elevation_val.get():.2f}")

        # Schedule the next update
        self.master.after(50, self.update_localization_display) # Update every 50ms (adjust as needed)

    def control_style_changed(self, event=None):
        selected_style = self.control_style_dropdown.get()
        handler = self.control_styles_dict.get(selected_style)

        # Clear previous dynamic content
        for widget in self.dynamic_control_area.winfo_children(): # <--- CORRECT THIS LINE
            widget.destroy()

        if handler:
            handler(self.dynamic_control_area, event)
        else:
            print(f"No handler for control style: {selected_style}")

    def create_keyboard_control_area(self, parent_frame, event=None):
        # Clear any existing widgets in the dynamic area
        for widget in parent_frame.winfo_children():
            widget.destroy()

        # Add instructions for keyboard control
        instructions = ttk.Label(parent_frame,
                                 text="Direct Keyboard Control:\n"
                                      "w: Forward, s: Backward\n"
                                      "a: Left, d: Right\n"
                                      "q: Rotate CCW, e: Rotate CW",
                                 style="TLabel", justify="center")
        instructions.pack(pady=10, padx=10)

        # REMOVED: Speed slider creation from here. The global speed slider is handled in create_widgets.
        # REMOVED: self.current_control_method assignment from here. This is handled by control_style_changed.    

    def create_python_script_director(self, parent_frame, event=None):
        ttk.Label(parent_frame, text="External Python Script Director (Not Implemented Yet)",
                  background="lightgray").pack(padx=10, pady=10)

    def create_circle_control(self, parent_frame, event=None):
        ttk.Label(parent_frame, text="SVG/BMP Director (Not Implemented Yet)", background="lightgray").pack(padx=10,
                                                                                                         pady=10)

    def create_tarantino_director(self, parent_frame, event=None):
        ttk.Label(parent_frame, text="Tarantino as Director (Not Implemented Yet)", background="lightgray").pack(padx=10,
                                                                                                           pady=10)
        # Add specific widgets and logic for the Tarantino control style here

    def create_frickin_shoot_everyone_director(self, parent_frame, event=None):
        ttk.Label(parent_frame, text="Frickin Shoot Everyone Director (Not Implemented Yet)",
                  background="lightgray").pack(padx=10, pady=10)
        # Add specific widgets and logic for this control style here

    def create_dxf_director(self, parent_frame, event=None):
        ttk.Label(parent_frame, text=".dxf Director (Not Implemented Yet)", background="lightgray").pack(padx=10,
                                                                                                      pady=10)

    def create_jpg_director(self, parent_frame, event=None):
        ttk.Label(parent_frame, text=".jpg (Python Contour) Director (Not Implemented Yet)",
                  background="lightgray").pack(padx=10, pady=10)

    def create_gcode_director(self, parent_frame, event=None):
        ttk.Label(parent_frame, text=".ngc/.nc/.gcode Director (Not Implemented Yet)", background="lightgray").pack(
            padx=10, pady=10)

    def create_svg_bmp_director(self, parent_frame, event=None):
        ttk.Label(parent_frame, text="SVG/BMP Director (Not Implemented Yet)", background="lightgray").pack(padx=10,
                                                                                                         pady=10)

    def create_joystick_control_area(self, parent_frame, event=None):
        ttk.Label(parent_frame, text="Joystick Control", background="lightgray").pack(padx=10,
                                                                                                         pady=10)

    def create_placeholder_style(self, parent_frame, event=None):
        ttk.Label(parent_frame, text="Placeholder Style (Not Implemented Yet)", background="lightgray").pack(padx=10,
                                                                                                           pady=10)

    def activate_laser_button(self):
        print("Laser Activated (Button).")
        self.laser_on.set(True)
        self.send_control_command()
        self.simulate_radio_transmission(True)

    def deactivate_laser_button(self):
        print("Laser Deactivated (Button).")
        self.laser_on.set(False)
        self.send_control_command()
        self.simulate_radio_transmission(True)

    def set_laser_power(self, power):
        print(f"Setting laser power to: {power}")
        self.current_laser_power.set(int(power))
        self.power_label.config(text=f"Power: {self.current_laser_power.get()}")
        self.send_control_command()
        self.simulate_radio_transmission(True)

    def simulate_radio_transmission(self, success):
        """Simulates a radio transmission and updates the status."""
        if success:
            self.update_radio_status("Success")
            self.master.after(100, self.reset_radio_status)  # Reset after a short delay
        else:
            self.update_radio_status("Failure")

    def update_radio_status(self, status):
        """Updates the radio status label and appearance."""
        self.radio_status.set(status)
        if status == "Success":
            self.radio_status_label.config(style="Success.TLabel")
        elif status == "Failure":
            self.radio_status_label.config(style="Failure.TLabel")
        elif status == "Error":
            self.radio_status_label.config(style="Error.TLabel")
        elif status == "Busy":
            self.radio_status_label.config(style="Busy.TLabel")
        elif status == "Disconnected":
            self.radio_status_label.config(style="Disconnected.TLabel")  # Add disconnected style
        else:
            self.radio_status_label.config(style="Idle.TLabel")  # Default state

    def reset_radio_status(self):
        if self.radio_status.get() in ["Success", "Failure"]:
            self.update_radio_status("Idle")

    def initiate_localization(self):
        print("Initiating Robot Location Initialization...")
        self.update_radio_status("Busy")
        # Simulate localization result after a short delay
        self.master.after(2000, self.simulate_localization_result)

    def simulate_localization_result(self):
        """Simulates the result of the localization procedure."""
        success = True  # For this simulation, assume success
        if success:
            print("Robot Location Initialization Successful.")
            self.update_radio_status("Success")
            # Optionally update the Robot Status display with initial position/orientation
        else:
            print("Robot Location Initialization Failed.")
            self.update_radio_status("Failure")

    def read_keyboard(self, event):
        keysym = event.keysym.lower() # Get keysym and convert to lowercase for consistent checking
        print(f"DEBUG: read_keyboard - keysym: {keysym!r}, control_method: {self.current_control_method!r}")

        # Handle spacebar first as it's common across control methods and a direct action
        if keysym == 'space':
            self.laser_on.set(True)
            self.spacebar_pressed = True
            # Send the command immediately for laser state change
            self.send_control_command()
            print(f"DEBUG: Current Control Method: {self.current_control_method}")
            return # Exit after handling spacebar, it's a direct action


        if self.current_control_method == "Direct X/Y/R Buttons":
            speed = self.speed_var.get()

            # First, update the state of the *just pressed* key using keysym
            if keysym == 'w':
                self.is_moving['forward'] = True
            elif keysym == 's':
                self.is_moving['backward'] = True
            elif keysym == 'd':
                self.is_moving['right'] = True
            elif keysym == 'a':
                self.is_moving['left'] = True
            elif keysym == 'q':
                self.is_moving['CCW'] = True
                self.north_angle = (self.north_angle + 5) % 360  # Example rotation update
                print(f"North Angle: {self.north_angle}")
            elif keysym == 'e':
                self.is_moving['CW'] = True
                self.north_angle = (self.north_angle - 5) % 360  # Example rotation update
                print(f"North Angle: {self.north_angle}")
            else:
                # If it's not a relevant movement key, do nothing further in this method
                print(f"DEBUG: read_keyboard - Unhandled key {keysym!r} for Direct X/Y/R Buttons control.")
                return

            # Now, calculate the *cumulative* motion based on ALL currently pressed movement keys
            # (whose 'is_moving' flags are True)
            vx, vy, omega = 0.0, 0.0, 0.0
            if self.is_moving['forward']:
                vy += speed
            if self.is_moving['backward']:
                vy -= speed
            if self.is_moving['right']:
                vx += speed
            if self.is_moving['left']:
                vx -= speed
            if self.is_moving['CCW']:
                omega += speed
            if self.is_moving['CW']:
                omega -= speed

            # Update the motion_command dictionary with these cumulative values
            self.motion_command["x"] = vx
            self.motion_command["y"] = vy
            self.motion_command["rotation"] = omega

            # Start the continuous sending loop if it's not already running
            if not self.motion_update_job: # Only start if not already active
                print("DEBUG: read_keyboard - motion_update_job is None, starting loop.")
                self._start_motion_sending_loop()
            else:
                print("DEBUG: read_keyboard - motion_update_job already active, not restarting loop.")

        else:
            print(f"DEBUG: read_keyboard - Control method not Direct X/Y/R Buttons ({self.current_control_method!r}), skipping keyboard motion.")    

    def create_gcode_director(self, parent_frame, event = None):
        # Clear previous dynamic content
        for widget in parent_frame.winfo_children():
            widget.destroy()

        # Add G-code specific widgets
        row_counter = 0
        parent_frame.grid_columnconfigure(0, weight=1)
        parent_frame.grid_columnconfigure(1, weight=1) # For buttons next to each other

        # Load G-code File Button
        btn_load_gcode = ttk.Button(parent_frame, text="Load G-code File", command=self.select_gcode_file) # <--- CHANGE 'browse_gcode_file' to 'select_gcode_file' command=self.browse_gcode_file)
        btn_load_gcode.grid(row=row_counter, column=0, columnspan=2, padx=5, pady=5, sticky="ew")
        row_counter += 1

        # Display selected G-code file path
        lbl_gcode_path = ttk.Label(parent_frame, textvariable=self.gcode_file_path, wraplength=300)
        lbl_gcode_path.grid(row=row_counter, column=0, columnspan=2, padx=5, pady=5, sticky="ew")
        row_counter += 1

        # G-code Control Buttons (Start/Stop)
        self.btn_start_gcode = ttk.Button(parent_frame, text="Start G-code", command=self.start_gcode_execution, state=tk.DISABLED)
        self.btn_start_gcode.grid(row=row_counter, column=0, padx=5, pady=5, sticky="ew")

        self.btn_stop_gcode = ttk.Button(parent_frame, text="Stop G-code", command=self.stop_gcode_execution, state=tk.DISABLED)
        self.btn_stop_gcode.grid(row=row_counter, column=1, padx=5, pady=5, sticky="ew")
        row_counter += 1

        # REMOVE OR COMMENT OUT THIS LINE if it's present and overwriting your status:
        # self.update_gcode_status("No G-code loaded.")

    def start_gcode_execution(self):
        """Starts processing the loaded G-code queue, ensuring a fresh start."""
        # First, ensure a G-code file has been selected
        if not self.gcode_file_path.get():
            self.update_gcode_status("Error: No G-code file selected to start.")
            return

        # To ensure a fresh start and reset all internal G-code state variables,
        # we re-load the file. This will re-populate self.gcode_queue and reset
        # gcode_current_x, gcode_current_y, etc.
        # Note: self.load_gcode_file sets gcode_processing_active to False.
        self.load_gcode_file(self.gcode_file_path.get())

        # After load_gcode_file completes, check if the queue is actually populated.
        # (It might be empty if the file was empty or corrupted).
        if not self.gcode_queue:
            self.update_gcode_status("Error: G-code queue is empty after reset. Cannot start.")
            self.btn_start_gcode.config(state=tk.DISABLED)
            self.btn_stop_gcode.config(state=tk.DISABLED)
            return

        # Now, activate processing and begin the loop
        self.gcode_processing_active = True
        self.btn_start_gcode.config(state=tk.DISABLED) # Disable start button
        self.btn_stop_gcode.config(state=tk.NORMAL)   # Enable stop button
        self.update_gcode_status(f"Running G-code: {len(self.gcode_queue)} lines remaining.")
        print("[DEBUG GCODE] Manual start initiated. Calling _process_next_gcode_command.")
        self._process_next_gcode_command() # Start the processing loop

    def stop_gcode_execution(self):
        """Stops G-code processing and sends a stop command to the robot."""
        self.gcode_processing_active = False
        self.motion_command["x"] = 0.0 # Stop robot movement
        self.motion_command["y"] = 0.0
        self.motion_command["rotation"] = 0.0
        self.send_control_command() # Send the stop command

        self.btn_start_gcode.config(state=tk.NORMAL) # Enable start button
        self.btn_stop_gcode.config(state=tk.DISABLED) # Disable stop button
        self.update_gcode_status("G-code processing stopped by user.")
        print("[DEBUG GCODE] G-code processing stopped by user.")

        # You might want to clear the queue here if stopping means abandoning the current run
        # Or leave it if "Start" should resume from where it left off.
        # For now, let's keep it to allow resuming.

    def select_gcode_file(self):
        """
        Opens a file dialog to select a G-code (.nc, .gcode, .txt) file.
        If a file is selected, it triggers the G-code loading and parsing.
        """
        filepath = filedialog.askopenfilename(
            title="Select G-code File",
            filetypes=(("G-code files", "*.nc *.gcode"), ("Text files", "*.txt"), ("All files", "*.*"))
        )
        if filepath:
            self.gcode_status_label.config(text=f"Loading: {filepath.split('/')[-1]}")
            # Schedule the G-code loading to prevent GUI freezing during file read.
            self.master.after(100, lambda: self.load_gcode_file(filepath))

    def load_gcode_file(self, filepath):
        self.update_gcode_status("Loading G-code...")
        self.gcode_file_path.set(filepath)

        parsed_lines = []
        try:
            with open(filepath, 'r') as f:
                for line in f:
                    stripped_line = line.strip()
                    if stripped_line and not stripped_line.startswith(';'):
                        parsed_lines.append(stripped_line)
            
            print(f"[DEBUG GCODE] Successfully read {len(parsed_lines)} lines from file.")

            if parsed_lines:
                self.gcode_queue = parsed_lines # Store the loaded lines

                # Initialize G-code state for a new file load
                self.gcode_current_x = 0.0 # Reset G-code virtual position
                self.gcode_current_y = 0.0
                self.gcode_current_laser_on = False
                self.gcode_current_laser_power = 0
                # Initialize feed rate to a reasonable default, like your max (mm/min)
                self.gcode_current_feed_rate = self.ROBOT_MAX_LINEAR_VELOCITY_MM_PER_SEC 
                self.gcode_absolute_mode = True # Default to absolute mode (G90)

                self.gcode_processing_active = False # IMPORTANT: NOT ACTIVE YET
                
                self.update_gcode_status(f"Loaded {len(self.gcode_queue)} lines. Ready to start.")
                self.btn_start_gcode.config(state=tk.NORMAL) # Enable the Start button
                self.btn_stop_gcode.config(state=tk.DISABLED) # Disable the Stop button

                # --- REMOVE THESE LINES (They cause auto-start and duplicate logic) ---
                # self.gcode_processing_active = True
                # self.update_gcode_status(f"Loaded {len(self.gcode_queue)} lines. Starting...")
                # print("[DEBUG GCODE] Attempting to start G-code processing...")
                # self._process_next_gcode_command()
                # --- END REMOVAL ---

            else:
                self.update_gcode_status("No valid G-code lines found.")
                self.gcode_processing_active = False
                self.btn_start_gcode.config(state=tk.DISABLED)
                self.btn_stop_gcode.config(state=tk.DISABLED)

        except Exception as e:
            self.update_gcode_status(f"Error loading G-code: {e}")
            print(f"Error loading G-code file: {e}")
            self.gcode_processing_active = False
            self.btn_start_gcode.config(state=tk.DISABLED)
            self.btn_stop_gcode.config(state=tk.DISABLED)
            # No need for 'return' here, as the function implicitly ends.

    def _parse_gcode_line(self, line):
        """
        Parses a single G-code line and extracts relevant commands and parameters.
        Returns a dictionary representing the command.
        """
        cmd = {}
        
        # G90: Absolute positioning
        if 'G90' in line:
            cmd['type'] = 'G90'
            return cmd
        # G91: Relative positioning
        if 'G91' in line:
            cmd['type'] = 'G91'
            return cmd

        # G0, G1: Move commands (Rapid, Linear)
        move_match = re.match(r'G([01])(?: X([\-\d\.]+))?(?: Y([\-\d\.]+))?', line)
        if move_match:
            cmd['type'] = f"G{move_match.group(1)}"
            if move_match.group(2): cmd['X'] = float(move_match.group(2))
            if move_match.group(3): cmd['Y'] = float(move_match.group(3))
            
            # Look for F parameter on the same line
            f_match = re.search(r'F([\d\.]+)', line)
            if f_match: cmd['F'] = float(f_match.group(1))
            return cmd

        # F: Feed rate
        f_match = re.search(r'F([\d\.]+)', line)
        if f_match:
            cmd['type'] = 'F'
            cmd['F'] = float(f_match.group(1))
            return cmd
            
        # M3: Laser On
        if 'M3' in line:
            cmd['type'] = 'M3'
            s_match = re.search(r'S(\d+)', line) # M3 can also have S parameter on same line
            if s_match: cmd['S'] = int(s_match.group(1))
            return cmd

        # M5: Laser Off
        if 'M5' in line:
            cmd['type'] = 'M5'
            return cmd
        
        # S: Laser Power (if not part of M3)
        s_match = re.search(r'S(\d+)', line)
        if s_match and 'M3' not in line: # Avoid double-parsing S with M3
            cmd['type'] = 'S'
            cmd['S'] = int(s_match.group(1))
            return cmd

        # Add more G-code commands here as needed (e.g., G2, G3 for arcs, G28 for home)
        return None # Command not recognized

    def _process_next_gcode_command(self):
        if not self.gcode_processing_active:
            print("[DEBUG GCODE] G-code processing not active, returning.")
            return

        if not self.gcode_queue:
            print("[DEBUG GCODE] G-code queue empty. Execution complete.")
            self.update_gcode_status("G-code finished.")
            self.stop_gcode_execution() # Ensure cleanup
            return

        gcode_line = self.gcode_queue.pop(0).strip()
        print(f"[DEBUG GCODE] Processing line: '{gcode_line}'")
        self.update_gcode_status("Processing...")

        # --- Define Regex patterns here for clarity and reusability ---
        # Add (?:\s*\(.*?\))? to the end of regexes to optionally match comments in parentheses.
        # This allows commands like "G90 (use absolute coordinates)" to be parsed.

        # Regex for G1/G0 (Movement) commands, including optional X, Y, Z, F, S and comments
        match_g1_g0 = re.match(r'^(G[01])(?: X([\d.-]+))?(?: Y([\d.-]+))?(?: Z([\d.-]+))?(?: F([\d.]+))?(?: S(\d+))?(?:\s*\(.*?\))?$', gcode_line)
        
        # Regex for M3/M5 (Laser ON/OFF) commands, including optional S and comments
        match_m3_m5 = re.match(r'^(M[35])(?: S(\d+))?(?:\s*\(.*?\))?$', gcode_line)
        
        # Regex for standalone S (Laser Power) commands and comments
        s_only_match = re.match(r'^S(\d+)(?:\s*\(.*?\))?$', gcode_line)
        
        # Regex for F (Feed Rate) commands and comments
        f_match = re.match(r'^F([\d.]+)(?:\s*\(.*?\))?$', gcode_line)
        
        # Regex for G90/G91 (Absolute/Relative Positioning) and comments (already had it, but confirmed)
        g9x_match = re.match(r'^(G90|G91)(?:\s*\(.*?\))?$', gcode_line)

        # --- G-code Command Handling Logic ---

        # 1. Handle G1/G0 Commands (Movement, possibly with S for laser) - MOVED TO TOP PRIORITY
        if match_g1_g0:
            gcode_command = match_g1_g0.group(1) # G0 or G1
            target_x_str = match_g1_g0.group(2)
            target_y_str = match_g1_g0.group(3)
            feed_rate_str = match_g1_g0.group(5) # F parameter for movement speed
            s_value_in_g1_g0_str = match_g1_g0.group(6) # S parameter for laser power within G1/G0

            # Update feed rate if specified (F value in G-code is typically mm/min)
            # We convert it to mm/sec here, consistent with self.gcode_current_feed_rate
            if feed_rate_str:
                self.gcode_current_feed_rate = float(feed_rate_str) / 60.0 # Convert from mm/min to mm/second
                print(f"[DEBUG GCODE] G1/G0 with F value. Current Feed Rate: {self.gcode_current_feed_rate:.4f} mm/s")
            # Else, it uses the existing self.gcode_current_feed_rate which should be in mm/s from slider or prior F command

            # Update laser state if S value is specified in G1/G0
            if s_value_in_g1_g0_str:
                s_value = int(s_value_in_g1_g0_str)
                self.gcode_current_laser_power = s_value
                if s_value == 0:
                    self.gcode_current_laser_on = False
                else:
                    self.gcode_current_laser_on = True
                print(f"[DEBUG GCODE] G1/G0 with S value. Laser ON: {self.gcode_current_laser_on}, Power set to: {self.gcode_current_laser_power}")

            # Always ensure the latest laser state is pushed to motion_command
            self.motion_command["laser_on"] = self.gcode_current_laser_on
            self.motion_command["laser_power"] = self.gcode_current_laser_power
            
            # Handle G0/G1 commands that only change F or S, without X/Y movement
            if target_x_str is None and target_y_str is None:
                self.motion_command["x"] = 0.0
                self.motion_command["y"] = 0.0
                self.motion_command["rotation"] = 0.0
                self.send_control_command() # Send command with potentially updated laser/speed, but no movement
                if self.gcode_processing_active:
                    self.master.after(10, self._process_next_gcode_command)
                return # Handled this type of G0/G1, no further movement calculation needed

            # If X or Y are present for movement (actual move command)
            # Determine target_x and target_y based on absolute/relative mode
            target_x = float(target_x_str) if target_x_str is not None else self.gcode_current_x
            target_y = float(target_y_str) if target_y_str is not None else self.gcode_current_y

            if not self.gcode_absolute_mode: # G91 - Relative mode
                target_x += self.gcode_current_x
                target_y += self.gcode_current_y

            # Calculate movement vectors in millimetersw
            dx_mm = target_x - self.gcode_current_x
            dy_mm = target_y - self.gcode_current_y
            
            # --- Start G-code distance to robot velocity conversion (updated logic) ---
            MM_TO_M_SCALE = .0003 # Corrected: 1 millimeter = 0.001 meters

            # Convert distances to meters
            dx_m = dx_mm * MM_TO_M_SCALE
            dy_m = dy_mm * MM_TO_M_SCALE

            # Calculate total linear distance for the segment in meters
            total_distance_m = math.sqrt(dx_m**2 + dy_m**2)

            # Get the current feed rate in meters per second (self.gcode_current_feed_rate is already mm/s)
            current_feed_rate_mps = self.gcode_current_feed_rate * MM_TO_M_SCALE
            
            # Apply the GUI slider's speed factor as a global multiplier for G-code movements
            # This effectively caps the G-code commanded speed based on the GUI slider.
            # Convert slider value (0-1) to m/s based on max robot speed.
            effective_max_speed_mps = self.speed_var.get() * (self.ROBOT_MAX_LINEAR_VELOCITY_MM_PER_SEC * MM_TO_M_SCALE)
            
            # Limit the G-code commanded speed by the GUI slider's effective max speed
            if current_feed_rate_mps > effective_max_speed_mps:
                current_feed_rate_mps = effective_max_speed_mps
                print(f"[DEBUG GCODE] G-code feed rate capped by GUI slider to {current_feed_rate_mps:.4f} m/s")

            # Handle very small distances (effectively no movement)
            if total_distance_m < 1e-6: # Treat very small distances as no movement (e.g., less than 1 micrometer)
                print("[DEBUG GCODE] G0/G1: Very small movement (distance < 1e-6 m). Skipping actual move.")
                self.motion_command["x"] = 0.0
                self.motion_command["y"] = 0.0
                self.motion_command["rotation"] = 0.0
                self.send_control_command() # Send zero velocity
                if self.gcode_processing_active:
                    self.master.after(10, self._process_next_gcode_command)
                return
            
            # If a move is intended, but feed rate is zero or too small, assign a minimum velocity
            if current_feed_rate_mps < 1e-6: # Check if feed rate is effectively zero
                # Use a small default speed for non-zero distance moves if feed rate is 0
                # This ensures the robot still moves, albeit slowly, rather than stalling.
                current_feed_rate_mps = 0.005 # m/s - a sensible minimum velocity
                print("[DEBUG GCODE] G-code feed rate too low. Using minimum speed for movement.")

            # Calculate the time this segment should take based on distance and actual speed
            segment_duration_s = total_distance_m / current_feed_rate_mps
            
            # Calculate the component velocities (m/s) that the robot should execute
            self.motion_command["x"] = dx_m / segment_duration_s
            self.motion_command["y"] = dy_m / segment_duration_s
            self.motion_command["rotation"] = 0.0 # Linear G0/G1 moves have no rotation

            print(f"[DEBUG GCODE] Moving dx={dx_m:.4f}m, dy={dy_m:.4f}m at {current_feed_rate_mps:.4f} m/s for {segment_duration_s:.4f}s.")

            # Send the command (velocity) to the Arduino
            self.send_control_command()

            # Schedule the next G-code command *after* the calculated segment duration.
            # We call _stop_robot_and_continue_gcode to ensure the robot stops after
            # this segment, then proceeds to the next G-code command.
            delay_ms = int(segment_duration_s * 1000)
            delay_ms = max(50, delay_ms) # Ensure a minimum delay, e.g., 50ms, to allow command transmission

            print(f"[DEBUG GCODE] Scheduled move duration: {delay_ms}ms. Next command after robot stops.")
            self.master.after(delay_ms, self._stop_robot_and_continue_gcode)
            # --- End G-code distance to robot velocity conversion ---
            
            # Update current G-code position after a move is processed.
            # This should happen *before* the function returns for the next iteration.
            self.gcode_current_x = target_x
            self.gcode_current_y = target_y
            
            return # Exit this call, as the next step is already scheduled by master.after

        # 2. Handle M3/M5 (Laser ON/OFF) Commands
        elif match_m3_m5:
            gcode_command = match_m3_m5.group(1)
            s_value_str = match_m3_m5.group(2) # S value for M3

            if gcode_command == 'M3':
                self.gcode_current_laser_on = True
                self.gcode_current_laser_power = int(s_value_str) if s_value_str else 255
                print(f"[DEBUG GCODE] Matched M3. Laser ON. Power:{self.gcode_current_laser_power}")
            elif gcode_command == 'M5':
                self.gcode_current_laser_on = False
                self.gcode_current_laser_power = 0
                print(f"[DEBUG GCODE] Matched M5. Laser OFF.")
            
            self.laser_on.set(self.gcode_current_laser_on)
            self.current_laser_power.set(self.gcode_current_laser_power)
            
            self.motion_command["x"] = 0.0
            self.motion_command["y"] = 0.0
            self.motion_command["rotation"] = 0.0
            self.motion_command["laser_on"] = self.gcode_current_laser_on
            self.motion_command["laser_power"] = self.gcode_current_laser_power
            self.send_control_command()
            
            if self.gcode_processing_active:
                self.master.after(10, self._process_next_gcode_command)
            return

        # 3. Handle Standalone S Commands (Laser Power)
        elif s_only_match:
            s_value = int(s_only_match.group(1))
            self.gcode_current_laser_power = s_value
            
            if s_value == 0:
                self.gcode_current_laser_on = False
            else:
                self.gcode_current_laser_on = True
            
            print(f"[DEBUG GCODE] Matched Standalone S. Laser ON: {self.gcode_current_laser_on}, Power: {self.gcode_current_laser_power}")
            
            self.laser_on.set(self.gcode_current_laser_on)
            self.current_laser_power.set(self.gcode_current_laser_power) # Completed this line
            
            self.motion_command["x"] = 0.0
            self.motion_command["y"] = 0.0
            self.motion_command["rotation"] = 0.0
            self.motion_command["laser_on"] = self.gcode_current_laser_on
            self.motion_command["laser_power"] = self.gcode_current_laser_power
            self.send_control_command()
            
            if self.gcode_processing_active:
                self.master.after(10, self._process_next_gcode_command)
            return

        # 4. Handle F Commands (Feed Rate)
        elif f_match:
            feed_rate = float(f_match.group(1))
            self.gcode_current_feed_rate = feed_rate / 60.0 # Convert from mm/min to mm/second
            print(f"[DEBUG GCODE] Matched F. Current Feed Rate: {self.gcode_current_feed_rate:.4f} mm/s")
            # F commands do not involve immediate robot motion, just update state
            
            if self.gcode_processing_active:
                self.master.after(10, self._process_next_gcode_command)
            return

        # 5. Handle G90/G91 Commands (Absolute/Relative Positioning)
        elif g9x_match:
            command = g9x_match.group(1)
            if command == 'G90':
                self.gcode_absolute_mode = True
                print("[DEBUG GCODE] Matched G90. Absolute positioning mode.")
            elif command == 'G91':
                self.gcode_absolute_mode = False
                print("[DEBUG GCODE] Matched G91. Relative positioning mode.")
            # G90/G91 commands do not involve immediate robot motion, just update state
            
            if self.gcode_processing_active:
                self.master.after(10, self._process_next_gcode_command)
            return

        else:
            # For unrecognized commands or comments, simply skip and process the next line quickly.
            print(f"[DEBUG GCODE] Unrecognized G-code command: '{gcode_line}'. Skipping.")
            self.update_gcode_status(f"Unknown G-code: {gcode_line}")

        # Always schedule the next command if not already returned by a specific handler
        # This line ensures that even if a line is unrecognized, processing continues.
        if self.gcode_processing_active:
            self.master.after(10, self._process_next_gcode_command)


    def _stop_robot_and_continue_gcode(self):
        # Ensure robot is stopped after a move segment
        self.motion_command["x"] = 0.0
        self.motion_command["y"] = 0.0
        self.motion_command["rotation"] = 0.0
        self.send_control_command()
        print("[DEBUG GCODE] Robot stopped after segment.")

        # Continue processing the G-code queue after stopping
        if self.gcode_processing_active:
            self.master.after(10, self._process_next_gcode_command)

    def send_control_command(self):
        print(f"  [DEBUG] Entering send_control_command. is_sending_command: {self.is_sending_command}")

        # Ensure a serial port is connected (this is the serial connection to the desktop Nano bridge)
        if not self.arduino_connected or not self.serial_port or not self.serial_port.is_open:
            print("  [DEBUG] send_control_command: Serial port to desktop Nano not connected or open. Cannot send command.")
            self.radio_status.set("Bridge Not Connected") # Update status to reflect bridge
            return

        # Add this check back for safety against rapid calls if one is already in progress
        if self.is_sending_command:
            print("  [DEBUG] send_control_command: Command already in progress, skipping send.")
            return
        
        self.is_sending_command = True # Set flag to prevent re-entry
        print("  [DEBUG] is_sending_command set to True.")

        try:
            motion_x = self.motion_command["x"]
            motion_y = self.motion_command["y"]
            rotation = self.motion_command["rotation"]
            
            # --- THIS IS THE CRUCIAL CORRECTION FOR LASER CONTROL SOURCE ---
            # Get laser state and power from self.motion_command.
            # This dictionary is updated by both G-code commands (M3/M5) and manual controls.
            # Use .get() with a default value for robustness, in case a key isn't always present.
            laser = int(self.motion_command.get("laser_on", False)) # Get boolean, convert to 0 or 1
            laser_power = self.motion_command.get("laser_power", 0) # Get integer power, default to 0
            # --- END OF CORRECTION ---

            # The speed slider's value (0.0-1.0) is the `target_speed`
            # This should still come from the speed_var as it's a UI slider value
            target_speed = self.speed_var.get()

            # Construct the command string in the ORIGINAL WORKING FORMAT (that the Arduino expects)
            # Corrected: Removed extra commas after 'laser' and 'laser_power'
            command_string = (
                f"MX:{motion_x: .8f},"
                f"MY:{motion_y: .8f},"
                f"R:{rotation: .8f},"
                f"L:{laser},"       # Corrected: Removed trailing comma
                f"P:{laser_power}," # Corrected: Removed trailing comma
                f"S:{target_speed: .8f}\n" # Add newline for string termination
            )

            print(f"  [DEBUG] Attempting to send (via serial bridge): {command_string.strip()}")
            self.serial_port.write(command_string.encode('utf-8')) # Encode and send
            self.radio_status.set(f"Bridge Sent: {command_string.strip()}") # Update status
            print(f"  [SENT VIA BRIDGE] {command_string.strip()}") # CONFIRMATION PRINT

        except serial.SerialException as e:
            print(f"  [ERROR] Serial communication error during send to bridge: {e}")
            self.radio_status.set(f"Bridge Serial Error: {e}")
        except Exception as e:
            print(f"  [ERROR] send_control_command failed unexpectedly: {e}")
            self.radio_status.set(f"Command Error: {e}")
        finally:
            # Ensure the flag is reset in all cases (success or failure)
            self.is_sending_command = False
            print("  [DEBUG] is_sending_command set to False (finally block).")

    def update_gcode_status(self, message):
        """
        Updates the G-code status display and prints to console.
        Requires self.gcode_status_label to be initialized to a Tkinter Label widget.
        """
        if self.gcode_status_label: # Check if the label widget exists
           self.gcode_status_label.config(text=f"G-code Status: {message}")
        print(f"G-code Status: {message}") # Always print to console for debugging

if __name__ == '__main__':
    root = tk.Tk()
    gui = robotDirector(root)
    root.mainloop()


