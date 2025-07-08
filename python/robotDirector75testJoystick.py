import tkinter as tk
from tkinter import ttk
from tkinter import filedialog # Import filedialog
import serial
import time
import re
import math # Import math for sqrt
import socket
import json

class robotDirector:

    def __init__(self, master): 
        self.master = master
        master.title("Lyttle ReSearch Robot Director")
        master.config(bg="lightgreen")
        self.current_laser_power = tk.IntVar(master, value=0)
        self.laser_on = tk.BooleanVar(master, value=False)
        self.radio_status = tk.StringVar(master, value="Idle")
        self.motion_command = {"x": 0.0, "y": 0.0, "rotation": 0.0}
        self.north_angle = 0.0
        self.control_source = "keyboard"
        self.spacebar_pressed = False
        self.serial_port = None
        self.arduino_connected = False
        self.current_control_method = None
        self.speed_var = tk.DoubleVar(master, value=0.5)
        self.is_moving = {
            "forward": False,
            "backward": False,
            "left": False,
            "right": False,
            "CCW": False,
            "CW": False,}
        self.create_widgets()
        self.is_sending_command = False
        self.master.bind('<KeyPress>', self.read_keyboard)
        self.master.bind('<KeyRelease>', self.read_keyrelease)
        self.master.bind('<FocusIn>', self.focus_change_handler, add='+')
        self.master.bind('<FocusOut>', self.focus_change_handler, add='+')
        self.update_radio_status("Disconnected")
        self.port_name = '/dev/ttyUSB0'
        self.baud_rate = 115200
        self.connect_arduino_serial()
                # --- G-code Specific State Variables ---
        self.gcode_queue = [] # Queue to hold parsed G-code commands
        self.gcode_current_x = 0.0 # Robot's current X position in G-code coordinates (e.g., mm)
        self.gcode_current_y = 0.0 # Robot's current Y position in G-code coordinates (e.g., mm)
        self.gcode_current_laser_on = False
        self.gcode_current_laser_power = 0 # 0-255 scale
        self.gcode_current_feed_rate = 190.0 # Default feed rate in G-code units (e.g., mm/min)
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
        self._connect_to_joystick_server() # <<< ADD THIS LINE
        self.joystick_data_buffer = ''

        # --- Robot Physical Constants for G-code Scaling (Adjust these based on your robot's capabilities) ---
        # Define the maximum linear velocity your robot can physically achieve at speed_var=1.0.
        # This is CRITICAL for correctly mapping G-code F-values to your robot's control system.
        # Example: if your robot moves 50mm/second when speed_var is 1.0 in the director, and Arduino scales accordingly.
        self.ROBOT_MAX_LINEAR_VELOCITY_MM_PER_SEC = 50.0 # Adjust this to your robot's actual max linear speed in mm/sec

    # --- New Client-side Joystick Connection Method ---
    def _connect_to_joystick_server(self):
        """Attempts to connect to the joystick server as a client."""
        if self.client_connection:
            print("Already connected to joystick server.")
            return

        print(f"Attempting to connect to joystick server at {self.joystick_host}:{self.joystick_port}...")
        try:
            self.client_connection = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.client_connection.settimeout(1.0) # Set a timeout for the connection attempt
            self.client_connection.connect((self.joystick_host, self.joystick_port))
            self.client_connection.setblocking(False) # Make the client socket non-blocking for data reception
            print(f"Successfully connected to joystick server at {self.joystick_host}:{self.joystick_port}")
            self.joystick_data_buffer = b'' # Clear buffer for new connection
            
            # Start receiving data from the connected server
            self.master.after(10, self._receive_joystick_data) # Start polling for data
            # Update GUI status to show joystick connection
            self.update_radio_status("Joystick Connected") # Or a new status if you prefer

        except socket.timeout:
            print(f"Connection to joystick server timed out after 1 second.")
            self._close_joystick_client_connection() # Use the new close function
            self.master.after(2000, self._connect_to_joystick_server) # Retry after 2 seconds
        except socket.error as e:
            print(f"Error connecting to joystick server: {e}")
            self._close_joystick_client_connection() # Use the new close function
            self.master.after(2000, self._connect_to_joystick_server) # Retry after 2 seconds
        except Exception as e:
            print(f"Unexpected error during joystick client connection: {e}")
            self._close_joystick_client_connection() # Use the new close function
            self.master.after(2000, self._connect_to_joystick_server) # Retry after 2 seconds

    # --- Modified Close Connection Method (for client) ---
    def _close_joystick_client_connection(self):
        """Closes the joystick client connection."""
        if self.client_connection:
            self.client_connection.close()
            self.client_connection = None
            print("Joystick client connection closed.")
            self.update_radio_status("Joystick Disconnected") # Update status

    # --- You can remove or comment out these server-side methods ---
    # def _setup_joystick_listener(self): ...
    # def _accept_joystick_connection(self): ...
    # (The _receive_joystick_data method's internal logic is mostly reusable, but its scheduling needs to be linked to _connect_to_joystick_server)

    # --- Modify your create_joystick_control_area if you want a connect button in the GUI ---
    def create_joystick_control_area(self, parent_frame, event=None):
        ttk.Label(parent_frame, text="Joystick Control", background="lightgray").pack(padx=10, pady=10)
        # Optional: Add a connect/disconnect button here
        ttk.Button(parent_frame, text="Connect to Joystick Server", command=self._connect_to_joystick_server).pack(pady=5)
        ttk.Button(parent_frame, text="Disconnect Joystick", command=self._close_joystick_client_connection).pack(pady=5)

    # --- New Methods for Joystick Control ---

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
        """
        Receives and processes joystick data from the connected server.
        This method is designed to be called repeatedly by self.master.after().
        """
        if not self.client_connection:
            # If for some reason the connection isn't active, stop trying to receive.
            # print("Joystick client connection not active for receiving.") # Uncomment for debugging
            return

        try:
            # Attempt to receive data once. Will raise BlockingIOError if no data.
            data = self.client_connection.recv(self.joystick_buffer_size)

            if not data:
                # Client disconnected gracefully (received 0 bytes)
                print("Joystick client disconnected (graceful shutdown by server).")
                self._close_joystick_client_connection()
                return # Stop rescheduling this method

            # Decode and append to buffer for accumulating partial messages
            self.joystick_data_buffer += data.decode('utf-8')

            # Process all complete messages in the buffer (messages are newline-separated JSON)
            while '\n' in self.joystick_data_buffer:
                message_string, self.joystick_data_buffer = self.joystick_data_buffer.split('\n', 1)
                
                if not message_string.strip(): # Skip empty strings from split
                    continue

                # --- Your existing data processing logic for a single JSON message ---
                # print(f"Robot: Raw JSON received: '{message_string}'") # Uncomment for debugging

                try:
                    joystick_data = json.loads(message_string)
                    # print(f"Robot: Parsed joystick_data: {joystick_data}") # Uncomment for debugging

                    current_speed_factor = self.speed_var.get()
                    # print(f"Robot: Current Speed Factor: {current_speed_factor}") # Uncomment for debugging

                    self.motion_command["x"] = float(joystick_data.get("x", 0.0)) * current_speed_factor
                    self.motion_command["y"] = float(joystick_data.get("y", 0.0)) * current_speed_factor
                    self.motion_command["rotation"] = float(joystick_data.get("r", 0.0)) * current_speed_factor
                    self.motion_command["elevation"] = float(joystick_data.get("e", 0.0)) * current_speed_factor

                    # print(f"Robot: Motion Command (X, Y, R, E): ({self.motion_command['x']:.3f}, {self.motion_command['y']:.3f}, {self.motion_command['rotation']:.3f}, {self.motion_command['elevation']:.3f})") # Uncomment for debugging

                    # Handle laser and power (toggles)
                    laser_on_value = bool(joystick_data.get("laser", 0))
                    if self.laser_on.get() != laser_on_value:
                        self.laser_on.set(laser_on_value)
                        print(f"Laser {'ON' if laser_on_value else 'OFF'} (from joystick)")

                    power_on_value = bool(joystick_data.get("power", 0))
                    # You would integrate this with your robot's power control logic
                    if power_on_value != 0:
                        # print("Robot Power ON (from joystick)") # Uncomment for debugging
                        pass # Implement actual robot power control here
                    else:
                        # print("Robot Power OFF (from joystick)") # Uncomment for debugging
                        pass # Implement actual robot power control here

                    # Send the combined control command to the robot
                    self.send_control_command()
                    # print("Robot: send_control_command() called for joystick data.") # Uncomment for debugging

                except json.JSONDecodeError as e:
                    print(f"Robot: Received invalid JSON data: {e} - '{message_string}'")
                except ValueError as e:
                    print(f"Robot: Data conversion error: {e} in '{message_string}'")
                except Exception as e:
                    print(f"Robot: Error processing joystick data: {e}")

        except BlockingIOError:
            # This is expected when no data is currently available on a non-blocking socket.
            # Just means we'll check again in the next scheduled call.
            pass
        except socket.error as e:
            # Handle actual socket errors (e.g., connection reset by peer)
            print(f"Socket error during joystick data reception: {e}")
            self._close_joystick_client_connection() # Close connection on error
            return # Stop rescheduling this method
        except Exception as e:
            print(f"Unexpected error in _receive_joystick_data: {e}")
            self._close_joystick_client_connection() # Close connection on error
            return # Stop rescheduling this method
        
        # Schedule the next check for joystick data
        # This is how the method repeatedly polls for data without blocking the GUI
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

    def read_keyrelease(self, event):
        keysym = event.keysym.lower()
        print(f"read_keyrelease: event.keysym = {keysym!r}")

        if keysym == 'space':
            self.laser_on.set(False)
            self.send_control_command()
            self.spacebar_pressed = False
            return

        if self.current_control_method == "Direct X/Y/R Buttons (Legacy)":
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

            # Check if *any* movement key is still pressed.  If not, *then* stop.
            if not any(self.is_moving.values()):
                self.motion_command["x"] = 0.0
                self.motion_command["y"] = 0.0
                self.motion_command["rotation"] = 0.0
                self.send_control_command()

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
        try:
            self.serial_port = serial.Serial(self.port_name, self.baud_rate, timeout=1)
            print(f"Connected to Arduino on {self.port_name} at {self.baud_rate} baud.")
            self.arduino_connected = True
            self.update_radio_status("Connected")  # Update status to indicate connection
        except serial.SerialException as e:
            print(f"Error connecting to Arduino on {self.port_name}: {e}")
            self.serial_port = None  # Ensure serial_port is None if connection fails
            self.arduino_connected = False
            self.update_radio_status("Disconnected")  # Update status to indicate disconnection

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
            self.master.after(500, self._send_repeated_command, direction, speed)  # Repeat every 100ms

    def send_control_command(self):
        """This function packages and sends the control command over serial."""
        motion_x = self.motion_command["x"]
        motion_y = self.motion_command["y"]
        rotation = self.motion_command["rotation"]
        laser = int(self.laser_on.get())
        laser_power = self.current_laser_power.get()
        target_speed = self.speed_var.get()  # Get the current target speed

        command_string = f"MX:{motion_x:.2f},MY:{motion_y:.2f},R:{rotation:.2f},L:{laser},P:{laser_power},S:{target_speed:.2f}\n"
        print(f"Sending command to Arduino: {command_string.strip()}")

        if self.arduino_connected and self.serial_port:
            try:
                self.serial_port.write(command_string.encode('utf-8'))
                self.update_radio_status("Success")
            except serial.SerialException as e:
                print(f"Error sending data via serial: {e}")
                self.update_radio_status("Failure")
        else:
            print("Not connected to Arduino via serial.")
            self.update_radio_status("Disconnected")

    def control_style_changed(self, event=None):
        selected_style = self.control_style_dropdown.get()
        self.current_control_method = selected_style
        # Clear any existing widgets in the dynamic area
        for widget in self.dynamic_control_area.winfo_children():
            widget.destroy()
        if selected_style in self.control_styles_dict:
            handler = self.control_styles_dict[selected_style]
            handler(self.dynamic_control_area, event)  # Pass both parent and event
        else:
            not_implemented_label = ttk.Label(self.dynamic_control_area,
                                            text=f"{selected_style} - Not Implemented Yet",
                                            style="TLabel")
            not_implemented_label.pack(pady=20, padx=20)

        # For the "Legacy" style, ensure the current control method is set
        if selected_style == "Direct X/Y/R Buttons (Legacy)":
            self.current_control_method = "Direct X/Y/R Buttons (Legacy)"

    def create_widgets(self):
        # --- Styling ---
        style = ttk.Style()
        style.configure("TFrame", background="lightgreen")
        style.configure("TLabelframe", background="lightgreen")
        style.configure("TLabelframe.Label", background="lightgreen")
        style.configure("TLabel", background="lightgreen")
        style.configure("Emergency.TButton", foreground="white", background="red")
        style.map("Emergency.TButton",
                  foreground=[('active', 'white')],
                  background=[('active', 'darkred')])
        style.configure("Success.TLabel", foreground="green")
        style.configure("Failure.TLabel", foreground="red")
        style.configure("Busy.TLabel", foreground="orange")
        style.configure("Idle.TLabel", foreground="black")
        style.configure("Disconnected.TLabel", foreground="gray")

        # --- Top Frame for Emergency Stop and Radio Status ---
        top_frame = ttk.Frame(self.master, style="TFrame")
        top_frame.pack(pady=10, padx=10, fill="x")

        self.emergency_button = ttk.Button(top_frame, text="EMERGENCY STOP", command=self.emergency_stop,
                                            style="Emergency.TButton")
        self.emergency_button.pack(side=tk.LEFT)

        self.radio_status_label = ttk.Label(top_frame, textvariable=self.radio_status, style="Idle.TLabel")
        self.radio_status_label.pack(side=tk.RIGHT, padx=5)
        ttk.Label(top_frame, text="Radio Status:", background="lightgreen").pack(side=tk.RIGHT)

        # --- Control Style Selection ---
        control_frame = ttk.LabelFrame(self.master, text="Directorial Style", style="TLabelframe")
        control_frame.pack(padx=10, pady=10, fill="x")

        # Modify self.control_styles list:
        self.control_styles = ["Direct X/Y/R Buttons", "Circle Click Control", "Python Script Director",
                               "Tarantino as Director", "Frickin Shoot Everyone Director", ".dxf Director",
                               ".jpg (Python Contour) Director", ".ngc/.nc/.gcode Director", "SVG/BMP Director",
                               "Joystick Control", "Placeholder Style"] # Added "Joystick Control"

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
            "Joystick Control": self.create_joystick_control_area, # New entry for joystick
            "Placeholder Style": self.create_placeholder_style,
        }
        self.control_style_dropdown = ttk.Combobox(control_frame, values=self.control_styles, state="readonly")
        self.control_style_dropdown.set("Direct X/Y/R Buttons")  # Set default
        self.control_style_dropdown.pack(pady=5, padx=5, fill="x")
        self.control_style_dropdown.bind("<<ComboboxSelected>>", self.control_style_changed)

        # --- Dynamic Control Area ---
        self.dynamic_control_area = ttk.Frame(self.master, style="TFrame")
        self.dynamic_control_area.pack(padx=10, pady=10, fill="both", expand=True)

        self.control_style_changed()  # Call to set up initial style
        self.create_keyboard_control_area(self.dynamic_control_area) #added. call the fun

        # --- Laser Control Frame ---
        laser_frame = ttk.LabelFrame(self.master, text="Laser Control", style="TLabelframe")
        laser_frame.pack(padx=10, pady=10, fill="x")

        ttk.Button(laser_frame, text="Activate Laser", command=self.activate_laser_button).pack(side=tk.LEFT, padx=5)
        ttk.Button(laser_frame, text="Deactivate Laser", command=self.deactivate_laser_button).pack(side=tk.LEFT, padx=5)

        power_scale_frame = ttk.Frame(laser_frame, style="TFrame")
        power_scale_frame.pack(side=tk.LEFT, padx=5)
        ttk.Label(power_scale_frame, text="Power:", background="lightgreen").pack(side=tk.LEFT)
        self.power_label = ttk.Label(power_scale_frame,
                                      text=f"Power: {self.current_laser_power.get()}",
                                      background="lightgreen")
        self.power_label.pack(side=tk.LEFT)
        self.laser_power_slider = tk.Scale(laser_frame, from_=0, to=255, orient=tk.HORIZONTAL,
                                            command=self.set_laser_power)
        self.laser_power_slider.set(0)  # Initialize to 0
        self.laser_power_slider.pack(fill="x", padx=5)

        # --- Robot Location Initialization ---
        localization_frame = ttk.LabelFrame(self.master, text="Robot Location", style="TLabelframe")
        localization_frame.pack(padx=10, pady=10, fill="x")

        ttk.Button(localization_frame, text="Initialize Location", command=self.initiate_localization).pack(padx=5,
                                                                                                        pady=5)

    def create_keyboard_control_area(self, parent_frame, event=None): #moved out
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

        # Add the speed slider
        speed_frame = ttk.Frame(parent_frame, style="TFrame")
        speed_frame.pack(pady=5, padx=5, fill="x")
        speed_label = ttk.Label(speed_frame, text="Speed:", style="TLabel")
        speed_label.pack(side=tk.LEFT, padx=5)
        self.speed_slider = tk.Scale(speed_frame, from_=0.0, to=1.0, resolution=0.01, orient=tk.HORIZONTAL,
                                     label="Target Speed", variable=self.speed_var)
        self.speed_slider.pack(fill="x", padx=5)

        self.current_control_method = "Direct X/Y/R Buttons (Legacy)"    

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
        if self.current_control_method == "Direct X/Y/R Buttons (Legacy)":
            vx, vy, omega = 0.0, 0.0, 0.0
            speed = self.speed_var.get()
            if event.char == 'w':
                vy = speed
                self.is_moving['forward'] = True
            elif event.char == 's':
                vy = -speed
                self.is_moving['backward'] = True
            elif event.char == 'd':
                vx = speed
                self.is_moving['right'] = True
            elif event.char == 'a':
                vx = -speed
                self.is_moving['left'] = True
            elif event.char == 'q':
                omega = speed
                self.is_moving['CCW'] = True
                self.north_angle = (self.north_angle + 5) % 360  # Example rotation update
                print(f"North Angle: {self.north_angle}")
            elif event.char == 'e':
                omega = -speed
                self.is_moving['CW'] = True
                self.north_angle = (self.north_angle - 5) % 360  # Example rotation update
                print(f"North Angle: {self.north_angle}")
            elif event.char == ' ':
                self.laser_on.set(True)
                self.spacebar_pressed = True
            else:
                return  # No relevant key pressed

            self.motion_command["x"] = vx
            self.motion_command["y"] = vy
            self.motion_command["rotation"] = omega
            self.send_control_command()
            self.update_radio_status("Success")  # Assume KB input is successful
            self.master.after(100, self.reset_radio_status)
        elif event.char == ' ':  # Spacebar laser for other control methods
            self.laser_on.set(True)
            self.spacebar_pressed = True

    def create_gcode_director(self, parent_frame, event=None):
        """
        Creates the UI for loading and running G-code files.
        """
        for widget in parent_frame.winfo_children():
            widget.destroy() # Clear previous widgets

        ttk.Label(parent_frame, text="Load .nc/.gcode file to run laser engraving path.",
                  background="lightgreen").pack(padx=10, pady=10)

        load_button = ttk.Button(parent_frame, text="Load and Run G-code",
                                 command=self.select_gcode_file) # Calls a file dialog
        load_button.pack(pady=5)

        # A label to show the status of G-code loading/execution
        self.gcode_status_label = ttk.Label(parent_frame, text="Status: Ready", background="lightgreen")
        self.gcode_status_label.pack(pady=5)

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
        """
        Reads and parses the G-code file, populating the internal G-code queue.
        Initializes G-code state variables.
        """
        self.gcode_queue = []
        self.gcode_current_x = 0.0 # Reset robot's assumed position to origin
        self.gcode_current_y = 0.0
        self.gcode_current_laser_on = False
        self.gcode_current_laser_power = 0
        self.gcode_current_feed_rate = 190.0 # Reset to default for the file
        self.gcode_absolute_mode = True # G90 is typically default for start

        try:
            with open(filepath, 'r') as f:
                for line_num, line in enumerate(f, 1):
                    line = line.strip().upper() # Convert to uppercase for consistent parsing
                    if not line or line.startswith('(') or line.startswith(';'):
                        continue # Skip empty lines and comments

                    # Parse G-code commands
                    parsed_command = self._parse_gcode_line(line)
                    if parsed_command:
                        self.gcode_queue.append(parsed_command)
                    else:
                        print(f"Warning: Unrecognized G-code line {line_num}: {line}")
            
            self.gcode_status_label.config(text=f"Loaded {len(self.gcode_queue)} commands. Ready to Run.")
            print(f"G-code file loaded. {len(self.gcode_queue)} commands queued.")
            
            # Start processing the queue if not already active
            if not self.gcode_processing_active:
                self.gcode_processing_active = True
                self.master.after(100, self._process_next_gcode_command) # Start the G-code execution loop

        except Exception as e:
            self.gcode_status_label.config(text=f"Error loading file: {e}")
            print(f"Error loading G-code file: {e}")
            self.gcode_processing_active = False # Ensure flag is reset on error

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
        """
        Processes one command from the G-code queue. This function is called recursively
        using Tkinter's after() to prevent blocking the GUI.
        """
        if not self.gcode_queue:
            self.gcode_status_label.config(text="G-code execution finished.")
            self.update_radio_status("Idle")
            self.gcode_processing_active = False
            self.emergency_stop() # Ensure robot stops at the end of the job
            print("G-code processing complete. Robot stopped.")
            return

        current_cmd = self.gcode_queue.pop(0) # Get the next command from the front of the queue
        print(f"Executing command: {current_cmd}")
        self.gcode_status_label.config(text=f"Executing: {current_cmd.get('type')}...")
        
        delay_ms = 0 # Default delay until next command

        if current_cmd['type'] == 'G90':
            self.gcode_absolute_mode = True
            print("  Set Absolute Positioning (G90)")
        elif current_cmd['type'] == 'G91':
            self.gcode_absolute_mode = False
            print("  Set Relative Positioning (G91)")
        elif current_cmd['type'] == 'F':
            self.gcode_current_feed_rate = current_cmd['F']
            print(f"  Set Feed Rate: {self.gcode_current_feed_rate} mm/min")
        elif current_cmd['type'] == 'S':
            self.gcode_current_laser_power = current_cmd['S']
            self.gcode_current_laser_on = (self.gcode_current_laser_power > 0)
            self.laser_on.set(self.gcode_current_laser_on) # Update UI
            self.current_laser_power.set(self.gcode_current_laser_power) # Update UI
            self.send_control_command() # Send laser state update
            print(f"  Set Laser Power: {self.gcode_current_laser_power}, On: {self.gcode_current_laser_on}")
        elif current_cmd['type'] == 'M3':
            self.gcode_current_laser_on = True
            if 'S' in current_cmd: # M3 can optionally carry an S parameter
                self.gcode_current_laser_power = current_cmd['S']
            else: # If no S, assume full power or last set power for M3
                if self.gcode_current_laser_power == 0: self.gcode_current_laser_power = 255 # Default to full if off
            self.laser_on.set(True) # Update UI
            self.current_laser_power.set(self.gcode_current_laser_power) # Update UI
            self.send_control_command() # Send laser state update
            print(f"  Laser M3 On (Power: {self.gcode_current_laser_power})")
        elif current_cmd['type'] == 'M5':
            self.gcode_current_laser_on = False
            self.gcode_current_laser_power = 0
            self.laser_on.set(False) # Update UI
            self.current_laser_power.set(0) # Update UI
            self.send_control_command() # Send laser state update
            print("  Laser M5 Off")
        elif current_cmd['type'] in ['G0', 'G1']:
            target_x = current_cmd.get('X', self.gcode_current_x)
            target_y = current_cmd.get('Y', self.gcode_current_y)

            # Convert to absolute coordinates if currently in relative mode
            if not self.gcode_absolute_mode: # G91 active
                target_x = self.gcode_current_x + target_x
                target_y = self.gcode_current_y + target_y

            dx = target_x - self.gcode_current_x
            dy = target_y - self.gcode_current_y
            distance_mm = math.sqrt(dx**2 + dy**2)

            # Determine the speed for this move
            # If F is specified on the G0/G1 line, use it; otherwise use the last set F
            move_feed_rate_mm_per_min = current_cmd.get('F', self.gcode_current_feed_rate)
            if current_cmd['type'] == 'G0': # Rapid move overrides F for speed
                move_feed_rate_mm_per_min = self.ROBOT_MAX_LINEAR_VELOCITY_MM_PER_SEC * 60 # Rapid moves at max speed
                
            move_linear_vel_mm_per_sec = move_feed_rate_mm_per_min / 60.0

            # Calculate the time duration for this move segment (in milliseconds)
            if move_linear_vel_mm_per_sec > 0 and distance_mm > 0:
                delay_ms = int((distance_mm / move_linear_vel_mm_per_sec) * 1000)
            else:
                delay_ms = 0 # No movement or zero speed means no delay for movement

            # Calculate normalized velocity components for motion_command["x"] and "y"
            # These represent the direction and will be multiplied by speed_var on Arduino side
            normalized_vx = 0.0
            normalized_vy = 0.0
            if distance_mm > 0:
                normalized_vx = dx / distance_mm
                normalized_vy = dy / distance_mm

            # Set the speed_var that will be sent to Arduino's S parameter
            # This maps the G-code feed rate to your robot's effective speed (0.0-1.0)
            target_speed_var = 0.0
            if self.ROBOT_MAX_LINEAR_VELOCITY_MM_PER_SEC > 0:
                target_speed_var = min(1.0, move_linear_vel_mm_per_sec / self.ROBOT_MAX_LINEAR_VELOCITY_MM_PER_SEC)
            
            # Update the UI speed slider (optional, but good for visual feedback)
            self.speed_var.set(target_speed_var)

            # Send command to Arduino
            self.motion_command["x"] = normalized_vx
            self.motion_command["y"] = normalized_vy
            self.motion_command["rotation"] = 0.0 # G-code linear moves typically don't involve robot rotation
            self.laser_on.set(self.gcode_current_laser_on) # Update laser state
            self.current_laser_power.set(self.gcode_current_laser_power) # Update laser power
            self.send_control_command() # Sends current motion and laser state

            print(f"  Move {current_cmd['type']} to X:{target_x:.2f} Y:{target_y:.2f}, Speed Factor: {target_speed_var:.2f}, Laser: {self.gcode_current_laser_on}")

            # Update robot's internal position after the move
            self.gcode_current_x = target_x
            self.gcode_current_y = target_y
            
            # Set minimum delay to avoid rapid successive commands causing issues or visual flickering
            if delay_ms < 50: # Minimum 50ms delay for very short segments
                delay_ms = 50
                # If a very short move, the actual speed will be higher than F
                # If this happens frequently, you might want to adjust your ROBOT_MAX_LINEAR_VELOCITY_MM_PER_SEC
                # or consider a more advanced motion planner.
                # print("  Adjusted segment delay to minimum 50ms.")

            # After the move duration, send a stop command to ensure it stops.
            # Then, schedule the next G-code command.
            self.master.after(delay_ms, self._stop_robot_and_continue_gcode)
            return # Exit this call, wait for the scheduled continuation

        # If a command was processed, schedule the next one.
        # This will be called after a very short delay for non-motion commands,
        # or after `_stop_robot_and_continue_gcode` for motion commands.
        if self.gcode_processing_active: # Only if still active (not emergency stopped etc.)
            self.master.after(10, self._process_next_gcode_command) # Process next command quickly

    def _stop_robot_and_continue_gcode(self):
        """
        Helper to stop robot motion after a G-code segment and then continue processing the queue.
        """
        # Ensure robot is stopped after a move segment
        self.motion_command["x"] = 0.0
        self.motion_command["y"] = 0.0
        self.motion_command["rotation"] = 0.0
        self.send_control_command()
        print("  Robot stopped after segment.")

        # Continue processing the G-code queue after stopping
        if self.gcode_processing_active:
            self.master.after(10, self._process_next_gcode_command)

if __name__ == '__main__':
    root = tk.Tk()
    gui = robotDirector(root)
    root.mainloop()


