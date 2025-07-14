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
import queue

class robotDirector:

    def __init__(self, master): 
        self.master = master
        master.title("Lyttle ReSearch Robot Director")
        master.config(bg="lightgreen")
        CE_PIN = 10 # GPIO17 (Physical pin 11)
        CSN_PIN = 9 # GPIO8 (Physical pin 24)

        self.port = tk.StringVar(value="/dev/ttyUSB0") # <--- Define self.port FIRST!
        self.baud_rate = 115200 # <--- Define self.baud_rate FIRST!

        # NEW: Queue for commands to be sent by a dedicated sending thread
        self.command_send_queue = queue.Queue()
        self.command_send_thread = None
        self.command_send_thread_running = False

        self.gcode_file_path = tk.StringVar(master) # Add this line
        self.gcode_status_label = None # Add this line as well, if you haven't already

        self.current_laser_power = tk.IntVar(master, value=0)
        self.laser_on = tk.BooleanVar(master, value=False)
        self.radio_status = tk.StringVar(master, value="Idle")
        self.serial_port = None  # <--- ADD THIS LINE (or ensure it's here)
        self.arduino_connected = False
        self.running = False # Flag for controlling the serial read thread
        self.motion_command = {"x": 0.0, "y": 0.0, "rotation": 0.0, "laser_on": False, "laser_power": 0}
        # Initialize last_sent_motion_command for comparison to prevent excessive queuing
        self.last_sent_motion_command = self.motion_command.copy()
        self.last_sent_motion_command["speed_factor"] = 0.0 # Initialize with a default speed factor

        self.north_angle = 0.0
        self.control_source = "keyboard"
        self.spacebar_pressed = False
        # self.arduino_connected = False # Already initialized above

        self.current_control_method = "Direct X/Y/R Buttons" # Default control method

        self.speed_var = tk.DoubleVar(master, value=0.5)
        self.motion_update_job = None
        self.is_moving = {
            "forward": False,
            "backward": False,
            "left": False,
            "right": False,
            "CCW": False,
            "CW": False,}

         # --- Localization variables (now for display only, not main control) ---
        self.x_pos = tk.DoubleVar(master, value=0.0)
        self.y_pos = tk.DoubleVar(master, value=0.0)
        self.rotation_val = tk.DoubleVar(master, value=0.0)
        self.elevation_val = tk.DoubleVar(master, value=0.0)

        # --- New: Command Throttle Variable (in milliseconds) ---
        self.command_throttle_ms = tk.IntVar(master, value=10) # Default to 100ms throttle

        self.create_widgets()
        self.master.bind('<KeyPress>', self.read_keyboard)
        self.master.bind('<KeyRelease>', self.read_keyrelease)
        self.master.bind('<FocusIn>', self.focus_change_handler, add='+')
        self.master.bind('<FocusOut>', self.focus_change_handler, add='+')
        self.update_radio_status("Disconnected")
        self.port = tk.StringVar(value="/dev/ttyUSB0")
        self.baud_rate = 115200
        self.connect_arduino_serial() # This will start serial and command sending threads

        # --- G-code Specific State Variables ---
        self.gcode_queue = [] # Queue to hold parsed G-code commands
        self.gcode_current_x = 0.0 # Robot's current X position in G-code coordinates (e.g., mm)
        self.gcode_current_y = 0.0 # Robot's current Y position in G-code coordinates (e.g., mm)
        self.gcode_current_laser_on = False
        self.gcode_current_laser_power = 0 # 0-255 scale
        self.gcode_current_feed_rate = 1.0 # Default feed rate in G-code units (e.g., mm/min)
        self.gcode_absolute_mode = True # True for G90 (absolute), False for G91 (relative)
        self.gcode_processing_active = False # Flag to indicate if G-code is currently being processed

        ## --- Joystick Client Variables (ENSURE THESE ARE INITIALIZED BEFORE _connect_to_joystick_server) ---
        self.joystick_socket = None # This will be the client socket
        self.joystick_connected = False # New flag for client connection status
        self.joystick_port = 52345
        self.joystick_host = '127.0.0.1'
        self.joystick_buffer_size = 1024 
        self.joystick_data_buffer = '' # Ensure this is a string
        self.joystick_data_queue = queue.Queue() # Queue to pass data from thread to main GUI
        self.joystick_read_thread = None # To hold the reference to the reading thread
        self.joystick_thread_running = False # Flag to control the thread's loop
        self._connect_to_joystick_server() # <<< ADD THIS LINE

        self.ROBOT_MAX_LINEAR_VELOCITY_MM_PER_SEC = 10000 # Adjust this to your robot's actual max linear speed in mm/sec
        # self.gcode_current_feed_rate = self.speed_var.get() * self.ROBOT_MAX_LINEAR_VELOCITY_MM_PER_SEC # This line is redundant, gcode_current_feed_rate is handled by F commands or defaults

        # --- Graceful shutdown protocol ---
        self.master.protocol("WM_DELETE_WINDOW", self.on_closing)

    def on_closing(self):
        """Handles graceful shutdown of the application."""
        print("Closing application. Attempting graceful shutdown of threads and connections...")
        self.running = False # Stop serial read thread
        self.command_send_thread_running = False # Stop command send thread
        self.joystick_thread_running = False # Stop joystick read thread
        self.gcode_processing_active = False # Stop G-code processing

        # Join serial read thread
        if self.serial_read_thread and self.serial_read_thread.is_alive():
            print("Joining serial read thread...")
            self.serial_read_thread.join(timeout=1)
            if self.serial_read_thread.is_alive():
                print("[Warning] Serial read thread did not terminate gracefully.")
        
        # Join command send thread
        if self.command_send_thread and self.command_send_thread.is_alive():
            print("Joining command send thread...")
            # Put a dummy item in queue to unblock it if it's waiting
            self.command_send_queue.put(None) 
            self.command_send_thread.join(timeout=1)
            if self.command_send_thread.is_alive():
                print("[Warning] Command send thread did not terminate gracefully.")

        # Join joystick read thread
        if self.joystick_read_thread and self.joystick_read_thread.is_alive():
            print("Joining joystick read thread...")
            # Put a dummy item in queue to unblock it if it's waiting
            self.joystick_data_queue.put(None)
            self.joystick_read_thread.join(timeout=1)
            if self.joystick_read_thread.is_alive():
                print("[Warning] Joystick read thread did not terminate gracefully.")

        # Close serial port
        if self.serial_port and self.serial_port.is_open:
            print("Closing serial port...")
            self.serial_port.close()

        # Close joystick socket
        if self.joystick_socket:
            print("Closing joystick socket...")
            try:
                self.joystick_socket.shutdown(socket.SHUT_RDWR)
                self.joystick_socket.close()
            except OSError as e:
                print(f"Error during joystick socket shutdown/close on exit: {e}")

        print("All threads and connections shut down. Destroying main window.")
        self.master.destroy()


    def _start_motion_sending_loop(self):
        # Cancel any existing loop to prevent duplicates
        if self.motion_update_job:
            self.master.after_cancel(self.motion_update_job)
            self.motion_update_job = None # Clear the old job ID

        # Start the continuous sending loop
        # The _send_repeated_command method will now handle its own rescheduling.
        self._send_repeated_command()
        #print("DEBUG: _start_motion_sending_loop - continuous sending loop initiated.")

    def disconnect_serial(self):
        # Stop the serial reading thread
        self.running = False
        if self.serial_read_thread and self.serial_read_thread.is_alive():
            self.serial_read_thread.join(timeout=1)
            if self.serial_read_thread.is_alive():
                print("[Warning] Serial read thread did not terminate gracefully.")
        self.serial_read_thread = None

        # Stop the command sending thread
        self.command_send_thread_running = False
        if self.command_send_thread and self.command_send_thread.is_alive():
            # Put a dummy item in queue to unblock it if it's waiting
            self.command_send_queue.put(None) 
            self.command_send_thread.join(timeout=1)
            if self.command_send_thread.is_alive():
                print("[Warning] Command sending thread did not terminate gracefully.")
        self.command_send_thread = None

        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            self.arduino_connected = False
            self.update_radio_status("Disconnected.")
            print("Serial port disconnected.")
        else:
            self.update_radio_status("Not connected.")
            print("Serial port not open.")

    # --- New Client-side Joystick Connection Method ---
    def _connect_to_joystick_server(self):
        """Attempts to connect to the joystick server as a client."""
        # Check if the thread is already running and connected
        if self.joystick_read_thread and self.joystick_read_thread.is_alive():
            print("Joystick client connection already active.")
            return

        print(f"Attempting to connect to joystick server at {self.joystick_host}:{self.joystick_port}...")
        try:
            self.joystick_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM) # Initialize the actual socket
            self.joystick_socket.settimeout(1.0) # Set a timeout for the connection attempt
            self.joystick_socket.connect((self.joystick_host, self.joystick_port)) # Connect the socket
            self.joystick_socket.setblocking(False) # Make the client socket non-blocking for data reception

            print(f"Successfully connected to joystick server at {self.joystick_host}:{self.joystick_port}")
            self.joystick_data_buffer = '' # Clear buffer for new connection
            
            # Start the dedicated thread for receiving joystick data
            self.joystick_thread_running = True
            self.joystick_read_thread = threading.Thread(target=self._joystick_read_thread_target, daemon=True)
            self.joystick_read_thread.start()

            # Schedule a periodic check of the queue in the main Tkinter thread
            self.master.after(50, self._process_joystick_queue) # Check queue every 50ms
            
            # Update GUI status to show joystick connection
            self.update_radio_status("Joystick Connected")

        except socket.timeout:
            print(f"Connection to joystick server timed out after 1 second.")
            self._close_joystick_client_connection()
            # self.master.after(2000, self._connect_to_joystick_server) # Retry after 2 seconds - handled by _close_joystick_client_connection
        except socket.error as e:
            print(f"Error connecting to joystick server: {e}")
            self._close_joystick_client_connection()
            # self.master.after(2000, self._connect_to_joystick_server) # Retry after 2 seconds - handled by _close_joystick_client_connection
        except Exception as e:
            print(f"Unexpected error during joystick client connection: {e}")
            self._close_joystick_client_connection()
            # self.master.after(2000, self._connect_to_joystick_server) # Retry after 2 seconds - handled by _close_joystick_client_connection

    def _joystick_read_thread_target(self):
        """
        Target function for the joystick reading thread.
        Continuously receives data from the joystick server and puts it into a queue.
        """
        print("[Joystick Thread] Starting joystick data reception thread.")
        while self.joystick_thread_running and self.joystick_socket:
            try:
                # Attempt to receive data once. Will raise BlockingIOError if no data.
                chunk = self.joystick_socket.recv(self.joystick_buffer_size)
                if not chunk:
                    # Server closed connection gracefully (received 0 bytes)
                    print("[Joystick Thread] Server disconnected gracefully.")
                    self.joystick_thread_running = False # Signal thread to stop
                    # Put a special message in queue to signal disconnection to main thread
                    self.joystick_data_queue.put("DISCONNECTED") 
                    break # Exit loop
                
                # Decode and put the chunk into the queue
                self.joystick_data_queue.put(chunk.decode('utf-8'))

            except BlockingIOError:
                # No data available, just continue loop and try again
                time.sleep(0.01) # Small sleep to prevent busy-waiting and high CPU usage
            except socket.error as e:
                print(f"[Joystick Thread] Socket error: {e}")
                self.joystick_thread_running = False # Signal thread to stop
                self.joystick_data_queue.put("ERROR_SOCKET") # Signal error to main thread
                break # Exit loop
            except Exception as e:
                print(f"[Joystick Thread] Unexpected error in joystick thread: {e}")
                self.joystick_thread_running = False # Signal thread to stop
                self.joystick_data_queue.put("ERROR_UNKNOWN") # Signal error to main thread
                break # Exit loop
        print("[Joystick Thread] Exiting joystick data reception thread.")

    def _process_joystick_queue(self):
        """
        Processes joystick data from the queue in the main Tkinter thread.
        This method is called periodically by self.master.after().
        """
        # --- NEW: Only process joystick data if "Joystick Control" is the active method ---
        if self.current_control_method != "Joystick Control":
            # If not in joystick control mode, clear the queue to prevent backlog
            while not self.joystick_data_queue.empty():
                try:
                    self.joystick_data_queue.get_nowait()
                except queue.Empty:
                    break # Should not happen with empty() check, but for safety
            # Reschedule itself to keep checking the queue, only if the thread is still alive
            if self.joystick_read_thread and self.joystick_read_thread.is_alive():
                self.master.after(50, self._process_joystick_queue)
            return # Exit if not in joystick control mode

        while not self.joystick_data_queue.empty():
            item = self.joystick_data_queue.get_nowait() # Get item without blocking
            
            # Check for dummy item from on_closing to gracefully exit
            if item is None:
                print("[Main Thread] Received None from joystick queue, stopping processing.")
                # self.joystick_data_queue.task_done() # Not using task_done for simple queue
                return

            if item == "DISCONNECTED" or item == "ERROR_SOCKET" or item == "ERROR_UNKNOWN":
                print(f"[Main Thread] Joystick thread signaled: {item}")
                self._close_joystick_client_connection() # Handle disconnection/error
                return # Stop processing queue for now

            # Append the received string chunk to the buffer
            self.joystick_data_buffer += item 

            # Process all complete messages in the buffer (messages are newline-separated JSON)
            while '\n' in self.joystick_data_buffer:
                message_string, self.joystick_data_buffer = self.joystick_data_buffer.split('\n', 1)
                
                if not message_string.strip(): # Skip empty strings from split
                    continue

                try:
                    joystick_data = json.loads(message_string)

                    # Update speed_var from joystick data if 'speed' key is present
                    new_speed_multiplier_from_joystick = float(joystick_data.get("speed", self.speed_var.get()))
                    
                    # Only update speed_var if it's actually different to avoid unnecessary callbacks
                    if abs(self.speed_var.get() - new_speed_multiplier_from_joystick) > 1e-6:
                        self.speed_var.set(new_speed_multiplier_from_joystick) # Update Tkinter var, will trigger _update_gcode_feed_rate_from_slider

                    current_speed_factor = self.speed_var.get() # Get the *current* speed from the Tkinter variable

                    temp_motion_command = {
                        "x": float(joystick_data.get("x", 0.0)) * current_speed_factor,
                        "y": float(joystick_data.get("y", 0.0)) * current_speed_factor,
                        "rotation": float(joystick_data.get("r", 0.0)) * current_speed_factor,
                        "laser_on": bool(joystick_data.get("laser", 0)),
                        "laser_power": int(joystick_data.get("power", self.current_laser_power.get()))
                    }

                    # Only update Tkinter vars if there's a change to prevent unnecessary GUI updates
                    if self.laser_on.get() != temp_motion_command["laser_on"]:
                        self.laser_on.set(temp_motion_command["laser_on"])
                    if self.current_laser_power.get() != temp_motion_command["laser_power"]:
                        self.current_laser_power.set(temp_motion_command["laser_power"])
                    
                    # Store the current state in self.motion_command
                    self.motion_command.update(temp_motion_command)

                    # --- REVISED CRITICAL CHANGE: Only queue if there's a meaningful change ---
                    # Check for changes in actual motion (X, Y, R)
                    motion_x_changed = abs(self.motion_command["x"] - self.last_sent_motion_command["x"]) > 1e-6
                    motion_y_changed = abs(self.motion_command["y"] - self.last_sent_motion_command["y"]) > 1e-6
                    motion_r_changed = abs(self.motion_command["rotation"] - self.last_sent_motion_command["rotation"]) > 1e-6
                    
                    # Check for changes in laser state
                    laser_on_changed = self.motion_command["laser_on"] != self.last_sent_motion_command["laser_on"]
                    laser_power_changed = self.motion_command["laser_power"] != self.last_sent_motion_command["laser_power"]
                    
                    # Check for changes in speed factor.
                    speed_factor_changed = abs(current_speed_factor - self.last_sent_motion_command.get("speed_factor", 0.0)) > 1e-6
                    
                    # Determine if a command needs to be queued
                    should_queue_command = False
                    
                    # Rule 1: Always send if motion (X, Y, R) or laser state changes
                    if motion_x_changed or motion_y_changed or motion_r_changed or laser_on_changed or laser_power_changed:
                        should_queue_command = True
                    # Rule 2: Send if speed factor changes AND there is active motion
                    elif speed_factor_changed and (abs(self.motion_command["x"]) > 1e-6 or abs(self.motion_command["y"]) > 1e-6 or abs(self.motion_command["rotation"]) > 1e-6):
                        should_queue_command = True
                    # Rule 3: Send if speed factor changes to or from zero, even if robot is idle (for explicit speed context)
                    elif speed_factor_changed and (
                        (abs(self.last_sent_motion_command.get("speed_factor", 0.0)) < 1e-6 and abs(current_speed_factor) >= 1e-6) or # From zero to non-zero
                        (abs(self.last_sent_motion_command.get("speed_factor", 0.0)) >= 1e-6 and abs(current_speed_factor) < 1e-6)    # From non-zero to zero
                    ):
                        should_queue_command = True
                    # Rule 4: Send if speed factor changes significantly AND robot is idle AND new speed is NOT zero
                    # This ensures the Arduino is aware of the new 'max speed' setting for future moves.
                    elif speed_factor_changed and not (abs(self.motion_command["x"]) > 1e-6 or abs(self.motion_command["y"]) > 1e-6 or abs(self.motion_command["rotation"]) > 1e-6) and abs(current_speed_factor) >= 1e-6:
                        should_queue_command = True

                    if should_queue_command:
                        self.command_send_queue.put(self.motion_command.copy())
                        self.last_sent_motion_command = self.motion_command.copy()
                        self.last_sent_motion_command["speed_factor"] = current_speed_factor # Store for comparison

                except json.JSONDecodeError as e:
                    print(f"Robot: Received invalid JSON data: {e} - '{message_string}'")
                except ValueError as e:
                    print(f"Robot: Data conversion error: {e} in '{message_string}'")
                except Exception as e:
                    print(f"Robot: Error processing joystick data: {e}")

        # Reschedule itself to keep checking the queue, only if the thread is still alive
        if self.joystick_read_thread and self.joystick_read_thread.is_alive():
            self.master.after(50, self._process_joystick_queue)

    # --- Modified Close Connection Method (for client) ---
    def _close_joystick_client_connection(self):
        """
        Closes the joystick client connection and stops the reading thread.
        Includes logic to attempt auto-reconnect.
        """
        print("[DEBUG] _close_joystick_client_connection called.")
        self.joystick_thread_running = False # Signal the reading thread to stop
        
        # Close the socket if it exists
        if self.joystick_socket: # Use self.joystick_socket consistently
            try:
                self.joystick_socket.shutdown(socket.SHUT_RDWR) # Attempt graceful shutdown
                self.joystick_socket.close()
                print("Joystick client socket closed.")
            except OSError as e:
                print(f"Error during joystick socket shutdown/close: {e}")
            self.joystick_socket = None # Clear the socket reference
        else:
            print("No joystick socket to close.")

        # Wait for the joystick reading thread to finish
        if self.joystick_read_thread and self.joystick_read_thread.is_alive():
            print("Waiting for joystick reading thread to terminate...")
            # Put a dummy item in queue to unblock it if it's waiting
            self.joystick_data_queue.put(None) 
            self.joystick_read_thread.join(timeout=1) # Wait for up to 1 second
            if self.joystick_read_thread.is_alive():
                print("[Warning] Joystick reading thread did not terminate gracefully.")
        self.joystick_read_thread = None # Clear thread reference
        print("Joystick reading thread reference cleared.")

        self.update_radio_status("Joystick Disconnected") # Update GUI status

        # --- Auto-reconnect logic ---
        print("Attempting to re-establish joystick connection in 2 seconds...")
        self.master.after(2000, self._connect_to_joystick_server) # Retry connection after 2 seconds

    # --- Modify your create_joystick_control_area if you want a connect button in the GUI ---
    def create_joystick_control_area(self, parent_frame, event=None):
        ttk.Label(parent_frame, text="Joystick Control", background="lightgray").pack(padx=10, pady=10)
        # Optional: Add a connect/disconnect button here
        ttk.Button(parent_frame, text="Connect to Joystick Server", command=self._connect_to_joystick_server).pack(pady=5)
        ttk.Button(parent_frame, text="Disconnect Joystick", command=self._close_joystick_client_connection).pack(pady=5)

    def send_control_command(self):
        """
        Queues the current motion_command for sending by the dedicated sending thread.
        This method is called from the main Tkinter thread by various controls (keyboard, buttons, G-code).
        """
        if not self.arduino_connected or not self.serial_port or not self.serial_port.is_open:
            print("  [DEBUG] send_control_command (main thread): Serial port to desktop Nano not connected or open. Cannot queue command.")
            self.radio_status.set("Bridge Not Connected")
            return
        
        # Put a copy of the current motion_command into the queue
        # Ensure the current speed_var value is also part of the command for the sending thread
        command_to_queue = self.motion_command.copy()
        command_to_queue["speed_factor"] = self.speed_var.get() # Add speed factor to the command
        self.command_send_queue.put(command_to_queue)
        # print(f"  [DEBUG] Command queued: {command_to_queue}") # Uncomment for debugging


    # Renamed from send_control_command
    def _send_command_to_serial_bridge(self, command_data): # NEW: Takes command_data as argument
        # print(f"  [DEBUG] Entering _send_command_to_serial_bridge.") # Debug print

        # Ensure a serial port is connected (this is the serial connection to the desktop Nano bridge)
        if not self.arduino_connected or not self.serial_port or not self.serial_port.is_open:
            print("  [DEBUG] _send_command_to_serial_bridge: Serial port to desktop Nano not connected or open. Cannot send command.")
            # Update GUI status on main thread using master.after
            self.master.after(0, lambda: self.radio_status.set("Bridge Not Connected"))
            return
        
        try:
            motion_x = command_data["x"] # Use passed command_data
            motion_y = command_data["y"] # Use passed command_data
            rotation = command_data["rotation"] # Use passed command_data
            
            laser = int(command_data.get("laser_on", False)) # Use passed command_data
            laser_power = command_data.get("laser_power", 0) # Use passed command_data

            target_speed = command_data.get("speed_factor", 0.5) # Use speed_factor from command_data, default to 0.5

            command_string = (
                f"MX:{motion_x: .8f},"
                f"MY:{motion_y: .8f},"
                f"R:{rotation: .8f},"
                f"L:{laser},"       
                f"P:{laser_power}," 
                f"S:{target_speed: .8f}\n" 
            )

            self.serial_port.write(command_string.encode('utf-8'))
            # Update GUI status on main thread (must use master.after for thread safety)
            self.master.after(0, lambda s=command_string.strip(): self.radio_status.set(f"Bridge Sent: {s}"))
            print(f"  [SENT VIA BRIDGE] {command_string.strip()}")

        except serial.SerialException as e:
            print(f"  [ERROR] Serial communication error during send to bridge: {e}")
            self.master.after(0, lambda: self.radio_status.set(f"Bridge Serial Error: {e}"))
        except Exception as e:
            print(f"  [ERROR] _send_command_to_serial_bridge failed unexpectedly: {e}")
            self.master.after(0, lambda: self.radio_status.set(f"Command Error: {e}"))

    def focus_change_handler(self,event):
        # This handler can be used for debugging focus changes, but the main
        # keyboard event filtering is done in read_keyboard/read_keyrelease.
        # print(f"Focus changed to: {event.widget}")
        pass

    def deactivate_laser_button(self):
        """Turns the laser OFF via button press."""
        self.laser_on.set(False) # Update Tkinter variable for GUI
        self.motion_command["laser_on"] = False # Update motion_command for send_control_command
        self.motion_command["laser_power"] = 0 # Turn off power
        self.send_control_command()
        self.simulate_radio_transmission(True) # Keep for simulation feedback

    def read_keyboard(self, event):
        # --- NEW: Check if an Entry widget has focus ---
        focused_widget = self.master.focus_get()
        if isinstance(focused_widget, ttk.Entry) or isinstance(focused_widget, tk.Entry):
            # If an Entry widget has focus, do not process keyboard events for robot control
            return

        keysym = event.keysym.lower() # Get keysym and convert to lowercase for consistent checking
        #print(f"DEBUG: read_keyboard - keysym: {keysym!r}, control_method: {self.current_control_method!r}")

        # Handle spacebar first as it's common across control methods and a direct action
        if keysym == 'space':
            self.laser_on.set(True)
            self.spacebar_pressed = True
            self.send_control_command() # Send the command immediately for laser state change
            # print(f"DEBUG: Current Control Method: {self.current_control_method}") # Debug print
            return # Exit after handling spacebar, it's a direct action


        if self.current_control_method == "Direct X/Y/R Buttons":
            speed = self.speed_var.get() # Get speed from slider

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
                # print(f"North Angle: {self.north_angle}") # Debug print
            elif keysym == 'e':
                self.is_moving['CW'] = True
                self.north_angle = (self.north_angle - 5) % 360  # Example rotation update
                # print(f"North Angle: {self.north_angle}") # Debug print
            else:
                return # If it's not a relevant movement key, do nothing further in this method

            # Now, calculate the *cumulative* motion based on ALL currently pressed movement keys
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
                self._start_motion_sending_loop()

        else: # This 'else' belongs to the 'if self.current_control_method == "Direct X/Y/R Buttons":'
            pass # Use 'pass' if you want to explicitly do nothing here    

    # --- COMMENTED OUT: This was a duplicate and problematic send_control_command ---
    # This method was causing issues by being defined multiple times and containing
    # blocking serial write logic on the main thread. It is now superseded by the
    # single, queuing `send_control_command` method (around line 593) and the
    # `_send_command_to_serial_bridge` method (around line 617) which runs in a thread.
    # def send_control_command(self):
    #     #print(f"  [DEBUG] Entering send_control_command. is_sending_command: {self.is_sending_command}")
    #     if not self.arduino_connected or not self.serial_port or not self.serial_port.is_open:
    #         #print("  [DEBUG] send_control_command: Serial port to desktop Nano not connected or open. Cannot send command.")
    #         self.radio_status.set("Bridge Not Connected") # Update status to reflect bridge
    #         return
    #     # if self.is_sending_command:
    #     #     #print("  [DEBUG] send_control_command: Command already in progress, skipping send.")
    #     #     return
    #     # self.is_sending_command = True # Set flag to prevent re-entry
    #     # #print("  [DEBUG] is_sending_command set to True.")
    #     try:
    #         motion_x = self.motion_command["x"]
    #         motion_y = self.motion_command["y"]
    #         rotation = self.motion_command["rotation"]
    #         laser = int(self.motion_command.get("laser_on", False))
    #         laser_power = self.motion_command.get("laser_power", 0)
    #         target_speed = self.speed_var.get()
    #         command_string = (
    #             f"MX:{motion_x: .8f},"
    #             f"MY:{motion_y: .8f},"
    #             f"R:{rotation: .8f},"
    #             f"L:{laser},"       
    #             f"P:{laser_power}," 
    #             f"S:{target_speed: .8f}\n" 
    #         )
    #         #print(f"  [DEBUG] Attempting to send (via serial bridge): {command_string.strip()}")
    #         self.serial_port.write(command_string.encode('utf-8'))
    #         self.radio_status.set(f"Bridge Sent: {command_string.strip()}")
    #         print(f"  [SENT VIA BRIDGE] {command_string.strip()}")
    #     except serial.SerialException as e:
    #         print(f"  [ERROR] Serial communication error during send to bridge: {e}")
    #         self.radio_status.set(f"Bridge Serial Error: {e}")
    #     except Exception as e:
    #         print(f"  [ERROR] send_control_command failed unexpectedly: {e}")
    #         self.radio_status.set(f"Command Error: {e}")
    #     finally:
    #         # self.is_sending_command = False
    #         #print("  [DEBUG] is_sending_command set to False (finally block).")
    #     pass # END OF COMMENTED OUT BLOCK

    def read_keyrelease(self, event):
        # --- NEW: Check if an Entry widget has focus ---
        focused_widget = self.master.focus_get()
        if isinstance(focused_widget, ttk.Entry) or isinstance(focused_widget, tk.Entry):
            # If an Entry widget has focus, do not process keyboard events for robot control
            return

        keysym = event.keysym.lower()
        # print(f"read_keyrelease: event.keysym = {keysym!r}") # Debug print

        if keysym == 'space':
            self.laser_on.set(False) # Update Tkinter variable for GUI
            self.motion_command["laser_on"] = False
            self.motion_command["laser_power"] = 0 # Turn off laser power on release
            self.send_control_command()
            self.spacebar_pressed = False
            # print("Spacebar released: Laser OFF") # Debug print
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
                self._stop_motion_sending_loop() # Stop the repeating loop!
            else:
                # If keys are still pressed, ensure the motion command is updated and queued
                # This ensures continuous movement when multiple keys are released sequentially
                vx, vy, omega = 0.0, 0.0, 0.0
                speed = self.speed_var.get() # Get speed from slider
                if self.is_moving['forward']: vy += speed
                if self.is_moving['backward']: vy -= speed
                if self.is_moving['right']: vx += speed
                if self.is_moving['left']: vx -= speed
                if self.is_moving['CCW']: omega += speed
                if self.is_moving['CW']: omega -= speed
                self.motion_command["x"] = vx
                self.motion_command["y"] = vy
                self.motion_command["rotation"] = omega
                self.send_control_command() # Queue the updated (potentially reduced) motion command

    def _stop_motion_sending_loop(self):
        if self.motion_update_job:
            self.master.after_cancel(self.motion_update_job)
            self.motion_update_job = None
            #print("DEBUG: _stop_motion_sending_loop - Motion sending loop stopped.")

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

            # NEW: Start the command sending thread
            self.command_send_thread_running = True
            self.command_send_thread = threading.Thread(target=self._command_sending_thread_target, daemon=True)
            self.command_send_thread.start()

            # --- Send an initial stop command to ensure robot is halted ---
            # This ensures motion_command is explicitly zeroed and sent immediately after connection.
            self.motion_command = {"x": 0.0, "y": 0.0, "rotation": 0.0, "laser_on": False, "laser_power": 0}
            self.send_control_command() # Queue this initial stop command
            self.last_sent_motion_command = self.motion_command.copy() # Initialize last sent command after the first send
            self.last_sent_motion_command["speed_factor"] = self.speed_var.get() # Also store initial speed factor


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
                    time.sleep(1.0) # Small delay to prevent busy-waiting on error
            else:
                time.sleep(0.5) # Sleep longer if not connected to avoid CPU hogging

    def emergency_stop(self):
        print("Emergency Stop Activated!")
        self.update_radio_status("Error")
        self.motion_command = {"x": 0.0, "y": 0.0, "rotation": 0.0, "laser_on": False, "laser_power": 0} # Ensure all fields are reset
        self.laser_on.set(False)
        self.current_laser_power.set(0)
        self.send_control_command()

    def move_robot(self, direction, speed, start_event=None):
        print(f"Start moving robot {direction} at {speed} speed.")
        self.is_moving[direction] = True  # Track which direction is active
        # The _send_repeated_command now calculates motion based on self.is_moving
        # and schedules itself. We just need to start the loop if it's not running.
        if not self.motion_update_job: # Only start if not already active
            self._start_motion_sending_loop() # <--- Call the method that starts the loop

    def _send_repeated_command(self): # No need for direction, speed arguments here anymore
        # Calculate the cumulative motion based on ALL currently pressed movement keys
        vx, vy, omega = 0.0, 0.0, 0.0
        speed = self.speed_var.get() # Get current speed from slider

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

        # Check if there is any active movement requested
        # Only queue if there's actual motion or if it's a transition to zero motion
        is_moving_now = (abs(self.motion_command["x"]) > 1e-6 or 
                         abs(self.motion_command["y"]) > 1e-6 or 
                         abs(self.motion_command["rotation"]) > 1e-6)
        
        was_moving_last = (abs(self.last_sent_motion_command["x"]) > 1e-6 or 
                           abs(self.last_sent_motion_command["y"]) > 1e-6 or 
                           abs(self.last_sent_motion_command["rotation"]) > 1e-6)

        speed_changed_significantly = abs(speed - self.last_sent_motion_command.get("speed_factor", 0.0)) > 1e-6

        # Queue if:
        # 1. Motion state changes (moving to stopped, or stopped to moving)
        # 2. Motion is active AND speed changes significantly
        # 3. Motion is zero, but speed changes to/from zero explicitly (for explicit speed context)
        # 4. Laser state changes (added this back explicitly for completeness, though it's also in read_keyboard/joystick)
        laser_on_changed = self.motion_command["laser_on"] != self.last_sent_motion_command["laser_on"]
        laser_power_changed = self.motion_command["laser_power"] != self.last_sent_motion_command["laser_power"]


        if is_moving_now or was_moving_last or \
           (speed_changed_significantly and is_moving_now) or \
           (speed_changed_significantly and not is_moving_now and (abs(speed) < 1e-6 or abs(self.last_sent_motion_command.get("speed_factor", 0.0)) < 1e-6)) or \
           laser_on_changed or laser_power_changed: # Added laser changes here for _send_repeated_command
            
            self.send_control_command() # Queue the current (possibly zero) motion command
            # Update last_sent_motion_command only if a command was actually queued
            self.last_sent_motion_command = self.motion_command.copy()
            self.last_sent_motion_command["speed_factor"] = speed # Store current speed for comparison
            
        # Reschedule next call if any key is still pressed, or if motion is non-zero (for continuous update)
        if any(self.is_moving.values()) or is_moving_now:
            self.motion_update_job = self.master.after(150, self._send_repeated_command) # Schedule next call
        else:
            # No movement keys are pressed AND motion_command is already zero, so stop the loop
            if self.motion_update_job: # Only clear if a job is active
                self.master.after_cancel(self.motion_update_job)
                self.motion_update_job = None # Clear the job ID
            print("DEBUG: _send_repeated_command - No active movement, stopping loop.")
            # Ensure a final stop command is sent if it wasn't already (redundant but safe)
            # This is handled by the 'was_moving_last' check above for the transition to zero.
            # If the robot was already stopped and no keys are pressed, no need to send again.
            if was_moving_last: # Only send a final stop if it was moving and now isn't
                self.motion_command["x"] = 0.0
                self.motion_command["y"] = 0.0
                self.motion_command["rotation"] = 0.0
                self.send_control_command() # Send one final stop command to ensure robot halts


    def _update_gcode_feed_rate_from_slider(self, value):
        """
        Updates gcode_current_feed_rate when the speed slider is moved.
        Sends the command immediately to update the robot's speed.
        """
        new_speed_multiplier = float(value)
        # Update gcode_current_feed_rate for G-code processing, if applicable
        self.gcode_current_feed_rate = new_speed_multiplier * self.ROBOT_MAX_LINEAR_VELOCITY_MM_PER_SEC
        
        # --- REVISED LOGIC: Only send command if robot is moving or speed changes to/from zero ---
        is_robot_moving = (abs(self.motion_command["x"]) > 1e-6 or 
                           abs(self.motion_command["y"]) > 1e-6 or 
                           abs(self.motion_command["rotation"]) > 1e-6)
        
        speed_changed = abs(new_speed_multiplier - self.last_sent_motion_command.get("speed_factor", 0.0)) > 1e-6

        # Queue if:
        # 1. Robot is currently moving (to apply new speed mid-motion)
        # 2. Speed changes to or from zero (to explicitly set speed context when idle)
        # 3. Speed changes significantly AND robot is idle AND new speed is NOT zero (e.g. going from 0.5 to 0.8 while idle)
        #    This ensures the Arduino is aware of the new 'max speed' setting for future moves.
        if is_robot_moving or \
           (speed_changed and (abs(new_speed_multiplier) < 1e-6 or abs(self.last_sent_motion_command.get("speed_factor", 0.0)) < 1e-6)) or \
           (speed_changed and not is_robot_moving and abs(new_speed_multiplier) >= 1e-6): # New condition
            self.send_control_command()
            # Update last_sent_motion_command's speed_factor here as well
            self.last_sent_motion_command["speed_factor"] = new_speed_multiplier
        # print(f"Sent command with global speed factor: {new_speed_multiplier:.2f} (from slider)")

    def create_widgets(self):
        # Configure the master grid for responsiveness
        # Row 0: Top-level control, throttle
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
        # Removed grid_columnconfigure(2, weight=1) as the third column (Current Position) is removed.

        # --- Control Frame ---
        control_frame = ttk.LabelFrame(self.master, text="Robot Control", padding="10")
        # control_frame.grid(row=0, column=0, padx=10, pady=10, sticky="nsew") # Old placement
        control_frame.grid(row=0, column=0, padx=10, pady=10, sticky="nsew", columnspan=1) # Adjusted columnspan
        control_frame.grid_columnconfigure(0, weight=1)

        # RESTORED: G-code related control styles
        self.control_styles = ["Direct X/Y/R Buttons", "Circle Click Control", "Python Script Director",
                               "Tarantino as Director", "Frickin Shoot Everyone Director", ".dxf Director",
                               "G-code Director", "Joystick Control", "Placeholder Style"] # Removed .jpg and SVG/BMP for now, simplifying

        # RESTORED: G-code related control style handlers
        self.control_styles_dict = {
            "Direct X/Y/R Buttons": self.create_keyboard_control_area,
            "Circle Click Control": self.create_circle_control,
            "Python Script Director": self.create_python_script_director,
            "Tarantino as Director": self.create_tarantino_director,
            "Frickin Shoot Everyone Director": self.create_frickin_shoot_everyone_director,
            ".dxf Director": self.create_dxf_director,
            "G-code Director": self.create_gcode_director, # RESTORED
            "Joystick Control": self.create_joystick_control_area,
            "Placeholder Style": self.create_placeholder_style, # Ensure this is correctly referenced
        }
        self.control_style_dropdown = ttk.Combobox(control_frame, values=self.control_styles, state="readonly")
        self.control_style_dropdown.set("Direct X/Y/R Buttons")
        self.control_style_dropdown.grid(row=0, column=0, pady=5, padx=5, sticky="ew")
        self.control_style_dropdown.bind("<<ComboboxSelected>>", self.control_style_changed)

        # --- NEW: Command Throttle Input Frame ---
        throttle_frame = ttk.LabelFrame(self.master, text="Command Throttle", padding="10")
        # throttle_frame.grid(row=0, column=1, padx=10, pady=10, sticky="nsew") # Old placement
        throttle_frame.grid(row=0, column=1, padx=10, pady=10, sticky="nsew", columnspan=1) # Adjusted columnspan
        throttle_frame.grid_columnconfigure(1, weight=1) # Allow entry field to expand

        throttle_label = ttk.Label(throttle_frame, text="Delay (ms):")
        throttle_label.grid(row=0, column=0, padx=5, pady=5, sticky="w")

        self.throttle_entry = ttk.Entry(throttle_frame, textvariable=self.command_throttle_ms, width=8)
        self.throttle_entry.grid(row=0, column=1, padx=5, pady=5, sticky="ew")
        # Add validation for numerical input
        self.throttle_entry.bind("<FocusOut>", self._validate_throttle_input)
        self.throttle_entry.bind("<Return>", self._validate_throttle_input)

        # --- Removed: Current Position Display Frame ---
        # This frame and its labels have been removed as per user request.
        # self.x_label = ttk.Label(...)
        # self.y_label = ttk.Label(...)
        # self.r_label = ttk.Label(...)
        # self.e_label = ttk.Label(...)


        # --- Dynamic Control Area ---
        self.dynamic_control_area = ttk.Frame(self.master, style="TFrame")
        self.dynamic_control_area.grid(row=1, column=0, columnspan=2, padx=10, pady=10, sticky="nsew") # Adjusted columnspan
        self.dynamic_control_area.grid_columnconfigure(0, weight=1)
        self.dynamic_control_area.grid_rowconfigure(0, weight=1)

        self.control_style_changed() # Initialize dynamic control area


        # --- General Controls Frame (Speed Control) ---
        general_sliders_frame = ttk.LabelFrame(self.master, text="Speed Control", style="TLabelframe")
        general_sliders_frame.grid(row=2, column=0, columnspan=2, padx=10, pady=5, sticky="ew") # Adjusted columnspan
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
        laser_frame.grid(row=3, column=0, columnspan=2, padx=10, pady=10, sticky="ew") # Adjusted columnspan
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
        robot_location_init_frame.grid(row=4, column=0, columnspan=2, padx=10, pady=10, sticky="ew") # Adjusted columnspan
        robot_location_init_frame.grid_columnconfigure(0, weight=1)

        ttk.Button(robot_location_init_frame, text="Initialize Location", command=self.initiate_localization).grid(row=0, column=0, padx=5, pady=5)


        # --- Status Frame (NEW!) ---
        status_frame = ttk.Frame(self.master, style="TFrame")
        status_frame.grid(row=5, column=0, columnspan=2, padx=10, pady=5, sticky="ew") # Adjusted columnspan
        status_frame.grid_columnconfigure(0, weight=1) # Allow status label to expand

        # G-code Status Label - Place inside status_frame, at row 0, column 0
        self.gcode_status_label = tk.Label(status_frame, text="G-code Status: Idle", bg="lightgreen", font=("Arial", 10))
        self.gcode_status_label.grid(row=0, column=0, padx=5, pady=2, sticky="w") # Corrected grid placement and indentation

        # Radio Status Label - Place inside status_frame, at row 0, column 1 (or 0, 0 if you want it below gcode status)
        self.radio_status_label = ttk.Label(status_frame, textvariable=self.radio_status, style="TLabel")
        self.radio_status_label.grid(row=0, column=1, padx=5, pady=2, sticky="w") # Placed next to G-code status

    def _validate_throttle_input(self, event=None):
        """Validates the throttle input to ensure it's a positive integer."""
        try:
            value = int(self.command_throttle_ms.get())
            if value <= 0:
                messagebox.showerror("Invalid Input", "Throttle delay must be a positive integer (milliseconds). Setting to default 100ms.")
                self.command_throttle_ms.set(100)
        except ValueError:
            messagebox.showerror("Invalid Input", "Throttle delay must be a number. Setting to default 100ms.")
            self.command_throttle_ms.set(100)


    def activate_laser_button(self):
        print("Laser Activated (Button).")
        self.laser_on.set(True)
        self.motion_command["laser_on"] = True
        self.motion_command["laser_power"] = self.current_laser_power.get()
        self.send_control_command()
        self.simulate_radio_transmission(True)

    def deactivate_laser_button(self):
        """Turns the laser OFF via button press."""
        self.laser_on.set(False) # Update Tkinter variable for GUI
        self.motion_command["laser_on"] = False # Update motion_command for send_control_command
        self.motion_command["laser_power"] = 0 # Turn off power
        self.send_control_command()
        self.simulate_radio_transmission(True) # Keep for simulation feedback

    def set_laser_power(self, value): # 'value' is passed by the slider
        power = int(float(value))
        self.current_laser_power.set(power) # Update Tkinter variable for GUI
        self.motion_command["laser_power"] = power # Update motion_command
        # Only send command if laser is currently on, or if it's an M3/M5 G-code command
        if self.laser_on.get() or self.gcode_processing_active: # Only send if laser is on or G-code is active
         self.send_control_command()

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
        # --- NEW: Check if an Entry widget has focus ---
        focused_widget = self.master.focus_get()
        # If the focused widget is an Entry, do not process keyboard events for robot control
        if isinstance(focused_widget, ttk.Entry) or isinstance(focused_widget, tk.Entry):
            return

        keysym = event.keysym.lower() # Get keysym and convert to lowercase for consistent checking
        #print(f"DEBUG: read_keyboard - keysym: {keysym!r}, control_method: {self.current_control_method!r}")

        # Handle spacebar first as it's common across control methods and a direct action
        if keysym == 'space':
            self.laser_on.set(True)
            self.spacebar_pressed = True
            self.send_control_command() # Send the command immediately for laser state change
            # print(f"DEBUG: Current Control Method: {self.current_control_method}") # Debug print
            return # Exit after handling spacebar, it's a direct action


        if self.current_control_method == "Direct X/Y/R Buttons":
            speed = self.speed_var.get() # Get speed from slider

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
                # print(f"North Angle: {self.north_angle}") # Debug print
            elif keysym == 'e':
                self.is_moving['CW'] = True
                self.north_angle = (self.north_angle - 5) % 360  # Example rotation update
                # print(f"North Angle: {self.north_angle}") # Debug print
            else:
                return # If it's not a relevant movement key, do nothing further in this method

            # Now, calculate the *cumulative* motion based on ALL currently pressed movement keys
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
                self._start_motion_sending_loop()

        else: # This 'else' belongs to the 'if self.current_control_method == "Direct X/Y/R Buttons":'
            pass # Use 'pass' if you want to explicitly do nothing here    

    # --- COMMENTED OUT: This was a duplicate and problematic send_control_command ---
    # This method was causing issues by being defined multiple times and containing
    # blocking serial write logic on the main thread. It is now superseded by the
    # single, queuing `send_control_command` method (around line 593) and the
    # `_send_command_to_serial_bridge` method (around line 617) which runs in a thread.
    # def send_control_command(self):
    #     #print(f"  [DEBUG] Entering send_control_command. is_sending_command: {self.is_sending_command}")
    #     if not self.arduino_connected or not self.serial_port or not self.serial_port.is_open:
    #         #print("  [DEBUG] send_control_command: Serial port to desktop Nano not connected or open. Cannot send command.")
    #         self.radio_status.set("Bridge Not Connected") # Update status to reflect bridge
    #         return
    #     # if self.is_sending_command:
    #     #     #print("  [DEBUG] Command already in progress, skipping send.")
    #     #     return
    #     # self.is_sending_command = True # Set flag to prevent re-entry
    #     # #print("  [DEBUG] is_sending_command set to True.")
    #     try:
    #         motion_x = self.motion_command["x"]
    #         motion_y = self.motion_command["y"]
    #         rotation = self.motion_command["rotation"]
    #         laser = int(self.motion_command.get("laser_on", False))
    #         laser_power = self.motion_command.get("laser_power", 0)
    #         target_speed = self.speed_var.get()
    #         command_string = (
    #             f"MX:{motion_x: .8f},"
    #             f"MY:{motion_y: .8f},"
    #             f"R:{rotation: .8f},"
    #             f"L:{laser},"       
    #             f"P:{laser_power}," 
    #             f"S:{target_speed: .8f}\n" 
    #         )
    #         #print(f"  [DEBUG] Attempting to send (via serial bridge): {command_string.strip()}")
    #         self.serial_port.write(command_string.encode('utf-8'))
    #         self.radio_status.set(f"Bridge Sent: {command_string.strip()}")
    #         print(f"  [SENT VIA BRIDGE] {command_string.strip()}")
    #     except serial.SerialException as e:
    #         print(f"  [ERROR] Serial communication error during send to bridge: {e}")
    #         self.radio_status.set(f"Bridge Serial Error: {e}")
    #     except Exception as e:
    #         print(f"  [ERROR] send_control_command failed unexpectedly: {e}")
    #         self.radio_status.set(f"Command Error: {e}")
    #     finally:
    #         # self.is_sending_command = False
    #         #print("  [DEBUG] is_sending_command set to False (finally block).")
    #     pass # END OF COMMENTED OUT BLOCK


    def start_gcode_execution(self):
        """Starts processing the loaded G-code queue, ensuring a fresh start."""
        # First, ensure a G-code file has been selected
        if not self.gcode_file_path.get():
            self.update_gcode_status("Error: No G-code file selected to start.")
            return

        # To ensure a fresh start and reset all internal G-code state variables,
        # we re-load the file. This will re-populate self.gcode_queue and reset
        # gcode_current_x, gcode_code_current_y, etc.
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
        move_match = re.match(r'G([01])(?: X([\-\d\.]+))?(?: Y([\-\d\.]+))?(?: Z([\-\d\.]+))?(?: F([\d.]+))?(?: S(\d+))?(?:\s*\(.*?\))?$', line)
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
        #print(f"[DEBUG GCODE] Processing line: '{gcode_line}'")
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
                #else: # This else is problematic if S is non-zero but laser is off (e.g. M5 then S100)
                #    self.gcode_current_laser_on = True 
                #print(f"[DEBUG GCODE] G1/G0 with S value. Laser ON: {self.gcode_current_laser_on}, Power set to: {self.gcode_current_laser_power}")

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

            # Calculate movement vectors in millimeters
            dx_mm = target_x - self.gcode_current_x
            dy_mm = target_y - self.gcode_current_y
            
            # --- Start G-code distance to robot velocity conversion (updated logic) ---
            MM_TO_M_SCALE = .001 # Corrected: 1 millimeter = 0.001 meters

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
                #print("[DEBUG GCODE] G0/G1: Very small movement (distance < 1e-6 m). Skipping actual move.")
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

            #print(f"[DEBUG GCODE] Scheduled move duration: {delay_ms}ms. Next command after robot stops.")
            self.master.after(delay_ms, self._stop_robot_and_continue_gcode)
            
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
                #print(f"[DEBUG GCODE] Matched M3. Laser ON. Power:{self.gcode_current_laser_power}")
            elif gcode_command == 'M5':
                self.gcode_current_laser_on = False
                self.gcode_current_laser_power = 0
                #print(f"[DEBUG GCODE] Matched M5. Laser OFF.")
            
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
            
            # If S is non-zero, assume laser should be on if it's currently off
            if s_value > 0 and not self.gcode_current_laser_on:
                self.gcode_current_laser_on = True
            elif s_value == 0: # If S is zero, turn laser off
                self.gcode_current_laser_on = False
            
            #print(f"[DEBUG GCODE] Matched Standalone S. Laser ON: {self.gcode_current_laser_on}, Power: {self.gcode_current_laser_power}")
            
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
            #print(f"[DEBUG GCODE] Matched F. Current Feed Rate: {self.gcode_current_feed_rate:.4f} mm/s")
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
        #print("[DEBUG GCODE] Robot stopped after segment.")

        # Continue processing the G-code queue after stopping
        if self.gcode_processing_active:
            self.master.after(10, self._process_next_gcode_command)

    def _command_sending_thread_target(self):
        """
        Target function for the command sending thread.
        Continuously retrieves commands from the queue and sends them.
        """
        print("[Command Send Thread] Starting command sending thread.")
        while self.command_send_thread_running:
            try:
                # Get the command from the queue. block=True means it will wait until an item is available.
                # timeout=1 ensures it doesn't block indefinitely if the thread needs to stop.
                command_to_send = self.command_send_queue.get(block=True, timeout=1) 
                
                # Check for dummy item from on_closing to gracefully exit
                if command_to_send is None:
                    print("[Command Send Thread] Received None from queue, stopping processing.")
                    break # Exit the loop

                # Call the actual serial bridge sending method
                self._send_command_to_serial_bridge(command_to_send)

                # --- Introduce a delay in the sending thread to control rate ---
                # Use the user-defined throttle delay from the GUI
                throttle_delay_seconds = self.command_throttle_ms.get() / 1000.0
                time.sleep(throttle_delay_seconds) 

            except queue.Empty:
                # Queue was empty for 1 second, just loop again
                pass
            except Exception as e:
                print(f"[Command Send Thread] Error sending command: {e}")
                # Consider how to handle critical errors here (e.g., stop the thread)
        print("[Command Send Thread] Exiting command sending thread.")

    def update_gcode_status(self, message):
        """
        Updates the G-code status display and prints to console.
        Requires self.gcode_status_label to be initialized to a Tkinter Label widget.
        """
        if self.gcode_status_label: # Check if the label widget exists
           self.gcode_status_label.config(text=f"G-code Status: {message}")
        print(f"G-code Status: {message}") # Always print to console for debugging

    def control_style_changed(self, event=None):
        """
        Updates the dynamic control area based on the selected control style.
        """
        selected_style = self.control_style_dropdown.get()
        print(f"Control style changed to: {selected_style}") # Debug print
        
        # Clear existing widgets in the dynamic control area
        for widget in self.dynamic_control_area.winfo_children():
            widget.destroy()

        # Call the appropriate function to create widgets for the selected style
        creator_function = self.control_styles_dict.get(selected_style)
        if creator_function:
            creator_function(self.dynamic_control_area)
            self.current_control_method = selected_style # Update the active control method
        else:
            ttk.Label(self.dynamic_control_area, text="Control style not implemented.", background="lightgray").pack(padx=10, pady=10)
            self.current_control_method = None # No active control method

# --- All methods below are now explicitly included and correctly defined ---

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

    def create_gcode_director(self, parent_frame, event = None):
        # Clear previous dynamic content
        for widget in parent_frame.winfo_children():
            widget.destroy()

        # Add G-code specific widgets
        row_counter = 0
        parent_frame.grid_columnconfigure(0, weight=1)
        parent_frame.grid_columnconfigure(1, weight=1) # For buttons next to each other

        # Load G-code File Button
        btn_load_gcode = ttk.Button(parent_frame, text="Load G-code File", command=self.select_gcode_file)
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

    def create_svg_bmp_director(self, parent_frame, event=None):
        ttk.Label(parent_frame, text="SVG/BMP Director (Not Implemented Yet)", background="lightgray").pack(padx=10,
                                                                                                         pady=10)

    def create_placeholder_style(self, parent_frame, event=None):
        ttk.Label(parent_frame, text="Placeholder Style (Not Implemented Yet)", background="lightgray").pack(padx=10,
                                                                                                           pady=10)

# --- COMMENTED OUT: Old, redundant methods to avoid confusion and conflicts ---
# These methods were part of earlier iterations and are now superseded by the
# multi-threaded approach for joystick client and command sending.
# They are commented out to prevent accidental usage and simplify the codebase,
# as per user's instruction to not remove code.

# def _receive_joystick_data(self):
#     """
#     Receives and processes joystick data from the connected server.
#     This method is designed to be called repeatedly by self.master.after().
#     """
#     if not self.client_connection:
#         # print("Joystick client connection not active for receiving.") # Uncomment for debugging
#         return
#     try:
#         data = self.client_connection.recv(self.joystick_buffer_size)
#         if not data:
#             print("Joystick client disconnected (graceful shutdown by server).")
#             self._close_joystick_client_connection()
#             return
#         self.joystick_data_buffer += data.decode('utf-8')
#         while '\n' in self.joystick_data_buffer:
#             message_string, self.joystick_data_buffer = self.joystick_data_buffer.split('\n', 1)
#             if not message_string.strip():
#                 continue
#             try:
#                 joystick_data = json.loads(message_string)
#                 current_speed_factor = self.speed_var.get()
#                 self.motion_command["x"] = float(joystick_data.get("x", 0.0)) * current_speed_factor
#                 self.motion_command["y"] = float(joystick_data.get("y", 0.0)) * current_speed_factor
#                 self.motion_command["rotation"] = float(joystick_data.get("r", 0.0)) * current_speed_factor
#                 self.motion_command["elevation"] = float(joystick_data.get("e", 0.0)) * current_speed_factor
#                 laser_on_value = bool(joystick_data.get("laser", 0))
#                 if self.laser_on.get() != laser_on_value:
#                     self.laser_on.set(laser_on_value)
#                     print(f"Laser {'ON' if laser_on_value else 'OFF'} (from joystick)")
#                 power_on_value = bool(joystick_data.get("power", 0))
#                 if power_on_value != 0:
#                     pass
#                 else:
#                     pass
#                 self.send_control_command()
#             except json.JSONDecodeError as e:
#                 print(f"Robot: Received invalid JSON data: {e} - '{message_string}'")
#             except ValueError as e:
#                 print(f"Robot: Data conversion error: {e} in '{message_string}'")
#             except Exception as e:
#                 print(f"Robot: Error processing joystick data: {e}")
#     except BlockingIOError:
#         pass
#     except socket.error as e:
#         print(f"Socket error during joystick data reception: {e}")
#         self._close_joystick_client_connection()
#         return
#     except Exception as e:
#         print(f"Unexpected error in _receive_joystick_data: {e}")
#         self._close_joystick_client_connection()
#         return
#     self.master.after(10, self._receive_joystick_data)

# def _close_joystick_connection(self):
#     """Closes the joystick client connection and prepares for a new one."""
#     if self.joystick_socket:
#         try:
#             self.joystick_socket.shutdown(socket.SHUT_RDWR)
#             self.joystick_socket.close()
#         except OSError as e:
#             print(f"Error during socket shutdown/close: {e}")
#         self.joystick_socket = None
#     if self.joystick_listening and self.joystick_socket:
#         print("Listening for a new joystick connection...")
#         self.master.after(100, self._accept_joystick_connection)
#     elif self.joystick_socket:
#         self.joystick_socket.close()
#         self.joystick_socket = None

if __name__ == '__main__':
    root = tk.Tk()
    gui = robotDirector(root)
    root.mainloop()


