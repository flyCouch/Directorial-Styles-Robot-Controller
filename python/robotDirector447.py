import tkinter as tk
from tkinter import ttk
from tkinter import filedialog 
from tkinter import messagebox
import serial
import time
import re 
import math 
import socket
import json
import struct
import threading
import queue
from types import MethodType
from pathlib import Path
import os
import sys

# --- Class robotDirector (Unified Code) ---

class robotDirector:

    def __init__(self, master): 
        self.master = master
        master.title("Lyttle ReSearch Robot Director")
        master.config(bg="lightgreen") # Ensure the root window color is green
        
        # --- GUI Style Configuration (Fixes color issue) ---
        style = ttk.Style()
        # Set all standard widget backgrounds to match lightgreen, fixing the 'flash' and ensuring consistency
        style.configure("TFrame", background="lightgreen")
        style.configure("TLabelframe", background="lightgreen")
        style.configure("TLabelframe.Label", background="lightgreen") 
        style.configure("TLabel", background="lightgreen")
        style.configure("TCheckbutton", background="lightgreen")
        style.configure("TRadiobutton", background="lightgreen")
        style.configure("TButton", padding=6)
        # ---------------------------------------------------

        CE_PIN = 10 
        CSN_PIN = 9 

        self.port = tk.StringVar(value="/dev/ttyUSB0") 
        self.baud_rate = 115200 

        self.command_send_queue = queue.Queue()
        self.command_send_thread = None
        self.command_send_thread_running = False

        self.gcode_file_path = tk.StringVar(master) 
        self.gcode_status_label = None 
        self.btn_start_gcode = None
        self.btn_stop_gcode = None

        self.current_laser_power = tk.IntVar(master, value=0)
        self.laser_on = tk.BooleanVar(master, value=False)
        self.radio_status = tk.StringVar(master, value="Idle")
        self.serial_port = None  
        self.arduino_connected = False
        self.running = False 
        self.motion_command = {"x": 0.0, "y": 0.0, "rotation": 0.0, "laser_on": False, "laser_power": 0}
        self.last_sent_motion_command = self.motion_command.copy()
        self.last_sent_motion_command["speed_factor"] = 0.0 

        self.north_angle = 0.0
        self.control_source = "keyboard"
        self.spacebar_pressed = False

        # --- Control Style Definition (All styles visible) ---
        self.control_styles_dict = {
            "Direct X/Y/R Buttons": self.create_xyr_buttons_control,
            "Joystick Control": self.create_joystick_control_area,
            "G-code Director": self.create_gcode_director, 
            "Gcode with AprilTag Corrections": self.create_gcode_with_apriltag_director,
            "External Python Script Director": self.create_python_script_director,
            "SVG/BMP Director": self.create_circle_control, 
            "Tarantino as Director": self.create_tarantino_director,
            "Frickin Shoot Everyone Director": self.create_frickin_shoot_everyone_director,
            ".dxf Director": self.create_dxf_director,
            ".jpg (Python Contour) Director": self.create_jpg_director,
        }
        self.control_styles = list(self.control_styles_dict.keys())
        self.current_control_method = self.control_styles[0]
        # --------------------------------

        self.speed_var = tk.DoubleVar(master, value=0.5)
        self.motion_update_job = None
        self.is_moving = {
            "forward": False, "backward": False, "left": False, "right": False, "CCW": False, "CW": False,}

        self.x_pos = tk.DoubleVar(master, value=0.0)
        self.y_pos = tk.DoubleVar(master, value=0.0)
        self.rotation_val = tk.DoubleVar(master, value=0.0)
        self.elevation_val = tk.DoubleVar(master, value=0.0)

        self.command_throttle_ms = tk.IntVar(master, value=10) 

        # G-code State Variables
        self.gcode_queue = [] 
        self.gcode_current_x = 0.0 
        self.gcode_current_y = 0.0 
        self.gcode_current_laser_on = False
        self.gcode_current_laser_power = 0 
        self.gcode_current_feed_rate = 100.0
        self.gcode_absolute_mode = True
        self.gcode_processing_active = False 
        self.gcode_processing_thread = None
        self.ROBOT_MAX_LINEAR_VELOCITY_MM_PER_MIN = 10000.0 

        # Joystick State Variables
        self.joystick_socket = None 
        self.joystick_connected = False 
        self.joystick_port = 52345
        self.joystick_host = '127.0.0.1'
        self.joystick_buffer_size = 1024 
        self.joystick_data_buffer = '' 
        self.joystick_data_queue = queue.Queue() 
        self.joystick_read_thread = None 
        self.joystick_thread_running = False 
        self._connect_to_joystick_server() 

        # AprilTag State Variables
        self.apriltag_thread_running = False
        self.apriltag_socket = None
        self.apriltag_port = 65000
        self.apriltag_host = '127.0.0.1'
        self.latest_apriltag_pose = None
        
        self.work_area_width_mm = tk.DoubleVar(master, value=300.0) 
        self.work_area_height_mm = tk.DoubleVar(master, value=300.0) 
        
        self.create_widgets()
        self.master.bind('<KeyPress>', self.read_keyboard)
        self.master.bind('<KeyRelease>', self.read_keyrelease)
        self.master.bind('<FocusIn>', self.focus_change_handler, add='+')
        self.master.bind('<FocusOut>', self.focus_change_handler, add='+')
        self.update_radio_status("Disconnected")
        self.connect_arduino_serial() 
        
        self.master.protocol("WM_DELETE_WINDOW", self.on_closing)
        
    # --- Control Style Methods ---

    def create_xyr_buttons_control(self, parent_frame, event=None):
        """Creates the direct X/Y/R button control interface (required for the fix)."""
        ttk.Label(parent_frame, text="Direct X/Y/R Keyboard Control (W/S, A/D, Q/E)").pack(padx=10, pady=10)
        ttk.Label(parent_frame, text="Current Speed Multiplier (F):").pack(padx=10, pady=5)
        ttk.Scale(parent_frame, from_=0.0, to=1.0, orient='horizontal', variable=self.speed_var).pack(padx=10, pady=5, fill='x')
        ttk.Label(parent_frame, textvariable=self.speed_var).pack(padx=10, pady=5)

    def create_joystick_control_area(self, parent_frame, event=None):
        ttk.Label(parent_frame, text="Joystick Control").pack(padx=10, pady=10)
        ttk.Button(parent_frame, text="Connect to Joystick Server", command=self._connect_to_joystick_server).pack(pady=5)
        ttk.Button(parent_frame, text="Disconnect Joystick", command=self._close_joystick_client_connection).pack(pady=5)

    def create_python_script_director(self, parent_frame, event=None):
        ttk.Label(parent_frame, text="External Python Script Director (Not Implemented Yet)").pack(padx=10, pady=10)

    def create_circle_control(self, parent_frame, event=None):
        ttk.Label(parent_frame, text="SVG/BMP Director (Not Implemented Yet)").pack(padx=10, pady=10)

    def create_tarantino_director(self, parent_frame, event=None):
        ttk.Label(parent_frame, text="Tarantino as Director (Not Implemented Yet)").pack(padx=10, pady=10)

    def create_frickin_shoot_everyone_director(self, parent_frame, event=None):
        ttk.Label(parent_frame, text="Frickin Shoot Everyone Director (Not Implemented Yet)").pack(padx=10, pady=10)

    def create_dxf_director(self, parent_frame, event=None):
        ttk.Label(parent_frame, text=".dxf Director (Not Implemented Yet)").pack(padx=10, pady=10)

    def create_jpg_director(self, parent_frame, event=None):
        ttk.Label(parent_frame, text=".jpg (Python Contour) Director (Not Implemented Yet)").pack(padx=10, pady=10)


    # --- G-code Director & File Loading ---
    
    def create_gcode_director(self, parent_frame, event=None):
        """Creates the widgets for the G-code Director style."""
        row_counter = 0

        btn_select_gcode = ttk.Button(parent_frame, text="Select G-code File", command=self.select_gcode_file)
        btn_select_gcode.grid(row=row_counter, column=0, columnspan=2, padx=5, pady=5, sticky="ew")
        row_counter += 1

        self.gcode_status_label = ttk.Label(parent_frame, text="G-code Status: Ready")
        self.gcode_status_label.grid(row=row_counter, column=0, columnspan=2, padx=5, pady=5, sticky="ew")
        row_counter += 1

        lbl_gcode_path = ttk.Label(parent_frame, textvariable=self.gcode_file_path, wraplength=300)
        lbl_gcode_path.grid(row=row_counter, column=0, columnspan=2, padx=5, pady=5, sticky="ew")
        row_counter += 1

        self.btn_start_gcode = ttk.Button(parent_frame, text="Start G-code", command=self.start_gcode_execution, state=tk.DISABLED)
        self.btn_start_gcode.grid(row=row_counter, column=0, padx=5, pady=5, sticky="ew")

        self.btn_stop_gcode = ttk.Button(parent_frame, text="Stop G-code", command=self.stop_gcode_execution, state=tk.DISABLED)
        self.btn_stop_gcode.grid(row=row_counter, column=1, padx=5, pady=5, sticky="ew")
        row_counter += 1

    def create_gcode_with_apriltag_director(self, parent_frame, event=None):
        """Creates the widgets for the G-code with AprilTag Corrections style."""
        self.create_gcode_director(parent_frame, event)
        
        tag_frame = ttk.LabelFrame(parent_frame, text="AprilTag Correction Client", padding="5", borderwidth=1, relief="groove")
        tag_frame.grid(row=10, column=0, columnspan=2, padx=5, pady=10, sticky="ew")
        
        ttk.Label(tag_frame, text="Corrections Active").pack(pady=5)
        ttk.Button(tag_frame, text="Connect AprilTag Client", command=self.start_apriltag_client).pack(pady=5)
        ttk.Button(tag_frame, text="Disconnect AprilTag Client", command=self.stop_apriltag_client).pack(pady=5)

    def select_gcode_file(self):
        """
        Opens a file dialog.
        Relies on the OS/toolkit to handle not displaying hidden files.
        """
        file_path = filedialog.askopenfilename(
            defaultextension=".gcode",
            filetypes=[
                ("All files", "*.*"),
                ("G-code files", "*.gcode"),
                ("Python scripts", "*.py"),
                ("Text files", "*.txt"),
            ]
        )
        if file_path:
            self.gcode_file_path.set(file_path)
            # Reset G-code state variables
            self.gcode_queue = []
            self.gcode_current_x = 0.0
            self.gcode_current_y = 0.0
            self.gcode_current_laser_on = False
            self.gcode_current_laser_power = 0
            self.gcode_current_feed_rate = 100.0
            self.gcode_absolute_mode = True
            
            # Enable start button 
            if self.btn_start_gcode:
                self.btn_start_gcode.config(state=tk.NORMAL)
            if self.gcode_status_label:
                self.gcode_status_label.config(text=f"G-code Status: File Loaded")

    # --- G-code Execution Logic (Fixed) ---

    def parse_gcode_line(self, line):
        """Parses a G-code line for relevant commands and values."""
        line = line.split(';')[0].strip().upper()
        if not line:
            return None

        command = {}
        
        match_g = re.search(r'G(\d+)', line)
        if match_g:
            command['G'] = int(match_g.group(1))

        match_m = re.search(r'M(\d+)', line)
        if match_m:
            command['M'] = int(match_m.group(1))

        for code in ['X', 'Y', 'F', 'S']:
            match_val = re.search(rf'{code}([-+]?\d*\.?\d+)', line)
            if match_val:
                command[code] = float(match_val.group(1))
        
        return command if command else None

    def start_gcode_execution(self):
        """Reads the selected file and starts the G-code processing thread."""
        file_path = self.gcode_file_path.get()
        if not file_path:
            messagebox.showerror("Error", "Please select a G-code file first.")
            return

        try:
            with open(file_path, 'r') as f:
                self.gcode_current_x = 0.0 
                self.gcode_current_y = 0.0 
                self.gcode_current_laser_on = False
                self.gcode_current_laser_power = 0 
                self.gcode_current_feed_rate = 100.0 
                self.gcode_absolute_mode = True
                
                self.gcode_queue = [self.parse_gcode_line(line) for line in f if self.parse_gcode_line(line)]
        except Exception as e:
            messagebox.showerror("Error", f"Failed to read file: {e}")
            return
        
        if not self.gcode_queue:
            messagebox.showwarning("Warning", "The G-code file contains no recognized commands.")
            return

        self.gcode_processing_active = True
        if self.btn_start_gcode: self.btn_start_gcode.config(state=tk.DISABLED)
        if self.btn_stop_gcode: self.btn_stop_gcode.config(state=tk.NORMAL)
        
        self.gcode_processing_thread = threading.Thread(target=self._process_gcode_thread_target, daemon=True)
        self.gcode_processing_thread.start()
        self.gcode_status_label.config(text="G-code Status: Running...")

    def stop_gcode_execution(self):
        """Stops the G-code processing thread and resets buttons."""
        self.gcode_processing_active = False
        if self.gcode_processing_thread and self.gcode_processing_thread.is_alive():
            # Send stop command to robot immediately
            self.motion_command.update({"x": 0.0, "y": 0.0, "rotation": 0.0})
            self.command_send_queue.put({"x": 0.0, "y": 0.0, "rotation": 0.0, "laser_on": self.gcode_current_laser_on, "laser_power": int(self.gcode_current_laser_power), "speed_factor": 0.0})
            
            self.gcode_processing_thread.join(timeout=0.1)
        self.gcode_processing_thread = None
        
        if self.btn_start_gcode: self.btn_start_gcode.config(state=tk.NORMAL)
        if self.btn_stop_gcode: self.btn_stop_gcode.config(state=tk.DISABLED)
        if self.gcode_status_label: self.gcode_status_label.config(text="G-code Status: Stopped.")


    def _process_gcode_thread_target(self):
        """Target for the G-code execution thread."""
        try:
            for i, command in enumerate(self.gcode_queue):
                if not self.gcode_processing_active:
                    break
                
                self.master.after(0, lambda i=i: self.gcode_status_label.config(text=f"G-code Status: Line {i+1}/{len(self.gcode_queue)}"))
                
                self._process_next_gcode_command(command)

        except Exception as e:
            self.master.after(0, lambda e=e: messagebox.showerror("G-code Error", f"Execution failed: {e}"))
        finally:
            self.master.after(0, self.stop_gcode_execution)
            self.master.after(0, lambda: self.gcode_status_label.config(text="G-code Status: Done."))


    def _process_next_gcode_command(self, command):
        """
        Processes a single parsed G-code command. 
        FIXED: Calculates a directional vector (MX/MY) and a speed factor (S) 
        based on G-code F rate and time.
        """
        
        if 'M' in command:
            m_code = command['M']
            if m_code == 3: # Laser/Spindle ON
                self.gcode_current_laser_on = True
                self.gcode_current_laser_power = command.get('S', 100)
            elif m_code == 5: # Laser/Spindle OFF
                self.gcode_current_laser_on = False
                self.gcode_current_laser_power = 0
            
            self.command_send_queue.put({
                "x": 0.0, "y": 0.0, "rotation": 0.0, 
                "laser_on": self.gcode_current_laser_on,
                "laser_power": int(self.gcode_current_laser_power),
                "speed_factor": 0.0
            })
            time.sleep(0.01)
            return
        
        g_code = command.get('G')
        
        if g_code == 90: 
            self.gcode_absolute_mode = True
            return
        elif g_code == 91:
            self.gcode_absolute_mode = False
            return
            
        if 'F' in command:
            self.gcode_current_feed_rate = command['F']

        if g_code in [0, 1]: 
            target_x = command.get('X', self.gcode_current_x)
            target_y = command.get('Y', self.gcode_current_y)
            
            if not self.gcode_absolute_mode:
                target_x = self.gcode_current_x + target_x
                target_y = self.gcode_current_y + target_y

            # AprilTag Correction
            if self.current_control_method == "Gcode with AprilTag Corrections":
                corrected_x, corrected_y = self.perform_apriltag_correction(target_x, target_y)
            else:
                corrected_x, corrected_y = target_x, target_y


            target_dx = corrected_x - self.gcode_current_x
            target_dy = corrected_y - self.gcode_current_y

            distance_mm = math.sqrt(target_dx**2 + target_dy**2)
            
            if distance_mm > 1e-6:
                vx_norm = target_dx / distance_mm
                vy_norm = target_dy / distance_mm

                MAX_ROBOT_SPEED_MM_PER_MIN = self.ROBOT_MAX_LINEAR_VELOCITY_MM_PER_MIN
                
                if g_code == 0:
                    target_feed_rate_mm_per_min = MAX_ROBOT_SPEED_MM_PER_MIN
                else:
                    target_feed_rate_mm_per_min = self.gcode_current_feed_rate

                speed_factor_f = min(1.0, target_feed_rate_mm_per_min / MAX_ROBOT_SPEED_MM_PER_MIN if MAX_ROBOT_SPEED_MM_PER_MIN > 0 else 0.0)
                
                if target_feed_rate_mm_per_min > 0:
                    move_duration = (distance_mm / target_feed_rate_mm_per_min) * 60.0
                else:
                    move_duration = 0.01 

                # --- START MOVEMENT ---
                command_start = {
                    "x": vx_norm,
                    "y": vy_norm,
                    "rotation": 0.0, 
                    "laser_on": self.gcode_current_laser_on,
                    "laser_power": int(self.gcode_current_laser_power),
                    "speed_factor": speed_factor_f 
                }
                self.command_send_queue.put(command_start) 
                
                time.sleep(move_duration)
                
                # --- STOP MOVEMENT ---
                command_stop = {
                    "x": 0.0, "y": 0.0, "rotation": 0.0, 
                    "laser_on": self.gcode_current_laser_on,
                    "laser_power": int(self.gcode_current_laser_power),
                    "speed_factor": 0.0
                }
                self.command_send_queue.put(command_stop)
            else:
                time.sleep(0.01)

            # Update internal position state
            self.gcode_current_x = corrected_x
            self.gcode_current_y = corrected_y

    # --- AprilTag Client Logic ---

    def start_apriltag_client(self, host='127.0.0.1', port=65000):
        if getattr(self, 'apriltag_thread_running', False):
            return
        
        self.apriltag_host = host
        self.apriltag_port = port
        self.apriltag_thread_running = True
        self.latest_apriltag_pose = None
        self.apriltag_data_buffer = ''

        try:
            self.apriltag_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.apriltag_socket.settimeout(2.0)
            self.apriltag_socket.connect((self.apriltag_host, self.apriltag_port))
            self.apriltag_socket.setblocking(False)

            self.apriltag_read_thread = threading.Thread(target=self._apriltag_read_thread_target, daemon=True)
            self.apriltag_read_thread.start()
            self.update_radio_status("AprilTag Client Connected")
        except Exception as e:
            self.apriltag_thread_running = False
            self.apriltag_socket = None
            self.update_radio_status(f"AprilTag Client Error: {e}")

    def _apriltag_read_thread_target(self):
        while self.apriltag_thread_running and self.apriltag_socket:
            try:
                chunk = self.apriltag_socket.recv(1024)
                if not chunk:
                    self.apriltag_thread_running = False
                    self.master.after(0, lambda: self.update_radio_status("AprilTag Server Disconnected"))
                    break
                
                self.apriltag_data_buffer += chunk.decode('utf-8')

                while '\n' in self.apriltag_data_buffer:
                    message_string, self.apriltag_data_buffer = self.apriltag_data_buffer.split('\n', 1)
                    if message_string.strip():
                        try:
                            pose_data = json.loads(message_string)
                            self.latest_apriltag_pose = pose_data
                        except json.JSONDecodeError:
                            pass
            
            except BlockingIOError:
                time.sleep(0.01)
            except Exception:
                self.apriltag_thread_running = False
                self.master.after(0, lambda: self.update_radio_status("AprilTag Client Read Error"))
                break

    def stop_apriltag_client(self):
        self.apriltag_thread_running = False
        if self.apriltag_socket:
            try:
                self.apriltag_socket.shutdown(socket.SHUT_RDWR)
                self.apriltag_socket.close()
            except OSError:
                pass
            self.apriltag_socket = None
        self.latest_apriltag_pose = None
        self.update_radio_status("AprilTag Client Disconnected")

    def perform_apriltag_correction(self, target_x_mm, target_y_mm):
        """Calculates a corrected target position using the latest AprilTag pose."""
        if not self.latest_apriltag_pose:
            return target_x_mm, target_y_mm

        # Error is the difference between *expected* G-code position and *measured* real-world position
        error_x = self.gcode_current_x - self.latest_apriltag_pose.get('x', self.gcode_current_x)
        error_y = self.gcode_current_y - self.latest_apriltag_pose.get('y', self.gcode_current_y)
        
        # Corrected Target = Original Target + Error Offset
        corrected_x = target_x_mm + error_x 
        corrected_y = target_y_mm + error_y

        return corrected_x, corrected_y

    # --- Other Methods (Standard functionality) ---

    def on_closing(self):
        """Handles graceful shutdown of the application."""
        print("Closing application. Attempting graceful shutdown of threads and connections...")
        self.running = False
        self.command_send_thread_running = False
        self.joystick_thread_running = False
        self.gcode_processing_active = False
        self.stop_apriltag_client() 
        
        if hasattr(self, 'serial_read_thread') and self.serial_read_thread and self.serial_read_thread.is_alive():
            print("Joining serial read thread...")
            self.serial_read_thread.join(timeout=0.5)
        
        if self.command_send_thread and self.command_send_thread.is_alive():
            print("Joining command send thread...")
            self.command_send_queue.put(None) 
            self.command_send_thread.join(timeout=0.5)
        
        if self.gcode_processing_thread and self.gcode_processing_thread.is_alive():
            print("Joining G-code processing thread...")
            self.gcode_processing_thread.join(timeout=0.5)


        if self.joystick_read_thread and self.joystick_read_thread.is_alive():
            print("Joining joystick read thread...")
            self.joystick_data_queue.put(None)
            self.joystick_read_thread.join(timeout=0.5)

        if self.serial_port and self.serial_port.is_open:
            print("Closing serial port...")
            self.serial_port.close()

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
        if self.motion_update_job:
            self.master.after_cancel(self.motion_update_job)
            self.motion_update_job = None
        self._send_repeated_command()

    def disconnect_serial(self):
        self.running = False
        if hasattr(self, 'serial_read_thread') and self.serial_read_thread and self.serial_read_thread.is_alive():
            self.serial_read_thread.join(timeout=0.5)
        setattr(self, 'serial_read_thread', None) 

        self.command_send_thread_running = False
        if self.command_send_thread and self.command_send_thread.is_alive():
            self.command_send_queue.put(None) 
            self.command_send_thread.join(timeout=0.5)
        self.command_send_thread = None

        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            self.arduino_connected = False
            self.update_radio_status("Disconnected.")
            print("Serial port disconnected.")
        else:
            self.update_radio_status("Not connected.")
            print("Serial port not open.")

    def _connect_to_joystick_server(self):
        if self.joystick_read_thread and self.joystick_read_thread.is_alive():
            return

        try:
            self.joystick_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.joystick_socket.settimeout(1.0)
            self.joystick_socket.connect((self.joystick_host, self.joystick_port))
            self.joystick_socket.setblocking(False)
            
            self.joystick_data_buffer = ''
            
            self.joystick_thread_running = True
            self.joystick_read_thread = threading.Thread(target=self._joystick_read_thread_target, daemon=True)
            self.joystick_read_thread.start()

            self.master.after(50, self._process_joystick_queue)
            self.update_radio_status("Joystick Connected")

        except Exception as e:
            self._close_joystick_client_connection()

    def _joystick_read_thread_target(self):
        while self.joystick_thread_running and self.joystick_socket:
            try:
                chunk = self.joystick_socket.recv(self.joystick_buffer_size)
                if not chunk:
                    self.joystick_thread_running = False
                    self.joystick_data_queue.put("DISCONNECTED") 
                    break
                
                self.joystick_data_queue.put(chunk.decode('utf-8'))

            except BlockingIOError:
                time.sleep(0.01)
            except Exception:
                self.joystick_thread_running = False
                self.joystick_data_queue.put("ERROR_SOCKET")
                break

    def _process_joystick_queue(self):
        if self.current_control_method != "Joystick Control":
            while not self.joystick_data_queue.empty():
                try: self.joystick_data_queue.get_nowait()
                except queue.Empty: break
            if self.joystick_read_thread and self.joystick_read_thread.is_alive():
                self.master.after(50, self._process_joystick_queue)
            return

        while not self.joystick_data_queue.empty():
            item = self.joystick_data_queue.get_nowait()
            
            if item is None: return

            if item in ["DISCONNECTED", "ERROR_SOCKET", "ERROR_UNKNOWN"]:
                self._close_joystick_client_connection()
                return

            self.joystick_data_buffer += item 

            while '\n' in self.joystick_data_buffer:
                message_string, self.joystick_data_buffer = self.joystick_data_buffer.split('\n', 1)
                
                if not message_string.strip(): continue

                try:
                    joystick_data = json.loads(message_string)
                    new_speed_multiplier_from_joystick = float(joystick_data.get("speed", self.speed_var.get()))
                    
                    if abs(self.speed_var.get() - new_speed_multiplier_from_joystick) > 1e-6:
                        self.speed_var.set(new_speed_multiplier_from_joystick)

                    current_speed_factor = self.speed_var.get()

                    temp_motion_command = {
                        "x": float(joystick_data.get("x", 0.0)) * current_speed_factor,
                        "y": float(joystick_data.get("y", 0.0)) * current_speed_factor,
                        "rotation": float(joystick_data.get("r", 0.0)) * current_speed_factor,
                        "laser_on": bool(joystick_data.get("laser", 0)),
                        "laser_power": int(joystick_data.get("power", self.current_laser_power.get()))
                    }

                    if self.laser_on.get() != temp_motion_command["laser_on"]:
                        self.laser_on.set(temp_motion_command["laser_on"])
                    if self.current_laser_power.get() != temp_motion_command["laser_power"]:
                        self.current_laser_power.set(temp_motion_command["laser_power"])
                    
                    self.motion_command.update(temp_motion_command)

                    # --- Motion/State Change Check ---
                    motion_x_changed = abs(self.motion_command["x"] - self.last_sent_motion_command["x"]) > 1e-6
                    motion_y_changed = abs(self.motion_command["y"] - self.last_sent_motion_command["y"]) > 1e-6
                    motion_r_changed = abs(self.motion_command["rotation"] - self.last_sent_motion_command["rotation"]) > 1e-6
                    laser_on_changed = self.motion_command["laser_on"] != self.last_sent_motion_command["laser_on"]
                    laser_power_changed = self.motion_command["laser_power"] != self.last_sent_motion_command["laser_power"]
                    speed_factor_changed = abs(current_speed_factor - self.last_sent_motion_command.get("speed_factor", 0.0)) > 1e-6
                    
                    should_queue_command = False
                    
                    if motion_x_changed or motion_y_changed or motion_r_changed or laser_on_changed or laser_power_changed:
                        should_queue_command = True
                    elif speed_factor_changed and (abs(self.motion_command["x"]) > 1e-6 or abs(self.motion_command["y"]) > 1e-6 or abs(self.motion_command["rotation"]) > 1e-6):
                        should_queue_command = True
                    elif speed_factor_changed and not (abs(self.motion_command["x"]) > 1e-6 or abs(self.motion_command["y"]) > 1e-6 or abs(self.motion_command["rotation"]) > 1e-6) and abs(current_speed_factor) >= 1e-6:
                        should_queue_command = True


                    if should_queue_command:
                        self.command_send_queue.put(self.motion_command.copy())
                        self.last_sent_motion_command = self.motion_command.copy()
                        self.last_sent_motion_command["speed_factor"] = current_speed_factor

                except Exception:
                    pass

        if self.joystick_read_thread and self.joystick_read_thread.is_alive():
            self.master.after(50, self._process_joystick_queue)

    def _close_joystick_client_connection(self):
        self.joystick_thread_running = False
        
        if self.joystick_socket:
            try:
                self.joystick_socket.shutdown(socket.SHUT_RDWR)
                self.joystick_socket.close()
            except OSError:
                pass
            self.joystick_socket = None

        if self.joystick_read_thread and self.joystick_read_thread.is_alive():
            self.joystick_data_queue.put(None) 
            self.joystick_read_thread.join(timeout=0.5)
        self.joystick_read_thread = None

        self.update_radio_status("Joystick Disconnected")

    def send_control_command(self):
        if not self.arduino_connected or not self.serial_port or not self.serial_port.is_open:
            self.radio_status.set("Bridge Not Connected")
            return
        
        command_to_queue = self.motion_command.copy()
        command_to_queue["speed_factor"] = self.speed_var.get()
        self.command_send_queue.put(command_to_queue)

    def _send_command_to_serial_bridge(self, command_data):
        if not self.arduino_connected or not self.serial_port or not self.serial_port.is_open:
            self.master.after(0, lambda: self.radio_status.set("Bridge Not Connected"))
            return
        
        try:
            motion_x = command_data.get("x", 0.0)
            motion_y = command_data.get("y", 0.0)
            rotation = command_data.get("rotation", 0.0)
            laser = int(command_data.get("laser_on", False))
            laser_power = command_data.get("laser_power", 0)
            target_speed = command_data.get("speed_factor", self.speed_var.get())

            command_string = (
                f"MX:{motion_x: .8f},"
                f"MY:{motion_y: .8f},"
                f"R:{rotation: .8f},"
                f"L:{laser},"       
                f"P:{laser_power}," 
                f"S:{target_speed: .8f}\n" 
            )

            self.serial_port.write(command_string.encode('utf-8'))
            self.master.after(0, lambda s=command_string.strip(): self.radio_status.set(f"Bridge Sent: {s}"))

        except serial.SerialException as e:
            self.master.after(0, lambda: self.radio_status.set(f"Bridge Serial Error: {e}"))
        except Exception as e:
            self.master.after(0, lambda: self.radio_status.set(f"Command Error: {e}"))

    def focus_change_handler(self,event):
        pass

    def read_keyboard(self, event):
        focused_widget = self.master.focus_get()
        if isinstance(focused_widget, ttk.Entry) or isinstance(focused_widget, tk.Entry):
            return

        keysym = event.keysym.lower()

        if keysym == 'space':
            self.laser_on.set(True)
            self.motion_command["laser_on"] = True
            self.motion_command["laser_power"] = self.current_laser_power.get()
            self.send_control_command()
            self.spacebar_pressed = True
            return

        if self.current_control_method == "Direct X/Y/R Buttons":
            speed = self.speed_var.get()

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
                self.north_angle = (self.north_angle + 5) % 360
            elif keysym == 'e':
                self.is_moving['CW'] = True
                self.north_angle = (self.north_angle - 5) % 360
            else:
                return

            vx, vy, omega = 0.0, 0.0, 0.0
            if self.is_moving['forward']: vy += speed
            if self.is_moving['backward']: vy -= speed
            if self.is_moving['right']: vx += speed
            if self.is_moving['left']: vx -= speed
            if self.is_moving['CCW']: omega += speed
            if self.is_moving['CW']: omega -= speed

            self.motion_command["x"] = vx
            self.motion_command["y"] = vy
            self.motion_command["rotation"] = omega

            if not self.motion_update_job:
                self._start_motion_sending_loop()

        else:
            pass

    def read_keyrelease(self, event):
        focused_widget = self.master.focus_get()
        if isinstance(focused_widget, ttk.Entry) or isinstance(focused_widget, tk.Entry):
            return

        keysym = event.keysym.lower()

        if keysym == 'space':
            self.laser_on.set(False)
            self.motion_command["laser_on"] = False
            self.motion_command["laser_power"] = 0
            self.send_control_command()
            self.spacebar_pressed = False
            return

        control_method_value = self.current_control_method

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

            if not any(self.is_moving.values()):
                self.motion_command["x"] = 0.0
                self.motion_command["y"] = 0.0
                self.motion_command["rotation"] = 0.0
                self.send_control_command()
                self._stop_motion_sending_loop()
            else:
                vx, vy, omega = 0.0, 0.0, 0.0
                speed = self.speed_var.get()
                if self.is_moving['forward']: vy += speed
                if self.is_moving['backward']: vy -= speed
                if self.is_moving['right']: vx += speed
                if self.is_moving['left']: vx -= speed
                if self.is_moving['CCW']: omega += speed
                if self.is_moving['CW']: omega -= speed
                self.motion_command["x"] = vx
                self.motion_command["y"] = vy
                self.motion_command["rotation"] = omega
                self.send_control_command()

    def _stop_motion_sending_loop(self):
        if self.motion_update_job:
            self.master.after_cancel(self.motion_update_job)
            self.motion_update_job = None
        
    def _send_repeated_command(self):
        vx, vy, omega = 0.0, 0.0, 0.0
        speed = self.speed_var.get()
        if self.is_moving['forward']: vy += speed
        if self.is_moving['backward']: vy -= speed
        if self.is_moving['right']: vx += speed
        if self.is_moving['left']: vx -= speed
        if self.is_moving['CCW']: omega += speed
        if self.is_moving['CW']: omega -= speed

        self.motion_command["x"] = vx
        self.motion_command["y"] = vy
        self.motion_command["rotation"] = omega

        is_moving_now = (abs(self.motion_command["x"]) > 1e-6 or abs(self.motion_command["y"]) > 1e-6 or abs(self.motion_command["rotation"]) > 1e-6)
        
        if self.current_control_method == "Direct X/Y/R Buttons" and (is_moving_now or self.motion_command["laser_on"] != self.last_sent_motion_command["laser_on"] or self.motion_command["laser_power"] != self.last_sent_motion_command["laser_power"]):
             self.send_control_command()

        if is_moving_now:
            delay = self.command_throttle_ms.get()
            self.motion_update_job = self.master.after(delay, self._send_repeated_command)
        else:
            self.motion_update_job = None


    def _send_command_thread_target(self):
        self.command_send_thread_running = True
        while self.command_send_thread_running:
            try:
                command_data = self.command_send_queue.get(timeout=0.1) 
                
                if command_data is None: 
                    self.command_send_thread_running = False
                    break 

                self._send_command_to_serial_bridge(command_data)
                self.command_send_queue.task_done()
                
            except queue.Empty:
                continue
            except Exception as e:
                self.master.after(0, lambda: self.radio_status.set(f"Command Sender Error: {e}"))
                time.sleep(0.5)
                
        print("Command sender thread finished.")


    def serial_read_thread_target(self):
        self.running = True
        while self.running and self.serial_port and self.serial_port.is_open:
            try:
                if self.serial_port.in_waiting > 0:
                    line = self.serial_port.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        self.master.after(0, lambda l=line: self.update_radio_status(f"Bridge Recv: {l}"))
            except serial.SerialException:
                break
            except Exception:
                break
            time.sleep(0.01)
        self.running = False
        self.master.after(0, lambda: self.connect_button.config(text="Connect", command=self.connect_arduino_serial))
        self.master.after(0, lambda: self.update_radio_status("Disconnected"))
        print("Serial reader thread finished.")

    def connect_arduino_serial(self, event=None):
        if self.arduino_connected:
            self.disconnect_serial()
            return

        port_val = self.port.get()
        try:
            self.serial_port = serial.Serial(port_val, self.baud_rate, timeout=0.1)
            time.sleep(2)

            self.arduino_connected = True
            self.update_radio_status("Connected")
            self.connect_button.config(text="Disconnect", command=self.disconnect_serial)
            
            self.command_send_thread = threading.Thread(target=self._send_command_thread_target, daemon=True)
            self.command_send_thread.start()

            self.serial_read_thread = threading.Thread(target=self.serial_read_thread_target, daemon=True)
            self.serial_read_thread.start()

        except serial.SerialException as e:
            self.arduino_connected = False
            self.update_radio_status(f"Error: {e}")
            messagebox.showerror("Connection Error", f"Could not connect to {port_val}: {e}")

    def update_radio_status(self, new_status):
        self.radio_status.set(new_status)

    def on_control_style_change(self, *args):
        for widget in self.control_frame.winfo_children():
            widget.destroy()

        self.current_control_method = self.control_var.get()

        create_func = self.control_styles_dict.get(self.current_control_method)
        if create_func:
            self.motion_command["x"] = 0.0
            self.motion_command["y"] = 0.0
            self.motion_command["rotation"] = 0.0
            self.motion_command["laser_on"] = self.laser_on.get() 
            self.motion_command["laser_power"] = self.current_laser_power.get()
            self.send_control_command()
            self._stop_motion_sending_loop()
            
            if self.gcode_processing_active:
                self.stop_gcode_execution()

            create_func(self.control_frame)
            
            if self.current_control_method.startswith("G-code"):
                if self.gcode_file_path.get():
                    if self.btn_start_gcode: self.btn_start_gcode.config(state=tk.NORMAL)


    def create_widgets(self):
        """Sets up the main GUI structure."""
        
        main_frame = ttk.Frame(self.master, padding="10")
        main_frame.pack(fill='both', expand=True)

        # ------------------- STATUS & CONNECTION FRAME -------------------
        status_frame = ttk.LabelFrame(main_frame, text="Connection & Status", padding="10")
        status_frame.grid(row=0, column=0, padx=10, pady=5, sticky="ew")

        ttk.Label(status_frame, text="Serial Port:").grid(row=0, column=0, padx=5, pady=2, sticky="w")
        ttk.Entry(status_frame, textvariable=self.port, width=20).grid(row=0, column=1, padx=5, pady=2, sticky="ew")

        self.connect_button = ttk.Button(status_frame, text="Connect", command=self.connect_arduino_serial)
        self.connect_button.grid(row=0, column=2, padx=5, pady=2)

        ttk.Label(status_frame, text="Status:").grid(row=1, column=0, padx=5, pady=2, sticky="w")
        ttk.Label(status_frame, textvariable=self.radio_status, background="yellow").grid(row=1, column=1, columnspan=2, padx=5, pady=2, sticky="ew")

        ttk.Label(status_frame, text="Command Throttle (ms):").grid(row=2, column=0, padx=5, pady=2, sticky="w")
        ttk.Entry(status_frame, textvariable=self.command_throttle_ms, width=5).grid(row=2, column=1, padx=5, pady=2, sticky="w")
        
        # ------------------- CONTROL STYLE SELECTION -------------------
        style_frame = ttk.LabelFrame(main_frame, text="Director Style", padding="10")
        style_frame.grid(row=1, column=0, padx=10, pady=5, sticky="ew")

        self.control_var = tk.StringVar(self.master)
        self.control_var.set(self.current_control_method)
        self.control_var.trace_add("write", self.on_control_style_change)

        ttk.Label(style_frame, text="Select Director:").pack(padx=5, pady=2, fill='x')
        ttk.OptionMenu(style_frame, self.control_var, self.current_control_method, *self.control_styles).pack(padx=5, pady=2, fill='x')


        # ------------------- CONTROL SPECIFIC FRAME (DYNAMIC) -------------------
        self.control_frame = ttk.LabelFrame(main_frame, text="Director Controls", padding="10")
        self.control_frame.grid(row=2, column=0, padx=10, pady=5, sticky="nsew")

        self.on_control_style_change() 

        # ------------------- LASER CONTROL FRAME -------------------
        laser_frame = ttk.LabelFrame(main_frame, text="Laser/Spindle", padding="10")
        laser_frame.grid(row=3, column=0, padx=10, pady=5, sticky="ew")

        ttk.Checkbutton(laser_frame, text="Laser On (Hold SPACE)", variable=self.laser_on, command=lambda: self.send_control_command()).grid(row=0, column=0, padx=5, pady=2, sticky="w")

        ttk.Label(laser_frame, text="Power (0-255):").grid(row=1, column=0, padx=5, pady=2, sticky="w")
        ttk.Scale(laser_frame, from_=0, to=255, orient='horizontal', variable=self.current_laser_power, command=lambda *a: self.send_control_command()).grid(row=1, column=1, padx=5, pady=2, sticky="ew")
        ttk.Label(laser_frame, textvariable=self.current_laser_power).grid(row=1, column=2, padx=5, pady=2, sticky="w")
        
        main_frame.grid_columnconfigure(0, weight=1)
        main_frame.grid_rowconfigure(2, weight=1)

if __name__ == '__main__':
    root = tk.Tk()
    app = robotDirector(root)
    root.mainloop()
