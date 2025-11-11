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

# --- Imports needed for Joystick Control (v469/v480) ---
try:
    import pygame
    os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = "hide"
except ImportError:
    pygame = None
import atexit 
# --------------------------------------------------------

# --- Class robotDirector (Unified Code) ---
class robotDirector:

    def __init__(self, master): 
        self.master = master
        master.title("Lyttle ReSearch Robot Director")
        master.config(bg="lightgreen") 
        master.resizable(False, False) 
        
        # --- GUI Style Configuration (v469) ---
        style = ttk.Style()
        style.configure("TFrame", background="lightgreen")
        style.configure("TLabelframe", background="lightgreen")
        style.configure("TLabelframe.Label", background="lightgreen") 
        style.configure("TLabel", background="lightgreen")
        style.configure("TCheckbutton", background="lightgreen")
        style.configure("TRadiobutton", background="lightgreen")
        style.configure("TButton", padding=6)
        # -----------------------------

        CE_PIN = 10 
        CSN_PIN = 9 

        self.port = tk.StringVar(value="/dev/ttyUSB0") 
        self.baud_rate = 115200 

        # --- Threading/Command Management (v370/v469) ---
        self.command_send_queue = queue.Queue()
        self.command_send_thread = None
        self.command_send_thread_running = False
        self.command_throttle_ms = tk.IntVar(master, value=10) 

        # --- G-code Variables (v469) ---
        self.gcode_file_path = tk.StringVar(master) 
        self.gcode_status_label = None 
        self.btn_start_gcode = None
        self.btn_stop_gcode = None
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
        self.stop_gcode_flag = False 

        # --- Serial/Radio/Motion Variables (v370/v469) ---
        self.current_laser_power = tk.IntVar(master, value=0)
        self.laser_on = tk.BooleanVar(master, value=False)
        self.radio_status = tk.StringVar(master, value="Idle")
        self.serial_port = None  
        self.arduino_connected = False
        self.running = False 

        # Motion Command (v469 style)
        self.motion_command = {"x": 0.0, "y": 0.0, "rotation": 0.0, "laser_on": False, "laser_power": 0}
        self.last_sent_motion_command = self.motion_command.copy()
        self.last_sent_motion_command["speed_factor"] = 0.0 

        self.north_angle = 0.0
        self.control_source = "keyboard"
        self.spacebar_pressed = False
        self.speed_var = tk.DoubleVar(master, value=0.5) 
        self.motion_update_job = None 
        self.is_moving = { 
            "forward": False, "backward": False, "left": False, "right": False, "CCW": False, "CW": False,}

        # Localization/Position Display (CNC Coords)
        # Using StringVar for robust cross-thread GUI updates
        self.x_pos = tk.StringVar(master, value="0.00") 
        self.y_pos = tk.StringVar(master, value="0.00")
        self.rotation_val = tk.StringVar(master, value="0.00")
        self.elevation_val = tk.StringVar(master, value="0.00")
        
        # Work Area Variables (v469)
        self.work_area_width_mm = tk.DoubleVar(master, value=300.0) 
        self.work_area_height_mm = tk.DoubleVar(master, value=300.0) 

        # --- Vision (AprilTag) Tracking and Origin Management (v480) ---
        # CRITICAL FIX (V10): Initialize to a sentinel value to detect stale reads.
        self.current_tag_x_mm = -99999.9 
        self.current_tag_y_mm = -99999.9
        self.current_tag_r_deg = -99999.9
        self.vision_offset_x = tk.DoubleVar(master, value=0.0)
        self.vision_offset_y = tk.DoubleVar(master, value=0.0)
        self.vision_offset_r = tk.DoubleVar(master, value=0.0) 
        self.is_vision_calibrated = tk.BooleanVar(master, value=False)

        # AprilTag/Socket Control Variables (v480)
        self.tag_ip = tk.StringVar(master, value="127.0.0.1")
        # Default port changed to 65000 based on user feedback
        self.tag_port = tk.IntVar(master, value=65000) 
        self.tag_socket = None
        self.tag_thread = None
        self.tag_thread_running = False
        self.tag_status = tk.StringVar(master, value="Disconnected")
        self.tag_position = tk.StringVar(master, value="X:0.0 Y:0.0 R:0.0")
        
        # --- Control Style Definition (v469 + v480 additions) ---
        self.control_styles_dict = {
            "Direct X/Y/R Buttons": self.create_xyr_buttons_control,
            "Joystick Control": self.create_joystick_control_area,
            "G-code Director": self.create_gcode_director, 
            "Gcode with AprilTag Corrections": self.create_gcode_with_apriltag_director,
            "External Python Script Director": self.create_python_script_director,
            "SVG/BMP Director": self.create_svg_bmp_director, 
            "Tarantino as Director": self.create_tarantino_director,
            "Frickin Shoot Everyone Director": self.create_frickin_shoot_everyone_director,
            ".dxf Director": self.create_dxf_director,
            ".jpg (Python Contour) Director": self.create_jpg_director,
        }
        self.control_styles = list(self.control_styles_dict.keys())
        
        # --- Default style to Keyboard ---
        self.current_control_method = "Direct X/Y/R Buttons" 
        # ---------------------------------
        
        self.control_frame = None 
        
        # --- Joystick Client Variables (v469/v370) ---
        self.joystick_socket = None 
        self.joystick_connected = False 
        self.joystick_port = 52345
        self.joystick_host = '127.0.0.1'
        self.joystick_buffer_size = 1024 
        self.joystick_data_buffer = '' 
        self.joystick_data_queue = queue.Queue() 
        self.joystick_read_thread = None 
        self.joystick_thread_running = False 

        # --- Initial Setup ---
        self.create_widgets() # Full GUI from v469
        self.master.bind('<KeyPress>', self.read_keyboard)
        self.master.bind('<KeyRelease>', self.read_keyrelease)
        self.master.bind('<FocusIn>', self.focus_change_handler, add='+')
        self.master.bind('<FocusOut>', self.focus_change_handler, add='+')
        self.update_radio_status("Disconnected")
        threading.Thread(target=self.connect_arduino_serial, daemon=True).start()
        threading.Thread(target=self._connect_to_joystick_server, daemon=True).start()


        # Register cleanup function
        atexit.register(self.cleanup)
        self.master.protocol("WM_DELETE_WINDOW", self._on_closing)

    # -----------------------------------------------------------
    # --- GUI Creation Methods (v469) ---
    # -----------------------------------------------------------
    
    def create_widgets(self):
        """Creates the main GUI layout, incorporating v469's structure."""
        main_frame = ttk.Frame(self.master, padding="10 10 10 10")
        main_frame.grid(row=0, column=0, sticky="nsew")
        self.master.grid_columnconfigure(0, weight=1)
        self.master.grid_rowconfigure(0, weight=1)

        # ------------------- CONNECTION FRAME -------------------
        self.create_connection_area(main_frame)

        # ------------------- POSITION DISPLAY FRAME -------------------
        self.create_position_display(main_frame)
        
        # ------------------- WORK AREA FRAME -------------------
        work_area_frame = ttk.LabelFrame(main_frame, text="Work Area (mm)", padding="10")
        work_area_frame.grid(row=2, column=0, padx=10, pady=5, sticky="ew")
        
        ttk.Label(work_area_frame, text="Width:").grid(row=0, column=0, padx=5, pady=2, sticky="w")
        ttk.Entry(work_area_frame, textvariable=self.work_area_width_mm, width=10).grid(row=0, column=1, padx=5, pady=2, sticky="w")
        ttk.Label(work_area_frame, text="Height:").grid(row=1, column=0, padx=5, pady=2, sticky="w")
        ttk.Entry(work_area_frame, textvariable=self.work_area_height_mm, width=10).grid(row=1, column=1, padx=5, pady=2, sticky="w")


        # ------------------- CONTROL STYLE SELECTION -------------------
        self.create_control_style_selection(main_frame)

        # ------------------- CONTROL SPECIFIC FRAME (DYNAMIC) -------------------
        self.control_frame = ttk.LabelFrame(main_frame, text="Director Controls", padding="10")
        self.control_frame.grid(row=4, column=0, padx=10, pady=5, sticky="nsew")

        # Initialize the first control view (Keyboard style)
        self.on_control_style_change() 

        # ------------------- LASER CONTROL FRAME -------------------
        self.create_laser_control(main_frame)

    def create_connection_area(self, master_frame):
        conn_frame = ttk.LabelFrame(master_frame, text="Connection & Status", padding="10")
        conn_frame.grid(row=0, column=0, padx=10, pady=5, sticky="ew")
        ttk.Label(conn_frame, text="Serial Port:").grid(row=0, column=0, padx=5, pady=2, sticky="w")
        ttk.Entry(conn_frame, textvariable=self.port, width=20).grid(row=0, column=1, padx=5, pady=2, sticky="w")
        ttk.Button(conn_frame, text="Connect", command=self.connect_arduino_serial).grid(row=0, column=2, padx=5, pady=2)
        ttk.Button(conn_frame, text="Disconnect", command=self.disconnect_serial).grid(row=0, column=3, padx=5, pady=2)
        ttk.Label(conn_frame, text="Radio Status:").grid(row=1, column=0, padx=5, pady=2, sticky="w")
        ttk.Label(conn_frame, textvariable=self.radio_status).grid(row=1, column=1, padx=5, pady=2, sticky="w")
        ttk.Label(conn_frame, text="Command Throttle (ms):").grid(row=2, column=0, padx=5, pady=2, sticky="w")
        ttk.Entry(conn_frame, textvariable=self.command_throttle_ms, width=5).grid(row=2, column=1, padx=5, pady=2, sticky="w")

    def create_position_display(self, master_frame):
        # NOTE: This displays CNC/Work Area Coordinates
        pos_frame = ttk.LabelFrame(master_frame, text="Robot Position (mm, deg) [CNC Coords]", padding="10") 
        pos_frame.grid(row=1, column=0, padx=10, pady=5, sticky="ew")
        ttk.Label(pos_frame, text="X:").grid(row=0, column=0, padx=5, pady=2, sticky="w")
        ttk.Label(pos_frame, textvariable=self.x_pos, width=10, anchor='e').grid(row=0, column=1, padx=5, pady=2, sticky="e")
        ttk.Label(pos_frame, text="Y:").grid(row=0, column=2, padx=5, pady=2, sticky="w")
        ttk.Label(pos_frame, textvariable=self.y_pos, width=10, anchor='e').grid(row=0, column=3, padx=5, pady=2, sticky="e")
        ttk.Label(pos_frame, text="R:").grid(row=1, column=0, padx=5, pady=2, sticky="w")
        ttk.Label(pos_frame, textvariable=self.rotation_val, width=10, anchor='e').grid(row=1, column=1, padx=5, pady=2, sticky="e")
        ttk.Label(pos_frame, text="Elevation:").grid(row=1, column=2, padx=5, pady=2, sticky="w")
        ttk.Label(pos_frame, textvariable=self.elevation_val, width=10, anchor='e').grid(row=1, column=3, padx=5, pady=2, sticky="e")
        
    def create_control_style_selection(self, master_frame):
        style_frame = ttk.Frame(master_frame)
        style_frame.grid(row=3, column=0, padx=10, pady=5, sticky="ew")
        ttk.Label(style_frame, text="Control Method:").pack(side=tk.LEFT, padx=5)
        self.control_style_combo = ttk.Combobox(style_frame, 
                                                values=self.control_styles, 
                                                textvariable=tk.StringVar(value=self.current_control_method),
                                                state="readonly")
        self.control_style_combo.pack(side=tk.LEFT, fill='x', expand=True, padx=5)
        self.control_style_combo.bind("<<ComboboxSelected>>", self.on_control_style_change)

    def create_laser_control(self, master_frame):
        laser_frame = ttk.LabelFrame(master_frame, text="Laser/Spindle", padding="10")
        laser_frame.grid(row=5, column=0, padx=10, pady=5, sticky="ew")
        ttk.Checkbutton(laser_frame, text="Laser On (Hold SPACE)", variable=self.laser_on, command=lambda: self.send_control_command()).grid(row=0, column=0, padx=5, pady=2, sticky="w")
        ttk.Label(laser_frame, text="Power (0-255):").grid(row=1, column=0, padx=5, pady=2, sticky="w")
        ttk.Scale(laser_frame, from_=0, to=255, orient='horizontal', variable=self.current_laser_power, command=lambda *a: self.send_control_command()).grid(row=1, column=1, padx=5, pady=2, sticky="ew")
        ttk.Label(laser_frame, textvariable=self.current_laser_power).grid(row=1, column=2, padx=5, pady=2, sticky="w")


    def on_control_style_change(self, event=None):
        selected_style = self.control_style_combo.get() if hasattr(self, 'control_style_combo') else self.current_control_method
        self.current_control_method = selected_style
        if self.control_frame:
            for widget in self.control_frame.winfo_children():
                widget.destroy()
        creator = self.control_styles_dict.get(selected_style)
        if creator:
            self.control_frame.config(text=f"Director Controls: {selected_style}")
            creator(self.control_frame)
        else:
            # Fallback for safety, though the definition should prevent this
            ttk.Label(self.control_frame, text=f"Control style '{selected_style}' not found or implemented.").pack(padx=10, pady=10)

    # --- Control Style Specific Methods (v469 + v480 additions) ---

    def create_xyr_buttons_control(self, parent_frame, event=None):
        ttk.Label(parent_frame, text="Manual Keyboard/Button Control Active.").pack(padx=10, pady=10)
        ttk.Label(parent_frame, text="Use WASD (XY), QE (R) keys and +/- slider.").pack(padx=10, pady=2)
        speed_frame = ttk.Frame(parent_frame)
        speed_frame.pack(padx=10, pady=10)
        ttk.Label(speed_frame, text="Speed Multiplier:").pack(side=tk.LEFT)
        ttk.Scale(speed_frame, from_=0.1, to=1.0, orient='horizontal', variable=self.speed_var).pack(side=tk.LEFT, fill='x', expand=True)
        ttk.Label(speed_frame, textvariable=self.speed_var).pack(side=tk.LEFT)

    def create_joystick_control_area(self, parent_frame, event=None):
        ttk.Label(parent_frame, text="Joystick Control (Client/Server Mode)").pack(padx=10, pady=10)
        ttk.Label(parent_frame, text="Connection status and control logic managed by client threads.").pack(padx=10, pady=2)

    def create_gcode_director(self, parent_frame, event=None):
        # We reuse the internal G-code control creation for this standard director
        self._create_gcode_controls_internal(parent_frame)
        
    def create_gcode_with_apriltag_director(self, parent_frame, event=None):
        """G-code Director with Correction UI (v469 G-code + v480 Vision Controls)."""
        
        # 1. Standard G-code UI
        gcode_frame = ttk.LabelFrame(parent_frame, text="G-code File and Execution", padding="10")
        gcode_frame.pack(fill='x', padx=5, pady=5)
        self._create_gcode_controls_internal(gcode_frame)
        
        # 2. AprilTag Connection and Status (Including Calibrate Button)
        apriltag_frame = ttk.LabelFrame(parent_frame, text="AprilTag Correction Setup", padding="10")
        apriltag_frame.pack(fill='x', padx=5, pady=5)
        self._create_apriltag_connection_controls(apriltag_frame, include_calibration=True)

        ttk.Label(apriltag_frame, text="Correction will be applied if Vision is Connected AND Calibrated.").pack(padx=5, pady=5, anchor="w")

    def _create_gcode_controls_internal(self, parent_frame):
        """Internal method to create G-code specific controls."""
        row_counter = 0
        ttk.Button(parent_frame, text="Select G-code File (.txt, .gcode)", command=self.select_gcode_file).grid(row=row_counter, column=0, columnspan=2, padx=5, pady=5, sticky="ew")
        row_counter += 1
        lbl_gcode_path = ttk.Label(parent_frame, textvariable=self.gcode_file_path, wraplength=300)
        lbl_gcode_path.grid(row=row_counter, column=0, columnspan=2, padx=5, pady=5, sticky="ew")
        row_counter += 1
        self.btn_start_gcode = ttk.Button(parent_frame, text="Start G-code", command=self.start_gcode_execution, state=tk.DISABLED)
        self.btn_start_gcode.grid(row=row_counter, column=0, padx=5, pady=5, sticky="ew")
        self.btn_stop_gcode = ttk.Button(parent_frame, text="Stop G-code", command=self.stop_gcode_execution, state=tk.DISABLED)
        self.btn_stop_gcode.grid(row=row_counter, column=1, padx=5, pady=5, sticky="ew")
        row_counter += 1
        self.gcode_status_label = ttk.Label(parent_frame, text="Status: Idle", anchor='w')
        self.gcode_status_label.grid(row=row_counter, column=0, columnspan=2, padx=5, pady=5, sticky="ew")

    def _create_apriltag_connection_controls(self, parent_frame, include_calibration=False):
        """Helper to create AprilTag connection and status UI."""
        
        # Connection Row
        conn_frame = ttk.Frame(parent_frame)
        conn_frame.pack(fill='x', padx=5, pady=5)
        ttk.Label(conn_frame, text="IP:").grid(row=0, column=0, padx=5, pady=2, sticky="w")
        ttk.Entry(conn_frame, textvariable=self.tag_ip, width=15).grid(row=0, column=1, padx=5, pady=2, sticky="w")
        ttk.Label(conn_frame, text="Port:").grid(row=0, column=2, padx=5, pady=2, sticky="w")
        ttk.Entry(conn_frame, textvariable=self.tag_port, width=8).grid(row=0, column=3, padx=5, pady=2, sticky="w")
        self.btn_tag_connect = ttk.Button(conn_frame, text="Connect Vision", command=self.toggle_tag_connection)
        self.btn_tag_connect.grid(row=0, column=4, padx=5, pady=2, sticky="w")

        # Status and Position
        status_frame = ttk.Frame(parent_frame)
        status_frame.pack(fill='x', padx=5, pady=5)
        ttk.Label(status_frame, text="Vision Status:").grid(row=0, column=0, padx=5, pady=2, sticky="w")
        ttk.Label(status_frame, textvariable=self.tag_status).grid(row=0, column=1, padx=5, pady=2, sticky="w")
        ttk.Label(status_frame, text="Tag Pos (Raw):").grid(row=1, column=0, padx=5, pady=2, sticky="w")
        ttk.Label(status_frame, textvariable=self.tag_position).grid(row=1, column=1, padx=5, pady=2, sticky="w")
        ttk.Label(status_frame, text="Calibrated:").grid(row=2, column=0, padx=5, pady=2, sticky="w")
        ttk.Label(status_frame, textvariable=self.is_vision_calibrated).grid(row=2, column=1, padx=5, pady=2, sticky="w")


        if include_calibration:
            # Calibration button
            calib_frame = ttk.LabelFrame(parent_frame, text="Origin Calibration (V11: Calibrate to Work Center)", padding="10")
            calib_frame.pack(fill='x', padx=5, pady=5)
            
            ttk.Label(calib_frame, text="1. Manually move robot to physical center of the work area (Camera Origin).").pack(padx=5, pady=2, anchor='w')
            ttk.Button(calib_frame, text="2. Calibrate Vision Origin", command=self.calibrate_vision_origin).pack(padx=5, pady=5, fill='x')
            
            # Current Offsets Display
            offset_frame = ttk.Frame(calib_frame)
            offset_frame.pack(fill='x', padx=5, pady=5)
            ttk.Label(offset_frame, text="X Offset:").grid(row=0, column=0, padx=5, pady=2, sticky="w")
            ttk.Label(offset_frame, textvariable=self.vision_offset_x).grid(row=0, column=1, padx=5, pady=2, sticky="w")
            ttk.Label(offset_frame, text="Y Offset:").grid(row=1, column=0, padx=5, pady=2, sticky="w")
            ttk.Label(offset_frame, textvariable=self.vision_offset_y).grid(row=1, column=1, padx=5, pady=2, sticky="w")

    # --- RESTORED PLACEHOLDER STYLES ---
    def create_python_script_director(self, parent_frame, event=None):
        ttk.Label(parent_frame, text="External Python Script (Not Implemented)", background="lightgray").pack(padx=10, pady=10)
    
    def create_svg_bmp_director(self, parent_frame, event=None):
        ttk.Label(parent_frame, text="SVG/BMP Director (Not Implemented)", background="lightgray").pack(padx=10, pady=10)

    def create_tarantino_director(self, parent_frame, event=None):
        ttk.Label(parent_frame, text="Tarantino as Director (Not Implemented)", background="lightgray").pack(padx=10, pady=10)

    def create_frickin_shoot_everyone_director(self, parent_frame, event=None):
        ttk.Label(parent_frame, text="Frickin Shoot Everyone Director (Not Implemented)", background="lightgray").pack(padx=10, pady=10)

    def create_dxf_director(self, parent_frame, event=None):
        ttk.Label(parent_frame, text=".dxf Director (Not Implemented)", background="lightgray").pack(padx=10, pady=10)

    def create_jpg_director(self, parent_frame, event=None):
        ttk.Label(parent_frame, text=".jpg (Python Contour) Director (Not Implemented)", background="lightgray").pack(padx=10, pady=10)
    # ------------------------------------

    # -----------------------------------------------------------
    # --- AprilTag/Vision Logic (v480/V12) ---
    # -----------------------------------------------------------

    def toggle_tag_connection(self):
        """Connects or disconnects to the AprilTag vision server (v480)."""
        if self.tag_thread_running:
            self.stop_tag_thread()
            self.tag_status.set("Disconnected")
            self.tag_position.set("X:0.0 Y:0.0 R:0.0")
            self.btn_tag_connect.config(text="Connect Vision")
        else:
            host = self.tag_ip.get()
            port = self.tag_port.get()
            try:
                self.tag_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.tag_socket.settimeout(3.0)
                self.tag_socket.connect((host, port))
                self.tag_socket.setblocking(True) 
                self.tag_status.set("Connected")
                self.btn_tag_connect.config(text="Disconnect Vision")
                
                self.tag_thread_running = True
                self.tag_thread = threading.Thread(target=self._tag_read_worker, daemon=True)
                self.tag_thread.start()
            except socket.error as e:
                self.tag_status.set("Error")
                self.tag_socket = None
                self.btn_tag_connect.config(text="Connect Vision")
                # Showing the specific error code to the user for debugging
                messagebox.showerror("Connection Error", f"Could not connect to Vision Server: {e}")

    def _tag_read_worker(self, buffer_size=4096):
        """Dedicated thread to read AprilTag data from the socket (v480)."""
        data_buffer = ""
        while self.tag_thread_running and self.tag_socket:
            try:
                chunk = self.tag_socket.recv(buffer_size).decode('utf-8')
                if not chunk:
                    self.tag_thread_running = False
                    self.master.after(0, self.tag_status.set, "Disconnected")
                    self.master.after(0, self.btn_tag_connect.config, {'text': "Connect Vision"})
                    break
                
                data_buffer += chunk
                
                while '\n' in data_buffer:
                    line, data_buffer = data_buffer.split('\n', 1)
                    if line.strip():
                        self._process_tag_data(line)

            except socket.timeout:
                pass
            except socket.error:
                if self.tag_thread_running:
                    self.tag_thread_running = False
                    self.master.after(0, self.tag_status.set, "Error")
                    self.master.after(0, self.btn_tag_connect.config, {'text': "Connect Vision"})
                break
            except Exception:
                break
            
            time.sleep(0.01) 
            
    def _process_tag_data(self, data):
        """
        Parses the JSON tag data, updates raw variables, and updates 
        the CNC position display if calibrated. 

        This version accepts the "flat" JSON format sent by your AprilTag sender
        where numeric pose fields live at the top level of each detection dict
        (e.g., "x_mm", "y_mm", "yaw_deg") *or* inside a nested "pose" dict.
        It also supports pixel-based fallbacks: "x_px", "y_px", "yaw_approx_deg".
        """
        try:
            print("RAW incoming from vision thread:", data[:400])
            tag_data = json.loads(data)
            # Update the shared state variables
            print("TAG incoming:", data)

            detections = tag_data.get("detections", [])

            if detections:
                # Use first detection (legacy behavior)
                d = detections[0]

                pose_valid = d.get("pose_valid", False)

                # Helper to read pose values from either nested 'pose' or flat structure
                def _val(det, *keys, default=0.0):
                    for k in keys:
                        if isinstance(det, dict) and k in det:
                            return det[k]
                    return default

                # Prefer nested 'pose' dictionary if present
                if isinstance(d.get("pose"), dict):
                    pose_src = d.get("pose")
                else:
                    pose_src = d

                # Read values with fallbacks (mm first, then px if present)
                x_val = _val(pose_src, "x_mm", "x_px", default=0.0)
                y_val = _val(pose_src, "y_mm", "y_px", default=0.0)
                yaw_val = _val(pose_src, "yaw_deg", "yaw", "r_deg", "yaw_approx_deg", default=0.0)

                # Convert to floats safely
                try:
                    self.current_tag_x_mm = float(x_val)
                except Exception:
                    self.current_tag_x_mm = 0.0
                try:
                    self.current_tag_y_mm = float(y_val)
                except Exception:
                    self.current_tag_y_mm = 0.0
                try:
                    self.current_tag_r_deg = float(yaw_val)
                except Exception:
                    self.current_tag_r_deg = 0.0

            else:
                # No detections: set sentinel so callers know it's invalid/stale
                self.current_tag_x_mm = -99999.9
                self.current_tag_y_mm = -99999.9
                self.current_tag_r_deg = -99999.9

            # Update raw tag position display
            self.master.after(0, self.tag_position.set, 
                              f"X:{self.current_tag_x_mm:.2f} Y:{self.current_tag_y_mm:.2f} R:{self.current_tag_r_deg:.2f}")

            # Update CNC position if calibrated
            if self.is_vision_calibrated.get() and abs(self.current_tag_x_mm) < 99999.0:
                self.update_cnc_position(self.current_tag_x_mm, self.current_tag_y_mm, self.current_tag_r_deg)
            # --- V12 DEBUG: Add print statement for uncalibrated state ---
            else:
                print("DEBUG: Vision data received but CNC position not updated (Not Calibrated or invalid data).") 
            # -------------------------------------------------------------
                
        except json.JSONDecodeError as e:
            print(f"WARNING: Could not parse tag data as JSON: {data[:50]}... Error: {e}")
        except Exception as e:
            print(f"ERROR during tag data parsing: {e}")
            
    def stop_tag_thread(self):
        """Stops the AprilTag reading thread and closes the socket (v480)."""
        self.tag_thread_running = False
        if self.tag_socket:
            try:
                self.tag_socket.shutdown(socket.SHUT_RDWR)
                self.tag_socket.close()
            except OSError:
                pass # Socket already closed
            self.tag_socket = None
        if self.tag_thread and self.tag_thread.is_alive():
            self.tag_thread.join(timeout=0.1)
            
    
    def update_cnc_position(self, T_x, T_y, T_r):
        """
        Calculates and updates the robot's position in the CNC coordinate system.
        This version updates the GUI even when not calibrated: when uncalibrated, offsets are treated as 0.0.
        It also correctly handles yaw (rotation) and uses .get() on Tk variables to avoid type errors.
        """
        try:
            # Get numeric work area
            W = float(self.work_area_width_mm.get())
            H = float(self.work_area_height_mm.get())

            # Use offsets only if calibrated; otherwise zero
            if self.is_vision_calibrated.get():
                O_x = float(self.vision_offset_x.get())
                O_y = float(self.vision_offset_y.get())
                O_r = float(self.vision_offset_r.get())
            else:
                O_x = 0.0
                O_y = 0.0
                O_r = 0.0

            # Convert camera (center-origin) -> CNC
            # Your physical setup: CNC origin is down/left of camera center by half width/height,
            # so camera->CNC = T - (W/2,H/2) + offsets
            C_x = T_x - (W / 2.0) + O_x
            C_y = T_y - (H / 2.0) + O_y
            C_r = T_r + O_r  # Always include yaw (rotation) offset additively

            # --- DEBUG: Confirm update is happening ---
            print(f"DEBUG V12: CNC Position Update. Raw: ({T_x:.2f}, {T_y:.2f}, {T_r:.2f}). Off: ({O_x:.2f}, {O_y:.2f}, {O_r:.2f}). CNC: ({C_x:.2f}, {C_y:.2f}, {C_r:.2f})")
            # ----------------------------------------------

            # Thread-safe GUI update
            self.master.after(0, self.x_pos.set, f"{C_x:.2f}")
            self.master.after(0, self.y_pos.set, f"{C_y:.2f}")
            self.master.after(0, self.rotation_val.set, f"{C_r:.2f}")

        except Exception as e:
            print(f"Error updating CNC position: {e}")
    def calibrate_vision_origin(self):
        """
        Sets the offset so that the current raw tag position (T_x, T_y) 
        corresponds to the center of the work area (C_x, C_y).
        
        V11: CNC Target is now (WorkAreaWidth/2, WorkAreaHeight/2).
        """
        if not self.tag_thread_running or self.tag_status.get() != "Connected":
            messagebox.showwarning("Calibration Error", "AprilTag Vision must be Connected to calibrate the origin.")
            return

        T_x = self.current_tag_x_mm 
        T_y = self.current_tag_y_mm 
        T_r = self.current_tag_r_deg
        
        # CRITICAL V10/V11 FIX: Check for sentinel value from disconnected or stale read
        if abs(T_x) > 99999.0 or abs(T_y) > 99999.0:
            messagebox.showwarning("Calibration Error", "Invalid Raw Tag data. Ensure the tag is visible and the raw position is updating before calibrating.")
            return
            
        # --- V11 CRITICAL FIX: Set CNC Target to the Work Area Center (Half W/H) ---
        C_x = 0.0
        C_y = 0.0

        C_r = 0.0 # Assuming robot is aligned to work area axis when calibrated

        # Calculate Offsets: Offset = CNC Target - Raw Reading
        offset_x = C_x - T_x
        offset_y = C_y - T_y
        offset_r = C_r - T_r 
        
        # Set the calculated offsets
        self.vision_offset_x.set(offset_x)
        self.vision_offset_y.set(offset_y)
        self.vision_offset_r.set(offset_r)
        self.is_vision_calibrated.set(True)
        
        # Immediately update position based on new offsets
        self.update_cnc_position(T_x, T_y, T_r)
        
        messagebox.showinfo("Calibration Complete", 
                            f"Vision Origin Calibrated! (Calibration Flag is now {self.is_vision_calibrated.get()})\n"
                            f"Current Position (CNC): X={C_x:.2f}, Y={C_y:.2f}\n"
                            f"Calculated Offsets: X={offset_x:.2f}, Y={offset_y:.2f}")


    # -----------------------------------------------------------
    # --- Serial/Robot Communication Methods (v469) ---
    # -----------------------------------------------------------

    def connect_arduino_serial(self):
        """Attempts to establish serial connection to Arduino."""
        if self.serial_port and self.serial_port.is_open:
            self.update_radio_status("Already Connected")
            return
        
        port_name = self.port.get()
        
        try:
            self.serial_port = serial.Serial(port_name, self.baud_rate, timeout=0.1)
            time.sleep(2) # Wait for Arduino to reset
            self.arduino_connected = True
            self.update_radio_status("Connected")
            
            # Start the command sending thread
            if not self.command_send_thread_running:
                self.command_send_thread_running = True
                self.command_send_thread = threading.Thread(target=self._command_sender_worker, daemon=True)
                self.command_send_thread.start()
                
            # Start the continuous motion update thread
            self._start_motion_update_thread()
            
        except serial.SerialException as e:
            self.update_radio_status(f"Error: {e}")
            self.serial_port = None
            self.arduino_connected = False
            messagebox.showerror("Connection Error", f"Could not connect to {port_name}: {e}")

    def disconnect_serial(self):
        """Disconnects the serial port."""
        self.arduino_connected = False
        self.stop_motion_update_thread()
        self.stop_command_sender_thread()

        if self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.close()
                self.serial_port = None
                self.update_radio_status("Disconnected")
            except Exception as e:
                self.update_radio_status(f"Error disconnecting: {e}")

    def update_radio_status(self, status):
        """Updates the radio status label in the GUI (thread-safe)."""
        self.master.after(0, self.radio_status.set, status)

    def _command_sender_worker(self):
        """Dedicated thread to send commands from the queue to the serial port."""
        while self.command_send_thread_running:
            try:
                # Blocks until an item is available
                command = self.command_send_queue.get(timeout=0.1) 
                
                if command is None: # Sentinel to stop thread
                    break
                    
                if self.serial_port and self.serial_port.is_open:
                    # G-code commands already include '\n'
                    command_with_newline = command if command.endswith('\n') else command + '\n'
                    self.serial_port.write(command_with_newline.encode())
                    self.serial_port.flush() 
                    
                self.command_send_queue.task_done()
                
            except queue.Empty:
                continue
            except serial.SerialException as e:
                print(f"Serial Error in sender thread: {e}")
                self.master.after(0, self.disconnect_serial) # Disconnect safely on main thread
                break
            except Exception as e:
                print(f"Error in sender thread: {e}")
                break

    def stop_command_sender_thread(self):
        """Stops the command sender thread gracefully."""
        self.command_send_thread_running = False
        if self.command_send_thread and self.command_send_thread.is_alive():
            self.command_send_queue.put(None) # Send sentinel value
            self.command_send_thread.join(timeout=1)
            
    def send_command(self, command):
        """Puts a command into the queue for sending."""
        if self.arduino_connected:
            self.command_send_queue.put(command)
            
    # --- Motion Control and Update (v469) ---

    def _start_motion_update_thread(self):
        """Starts the periodic sending of the current motion command."""
        self.stop_motion_update_thread()
        self.motion_update_job = self.master.after(self.command_throttle_ms.get(), self._send_motion_update)

    def _send_motion_update(self):
        """Sends the current state of movement (speed/direction/laser) to the robot."""
        if not self.arduino_connected or self.gcode_processing_active:
            # Re-schedule and exit if not connected or G-code is active
            self.motion_update_job = self.master.after(self.command_throttle_ms.get(), self._send_motion_update)
            return

        # 1. Determine the actual speed factor
        speed_factor = self.speed_var.get()
        
        # 2. Determine target motion based on keyboard state (for manual control)
        # Note: Joystick control bypasses this but also calls send_control_command to trigger it
        target_x, target_y, target_r = self.get_target_motion_from_keyboard(speed_factor)

        # 3. Apply current laser state
        target_laser_on = self.laser_on.get() or self.spacebar_pressed
        target_laser_power = self.current_laser_power.get() if target_laser_on else 0

        current_cmd = {
            "x": target_x, 
            "y": target_y, 
            "rotation": target_r, 
            "laser_on": target_laser_on, 
            "laser_power": target_laser_power,
            "speed_factor": speed_factor
        }

        # 4. Check if command has changed since last send
        if current_cmd != self.last_sent_motion_command:
            command_str = self.format_control_command(current_cmd)
            self.send_command(command_str)
            self.last_sent_motion_command = current_cmd
            
        # Re-schedule the next update
        self.motion_update_job = self.master.after(self.command_throttle_ms.get(), self._send_motion_update)

    def stop_motion_update_thread(self):
        """Stops the motion update thread."""
        if self.motion_update_job is not None:
            self.master.after_cancel(self.motion_update_job)
            self.motion_update_job = None
            
    def get_target_motion_from_keyboard(self, speed_factor):
        """Translates keyboard state into X/Y/R motion targets."""
        target_x = 0.0
        target_y = 0.0
        target_r = 0.0
        
        # Linear Movement (X, Y)
        if self.is_moving["forward"]:
            target_y += 1.0 * speed_factor
        if self.is_moving["backward"]:
            target_y -= 1.0 * speed_factor
        if self.is_moving["right"]:
            target_x += 1.0 * speed_factor
        if self.is_moving["left"]:
            target_x -= 1.0 * speed_factor
            
        # Rotation (R)
        if self.is_moving["CW"]:
            target_r += 1.0 * speed_factor
        if self.is_moving["CCW"]:
            target_r -= 1.0 * speed_factor
            
        # Normalize diagonal movement if necessary, but for simplicity, just cap
        # (This is better handled by a joystick, but for key control, we keep it simple)

        return target_x, target_y, target_r

    def format_control_command(self, cmd_dict):
        """Formats the motion command dictionary into a string suitable for the robot."""
        # Note: We use a simplified control command for direct control:
        # C,<X_target>,<Y_target>,<R_target>,<Laser_Power>
        # The robot code then scales these targets by a speed factor and executes them.
        
        # Scale X, Y, R by 255 for a robust command signal, ignoring the speed_factor
        # as the command is relative and the robot should apply its own scaling.
        # We cap the values at a max to represent full stick/key deflection.
        MAX_VAL = 255.0
        x_val = cmd_dict['x'] * MAX_VAL
        y_val = cmd_dict['y'] * MAX_VAL
        r_val = cmd_dict['rotation'] * MAX_VAL
        
        x_cmd = max(-MAX_VAL, min(MAX_VAL, x_val))
        y_cmd = max(-MAX_VAL, min(MAX_VAL, y_val))
        r_cmd = max(-MAX_VAL, min(MAX_VAL, r_val))

        l_power = cmd_dict['laser_power']
        
        command_str = f"C,{int(x_cmd)},{int(y_cmd)},{int(r_cmd)},{l_power}"
        return command_str

    def send_control_command(self):
        """Forces an immediate check and send of the current control state."""
        # This method is primarily used by button/slider commands to immediately update the robot state.
        # The periodic thread handles continuous movement.
        
        if self.gcode_processing_active:
            return

        # 1. Determine the actual speed factor
        speed_factor = self.speed_var.get()
        
        # 2. Determine target motion based on current state (keyboard or joystick)
        target_x, target_y, target_r = self.get_target_motion_from_keyboard(speed_factor)
        
        # 3. Apply current laser state (Checkbutton or spacebar)
        target_laser_on = self.laser_on.get() or self.spacebar_pressed
        target_laser_power = self.current_laser_power.get() if target_laser_on else 0

        current_cmd = {
            "x": target_x, 
            "y": target_y, 
            "rotation": target_r, 
            "laser_on": target_laser_on, 
            "laser_power": target_laser_power,
            "speed_factor": speed_factor
        }

        command_str = self.format_control_command(current_cmd)
        self.send_command(command_str)
        self.last_sent_motion_command = current_cmd # Update last sent to prevent resending by the periodic thread

    # --- Keyboard/Focus Handlers (v469) ---

    def read_keyboard(self, event):
        """Handles key presses for WASD/QE movement and Spacebar laser."""
        if event.char in ('w', 'W'):
            self.is_moving["forward"] = True
        elif event.char in ('s', 'S'):
            self.is_moving["backward"] = True
        elif event.char in ('a', 'A'):
            self.is_moving["left"] = True
        elif event.char in ('d', 'D'):
            self.is_moving["right"] = True
        elif event.char in ('q', 'Q'):
            self.is_moving["CCW"] = True
        elif event.char in ('e', 'E'):
            self.is_moving["CW"] = True
        elif event.keysym == 'space':
            # This is specifically for the momentary 'Laser On' function
            if not self.spacebar_pressed:
                self.spacebar_pressed = True
                self.send_control_command() # Send command immediately on press
            
    def read_keyrelease(self, event):
        """Handles key releases to stop motion and turn off momentary laser."""
        if event.char in ('w', 'W'):
            self.is_moving["forward"] = False
        elif event.char in ('s', 'S'):
            self.is_moving["backward"] = False
        elif event.char in ('a', 'A'):
            self.is_moving["left"] = False
        elif event.char in ('d', 'D'):
            self.is_moving["right"] = False
        elif event.char in ('q', 'Q'):
            self.is_moving["CCW"] = False
        elif event.char in ('e', 'E'):
            self.is_moving["CW"] = False
        elif event.keysym == 'space':
            # This is specifically for the momentary 'Laser Off' function
            if self.spacebar_pressed:
                self.spacebar_pressed = False
                self.send_control_command() # Send command immediately on release

    def focus_change_handler(self, event):
        """Stops all movement when the application loses focus."""
        if event.type == '9': # FocusOut
            if self.is_moving != {"forward": False, "backward": False, "left": False, "right": False, "CCW": False, "CW": False,}:
                # Only send a stop command if the robot was moving
                self.is_moving = {"forward": False, "backward": False, "left": False, "right": False, "CCW": False, "CW": False,}
                self.send_control_command()

    # -----------------------------------------------------------
    # --- G-code Processing Methods (v469) ---
    # -----------------------------------------------------------
    
    def select_gcode_file(self):
        """Opens a file dialog for selecting a G-code file."""
        file_path = filedialog.askopenfilename(
            defaultextension=".gcode",
            filetypes=[("G-code Files", "*.gcode"), ("Text Files", "*.txt"), ("All Files", "*.*")]
        )
        if file_path:
            self.gcode_file_path.set(file_path)
            if self.btn_start_gcode:
                self.btn_start_gcode.config(state=tk.NORMAL)
            if self.gcode_status_label:
                self.gcode_status_label.config(text=f"Status: File Loaded: {Path(file_path).name}")

    def start_gcode_execution(self):
        """Initiates the G-code parsing and execution process."""
        if not self.arduino_connected:
            messagebox.showwarning("Connection Required", "Connect to the robot first.")
            return

        if not self.gcode_file_path.get():
            messagebox.showwarning("File Required", "Select a G-code file first.")
            return

        # Reset state and load file
        self.stop_gcode_flag = False
        self.gcode_processing_active = True
        self.gcode_current_x = self.work_area_width_mm.get() / 2.0 # Start from center
        self.gcode_current_y = self.work_area_height_mm.get() / 2.0 # Start from center
        self.gcode_current_laser_on = False
        self.gcode_current_laser_power = 0
        self.gcode_absolute_mode = True # Default to G90
        
        self.stop_motion_update_thread() # Stop manual motion updates
        
        # Disable Start/Enable Stop
        if self.btn_start_gcode: self.btn_start_gcode.config(state=tk.DISABLED)
        if self.btn_stop_gcode: self.btn_stop_gcode.config(state=tk.NORMAL)
        
        # Start the processing thread
        if self.gcode_processing_thread and self.gcode_processing_thread.is_alive():
            self.gcode_processing_thread.join(0.1)
        
        self.gcode_processing_thread = threading.Thread(target=self._gcode_processing_worker, daemon=True)
        self.gcode_processing_thread.start()
        self.gcode_status_label.config(text="Status: Executing...")

    def stop_gcode_execution(self):
        """Stops the G-code execution thread."""
        self.stop_gcode_flag = True
        
        # Send a stop command to the robot immediately
        self.send_command("C,0,0,0,0") 
        
        # Wait for the thread to stop and clean up
        if self.gcode_processing_thread and self.gcode_processing_thread.is_alive():
            self.gcode_processing_thread.join(timeout=1)
            
        self.gcode_processing_active = False
        
        # Re-enable Start/Disable Stop
        if self.btn_start_gcode: self.btn_start_gcode.config(state=tk.NORMAL)
        if self.btn_stop_gcode: self.btn_stop_gcode.config(state=tk.DISABLED)
        
        self.gcode_status_label.config(text="Status: Stopped")
        
        self._start_motion_update_thread() # Resume manual motion updates

    def _gcode_processing_worker(self):
        """
        Parses and executes G-code line by line, managing the robot's target 
        position and state.
        """
        try:
            with open(self.gcode_file_path.get(), 'r') as f:
                for line_number, line in enumerate(f, 1):
                    if self.stop_gcode_flag:
                        break

                    gcode_command = line.strip().upper()
                    if not gcode_command or gcode_command.startswith(';'):
                        continue # Skip empty lines and comments

                    # 1. Parse Motion Parameters
                    m = re.match(r'G(\d+)(?: X([\d\.\-]+))?(?: Y([\d\.\-]+))?(?: Z([\d\.\-]+))?(?: R([\d\.\-]+))?(?: F([\d\.\-]+))?', gcode_command)
                    if m:
                        g_code = int(m.group(1))
                        x_target = m.group(2)
                        y_target = m.group(3)
                        r_target = m.group(5) # Using R for rotation/yaw, Z is elevation
                        f_rate = m.group(6)
                        
                        # Handle Feed Rate (F)
                        if f_rate is not None:
                            self.gcode_current_feed_rate = float(f_rate)
                            # F-rate is usually not a movement command itself, so we skip to next line
                            if g_code not in [0, 1]:
                                continue 
                        
                        # Handle Absolute/Relative Modes (G90/G91)
                        if g_code == 90:
                            self.gcode_absolute_mode = True
                            continue
                        elif g_code == 91:
                            self.gcode_absolute_mode = False
                            continue
                            
                        # Handle Movement (G0/G1)
                        if g_code in [0, 1]:
                            # Determine Target Position
                            target_x = self.gcode_current_x
                            target_y = self.gcode_current_y
                            target_r = self.north_angle # Assume R is yaw, and we maintain current heading unless specified

                            if x_target is not None:
                                val = float(x_target)
                                target_x = val if self.gcode_absolute_mode else self.gcode_current_x + val
                            if y_target is not None:
                                val = float(y_target)
                                target_y = val if self.gcode_absolute_mode else self.gcode_current_y + val
                            if r_target is not None:
                                val = float(r_target)
                                target_r = val # Rotation is typically absolute or relative to current, let's treat as absolute North angle for now

                            # Update Current Position (must be done before sending)
                            self.gcode_current_x = target_x
                            self.gcode_current_y = target_y
                            self.north_angle = target_r
                            
                            # Update CNC display position for user
                            self.master.after(0, self.x_pos.set, f"{target_x:.2f}")
                            self.master.after(0, self.y_pos.set, f"{target_y:.2f}")
                            self.master.after(0, self.rotation_val.set, f"{target_r:.2f}")
                            
                            # Convert CNC Target (target_x, target_y, target_r) back to Raw Tag Coordinates for the robot
                            if self.is_vision_calibrated.get() and self.tag_thread_running:
                                # Apply inverse transformation: Raw Target = CNC Target - Offset
                                W = self.work_area_width_mm.get()
                                H = self.work_area_height_mm.get()
                                raw_target_x = target_x + (W / 2.0) - self.vision_offset_x.get()
                                raw_target_y = target_y + (H / 2.0) - self.vision_offset_y.get()
                                raw_target_r = target_r - self.vision_offset_r.get()

                            else:
                                # If not calibrated, assume CNC coordinates ARE the raw coordinates
                                raw_target_x = target_x
                                raw_target_y = target_y
                                raw_target_r = target_r

                            # Format and Send Move Command: M,<X_mm>,<Y_mm>,<R_deg>,<F_rate>,<Laser_P>
                            command = f"M,{raw_target_x:.2f},{raw_target_y:.2f},{raw_target_r:.2f},{self.gcode_current_feed_rate:.1f},{self.gcode_current_laser_power}"
                            self.send_command(command)
                            
                            # Wait for move completion (blocking behavior is necessary for G-code)
                            # Simplistic wait: Assume time_to_wait = Distance / Velocity
                            # A real system would require an acknowledgment from the robot (e.g., "OK")
                            time.sleep(0.1) # Minimum delay
                            
                            # A real implementation would:
                            # 1. Send "M" command.
                            # 2. Block until the robot returns an "OK" response.
                            # Since we don't have the "OK" loop, we rely on a throttle.
                            
                            time.sleep(self.command_throttle_ms.get() / 1000.0 * 2) # Arbitrary buffer

                            # Continue to next line of G-code
                            continue

                    # 2. Parse Spindle/Laser Parameters (M3/M5)
                    m_laser = re.match(r'M(\d+)(?: S(\d+))?', gcode_command)
                    if m_laser:
                        m_code = int(m_laser.group(1))
                        s_power = m_laser.group(2)
                        
                        if m_code == 3: # Laser/Spindle ON
                            self.gcode_current_laser_on = True
                            if s_power is not None:
                                self.gcode_current_laser_power = int(s_power)
                            elif self.gcode_current_laser_power == 0:
                                self.gcode_current_laser_power = 255 # Default to max if no S parameter and not set
                                
                            self.master.after(0, self.current_laser_power.set, self.gcode_current_laser_power)
                            self.master.after(0, self.laser_on.set, True)

                        elif m_code == 5: # Laser/Spindle OFF
                            self.gcode_current_laser_on = False
                            self.gcode_current_laser_power = 0
                            
                            self.master.after(0, self.current_laser_power.set, 0)
                            self.master.after(0, self.laser_on.set, False)

                        # Send the new laser state command
                        command = f"L,{self.gcode_current_laser_power}"
                        self.send_command(command)
                        time.sleep(0.01) # Short delay for state change
                        continue
                        
                    # Handle other commands (e.g., G4 dwell, which we ignore for simplicity)
                    # ... 

                    # Update status to show current line being processed
                    self.master.after(0, self.gcode_status_label.config, {'text': f"Status: Executing L{line_number}: {gcode_command}"})
                    
        except FileNotFoundError:
            self.master.after(0, messagebox.showerror, "G-code Error", "G-code file not found.")
        except Exception as e:
            self.master.after(0, messagebox.showerror, "G-code Runtime Error", f"An error occurred on line {line_number}: {e}")
        finally:
            self.master.after(0, self.stop_gcode_execution) # Ensure clean stop

    # -----------------------------------------------------------
    # --- Joystick Client (v469) ---
    # -----------------------------------------------------------

    def _connect_to_joystick_server(self):
        """Tries to connect to the external Joystick server."""
        if self.joystick_connected:
            return

        try:
            # We attempt to connect in a non-blocking way to avoid GUI freeze
            self.joystick_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.joystick_socket.settimeout(1.0) 
            self.joystick_socket.connect((self.joystick_host, self.joystick_port))
            self.joystick_socket.setblocking(True) 
            self.joystick_connected = True
            
            self.joystick_thread_running = True
            self.joystick_read_thread = threading.Thread(target=self._joystick_read_worker, daemon=True)
            self.joystick_read_thread.start()
            print("Joystick client connected to server.")
            
        except socket.error as e:
            # print(f"Could not connect to Joystick Server: {e}")
            self.joystick_socket = None
            self.joystick_connected = False

    def _joystick_read_worker(self):
        """Dedicated thread to read joystick data from the socket."""
        while self.joystick_thread_running and self.joystick_socket:
            try:
                # Read data
                chunk = self.joystick_socket.recv(self.joystick_buffer_size).decode('utf-8')
                if not chunk:
                    self.joystick_thread_running = False
                    self.joystick_connected = False
                    break
                
                self.joystick_data_buffer += chunk
                
                # Process lines
                while '\n' in self.joystick_data_buffer:
                    line, self.joystick_data_buffer = self.joystick_data_buffer.split('\n', 1)
                    if line.strip():
                        self._process_joystick_data(line)

            except socket.timeout:
                pass 
            except socket.error:
                if self.joystick_thread_running:
                    print("Joystick socket error, disconnecting.")
                    self.joystick_thread_running = False
                    self.joystick_connected = False
                break
            except Exception as e:
                print(f"Error in joystick read worker: {e}")
                break
            
            time.sleep(0.01) # Small delay

    def _process_joystick_data(self, data):
        """Parses the joystick JSON data and updates motion commands."""
        try:
            joy_data = json.loads(data)
            
            # Only update if the control style is 'Joystick Control'
            if self.current_control_method == "Joystick Control":
                
                # Axes: x, y, r (Rotation)
                target_x = joy_data.get('x', 0.0) 
                target_y = joy_data.get('y', 0.0)
                target_r = joy_data.get('r', 0.0)

                # Laser/Button state
                target_laser_on = joy_data.get('L1', 0.0) > 0.1 # Example: L1 button activates laser
                
                # Set X/Y/R movement states based on joystick input for motion thread
                # This ensures the motion thread or send_control_command picks up the non-zero targets
                # Note: For joystick, we use direct values, not boolean flags
                
                self.motion_command['x'] = target_x
                self.motion_command['y'] = target_y
                self.motion_command['rotation'] = target_r
                
                # Update laser state
                # Note: Joystick state must override the UI checkbox if joystick is in use
                self.laser_on.set(target_laser_on) 
                
                # Immediately send the command based on joystick input
                self.send_control_command_from_joystick(target_x, target_y, target_r, target_laser_on)
                
        except json.JSONDecodeError as e:
            print(f"WARNING: Could not parse joystick data as JSON: {data[:50]}... Error: {e}")
        except Exception as e:
            print(f"ERROR during joystick data parsing: {e}")

    def send_control_command_from_joystick(self, x, y, r, laser_on):
        """Formats and sends the control command directly from joystick input."""
        if self.gcode_processing_active:
            return

        current_cmd = {
            "x": x, 
            "y": y, 
            "rotation": r, 
            "laser_on": laser_on, 
            "laser_power": self.current_laser_power.get() if laser_on else 0,
            "speed_factor": self.speed_var.get() # Speed factor still applies to cap joystick input
        }

        command_str = self.format_control_command(current_cmd)
        
        # Only send if motion or laser state has significantly changed
        # We need a custom comparison for float values here
        if (abs(x - self.last_sent_motion_command.get('x', 0.0)) > 0.05 or
            abs(y - self.last_sent_motion_command.get('y', 0.0)) > 0.05 or
            abs(r - self.last_sent_motion_command.get('rotation', 0.0)) > 0.05 or
            laser_on != self.last_sent_motion_command.get('laser_on', False)):
            
            self.send_command(command_str)
            self.last_sent_motion_command = current_cmd.copy()


    def stop_joystick_thread(self):
        """Stops the joystick reading thread and closes the socket."""
        self.joystick_thread_running = False
        if self.joystick_socket:
            try:
                self.joystick_socket.shutdown(socket.SHUT_RDWR)
                self.joystick_socket.close()
            except OSError:
                pass
            self.joystick_socket = None
        if self.joystick_read_thread and self.joystick_read_thread.is_alive():
            self.joystick_read_thread.join(timeout=0.1)

    # ----------------------------------------------------------
    # --- Cleanup ---
    # ----------------------------------------------------------

    def cleanup(self):
        self.stop_gcode_execution()
        self.stop_tag_thread()
        self.stop_joystick_thread()
        self.stop_motion_update_thread()
        self.stop_command_sender_thread()
        
        # Final serial disconnect
        if self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.close()
            except:
                pass

    def _on_closing(self):
        self.cleanup()
        self.master.destroy()

# --- MANDATORY: Application Entry Point ---
if __name__ == "__main__":
    try:
        root = tk.Tk()
        app = robotDirector(root)
        root.mainloop()
    except Exception as e:
        print(f"Fatal error starting the application: {e}")
        # Clean up resources if mainloop failed to start
        if 'app' in locals():
            app.cleanup()
