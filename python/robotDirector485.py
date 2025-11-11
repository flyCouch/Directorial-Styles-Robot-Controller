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
        self.x_pos = tk.DoubleVar(master, value=0.0)
        self.y_pos = tk.DoubleVar(master, value=0.0)
        self.rotation_val = tk.DoubleVar(master, value=0.0)
        self.elevation_val = tk.DoubleVar(master, value=0.0)
        
        # Work Area Variables (v469)
        self.work_area_width_mm = tk.DoubleVar(master, value=300.0) 
        self.work_area_height_mm = tk.DoubleVar(master, value=300.0) 

        # --- Vision (AprilTag) Tracking and Origin Management (v480) ---
        self.current_tag_x_mm = 0.0
        self.current_tag_y_mm = 0.0
        self.current_tag_r_deg = 0.0
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
        self.connect_arduino_serial() 
        self._connect_to_joystick_server() 

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
            calib_frame = ttk.LabelFrame(parent_frame, text="Origin Calibration", padding="10")
            calib_frame.pack(fill='x', padx=5, pady=5)
            
            ttk.Label(calib_frame, text="1. Manually move robot to CNC Origin (X0 Y0).").pack(padx=5, pady=2, anchor='w')
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
    # --- AprilTag/Vision Logic (v480) ---
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
        Parses the JSON tag data, updates raw variables, and now 
        updates the CNC position display if calibrated. (v480 FIX)
        """
        try:
            tag_data = json.loads(data)
            self.current_tag_x_mm = tag_data.get('x', 0.0)
            self.current_tag_y_mm = tag_data.get('y', 0.0)
            self.current_tag_r_deg = tag_data.get('r', 0.0)
            
            # Update raw tag position display
            self.master.after(0, self.tag_position.set, 
                              f"X:{self.current_tag_x_mm:.2f} Y:{self.current_tag_y_mm:.2f} R:{self.current_tag_r_deg:.2f}")

            # --- CRITICAL FIX: Update CNC Position Display if Calibrated ---
            if self.is_vision_calibrated.get():
                Cx, Cy, Cr = self.get_cnc_position_from_tag()
                self.master.after(0, self.x_pos.set, Cx)
                self.master.after(0, self.y_pos.set, Cy)
                self.master.after(0, self.rotation_val.set, Cr)
            # -------------------------------------------------------------
            
        except json.JSONDecodeError as e:
            print(f"WARNING: Could not parse tag data as JSON: {data[:50]}... Error: {e}")
        except Exception as e:
            print(f"ERROR during tag data parsing: {e}")

    def calibrate_vision_origin(self):
        """Calculates the vision offset (v480 logic)."""
        if self.tag_status.get() != "Connected":
            messagebox.showwarning("Warning", "Vision System must be connected and reporting data to calibrate.")
            return

        # CNC Origin is defined as (0, 0, 0) for calibration
        C_x, C_y, C_r = 0.0, 0.0, 0.0 
        # Raw Tag Position (Vision)
        T_x, T_y, T_r = self.current_tag_x_mm, self.current_tag_y_mm, self.current_tag_r_deg
        
        # Offset = CNC Target - Vision Reading
        O_x = C_x - T_x
        O_y = C_y - T_y
        O_r = C_r - T_r

        self.vision_offset_x.set(O_x)
        self.vision_offset_y.set(O_y)
        self.vision_offset_r.set(O_r) 
        self.is_vision_calibrated.set(True)

        # Immediately update the display with the new origin (which should now be 0, 0, 0)
        self.x_pos.set(C_x)
        self.y_pos.set(C_y)
        self.rotation_val.set(C_r)

        messagebox.showinfo("Calibration Complete", 
                            f"Vision Origin set. Offsets:\n"
                            f"X_offset: {O_x:.2f} mm\n"
                            f"Y_offset: {O_y:.2f} mm\n"
                            f"R_offset: {O_r:.2f} deg")

    def get_cnc_position_from_tag(self):
        """Converts tag position to CNC position (v480 logic)."""
        Tx = self.current_tag_x_mm
        Ty = self.current_tag_y_mm
        Tr = self.current_tag_r_deg
        Ox = self.vision_offset_x.get()
        Oy = self.vision_offset_y.get()
        Or = self.vision_offset_r.get() 

        # CNC Position = Raw Tag Position + Offset
        Cx = Tx + Ox
        Cy = Ty + Oy
        Cr = Tr + Or
        
        return Cx, Cy, Cr

    # -----------------------------------------------------------
    # --- Core Robot/System Methods (v370/v469/v480) ---
    # -----------------------------------------------------------
    
    # --- Connection/Communication (v370/v469) ---

    def update_radio_status(self, status):
        """Updates the GUI radio status label."""
        self.radio_status.set(status)

    def connect_arduino_serial(self):
        """Opens the serial port and starts the command sender thread."""
        if self.arduino_connected:
            messagebox.showwarning("Connection Warning", "Serial port is already open.")
            return

        try:
            self.serial_port = serial.Serial(self.port.get(), self.baud_rate, timeout=0.1)
            self.arduino_connected = True
            self.update_radio_status("Connected (Serial)")
            self.running = True
            
            # Start serial read thread
            self.serial_read_thread = threading.Thread(target=self._read_from_serial_port, daemon=True)
            self.serial_read_thread.start()

            # Start command send worker thread
            if not self.command_send_thread_running:
                self.command_send_thread_running = True
                self.command_send_thread = threading.Thread(target=self._command_send_worker, daemon=True)
                self.command_send_thread.start()
            
            # Start motion update scheduler (for keyboard control loop)
            self._send_repeated_command()

        except serial.SerialException as e:
            self.update_radio_status("Error")
            messagebox.showerror("Connection Error", f"Could not open serial port {self.port.get()}: {e}")

    def disconnect_serial(self):
        """Closes the serial port and stops associated threads."""
        if self.serial_port and self.serial_port.is_open:
            self.running = False
            self.arduino_connected = False
            self.serial_port.close()
            self.update_radio_status("Disconnected")
            if self.motion_update_job:
                self.master.after_cancel(self.motion_update_job)
                self.motion_update_job = None
        else:
            messagebox.showwarning("Disconnection Warning", "Serial port is not open.")

    def _read_from_serial_port(self):
        """Dedicated thread to read data from the serial port."""
        while self.running:
            if self.serial_port and self.serial_port.is_open:
                try:
                    line = self.serial_port.readline().decode('utf-8').strip()
                    if line:
                        print(f"Arduino: {line}")
                        # Example position parsing (basic v469 concept)
                        if line.startswith("POS:"):
                            parts = line.split(',')
                            if len(parts) >= 3:
                                try:
                                    x = float(parts[0].split(':')[1])
                                    y = float(parts[1])
                                    r = float(parts[2])
                                    self.master.after(0, self.x_pos.set, x)
                                    self.master.after(0, self.y_pos.set, y)
                                    self.master.after(0, self.rotation_val.set, r)
                                except ValueError:
                                    pass # Ignore bad data
                except serial.SerialException:
                    self.running = False
                    self.master.after(0, self.update_radio_status, "Connection Lost")
                    break
                except TypeError: # Occurs when `self.serial_port.readline()` is called during closing
                    break
            time.sleep(0.01)

    def _command_send_worker(self):
        """Dedicated thread to send commands from the queue to the serial port."""
        while self.command_send_thread_running:
            try:
                command = self.command_send_queue.get(timeout=0.1) 
                if command is None: # Sentinel value
                    break
                
                if self.serial_port and self.serial_port.is_open and self.arduino_connected:
                    try:
                        self.serial_port.write(command.encode('utf-8'))
                        # print(f"Sent: {command.strip()}")
                    except serial.SerialException:
                        print("Error writing to serial port.")
                        self.master.after(0, self.update_radio_status, "Connection Lost")
                        break
                    except Exception as e:
                        print(f"Unexpected error during command send: {e}")

                self.command_send_queue.task_done()
                throttle = self.command_throttle_ms.get() / 1000.0
                time.sleep(throttle) 

            except queue.Empty:
                continue
            except Exception as e:
                print(f"Error in command send worker: {e}")
                break

    # --- Joystick Client (v469/v480) ---

    def _connect_to_joystick_server(self):
        """Starts the thread to connect to the external joystick server."""
        if self.joystick_thread_running:
            return

        self.joystick_thread_running = True
        self.joystick_read_thread = threading.Thread(target=self._joystick_read_thread_target, daemon=True)
        self.joystick_read_thread.start()

    def _joystick_read_thread_target(self):
        """Attempts to connect to and read from the joystick server."""
        while self.joystick_thread_running:
            if self.joystick_connected:
                # If connected, process data and wait
                self._process_joystick_queue()
                time.sleep(0.01)
                continue

            # Attempt connection
            try:
                self.joystick_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.joystick_socket.settimeout(5.0) 
                self.joystick_socket.connect((self.joystick_host, self.joystick_port))
                self.joystick_connected = True
                print("Joystick Client: Connected to server.")

                # Start reading loop
                while self.joystick_connected and self.joystick_thread_running:
                    try:
                        chunk = self.joystick_socket.recv(self.joystick_buffer_size).decode('utf-8')
                        if not chunk:
                            raise ConnectionResetError("Server closed connection.")
                        
                        self.joystick_data_buffer += chunk
                        
                        while '\n' in self.joystick_data_buffer:
                            line, self.joystick_data_buffer = self.joystick_data_buffer.split('\n', 1)
                            if line.strip():
                                self.joystick_data_queue.put(line)

                    except socket.timeout:
                        continue
                    except (socket.error, ConnectionResetError):
                        print("Joystick Client: Connection lost.")
                        self.joystick_connected = False
                        break
                
            except socket.error as e:
                # print(f"Joystick Client: Connection attempt failed. Retrying in 5s. Error: {e}")
                self.joystick_socket = None
            
            if not self.joystick_connected and self.joystick_thread_running:
                time.sleep(5) 
                
    def _process_joystick_queue(self):
        """Processes joystick data and updates motion command."""
        while not self.joystick_data_queue.empty():
            data = self.joystick_data_queue.get()
            if data is None: 
                break
            
            try:
                # Assuming data is a simple string "X,Y,R"
                x_val, y_val, r_val = map(float, data.split(','))
                
                # Check if control style is set to Joystick Control
                if self.current_control_method == "Joystick Control":
                    self.motion_command['x'] = x_val
                    self.motion_command['y'] = y_val
                    self.motion_command['rotation'] = r_val
                    self.control_source = "joystick"
                    
            except ValueError:
                print(f"Warning: Failed to parse joystick data: {data}")
            except Exception as e:
                print(f"Error processing joystick data: {e}")
            
            self.joystick_data_queue.task_done()
            
    # --- Input and Command Sending (v370/v469) ---

    def read_keyboard(self, event):
        """Handles key press events for manual control."""
        if self.current_control_method != "Direct X/Y/R Buttons":
            return
        
        # Prevent movement if a control is focused (e.g., Entry widget)
        if isinstance(self.master.focus_get(), (tk.Entry, ttk.Entry, ttk.Combobox)):
            return

        key = event.keysym.lower()
        
        # Check for laser activation
        if key == 'space' and not self.spacebar_pressed:
            self.spacebar_pressed = True
            self.laser_on.set(True)
            self.send_control_command()
            return

        key_map = {'w': 'forward', 's': 'backward', 'a': 'left', 'd': 'right', 'q': 'CCW', 'e': 'CW'}
        
        if key in key_map and not self.is_moving[key_map[key]]:
            self.is_moving[key_map[key]] = True
            self.control_source = "keyboard"
            # The _send_repeated_command loop will handle the movement update

    def read_keyrelease(self, event):
        """Handles key release events for stopping manual control."""
        if self.current_control_method != "Direct X/Y/R Buttons":
            return

        key = event.keysym.lower()

        if key == 'space' and self.spacebar_pressed:
            self.spacebar_pressed = False
            self.laser_on.set(False)
            self.send_control_command()
            return
            
        key_map = {'w': 'forward', 's': 'backward', 'a': 'left', 'd': 'right', 'q': 'CCW', 'e': 'CW'}
        
        if key in key_map:
            self.is_moving[key_map[key]] = False
            self.control_source = "keyboard"
            # The _send_repeated_command loop will handle the movement update
            
    def focus_change_handler(self, event):
        """Stops motion if keyboard focus is lost."""
        if self.motion_update_job and self.current_control_method == "Direct X/Y/R Buttons" and event.type == '9': # '9' is FocusOut
             # Only cancel the command loop if we are in Direct control mode and focus is lost
             if not isinstance(event.widget, (tk.Entry, ttk.Entry, ttk.Combobox)):
                 # Stop movement if the main window loses focus
                 all_stop = True
                 for direction in self.is_moving:
                     self.is_moving[direction] = False
                     if self.is_moving[direction]: all_stop = False
                 if all_stop:
                      self.motion_command = {"x": 0.0, "y": 0.0, "rotation": 0.0, "laser_on": False, "laser_power": 0}
                      self.send_control_command()

    def _send_repeated_command(self):
        """Scheduler function to repeatedly check for motion and send commands."""
        if self.current_control_method == "Direct X/Y/R Buttons" and self.control_source == "keyboard":
            
            # Reset motion commands
            self.motion_command['x'] = 0.0
            self.motion_command['y'] = 0.0
            self.motion_command['rotation'] = 0.0

            # Determine X/Y motion
            if self.is_moving['forward']: self.motion_command['y'] += 1.0
            if self.is_moving['backward']: self.motion_command['y'] -= 1.0
            if self.is_moving['left']: self.motion_command['x'] -= 1.0
            if self.is_moving['right']: self.motion_command['x'] += 1.0

            # Determine Rotation
            if self.is_moving['CCW']: self.motion_command['rotation'] += 1.0
            if self.is_moving['CW']: self.motion_command['rotation'] -= 1.0

            # If any movement is active, send the command
            if any(self.is_moving.values()) or self.spacebar_pressed:
                self.send_control_command()
            elif self.last_sent_motion_command.get("speed_factor") != 0.0:
                 # If no keys are pressed but the last command was a movement, send a stop command
                self.motion_command = {"x": 0.0, "y": 0.0, "rotation": 0.0, "laser_on": self.laser_on.get(), "laser_power": self.current_laser_power.get()}
                self.send_control_command()
                
        # Reschedule the next check
        self.motion_update_job = self.master.after(50, self._send_repeated_command)


    def send_control_command(self):
        """Formats and queues the motion command to the robot."""
        
        speed_factor = 0.0
        # Determine speed factor based on control source
        if self.control_source == "keyboard":
            # For keyboard, use the speed slider value
            if any(self.is_moving.values()):
                speed_factor = self.speed_var.get()
            
            # Normalize vector (optional, but good practice if X/Y/R are combined)
            magnitude = math.sqrt(self.motion_command['x']**2 + self.motion_command['y']**2 + self.motion_command['rotation']**2)
            if magnitude > 1.0:
                self.motion_command['x'] /= magnitude
                self.motion_command['y'] /= magnitude
                self.motion_command['rotation'] /= magnitude

        elif self.control_source == "joystick":
            # For joystick, the commands are normalized [-1, 1], so speed_factor is 1.0
            speed_factor = 1.0
            # A magnitude check for joystick is handled by the server typically
        
        else: # Stop everything if source is unknown
            speed_factor = 0.0
            self.motion_command['x'] = 0.0
            self.motion_command['y'] = 0.0
            self.motion_command['rotation'] = 0.0

        # Update laser settings regardless of motion
        laser_on_val = self.laser_on.get()
        laser_power_val = self.current_laser_power.get() if laser_on_val else 0

        # Create the command packet
        command = f"CONTROL,{self.motion_command['x']:.4f},{self.motion_command['y']:.4f},{self.motion_command['rotation']:.4f},{speed_factor:.4f},{laser_power_val}\n"
        
        # Check if the command has changed significantly since last send
        if (abs(self.motion_command['x'] - self.last_sent_motion_command['x']) > 0.01 or
            abs(self.motion_command['y'] - self.last_sent_motion_command['y']) > 0.01 or
            abs(self.motion_command['rotation'] - self.last_sent_motion_command['rotation']) > 0.01 or
            abs(speed_factor - self.last_sent_motion_command.get("speed_factor", 0.0)) > 0.01 or
            laser_power_val != self.last_sent_motion_command['laser_power']):

            self.command_send_queue.put(command)

            # Update last sent command state
            self.last_sent_motion_command = self.motion_command.copy()
            self.last_sent_motion_command["speed_factor"] = speed_factor
            self.last_sent_motion_command["laser_power"] = laser_power_val

    # --- G-code Processing (v469) ---

    def send_gcode_command_to_serial(self, command):
        """Queues a G-code command for the serial worker."""
        # Use a different format/prefix for G-code commands if necessary, 
        # but for simplicity, we send raw G-code lines as requested by v469 logic.
        self.command_send_queue.put(f"{command.strip()}\n")

    def select_gcode_file(self):
        """Opens a file dialog to select a G-code file."""
        file_path = filedialog.askopenfilename(
            defaultextension=".gcode",
            filetypes=[("G-code Files", "*.gcode"), ("Text Files", "*.txt"), ("All Files", "*.*")]
        )
        if file_path:
            self.gcode_file_path.set(file_path)
            self.btn_start_gcode.config(state=tk.NORMAL)
            if self.gcode_status_label:
                 self.gcode_status_label.config(text=f"Status: Ready to load {Path(file_path).name}")

    def parse_gcode_line(self, line):
        """Parses a single G-code line to update internal position (concept)."""
        line = line.strip().upper()
        if not line or line.startswith('('):
            return # Ignore empty lines and comments
        
        # Check for G-commands
        g_match = re.search(r'G(\d+)(?:\s|$)', line)
        if g_match:
            g_code = int(g_match.group(1))
            
            # G0 (Rapid Move) or G1 (Controlled Move)
            if g_code in [0, 1]:
                x_match = re.search(r'X([\d\.\-]+)', line)
                y_match = re.search(r'Y([\d\.\-]+)', line)
                f_match = re.search(r'F([\d\.\-]+)', line)
                
                new_x = self.gcode_current_x
                new_y = self.gcode_current_y
                
                if x_match:
                    x_val = float(x_match.group(1))
                    if self.gcode_absolute_mode:
                        new_x = x_val
                    else:
                        new_x += x_val
                
                if y_match:
                    y_val = float(y_match.group(1))
                    if self.gcode_absolute_mode:
                        new_y = y_val
                    else:
                        new_y += y_val
                
                # Update current position (used for next relative move)
                self.gcode_current_x = new_x
                self.gcode_current_y = new_y
                
                # Update feed rate
                if f_match:
                    self.gcode_current_feed_rate = float(f_match.group(1))
                    
            # G20 (Inches) / G21 (Millimeters)
            elif g_code == 20: messagebox.showinfo("G-code", "G20 (Inches) detected. Proceeding, but robot uses MM.")
            elif g_code == 21: pass # Millimeters, default
                
            # G90 (Absolute) / G91 (Relative)
            elif g_code == 90: self.gcode_absolute_mode = True
            elif g_code == 91: self.gcode_absolute_mode = False
            
        # Check for M-commands
        m_match = re.search(r'M(\d+)(?:\s|$)', line)
        if m_match:
            m_code = int(m_match.group(1))
            
            # M3 (Spindle/Laser On) / M5 (Spindle/Laser Off)
            if m_code == 3:
                s_match = re.search(r'S(\d+)', line)
                power = int(s_match.group(1)) if s_match else 255 
                self.gcode_current_laser_power = power
                self.gcode_current_laser_on = True
            elif m_code == 5:
                self.gcode_current_laser_power = 0
                self.gcode_current_laser_on = False
                
        # Send command to robot
        self.send_gcode_command_to_serial(line)
        self.master.after(0, self.x_pos.set, self.gcode_current_x)
        self.master.after(0, self.y_pos.set, self.gcode_current_y)
        self.master.after(0, self.current_laser_power.set, self.gcode_current_laser_power)
        self.laser_on.set(self.gcode_current_laser_on)


    def start_gcode_execution(self):
        """Loads the G-code file and starts the processing thread."""
        file_path = self.gcode_file_path.get()
        if not file_path or not os.path.exists(file_path):
            messagebox.showerror("Error", "G-code file not selected or not found.")
            return

        if not self.arduino_connected:
            messagebox.showwarning("Warning", "Connect to the robot first.")
            return

        try:
            with open(file_path, 'r') as f:
                self.gcode_queue = f.readlines()

            # Reset internal state
            self.stop_gcode_flag = False
            self.gcode_current_x = 0.0
            self.gcode_current_y = 0.0
            self.gcode_absolute_mode = True
            self.gcode_processing_active = True
            
            self.btn_start_gcode.config(state=tk.DISABLED)
            self.btn_stop_gcode.config(state=tk.NORMAL)
            self.gcode_status_label.config(text=f"Status: Running {Path(file_path).name}")

            # Start processing in the GUI thread via after() calls
            self._process_next_gcode_command()

        except Exception as e:
            messagebox.showerror("Error", f"Could not read G-code file: {e}")
            self.stop_gcode_execution()


    def _process_next_gcode_command(self):
        """Processes the next line in the G-code queue."""
        if not self.gcode_queue or self.stop_gcode_flag or not self.gcode_processing_active:
            self.stop_gcode_execution()
            return
        
        line = self.gcode_queue.pop(0)
        
        # Check for AprilTag correction if in the specific mode
        if (self.current_control_method == "Gcode with AprilTag Corrections" and 
            self.is_vision_calibrated.get() and 
            self.tag_thread_running):
            
            # --- CORRECTION LOGIC CONCEPT ---
            current_cnc_x, current_cnc_y, current_cnc_r = self.get_cnc_position_from_tag()
            # In a full implementation, you would calculate the difference between 
            # (current_cnc_x, current_cnc_y) and (self.gcode_current_x, self.gcode_current_y) 
            # and inject an adjustment command before parsing the G-code line.
            # print(f"AprilTag Corrected Pos: X{current_cnc_x:.2f} Y{current_cnc_y:.2f}. Processing line: {line.strip()}")
            pass 

        try:
            self.parse_gcode_line(line)
        except Exception as e:
             print(f"Error parsing G-code line '{line.strip()}': {e}")
             
        # Schedule the next command after a small delay (adjust based on robot response time)
        self.master.after(50, self._process_next_gcode_command)


    def stop_gcode_execution(self):
        """Stops the G-code processing."""
        self.stop_gcode_flag = True
        self.gcode_processing_active = False
        self.gcode_queue = []
        
        if self.btn_start_gcode:
            self.btn_start_gcode.config(state=tk.NORMAL)
        if self.btn_stop_gcode:
            self.btn_stop_gcode.config(state=tk.DISABLED)
        if self.gcode_status_label:
            self.gcode_status_label.config(text="Status: Stopped")

        # Send an M5 command to turn off the laser/spindle
        self.send_gcode_command_to_serial("M5\n")
        self.gcode_current_laser_power = 0
        self.gcode_current_laser_on = False
        self.current_laser_power.set(0)
        self.laser_on.set(False)

    # --- Cleanup Methods (v480) ---

    def stop_tag_thread(self): 
        self.tag_thread_running = False
        if self.tag_socket:
            try:
                self.tag_socket.close() 
            except Exception:
                pass
        if self.tag_thread and self.tag_thread.is_alive():
            self.tag_thread.join(timeout=1)

    def stop_joystick_thread(self): 
        self.joystick_thread_running = False
        if self.joystick_socket:
            try:
                self.joystick_socket.shutdown(socket.SHUT_RDWR)
                self.joystick_socket.close()
            except OSError:
                pass
        if self.joystick_read_thread and self.joystick_read_thread.is_alive():
             self.joystick_data_queue.put(None)
             self.joystick_read_thread.join(timeout=1)

    def cleanup(self):
        """Handles graceful shutdown of all threads and connections."""
        print("Closing application. Attempting graceful shutdown of threads and connections...")
        
        self.running = False 
        self.command_send_thread_running = False 
        self.stop_gcode_execution()
        
        self.stop_tag_thread() 
        self.stop_joystick_thread() 

        if self.command_send_thread and self.command_send_thread.is_alive():
            self.command_send_queue.put(None) 
            self.command_send_thread.join(timeout=1)
        
        if hasattr(self, 'serial_read_thread') and self.serial_read_thread and self.serial_read_thread.is_alive():
            self.serial_read_thread.join(timeout=1)

        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()

    def _on_closing(self): 
        self.cleanup()
        self.master.destroy()

    on_closing = _on_closing 

# -----------------------------------------------------------
# --- MANDATORY: Application Entry Point ---
# -----------------------------------------------------------

if __name__ == "__main__":
    try:
        root = tk.Tk()
        app = robotDirector(root)
        print("Starting main loop...")
        root.mainloop()
        print("Main loop terminated.")
    except Exception as e:
        print(f"An error occurred during application startup or execution: {e}")
        sys.exit(1)
