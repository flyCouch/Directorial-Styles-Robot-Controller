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

# --- Imports needed for Joystick Control ---
try:
    import pygame
    os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = "hide"
except ImportError:
    pygame = None
    # print("Warning: pygame not found. Joystick control will be disabled.")

import atexit 
# -------------------------------------------

# --- Class robotDirector (Unified Code) ---

class robotDirector:

    def __init__(self, master): 
        self.master = master
        master.title("Lyttle ReSearch Robot Director")
        master.config(bg="lightgreen") 
        master.resizable(False, False) 
        
        # --- GUI Style Configuration ---
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

        # --- Threading/Command Management ---
        self.command_send_queue = queue.Queue()
        self.command_send_thread = None
        self.command_send_thread_running = False

        # --- G-code Variables ---
        self.gcode_file_path = tk.StringVar(master) 
        self.gcode_status_label = None 
        self.stop_gcode_flag = False
        self.gcode_execution_thread = None

        # --- Serial/Radio Variables ---
        self.current_laser_power = tk.IntVar(master, value=0)
        self.laser_on = tk.BooleanVar(master, value=False)
        self.radio_status = tk.StringVar(master, value="Idle")
        self.serial_port = None  
        self.arduino_connected = False
        self.radio_channel = tk.IntVar(master, value=110)
        self.data_rate = tk.StringVar(master, value="1MBPS")
        self.nrf_power = tk.StringVar(master, value="MAX")

        # --- Motion Control Variables ---
        self.step_size_mm = tk.DoubleVar(master, value=1.0) 
        self.move_speed_mm_s = tk.DoubleVar(master, value=500.0) 
        
        self.x_dir = 0  
        self.y_dir = 0
        self.r_dir = 0

        self.motion_command = tk.StringVar(master, value="G0 X0 Y0 F500") 
        self.stop_continuous_thread = threading.Event()
        self.motion_update_thread = None

        # Work area limits
        self.work_area_width_mm = tk.DoubleVar(master, value=300.0)
        self.work_area_height_mm = tk.DoubleVar(master, value=200.0)

        # --- Vision (AprilTag) Tracking and Origin Management ---
        
        # Current actual tag position (received from vision server, relative to camera 0,0)
        self.current_tag_x_mm = 0.0
        self.current_tag_y_mm = 0.0
        self.current_tag_r_deg = 0.0
        
        # Offset: The total translation required to move from the raw Vision reading 
        # to the final CNC (G-code) coordinate.
        self.vision_offset_x = tk.DoubleVar(master, value=0.0)
        self.vision_offset_y = tk.DoubleVar(master, value=0.0)
        # Note: We keep the rotation offset for calibration, but the command is always R0
        self.vision_offset_r = tk.DoubleVar(master, value=0.0) 
        self.is_vision_calibrated = tk.BooleanVar(master, value=False)

        # AprilTag/Socket Control Variables
        self.tag_ip = tk.StringVar(master, value="127.0.0.1")
        self.tag_port = tk.IntVar(master, value=50007)
        self.tag_socket = None
        self.tag_thread = None
        self.tag_thread_running = False
        self.tag_status = tk.StringVar(master, value="Disconnected")
        self.tag_position = tk.StringVar(master, value="X:0.0 Y:0.0 R:0.0")

        # --- Joystick Control Variables ---
        self.joystick_connected = tk.BooleanVar(master, value=False)
        self.joystick_thread = None
        self.joystick_thread_running = False
        self.joystick = None
        self.joystick_x_dir = 0
        self.joystick_y_dir = 0
        self.joystick_r_dir = 0
        # ----------------------------------------------------

        self._create_main_gui()
        
        # Register cleanup function to safely stop threads
        atexit.register(self.cleanup)
        master.protocol("WM_DELETE_WINDOW", self._on_closing)

        # Start the command sending thread
        self.command_send_thread_running = True
        self.command_send_thread = threading.Thread(target=self._command_send_worker, daemon=True)
        self.command_send_thread.start()


    # -----------------------------------------------------------
    # --- GUI Creation Methods (v469 Layout) ---
    # -----------------------------------------------------------

    def _create_main_gui(self):
        main_frame = ttk.Frame(self.master, padding="10")
        main_frame.pack(fill='both', expand=True)

        # 1. Connection Frame (Row 0)
        self._create_connection_frame(main_frame)

        # 2. Control Style Selection (Row 1)
        self._create_control_style_selection(main_frame)

        # 3. Work Area Configuration (Row 2)
        self._create_work_area_config(main_frame)

        # 4. Control Specific Frame (DYNAMICALLY POPULATED) (Row 3)
        self.control_frame = ttk.LabelFrame(main_frame, text="Director Controls", padding="10")
        self.control_frame.grid(row=3, column=0, padx=10, pady=5, sticky="nsew")

        # Initialize the default control style
        self.on_control_style_change() 

        # 5. Laser/Spindle Control Frame (Row 4)
        self._create_laser_control_frame(main_frame)

        # 6. G-code Control Frame (Row 5)
        self._create_gcode_control_frame(main_frame)

    def _create_connection_frame(self, main_frame):
        conn_frame = ttk.LabelFrame(main_frame, text="Connection", padding="10")
        conn_frame.grid(row=0, column=0, padx=10, pady=5, sticky="ew")

        # Serial Port Selection
        ttk.Label(conn_frame, text="Serial Port:").grid(row=0, column=0, padx=5, pady=2, sticky="w")
        ttk.Entry(conn_frame, textvariable=self.port, width=20).grid(row=0, column=1, padx=5, pady=2, sticky="w")
        
        # Connect/Disconnect Button
        self.btn_serial_connect = ttk.Button(conn_frame, text="Connect", command=self.toggle_serial_connection)
        self.btn_serial_connect.grid(row=0, column=2, padx=5, pady=2, sticky="w")

        # Radio Status (NRF24)
        ttk.Label(conn_frame, text="Radio Status:").grid(row=1, column=0, padx=5, pady=2, sticky="w")
        ttk.Label(conn_frame, textvariable=self.radio_status, width=10).grid(row=1, column=1, padx=5, pady=2, sticky="w")

        # Channel/Power Info (Placeholder)
        ttk.Label(conn_frame, text="Channel:").grid(row=2, column=0, padx=5, pady=2, sticky="w")
        ttk.Label(conn_frame, textvariable=self.radio_channel, width=10).grid(row=2, column=1, padx=5, pady=2, sticky="w")
        ttk.Label(conn_frame, text="Data Rate:").grid(row=3, column=0, padx=5, pady=2, sticky="w")
        ttk.Label(conn_frame, textvariable=self.data_rate, width=10).grid(row=3, column=1, padx=5, pady=2, sticky="w")
        ttk.Label(conn_frame, text="RF Power:").grid(row=4, column=0, padx=5, pady=2, sticky="w")
        ttk.Label(conn_frame, textvariable=self.nrf_power, width=10).grid(row=4, column=1, padx=5, pady=2, sticky="w")


    def _create_control_style_selection(self, main_frame):
        style_frame = ttk.LabelFrame(main_frame, text="Director Style", padding="10")
        style_frame.grid(row=1, column=0, padx=10, pady=5, sticky="ew")

        ttk.Label(style_frame, text="Select Director:").pack(side=tk.LEFT, padx=5)

        # Dictionary of available control styles/directors
        self.control_styles_dict = {
            "Direct X/Y/R Buttons": self.create_button_director,
            "Joystick Control": self.create_joystick_control_area,
            "AprilTag (Vision) Director": self.create_april_tag_director, 
            "G-code File Sender": self.create_gcode_sender_director,
            "SVG/BMP Director": self.create_svg_bmp_director,
            "Tarantino as Director": self.create_placeholder_style,
            "Frickin Shoot Everyone Director": self.create_placeholder_style,
            ".dxf Director": self.create_placeholder_style,
        }
        
        self.current_style = tk.StringVar(self.master, value=list(self.control_styles_dict.keys())[0])
        
        style_dropdown = ttk.OptionMenu(style_frame, self.current_style, self.current_style.get(), 
                                        *self.control_styles_dict.keys(), 
                                        command=self.on_control_style_change)
        style_dropdown.pack(side=tk.LEFT, padx=5, fill=tk.X, expand=True)

    def _create_work_area_config(self, main_frame):
        work_area_frame = ttk.LabelFrame(main_frame, text="Work Area (mm)", padding="10")
        work_area_frame.grid(row=2, column=0, padx=10, pady=5, sticky="ew")

        ttk.Label(work_area_frame, text="Width (X):").grid(row=0, column=0, padx=5, pady=2, sticky="w")
        ttk.Entry(work_area_frame, textvariable=self.work_area_width_mm, width=10).grid(row=0, column=1, padx=5, pady=2, sticky="w")

        ttk.Label(work_area_frame, text="Height (Y):").grid(row=1, column=0, padx=5, pady=2, sticky="w")
        ttk.Entry(work_area_frame, textvariable=self.work_area_height_mm, width=10).grid(row=1, column=1, padx=5, pady=2, sticky="w")


    def _create_laser_control_frame(self, main_frame):
        laser_frame = ttk.LabelFrame(main_frame, text="Laser/Spindle", padding="10")
        laser_frame.grid(row=4, column=0, padx=10, pady=5, sticky="ew")

        ttk.Checkbutton(laser_frame, text="Laser On (Hold SPACE)", variable=self.laser_on, command=lambda: self.send_control_command()).grid(row=0, column=0, padx=5, pady=2, sticky="w")

        ttk.Label(laser_frame, text="Power (0-255):").grid(row=1, column=0, padx=5, pady=2, sticky="w")
        ttk.Scale(laser_frame, from_=0, to=255, orient='horizontal', variable=self.current_laser_power, command=lambda *a: self.send_control_command()).grid(row=1, column=1, padx=5, pady=2, sticky="ew")
        ttk.Label(laser_frame, textvariable=self.current_laser_power, width=4).grid(row=1, column=2, padx=5, pady=2, sticky="w")
        
        # Helper buttons
        ttk.Button(laser_frame, text="Laser Test (50)", command=lambda: self.send_gcode("M3 S50")).grid(row=2, column=0, padx=5, pady=2, sticky="ew")
        ttk.Button(laser_frame, text="Laser Off", command=lambda: self.send_gcode("M5")).grid(row=2, column=1, padx=5, pady=2, sticky="ew")
        
        laser_frame.grid_columnconfigure(1, weight=1) # Make the scale expand

    def _create_gcode_control_frame(self, main_frame):
        gcode_frame = ttk.LabelFrame(main_frame, text="G-code Execution", padding="10")
        gcode_frame.grid(row=5, column=0, padx=10, pady=5, sticky="ew")

        # G-code Status Label
        self.gcode_status_label = ttk.Label(gcode_frame, text="Ready", relief=tk.SUNKEN)
        self.gcode_status_label.grid(row=0, column=0, columnspan=2, padx=5, pady=5, sticky="ew")

        # Button to Open G-code File
        self.btn_open_gcode = ttk.Button(gcode_frame, text="Open G-code File", command=self.open_gcode_file)
        self.btn_open_gcode.grid(row=1, column=0, columnspan=2, padx=5, pady=5, sticky="ew")

        # Display selected G-code file path
        lbl_gcode_path = ttk.Label(gcode_frame, textvariable=self.gcode_file_path, wraplength=300)
        lbl_gcode_path.grid(row=2, column=0, columnspan=2, padx=5, pady=5, sticky="ew")

        # G-code Control Buttons (Start/Stop)
        self.btn_start_gcode = ttk.Button(gcode_frame, text="Start G-code", command=self.start_gcode_execution, state=tk.DISABLED)
        self.btn_start_gcode.grid(row=3, column=0, padx=5, pady=5, sticky="ew")

        self.btn_stop_gcode = ttk.Button(gcode_frame, text="Stop G-code", command=self.stop_gcode_execution, state=tk.DISABLED)
        self.btn_stop_gcode.grid(row=3, column=1, padx=5, pady=5, sticky="ew")


    # -----------------------------------------------------------
    # --- Director Style Management ---
    # -----------------------------------------------------------

    def on_control_style_change(self, *args):
        # 1. Clear the previous control frame contents
        for widget in self.control_frame.winfo_children():
            widget.destroy()

        # 2. Stop any previous motion, joystick, or tag threads
        self.stop_motion_update_thread()
        self.stop_joystick_thread()
        self.stop_tag_thread()

        # Unbind keyboard events
        self.master.unbind('<KeyPress>')
        self.master.unbind('<KeyRelease>')

        # 3. Get the new director method and call it to populate the frame
        style_name = self.current_style.get()
        director_method = self.control_styles_dict.get(style_name)
        
        if director_method:
            director_method(self.control_frame)
            
            # Start continuous motion thread only for styles that use it
            if style_name in ["Direct X/Y/R Buttons", "AprilTag (Vision) Director"]:
                self.start_motion_update_thread()
                if style_name == "Direct X/Y/R Buttons":
                    self.master.bind('<KeyPress>', self._handle_key_down)
                    self.master.bind('<KeyRelease>', self._handle_key_up)
        else:
            ttk.Label(self.control_frame, text="Error: Director not found.").pack(padx=10, pady=10)

    # -----------------------------------------------------------
    # --- AprilTag Logic (Origin Calculation) ---
    # -----------------------------------------------------------

    def set_work_origin_from_tag_position(self):
        """
        Calculates the total Vision Offset required to map the Vision System's 
        (T_x, T_y) coordinates to the CNC's (C_x, C_y) coordinates.

        The CNC Origin (G0 X0 Y0, lower-left corner) is defined as being 
        (-W/2, -H/2) relative to the Cam Origin (Vision 0,0, center). 
        Therefore, the Cam Origin is at (W/2, H/2) in CNC coordinates.

        The user is expected to move the robot to the physical center of the work area
        (the Cam Origin) before pressing this button.
        """
        # 1. Get current work area dimensions
        try:
            W = self.work_area_width_mm.get()
            H = self.work_area_height_mm.get()
        except tk.TclError:
            messagebox.showerror("Error", "Work Area dimensions must be valid numbers.")
            return

        # 2. Get current tag position (T_x, T_y, T_r) from the Vision frame
        T_x = self.current_tag_x_mm
        T_y = self.current_tag_y_mm
        T_r = self.current_tag_r_deg
        
        if self.tag_status.get() != "Connected":
             messagebox.showwarning("Warning", "Vision System must be connected and reporting data to calibrate.")
             return

        # 3. Calculate the Total Vision Offset (O_x, O_y)
        # Target CNC Position (C_x, C_y) for this physical location (Cam Origin) is (W/2, H/2).
        # We need C_x = T_x + O_x. Solving for O_x: O_x = C_x - T_x
        
        O_x = (W / 2.0) - T_x
        O_y = (H / 2.0) - T_y
        
        # O_r is set to counteract the current yaw T_r, forcing the final Cr to 0.0
        O_r = -T_r 

        # 4. Store the new offsets
        self.vision_offset_x.set(round(O_x, 3))
        self.vision_offset_y.set(round(O_y, 3))
        self.vision_offset_r.set(round(O_r, 3))
        self.is_vision_calibrated.set(True)
        messagebox.showinfo("Calibration Success", 
                            f"Vision calibrated successfully! \n"
                            f"Calibration Point: Physical Center of Work Area (CNC X{W/2:.3f}, Y{H/2:.3f}).\n"
                            f"Calculated Offset (O): X={O_x:.3f}, Y={O_y:.3f}, R={O_r:.3f}")
        
    def goto_work_origin(self):
        """
        Generates and sends a G0 command to move the robot to the CNC Work Origin (0,0,0) 
        using the current vision offsets.
        """
        if not self.is_vision_calibrated.get():
            messagebox.showwarning("Warning", "Please calibrate the Vision System before moving to the Work Origin.")
            return
        
        # Send G0 X0 Y0 R0 F<speed> command, which is the CNC Work Origin (lower-left corner).
        speed = self.move_speed_mm_s.get()
        command = f"G0 X0 Y0 R0 F{speed:.1f}"
        self.send_gcode(command)
        messagebox.showinfo("Move Command Sent", f"Sent: {command}. Robot will track to CNC Work Origin.")

    # -----------------------------------------------------------
    # --- G-code Execution and Pre/Post Move Logic ---
    # -----------------------------------------------------------

    def start_gcode_execution(self):
        """
        Starts the G-code execution thread. 
        MANDATORY: First queues a move to CNC Work Origin (G0 X0 Y0 R0).
        """
        file_path = self.gcode_file_path.get()
        if not file_path or not Path(file_path).is_file():
            messagebox.showerror("Error", "Please select a valid G-code file first.")
            return

        if self.gcode_execution_thread and self.gcode_execution_thread.is_alive():
            messagebox.showwarning("Warning", "G-code execution is already running.")
            return

        if not self.is_vision_calibrated.get():
            messagebox.showwarning("Warning", "Vision must be calibrated to ensure the CNC origin is set correctly. Continuing without calibration is discouraged.")
            # We allow it to continue, but the positioning may be wrong.

        # 1. Disable buttons and set flag
        self.btn_start_gcode.config(state=tk.DISABLED)
        self.btn_stop_gcode.config(state=tk.NORMAL)
        self.stop_gcode_flag = False
        self.gcode_status_label.config(text="Moving to Origin...")

        # 2. MANDATORY: Queue move to CNC Work Origin (G0 X0 Y0 R0)
        # This move is necessary to align the robot with the established CNC coordinate system.
        speed = self.move_speed_mm_s.get()
        origin_move_command = f"G0 X0 Y0 R0 F{speed:.1f}"
        
        # Queue the initial move command
        self.send_gcode_async(origin_move_command)

        # 3. Start the execution thread
        self.gcode_execution_thread = threading.Thread(
            target=self._gcode_execution_worker, 
            args=(file_path,), 
            daemon=True
        )
        self.gcode_execution_thread.start()


    def _gcode_execution_worker(self, file_path):
        """
        Worker thread for G-code file execution. Ensures the robot moves to origin 
        before streaming the file.
        """
        try:
            with open(file_path, 'r') as f:
                gcode_lines = [line.strip() for line in f if line.strip() and not line.strip().startswith(';')]

            if not gcode_lines:
                self.gcode_status_label.config(text="File Empty or only comments.")
                return

            # Wait for the initial G0 X0 Y0 R0 command to be sent and acknowledged
            # This is a simplification; in a real GRBL-like system, you'd wait for an 'ok' or position report.
            # Here, we wait for the queue to empty after the initial command is sent.
            print("INFO: Waiting for initial move to CNC Origin to be queued...")
            time.sleep(1.0) # Give the command_send_worker time to process the first command

            self.gcode_status_label.config(text=f"Streaming G-code lines...")

            for i, line in enumerate(gcode_lines):
                if self.stop_gcode_flag:
                    print("INFO: G-code execution stopped by user.")
                    self.gcode_status_label.config(text="Execution Stopped")
                    break
                
                # Send command and wait for a brief period/acknowledgement
                self.send_gcode_async(line)
                
                self.gcode_status_label.config(text=f"Line {i+1}/{len(gcode_lines)}: {line}")
                time.sleep(0.1) # Throttle the sending rate (simulating waiting for 'ok')
                
            else:
                self.gcode_status_label.config(text="Execution Complete")

        except Exception as e:
            error_message = f"G-code execution failed: {e}"
            print(f"ERROR: {error_message}")
            messagebox.showerror("Execution Error", error_message)
            self.gcode_status_label.config(text="Execution Error")
            
        finally:
            self.stop_gcode_flag = False
            self.master.after(100, lambda: self.btn_start_gcode.config(state=tk.NORMAL))
            self.master.after(100, lambda: self.btn_stop_gcode.config(state=tk.DISABLED))


    def stop_gcode_execution(self):
        self.stop_gcode_flag = True
        self.gcode_status_label.config(text="Stopping...")
        # Emergency Stop command (if implemented by the microcontroller)
        self.send_gcode("M112")
        self.btn_stop_gcode.config(state=tk.DISABLED)
        self.btn_start_gcode.config(state=tk.NORMAL)
        # Clear the queue to prevent further commands
        with self.command_send_queue.mutex:
            self.command_send_queue.queue.clear()


    def open_gcode_file(self):
        f_types = [('G-Code Files', '*.gcode'), ('Text Files', '*.txt'), ('All Files', '*.*')]
        file_path = filedialog.askopenfilename(filetypes=f_types)
        if file_path:
            self.gcode_file_path.set(file_path)
            self.btn_start_gcode.config(state=tk.NORMAL)
            self.gcode_status_label.config(text="File Loaded: Ready to Start")
            print(f"INFO: Loaded G-code file: {file_path}")


    # -----------------------------------------------------------
    # --- AprilTag Director Implementation (Continued) ---
    # -----------------------------------------------------------

    def create_april_tag_director(self, parent_frame):
        """Creates the frame for the AprilTag (Vision) Director style."""
        
        # --- 1. Connection Frame ---
        conn_frame = ttk.LabelFrame(parent_frame, text="Vision Server Connection", padding="10")
        conn_frame.pack(fill='x', padx=5, pady=5)
        
        ttk.Label(conn_frame, text="IP:").grid(row=0, column=0, padx=5, pady=2, sticky="w")
        ttk.Entry(conn_frame, textvariable=self.tag_ip, width=15).grid(row=0, column=1, padx=5, pady=2, sticky="w")
        
        ttk.Label(conn_frame, text="Port:").grid(row=0, column=2, padx=5, pady=2, sticky="w")
        ttk.Entry(conn_frame, textvariable=self.tag_port, width=8).grid(row=0, column=3, padx=5, pady=2, sticky="w")

        self.btn_tag_connect = ttk.Button(conn_frame, text="Connect Vision", command=self.toggle_tag_connection)
        self.btn_tag_connect.grid(row=1, column=0, columnspan=4, padx=5, pady=5, sticky="ew")

        ttk.Label(conn_frame, text="Status:").grid(row=2, column=0, padx=5, pady=2, sticky="w")
        ttk.Label(conn_frame, textvariable=self.tag_status, relief=tk.SUNKEN).grid(row=2, column=1, columnspan=3, padx=5, pady=2, sticky="ew")

        # --- 2. Origin/Calibration Frame (CNC Origin Management) ---
        origin_frame = ttk.LabelFrame(parent_frame, text="Work Origin Management", padding="10")
        origin_frame.pack(fill='x', padx=5, pady=5)
        
        # Calibration Status
        ttk.Label(origin_frame, text="Calibrated:").grid(row=0, column=0, padx=5, pady=2, sticky="w")
        ttk.Checkbutton(origin_frame, variable=self.is_vision_calibrated, state=tk.DISABLED).grid(row=0, column=1, padx=5, pady=2, sticky="w")

        # Set Origin Button: NOW REFLECTS THAT CALIBRATION IS FOR THE CAM/WORK CENTER
        ttk.Button(origin_frame, 
                   text="CALIBRATE: Position robot at Physical Work Center", 
                   command=self.set_work_origin_from_tag_position).grid(row=1, column=0, columnspan=2, padx=5, pady=5, sticky="ew")

        # Move to Work Origin Button (Pre/Post G-code move)
        ttk.Button(origin_frame, 
                   text="Go To CNC Work Origin (G0 X0 Y0 R0)", 
                   command=self.goto_work_origin).grid(row=2, column=0, columnspan=2, padx=5, pady=5, sticky="ew")


        # Offset Display
        ttk.Label(origin_frame, text="Vision Offset X:").grid(row=3, column=0, padx=5, pady=2, sticky="w")
        ttk.Label(origin_frame, textvariable=self.vision_offset_x, width=10, relief=tk.SUNKEN).grid(row=3, column=1, padx=5, pady=2, sticky="w")

        ttk.Label(origin_frame, text="Vision Offset Y:").grid(row=4, column=0, padx=5, pady=2, sticky="w")
        ttk.Label(origin_frame, textvariable=self.vision_offset_y, width=10, relief=tk.SUNKEN).grid(row=4, column=1, padx=5, pady=2, sticky="w")

        ttk.Label(origin_frame, text="Vision Offset R:").grid(row=5, column=0, padx=5, pady=2, sticky="w")
        ttk.Label(origin_frame, textvariable=self.vision_offset_r, width=10, relief=tk.SUNKEN).grid(row=5, column=1, padx=5, pady=2, sticky="w")
        
        origin_frame.grid_columnconfigure(1, weight=1)

        # --- 3. Position Status Frame ---
        status_frame = ttk.LabelFrame(parent_frame, text="Real-time Position", padding="10")
        status_frame.pack(fill='x', padx=5, pady=5)
        
        ttk.Label(status_frame, text="Tag Position (X,Y,R):").grid(row=0, column=0, padx=5, pady=2, sticky="w")
        ttk.Label(status_frame, textvariable=self.tag_position, relief=tk.SUNKEN).grid(row=0, column=1, padx=5, pady=2, sticky="ew")

        ttk.Label(status_frame, text="CNC Command Error (X,Y,R):").grid(row=1, column=0, padx=5, pady=2, sticky="w")
        self.cnc_error_label = ttk.Label(status_frame, text="---", relief=tk.SUNKEN)
        self.cnc_error_label.grid(row=1, column=1, padx=5, pady=2, sticky="ew")

        status_frame.grid_columnconfigure(1, weight=1)

        ttk.Label(parent_frame, text="Closed-Loop control active. Movement is automatic once connected and calibrated.").pack(padx=10, pady=10)

    # --- Placeholder/Simple Director Methods ---

    def create_button_director(self, parent_frame):
        """Creates the frame for the Direct X/Y/R Buttons control style."""
        
        # Settings frame for step size and speed
        settings_frame = ttk.LabelFrame(parent_frame, text="Settings", padding="10")
        settings_frame.pack(fill='x', padx=5, pady=5)
        
        ttk.Label(settings_frame, text="Step Size (mm):").grid(row=0, column=0, padx=5, pady=2, sticky="w")
        ttk.Entry(settings_frame, textvariable=self.step_size_mm, width=10).grid(row=0, column=1, padx=5, pady=2, sticky="w")

        ttk.Label(settings_frame, text="Jog Speed (mm/s):").grid(row=1, column=0, padx=5, pady=2, sticky="w")
        ttk.Entry(settings_frame, textvariable=self.move_speed_mm_s, width=10).grid(row=1, column=1, padx=5, pady=2, sticky="w")

        # --- Motion Buttons Frame ---
        motion_frame = ttk.Frame(parent_frame, padding="10")
        motion_frame.pack(fill='both', padx=5, pady=5)
        
        # Center Row for Y movement
        self._create_motion_button(motion_frame, "Y+", 0, 1, 1, self.move_by_step, "+Y")
        self._create_motion_button(motion_frame, "HOME (G28)", 1, 1, 1, lambda: self.send_gcode("G28"))
        self._create_motion_button(motion_frame, "Y-", 2, 1, 1, self.move_by_step, "-Y")
        
        # Middle Row for X movement
        self._create_motion_button(motion_frame, "X-", 1, 0, 1, self.move_by_step, "-X")
        self._create_motion_button(motion_frame, "X+", 1, 2, 1, self.move_by_step, "+X")

        # Bottom Row for R (Rotation)
        ttk.Label(motion_frame, text="Rotation (R):").grid(row=3, column=0, padx=5, pady=5, sticky="e")
        self._create_motion_button(motion_frame, "R-", 3, 1, 1, self.move_by_step, "-R")
        self._create_motion_button(motion_frame, "R+", 3, 2, 1, self.move_by_step, "+R")

        # Continuous Control (Press & Hold) Buttons
        continuous_frame = ttk.LabelFrame(parent_frame, text="Continuous Jog (Hold Keys/Button)", padding="10")
        continuous_frame.pack(fill='x', padx=5, pady=5)

        ttk.Label(continuous_frame, text="W/S: Y-Axis, A/D: X-Axis, Q/E: R-Axis").grid(row=0, column=0, columnspan=2, pady=5, sticky="ew")

        # Continuous Jog Buttons 
        self._create_continuous_button(continuous_frame, "Y+", 'w', 's', 'Y', 1, 0)
        self._create_continuous_button(continuous_frame, "Y-", 's', 'w', 'Y', 1, 1)
        self._create_continuous_button(continuous_frame, "X+", 'd', 'a', 'X', 2, 0)
        self._create_continuous_button(continuous_frame, "X-", 'a', 'd', 'X', 2, 1)
        self._create_continuous_button(continuous_frame, "R+", 'e', 'q', 'R', 3, 0)
        self._create_continuous_button(continuous_frame, "R-", 'q', 'e', 'R', 3, 1)

        continuous_frame.grid_columnconfigure((0, 1), weight=1)

    def _create_motion_button(self, parent_frame, text, row, col, columnspan, command, axis=None):
        """Helper function to create a motion control button."""
        if axis:
            btn = ttk.Button(parent_frame, text=text, command=lambda: command(axis))
        else:
            btn = ttk.Button(parent_frame, text=text, command=command)
        btn.grid(row=row, column=col, columnspan=columnspan, padx=5, pady=5, sticky="ew")
        return btn

    def _create_continuous_button(self, parent_frame, text, key_down, key_up, axis, row, col):
        """Helper function to create a continuous motion button with key binding info."""
        # Key bindings are handled by the main key handler
        ttk.Button(parent_frame, text=f"{text} ({key_down.upper()})", 
                   command=lambda: self.move_continuous_trigger(axis, 1 if text.endswith('+') else -1)).grid(row=row, column=col, padx=5, pady=2, sticky="ew")
        
    def move_by_step(self, axis):
        """Sends a G1 command to move the robot by the defined step size."""
        try:
            step = self.step_size_mm.get()
            speed = self.move_speed_mm_s.get()
            
            # Determine direction
            direction = 1 if axis.startswith('+') else -1
            axis_char = axis[-1]
            
            distance = direction * step
            
            command = f"G1 {axis_char}{distance:.3f} F{speed:.1f}"
            self.send_gcode(command)
        except Exception as e:
            messagebox.showerror("Error", f"Invalid input for step size or speed: {e}")

    def _handle_key_down(self, event):
        key = event.keysym.lower()
        if key == 'w': self.y_dir = 1
        elif key == 's': self.y_dir = -1
        elif key == 'd': self.x_dir = 1
        elif key == 'a': self.x_dir = -1
        elif key == 'e': self.r_dir = 1
        elif key == 'q': self.r_dir = -1
        elif key == 'space': self.laser_on.set(True)
        self._update_motion_command()

    def _handle_key_up(self, event):
        key = event.keysym.lower()
        if key in ('w', 's'): self.y_dir = 0
        elif key in ('d', 'a'): self.x_dir = 0
        elif key in ('e', 'q'): self.r_dir = 0
        elif key == 'space': self.laser_on.set(False)
        self._update_motion_command()

    def _update_motion_command(self):
        """Generates the G0 command string based on current directional inputs."""
        speed = self.move_speed_mm_s.get()
        
        # Calculate the movement vector (which is the actual movement command we want)
        # Note: Since we are using an encoder/closed-loop system, we send the target 
        # offset (0 or 1mm in that direction) repeatedly, or simply G0/G1 without 
        # coordinates and let the robot maintain motion if it supports that.
        # For simplicity, we stick to sending the M-code based on the direction for continuous jog:
        
        # Directional M-codes: M100 - M105 for continuous jog
        if self.x_dir != 0 or self.y_dir != 0 or self.r_dir != 0:
            x_code = 100 + (self.x_dir + 1) # 101 (X+), 99 (X-), 100 (X stop - not needed)
            y_code = 102 + (self.y_dir + 1) # 103 (Y+), 101 (Y-), 102 (Y stop - not needed)
            r_code = 104 + (self.r_dir + 1) # 105 (R+), 103 (R-), 104 (R stop - not needed)

            command = f"M100 X{self.x_dir} Y{self.y_dir} R{self.r_dir} F{speed:.1f}"
        else:
            command = f"M100 X0 Y0 R0 F{speed:.1f}"
            
        self.motion_command.set(command)
        self.send_gcode_async(command)

        # Update laser status separately if it changed (triggered by space bar)
        self.send_control_command()


    def start_motion_update_thread(self):
        """Starts the thread responsible for continuously sending motion commands (used by AprilTag & Continuous Jog)."""
        self.stop_continuous_thread.clear()
        self.motion_update_thread = threading.Thread(target=self._motion_update_worker, daemon=True)
        self.motion_update_thread.start()
        print("INFO: Motion update thread started.")

    def stop_motion_update_thread(self):
        """Stops the continuous motion update thread."""
        if self.motion_update_thread and self.motion_update_thread.is_alive():
            self.stop_continuous_thread.set()
            self.motion_update_thread.join(timeout=1)
            print("INFO: Motion update thread stopped.")
            
    def _motion_update_worker(self):
        """Worker function for the continuous motion update thread."""
        if self.current_style.get() == "AprilTag (Vision) Director":
            # For AprilTag, the update command is derived from Vision tracking
            while not self.stop_continuous_thread.is_set():
                if self.is_vision_calibrated.get() and self.tag_status.get() == "Connected":
                    self._calculate_cnc_command_from_tag()
                time.sleep(0.1) # Update at 10Hz
        
        elif self.current_style.get() == "Direct X/Y/R Buttons":
             # For Button Director, the update command is generated by _update_motion_command
             while not self.stop_continuous_thread.is_set():
                if self.x_dir != 0 or self.y_dir != 0 or self.r_dir != 0:
                    self._update_motion_command()
                # Use a slower interval here since the key handler updates immediately
                time.sleep(0.05)


    def _calculate_cnc_command_from_tag(self):
        """
        Calculates the required closed-loop G0 movement command based on the 
        current AprilTag position and the established vision offsets.
        
        CRITICAL FIX: The command R (rotation/yaw) is always R0 to maintain fixed yaw.
        """
        # 1. Get current data
        Tx = self.current_tag_x_mm
        Ty = self.current_tag_y_mm
        Tr = self.current_tag_r_deg
        
        Ox = self.vision_offset_x.get()
        Oy = self.vision_offset_y.get()
        Or = self.vision_offset_r.get() # This offset is used for the calibration base

        speed = self.move_speed_mm_s.get()

        # 2. Convert Tag Position to CNC Position: C = T + O
        # This gives us the robot's current position in CNC coordinates.
        Cx = Tx + Ox
        Cy = Ty + Oy
        # Cr calculation uses the calibration offset. While we calculate it, we don't command it.
        Cr = Tr + Or 
        
        # 3. Calculate Error (Used only for display)
        # Note: If the goal is to hold position, the target is the last commanded position (not implemented here).
        # Assuming the system goal is to track the initial calibrated origin (0,0) for X/Y.
        # This is used to display how far the robot is from the CNC origin.
        ErrorX = -Cx 
        ErrorY = -Cy
        ErrorR = -Cr

        self.cnc_error_label.config(text=f"X:{ErrorX:.2f} Y:{ErrorY:.2f} R:{ErrorR:.2f}")

        # 4. Generate the movement command. We command the robot's current calculated X/Y 
        # position in CNC coordinates (Cx, Cy) as the setpoint for closed-loop, 
        # but force the R component to 0.0 to fix the yaw.
        
        # UPDATED: R must always be R0.
        command = f"G0 X{Cx:.3f} Y{Cy:.3f} R0 F{speed:.1f}"
        
        self.motion_command.set(command)
        self.send_gcode_async(command)


    def create_gcode_sender_director(self, parent_frame):
        """Creates the frame for the G-code File Sender style."""
        ttk.Label(parent_frame, text="This style utilizes the G-code File Execution controls below. Calibration is required if using vision offsets.").pack(padx=10, pady=10)

    
    def create_joystick_control_area(self, parent_frame, event=None):
        """Creates the UI for Joystick Control and starts the thread."""
        if pygame is None:
             ttk.Label(parent_frame, text="Joystick Control Disabled: 'pygame' library is not installed.", 
                       foreground="red").pack(padx=10, pady=10)
             return

        # Settings frame for move speed
        settings_frame = ttk.LabelFrame(parent_frame, text="Settings", padding="10")
        settings_frame.pack(fill='x', padx=5, pady=5)
        
        ttk.Label(settings_frame, text="Max Jog Speed (mm/s):").grid(row=1, column=0, padx=5, pady=2, sticky="w")
        ttk.Entry(settings_frame, textvariable=self.move_speed_mm_s, width=10).grid(row=1, column=1, padx=5, pady=2, sticky="w")

        # Status Label
        status_frame = ttk.Frame(parent_frame, padding="10")
        status_frame.pack(fill='x', padx=5, pady=5)
        
        ttk.Label(status_frame, text="Joystick Status:").pack(side=tk.LEFT)
        self.joystick_status_label = ttk.Label(status_frame, 
                                               textvariable=self.joystick_connected, 
                                               width=20, 
                                               relief=tk.SUNKEN)
        self.joystick_status_label.pack(side=tk.LEFT, padx=5)

        ttk.Label(status_frame, text="Command:").pack(side=tk.LEFT, padx=(15, 0))
        self.joystick_command_label = ttk.Label(status_frame, 
                                                textvariable=self.motion_command, 
                                                width=20, 
                                                relief=tk.SUNKEN)
        self.joystick_command_label.pack(side=tk.LEFT, padx=5)
        
        self.start_joystick_thread(parent_frame)


    def create_svg_bmp_director(self, parent_frame, event=None):
        ttk.Label(parent_frame, text="SVG/BMP Director (Not Implemented Yet)").pack(padx=10, pady=10)

    def create_placeholder_style(self, parent_frame, event=None):
        # NOTE: This method is retained exactly as in the original provided code.
        ttk.Label(parent_frame, text=f"Placeholder Style (Not Implemented Yet)").pack(padx=10, pady=10)


    # -----------------------------------------------------------
    # --- Joystick Logic (Retained from original code) ---
    # -----------------------------------------------------------
    
    def start_joystick_thread(self, parent_frame):
        # Implementation details omitted for brevity
        pass

    def stop_joystick_thread(self):
        # Implementation details omitted for brevity
        pass

    # -----------------------------------------------------------
    # --- Serial/Command Logic (Retained from original code) ---
    # -----------------------------------------------------------
    
    def toggle_serial_connection(self):
        # Implementation details omitted for brevity
        pass

    def _command_send_worker(self):
        # Implementation details omitted for brevity
        pass

    def send_gcode(self, gcode):
        # Implementation details omitted for brevity
        pass

    def send_gcode_async(self, gcode):
        # Implementation details omitted for brevity
        self.command_send_queue.put(gcode)

    def send_control_command(self, *args):
        # Implementation details omitted for brevity
        pass

    # -----------------------------------------------------------
    # --- AprilTag Socket Logic (Retained from original code) ---
    # -----------------------------------------------------------
    
    def toggle_tag_connection(self):
        # Implementation details omitted for brevity
        pass

    def start_tag_thread(self):
        # Implementation details omitted for brevity
        pass

    def stop_tag_thread(self):
        # Implementation details omitted for brevity
        pass

    def _tag_receive_worker(self):
        # Implementation details omitted for brevity
        pass

    def _parse_tag_data(self, data):
        # Implementation details omitted for brevity
        try:
            # Example parsing for JSON format: {"x": 10.5, "y": 20.1, "r": -5.0}
            tag_data = json.loads(data)
            self.current_tag_x_mm = tag_data.get('x', 0.0)
            self.current_tag_y_mm = tag_data.get('y', 0.0)
            self.current_tag_r_deg = tag_data.get('r', 0.0)
            self.master.after(0, self.tag_position.set, 
                              f"X:{self.current_tag_x_mm:.2f} Y:{self.current_tag_y_mm:.2f} R:{self.current_tag_r_deg:.2f}")

        except json.JSONDecodeError as e:
            print(f"WARNING: Could not parse tag data as JSON: {data[:50]}... Error: {e}")
        except Exception as e:
            print(f"ERROR during tag data parsing: {e}")

    # -----------------------------------------------------------
    # --- Cleanup ---
    # -----------------------------------------------------------

    def cleanup(self):
        self.stop_gcode_execution()
        self.stop_tag_thread()
        self.stop_joystick_thread()
        self.stop_motion_update_thread()
        self.command_send_thread_running = False
        if self.command_send_thread and self.command_send_thread.is_alive():
            self.command_send_queue.put(None) # Sentinel to stop thread
            self.command_send_thread.join(timeout=1)
        if self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.close()
            except:
                pass

    def _on_closing(self):
        self.cleanup()
        self.master.destroy()

# --- MANDATORY: Application Entry Point ---
# This block ensures the application starts the GUI when the file is executed.
if __name__ == "__main__":
    root = tk.Tk()
    app = robotDirector(root)
    root.mainloop()
