import tkinter as tk
from tkinter import ttk
import cv2
import numpy as np
import pickle
import time
import urllib.request
import urllib.error
import threading 
from PIL import Image, ImageTk

# --- Calibration Parameters ---
# IMPORTANT: Adjust these to match your checkerboard
CHECKERBOARD = (4, 6)  # Inner corners: (columns - 1, rows - 1). Standard 7x10 board is (6, 9).
SQUARE_SIZE_MM = 25.0  # Size of a single square on the checkerboard in millimeters

# --- Configuration for Cycling ---
IP_START_RANGE = 201
IP_END_RANGE = 209
IP_BASE = "192.168.43" # Base for all 9 camera IPs (e.g., 192.168.43.XXX)

# --- Known ESP32-CAM Stream Endpoints ---
# Added '/' and '/cam' as primary endpoints
STREAM_ENDPOINTS = ["/", "/cam", "/stream", "/video", "/mjpeg/1"]

# --- Browser Headers to bypass anti-bot measures ---
# Explicitly set Accept to prefer the MJPEG stream type
BROWSER_HEADERS = {
    'User-Agent': 'Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/91.0.4472.124 Safari/537.36',
    'Accept': 'multipart/x-mixed-replace, image/jpeg, */*',
    'Accept-Language': 'en-US,en;q=0.5',
    'Connection': 'keep-alive'
}

# --- Frame Reader Class for MJPEG Streams ---
class MJPEGFrameReader:
    """A dedicated reader to handle multipart MJPEG streams reliably."""
    def __init__(self, url):
        self.url = url
        self.stream = None
        self.boundary = b''
        self.is_open = False
        self.last_frame = None

    def open(self):
        """Attempts to open the stream and find the MJPEG boundary."""
        try:
            req = urllib.request.Request(self.url, headers=BROWSER_HEADERS)
            self.stream = urllib.request.urlopen(req, timeout=5)
            self.is_open = True
            
            # Read the header to find the boundary
            header = self.stream.readline().strip()
            if not header.startswith(b'--'):
                # Read until we find the start of the first boundary
                while not header.startswith(b'--'):
                    header = self.stream.readline().strip()

            # The boundary is the first line starting with '--'
            self.boundary = header
            return True

        except urllib.error.URLError as e:
            print(f"URLError opening stream {self.url}: {e}")
            self.is_open = False
            return False
        except Exception as e:
            print(f"Error opening stream {self.url}: {e}")
            self.is_open = False
            return False

    def read(self):
        """Reads the next full frame (JPEG image)."""
        if not self.is_open:
            return False, None

        try:
            # 1. Skip headers until the boundary is found (or the end of stream)
            while True:
                line = self.stream.readline()
                if not line:
                    self.is_open = False # Stream ended
                    return False, None
                if line.strip() == self.boundary:
                    break
            
            # 2. Skip Content-Type and Content-Length headers
            content_length = None
            while True:
                line = self.stream.readline().strip()
                if not line: break # Empty line separates headers from image data
                if line.startswith(b'Content-Length:'):
                    try:
                        content_length = int(line.split(b':')[1].strip())
                    except:
                        pass # Ignore if Content-Length is malformed

            # 3. Read the image data
            if content_length:
                # Optimized read if Content-Length is provided
                image_data = self.stream.read(content_length)
            else:
                # Read chunks until the next boundary is reached (less efficient)
                image_data = b''
                while True:
                    chunk = self.stream.read(1024)
                    if not chunk or self.boundary in chunk:
                        # If boundary is found in chunk, we've read too far
                        if self.boundary in chunk:
                            part_before_boundary, _ = chunk.split(self.boundary, 1)
                            image_data += part_before_boundary
                        break
                    image_data += chunk
            
            # Convert bytes to a numpy array (for OpenCV)
            np_array = np.frombuffer(image_data, dtype=np.uint8)
            frame = cv2.imdecode(np_array, cv2.IMREAD_COLOR)

            if frame is not None:
                self.last_frame = frame
                return True, frame
            else:
                return False, None

        except Exception as e:
            # print(f"Error reading stream frame: {e}") # Suppress frequent errors
            self.is_open = False
            return False, None
    
    def release(self):
        """Closes the underlying stream connection."""
        if self.stream:
            self.stream.close()
        self.is_open = False

class CameraCalibrationApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Multi-Camera Calibration Utility")
        
        # Internal state
        self.current_ip_end = IP_START_RANGE
        self.cap = None # Use this now to hold the MJPEGFrameReader instance
        self.is_running = True # Flag to control the update thread
        self.last_frame = None # Holds the last retrieved OpenCV frame
        
        # Calibration storage
        self.obj_points = []
        self.img_points = []
        self.camera_matrix = None
        self.dist_coeffs = None
        self.new_camera_matrix = None

        # Prepare 3D object points (0,0,0), (1,0,0), (2,0,0) ...
        self.objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
        # Apply the physical square size to the 3D points
        self.objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2) * SQUARE_SIZE_MM

        # Tkinter UI elements
        self.create_widgets()
        
        # Initialize the first camera stream
        self.load_stream()

        # Start the thread to continuously update the stream
        self.update_thread = threading.Thread(target=self.stream_update_loop, daemon=True)
        self.update_thread.start()
        
        # Bind keyboard shortcuts
        self.root.bind('<Right>', self.next_cam)
        self.root.bind('<Left>', self.prev_cam)
        self.root.bind('c', self.capture_frame)
        self.root.bind('k', self.calibrate_cam)
        self.root.bind('s', self.save_calibration)
        self.root.bind('d', self.undistort_toggle)

    def create_widgets(self):
        # Frame for controls (top)
        control_frame = ttk.Frame(self.root, padding="10")
        control_frame.pack(fill='x')
        
        # Current Camera Label
        self.cam_label = ttk.Label(control_frame, text=f"Current Cam: {IP_BASE}.{self.current_ip_end} (Status: Disconnected)", font=('Arial', 14, 'bold'))
        self.cam_label.grid(row=0, column=0, columnspan=4, sticky='w')

        # Camera Navigation Buttons
        ttk.Button(control_frame, text="< Prev", command=self.prev_cam).grid(row=1, column=0, padx=5, pady=5)
        ttk.Button(control_frame, text="Next >", command=self.next_cam).grid(row=1, column=1, padx=5, pady=5)
        
        # Calibration Buttons (Row 2)
        ttk.Button(control_frame, text="[c] Capture Frame (0)", command=self.capture_frame).grid(row=2, column=0, padx=5, pady=5)
        self.capture_count_var = tk.StringVar(value="0")
        ttk.Label(control_frame, textvariable=self.capture_count_var).grid(row=2, column=1, sticky='w')
        
        ttk.Button(control_frame, text="[k] Calibrate", command=self.calibrate_cam).grid(row=2, column=2, padx=5, pady=5)
        ttk.Button(control_frame, text="[s] Save", command=self.save_calibration).grid(row=2, column=3, padx=5, pady=5)

        # Undistort Toggle Checkbox (Row 3)
        self.undistort_var = tk.BooleanVar()
        self.undistort_check = ttk.Checkbutton(control_frame, text="[d] Undistort View", variable=self.undistort_var, command=self.undistort_toggle)
        self.undistort_check.grid(row=3, column=0, columnspan=2, sticky='w', padx=5, pady=5)

        # Frame for Video and Messages (bottom)
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.pack(fill='both', expand=True)

        # Video Display (Left)
        self.video_label = ttk.Label(main_frame)
        self.video_label.pack(side='left', fill='both', expand=True, padx=5, pady=5)

        # Message Display (Right)
        message_frame = ttk.Frame(main_frame)
        message_frame.pack(side='right', fill='y', padx=5, pady=5)
        
        ttk.Label(message_frame, text="Status Messages:", font=('Arial', 10, 'bold')).pack(fill='x')
        
        self.message_display = tk.Text(message_frame, width=50, height=25, state='disabled', wrap='word', bg='#2c2c2c', fg='#f0f0f0')
        self.message_display.pack(side='left', fill='y')
        
        scrollbar = ttk.Scrollbar(message_frame, command=self.message_display.yview)
        scrollbar.pack(side='right', fill='y')
        self.message_display.config(yscrollcommand=scrollbar.set)
        
        # Display instructions
        self.display_message("--- Instructions ---")
        self.display_message("1. Point camera at checkerboard.")
        self.display_message("2. Press [c] to capture frame (need ~10).")
        self.display_message("3. Press [k] to calculate calibration.")
        self.display_message("4. Press [s] to save the .pkl file.")
        self.display_message("5. Press [d] to toggle distortion correction.")
        self.display_message("--------------------")

    def display_message(self, message):
        """Appends a message to the text area and scrolls to the bottom."""
        self.message_display.config(state='normal')
        self.message_display.insert(tk.END, message + "\n")
        self.message_display.see(tk.END)
        self.message_display.config(state='disabled')
        
    def stream_update_loop(self):
        """Runs in a separate thread to continuously read the stream."""
        while self.is_running:
            if self.cap and self.cap.is_open:
                # Read the latest frame using the custom reader
                ret, frame = self.cap.read()
                if ret:
                    self.last_frame = frame
                    # Schedule the UI update on the main thread
                    self.root.after(0, self.update_display)
                else:
                    # If stream fails mid-read, try to reopen
                    self.release_stream()
                    self.root.after(0, self.load_stream)
            else:
                # If stream is closed or failed, try loading it again
                self.root.after(0, self.load_stream)
            
            # Control the refresh rate (adjust as needed, 50ms is 20 FPS max)
            time.sleep(0.05)    

    def update_display(self):
        """Updates the Tkinter image on the main thread."""
        if self.last_frame is None:
            return

        frame = self.last_frame.copy()
        
        # 1. Apply Undistortion if requested and calibration exists
        if self.undistort_var.get() and self.camera_matrix is not None and self.dist_coeffs is not None:
             # Calculate optimal new camera matrix only once after calibration
            if self.new_camera_matrix is None:
                h, w = frame.shape[:2]
                self.new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(self.camera_matrix, self.dist_coeffs, (w, h), 1, (w, h))

            # Undistort the frame
            frame = cv2.undistort(frame, self.camera_matrix, self.dist_coeffs, None, self.new_camera_matrix)

        # 2. Convert and Display
        # Convert BGR to RGB (Tkinter/PIL expects RGB)
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        
        # Convert to PIL Image
        img_pil = Image.fromarray(frame_rgb)
        
        # Resize to fit the UI area (example resize, adjust as necessary)
        # Using a fixed size for simplicity, or grab the video_label size
        target_width = 800
        target_height = int(target_width * frame.shape[0] / frame.shape[1])
        img_pil = img_pil.resize((target_width, target_height), Image.Resampling.LANCZOS)
        
        # Convert to ImageTk format
        img_tk = ImageTk.PhotoImage(image=img_pil)

        # Update the Tkinter label
        self.video_label.imgtk = img_tk  # Keep a reference!
        self.video_label.config(image=img_tk)

    def trigger_stream_if_needed(self, ip_address):
        """Pings the camera's root IP to ensure the streaming service is initialized."""
        root_url = f"http://{ip_address}/"
        try:
            # We don't care about the content, just that it executes the camera's setup code
            self.display_message(f"Pinging {root_url} to wake up the stream...")
            req = urllib.request.Request(root_url, headers=BROWSER_HEADERS)
            with urllib.request.urlopen(req, timeout=5) as response:
                self.display_message(f"Ping successful. Status: {response.getcode()}")
        except urllib.error.URLError as e:
            self.display_message(f"Warning: Ping to {root_url} failed ({e}). Stream may not start.")
        except Exception as e:
             self.display_message(f"Warning: Ping to {root_url} failed with general error: {e}")


    def load_stream(self):
        """Attempts to connect to the current camera's stream."""
        self.release_stream() # Ensure previous stream is closed
        
        full_ip = f"{IP_BASE}.{self.current_ip_end}"
        self.cam_label.config(text=f"Current Cam: {full_ip} (Status: Connecting...)")
        
        # STEP 1: Wake up the stream service by hitting the root URL
        self.trigger_stream_if_needed(full_ip)
        
        # STEP 2: Try the known stream endpoints
        stream_reader = None
        found_endpoint = False
        
        for endpoint in STREAM_ENDPOINTS:
            url = f"http://{full_ip}{endpoint}"
            self.display_message(f"Attempting to open stream: {url}")
            
            # Initialize the custom MJPEG reader
            reader = MJPEGFrameReader(url)
            # 5-second timeout is set in MJPEGFrameReader.open()
            if reader.open():
                stream_reader = reader
                found_endpoint = True
                self.display_message(f"SUCCESS: Stream opened on {endpoint}.")
                break
            else:
                self.display_message(f"Failed on {endpoint}. Trying next...")
        
        if found_endpoint:
            self.cap = stream_reader
            self.cam_label.config(text=f"Current Cam: {full_ip} (Status: Connected)", foreground='green')
            # Check for existing calibration file
            self.load_calibration_file(full_ip)
        else:
            self.cam_label.config(text=f"Current Cam: {full_ip} (Status: Disconnected/Failed)", foreground='red')
            self.display_message(f"FAILURE: Could not connect to {full_ip} on any endpoint.")
            self.last_frame = np.zeros((480, 640, 3), dtype=np.uint8) # Display black frame
            self.root.after(0, self.update_display) # Update to show black screen

    def release_stream(self):
        """Closes the current stream connection."""
        if self.cap:
            self.cap.release()
            self.cap = None

    def next_cam(self, event=None):
        """Moves to the next camera IP."""
        if self.current_ip_end < IP_END_RANGE:
            self.current_ip_end += 1
            self.reset_for_new_cam()

    def prev_cam(self, event=None):
        """Moves to the previous camera IP."""
        if self.current_ip_end > IP_START_RANGE:
            self.current_ip_end -= 1
            self.reset_for_new_cam()

    def reset_for_new_cam(self):
        """Resets calibration data and loads the new stream."""
        self.release_stream()
        self.obj_points = []
        self.img_points = []
        self.camera_matrix = None
        self.dist_coeffs = None
        self.new_camera_matrix = None # Reset new matrix calculation
        self.undistort_var.set(False)
        self.capture_count_var.set("0")
        self.display_message(f"\n--- Switching to new camera: {IP_BASE}.{self.current_ip_end} ---")
        self.load_stream()

    def load_calibration_file(self, ip):
        """Loads calibration data if a .pkl file exists for the current IP."""
        filename = f"cam_calibration_{ip.split('.')[-1]}.pkl"
        try:
            with open(filename, 'rb') as f:
                data = pickle.load(f)
                self.camera_matrix = data['camera_matrix']
                self.dist_coeffs = data['dist_coeffs']
                self.display_message(f"SUCCESS: Loaded existing calibration file: {filename}")
                self.display_message("Calibration ready. Press [d] to toggle undistortion.")
        except FileNotFoundError:
            self.display_message(f"No existing calibration found for this camera. Start capturing frames.")
        except Exception as e:
            self.display_message(f"ERROR loading calibration file {filename}: {e}")

    def capture_frame(self, event=None):
        """Captures a frame and attempts to find the checkerboard."""
        if self.last_frame is None or self.cap is None or not self.cap.is_open:
            self.display_message("Error: No active camera stream to capture from.")
            return

        frame = self.last_frame.copy()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Find the checkerboard corners
        ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)

        if ret:
            # Refine the corner locations
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            
            # Draw and save (The draw command is kept to visually mark corners on the frame itself)
            cv2.drawChessboardCorners(frame, CHECKERBOARD, corners, ret)
            self.obj_points.append(self.objp)
            self.img_points.append(corners)
            
            count = len(self.img_points)
            self.capture_count_var.set(str(count))
            self.display_message(f"Captured frame {count}. Corners found successfully.")
            
            # --- LINES REMOVED: These lines previously caused the separate window to appear ---
            # cv2.imshow("Capture Check", frame)
            # cv2.waitKey(1)
            # ---------------------------------------------------------------------------------
            
        else:
            self.display_message("Failed to find checkerboard corners. Adjust position/lighting.")

    def calibrate_cam(self, event=None):
        """Performs camera calibration using captured points."""
        if len(self.img_points) < 5:
            self.display_message(f"Error: Need at least 5 good capture points. Currently have {len(self.img_points)}.")
            return

        h, w = self.last_frame.shape[:2] if self.last_frame is not None else (480, 640)
        
        self.display_message(f"Starting calibration with {len(self.img_points)} points...")
        try:
            ret, self.camera_matrix, self.dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
                self.obj_points, self.img_points, (w, h), None, None
            )
            
            if ret:
                mean_error = self.calculate_reprojection_error(rvecs, tvecs, self.obj_points, self.img_points, self.camera_matrix, self.dist_coeffs)
                self.display_message("Calibration successful!")
                self.display_message(f"Camera Matrix (K): \n{self.camera_matrix}")
                self.display_message(f"Reprojection Error: {mean_error:.4f} pixels (Lower is better)")
                self.display_message("Press [s] to save the calibration data.")
            else:
                self.display_message("Calibration failed. Try capturing more frames.")
        except Exception as e:
            self.display_message(f"An error occurred during calibration: {e}")

    def calculate_reprojection_error(self, rvecs, tvecs, obj_points, img_points, K, D):
        """Calculates the mean reprojection error."""
        total_error = 0
        for i in range(len(obj_points)):
            # Ensure the points are correctly shaped for projectPoints
            objp_reshaped = obj_points[i].reshape(-1, 1, 3) 
            imgpoints2, _ = cv2.projectPoints(objp_reshaped, rvecs[i], tvecs[i], K, D)
            error = cv2.norm(img_points[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
            total_error += error
        return total_error / len(obj_points)

    def save_calibration(self, event=None):
        """Saves the camera matrix and distortion coefficients to a pickle file."""
        if self.camera_matrix is None or self.dist_coeffs is None:
            self.display_message("Error: Calibration must be performed successfully before saving.")
            return

        filename = f"cam_calibration_{self.current_ip_end}.pkl"
        data = {
            'camera_matrix': self.camera_matrix,
            'dist_coeffs': self.dist_coeffs
        }
        
        try:
            with open(filename, 'wb') as f:
                pickle.dump(data, f)
            self.display_message(f"SUCCESS: Calibration data saved to {filename}")
        except Exception as e:
            self.display_message(f"ERROR saving calibration file: {e}")
            
    def undistort_toggle(self, event=None):
        """Toggles the undistorted view."""
        if self.camera_matrix is None or self.dist_coeffs is None:
            self.display_message("Undistort failed: Calibration data not available for this camera.")
            self.undistort_var.set(False)
        else:
            state = "ON" if self.undistort_var.get() else "OFF"
            self.display_message(f"Undistorted view is now {state}.")


    def on_closing(self):
        """Ensures the camera is released and all threads/windows are closed."""
        self.is_running = False # Signal the thread to stop
        
        # Wait for the thread to finish (optional, but cleaner)
        if self.update_thread.is_alive():
            self.update_thread.join(timeout=1) 
        
        self.release_stream() # Closes the urllib stream
        
        cv2.destroyAllWindows() # Close any OpenCV windows (like 'Capture Check')
        self.root.destroy()

# -------------------------------------------
# --- Clean Exit Handler ---
# -------------------------------------------

def on_closing_wrapper(app_instance):
    """Initial wrapper for clean exit."""
    app_instance.on_closing()

if __name__ == "__main__":
    # --- IMPORTANT CHECKERBOARD INSTRUCTIONS ---
    print(f"Calibration starting. Please ensure your physical checkerboard has:")
    print(f"  - Inner Corners: {CHECKERBOARD[1]} x {CHECKERBOARD[0]} (e.g., a 7x10 board has 6x9 inner corners)")
    print(f"  - Square Size: {SQUARE_SIZE_MM} mm")
    print("These parameters must be accurate for correct pose estimation!")
    # -------------------------------------------
    
    # Initialize Tkinter
    root = tk.Tk()
    app = CameraCalibrationApp(root)
    
    # Use a lambda function to pass the app instance to the closing protocol
    root.protocol("WM_DELETE_WINDOW", lambda: on_closing_wrapper(app))
    
    # Start the Tkinter main loop
    root.mainloop()

