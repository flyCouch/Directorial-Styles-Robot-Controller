import tkinter as tk
from tkinter import ttk
import cv2
import numpy as np
import pickle
import time
import urllib.request
import re # Added for robust string parsing/incrementing

# --- Calibration Parameters ---
# IMPORTANT: Adjust these to match your checkerboard
CHECKERBOARD = (5, 7)  # Inner corners: (columns - 1, rows - 1). Standard 7x10 board is (6, 9).
SQUARE_SIZE_MM = 25.0  # Size of a single square on the checkerboard in millimeters

# --- Configuration for Cycling ---
IP_START_RANGE = 201
IP_END_RANGE = 209

class CameraCalibrationApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Multi-Camera Calibration Utility")

        # Calibration storage
        self.obj_points = []  # 3D points in real world space
        self.img_points = []  # 2D points in image plane
        self.camera_matrix = None
        self.dist_coeffs = None
        self.new_camera_matrix = None
        self.cap = None

        # Prepare 3D object points (0,0,0), (1,0,0), (2,0,0) ...
        self.objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
        self.objp[:, :2] = np.mgrid[0:CHECKERBOARD[1], 0:CHECKERBOARD[0]].T.reshape(-1, 2) * SQUARE_SIZE_MM

        # --- GUI Setup ---
        frame = ttk.Frame(root, padding="10")
        frame.pack(fill='both', expand=True)

        # 1. Camera Source Input
        ttk.Label(frame, text="Camera Source (IP or Index):").grid(row=0, column=0, sticky="w", pady=2)
        self.source_var = tk.StringVar(value=f"192.168.1.{IP_START_RANGE}") # Default IP example
        self.source_entry = ttk.Entry(frame, textvariable=self.source_var, width=30)
        self.source_entry.grid(row=0, column=1, sticky="we", pady=2)
        
        self.connect_button = ttk.Button(frame, text="Connect & Start Live Stream", command=self.connect_camera)
        self.connect_button.grid(row=1, column=0, columnspan=2, pady=5, sticky="we")

        # 2. Capture and Calibration Controls
        self.instruction_label = ttk.Label(frame, text="Status: Not Connected")
        self.instruction_label.grid(row=2, column=0, columnspan=2, sticky="w", pady=5)

        self.capture_button = ttk.Button(frame, text="Capture Image (0/20)", command=self.capture_image)
        self.capture_button.grid(row=3, column=0, sticky="we", padx=2)
        self.capture_button.config(state="disabled")

        self.calibrate_button = ttk.Button(frame, text="Calibrate Camera", command=self.calibrate)
        self.calibrate_button.grid(row=3, column=1, sticky="we", padx=2)
        self.calibrate_button.config(state="disabled")

        self.reset_button = ttk.Button(frame, text="Reset Capture Data", command=self.reset_data)
        self.reset_button.grid(row=4, column=0, sticky="we", padx=2, pady=5)
        
        self.cycle_button = ttk.Button(frame, text=f"Cycle to Next Cam ({IP_START_RANGE}-{IP_END_RANGE})", command=self.cycle_to_next_cam)
        self.cycle_button.grid(row=4, column=1, sticky="we", padx=2, pady=5)

        # 3. Save Output
        ttk.Label(frame, text="Output Filename (.pkl):").grid(row=5, column=0, sticky="w", pady=2)
        self.filename_var = tk.StringVar(value=f"calib_{IP_START_RANGE}.pkl") # Default filename example
        self.filename_entry = ttk.Entry(frame, textvariable=self.filename_var, width=30)
        self.filename_entry.grid(row=5, column=1, sticky="we", pady=2)

        self.save_button = ttk.Button(frame, text="Save Calibration Data", command=self.save_calibration)
        self.save_button.grid(row=6, column=0, columnspan=2, pady=5, sticky="we")
        self.save_button.config(state="disabled")

        # 4. Message Display
        self.message_display = tk.Text(frame, height=10, state='normal', wrap='word')
        self.message_display.grid(row=7, column=0, columnspan=2, sticky="nsew", pady=5)
        frame.grid_rowconfigure(7, weight=1)
        frame.grid_columnconfigure(1, weight=1)

    def connect_camera(self):
        """Initializes video capture from index or IP address."""
        # Ensure any existing camera connection is released
        if self.cap and self.cap.isOpened():
            self.cap.release()
            cv2.destroyAllWindows()
            self.cap = None

        source = self.source_var.get().strip()
        
        try:
            # Check if it's a numeric index or an IP
            if source.isdigit():
                self.display_message(f"Attempting to connect to local camera index {source}...")
                self.cap = cv2.VideoCapture(int(source))
            else:
                # Use the raw source string if it already includes "http://" or similar, otherwise prepend "http://"
                stream_url = source if source.startswith('http') else f"http://{source}/stream"
                self.display_message(f"Attempting to connect to IP stream: {stream_url}")
                
                try:
                    # Quick URL check to provide better feedback
                    urllib.request.urlopen(stream_url, timeout=5)
                except Exception:
                    self.display_message("ERROR: Stream URL seems unreachable or timed out. Check IP/connection.")
                    return
                    
                self.cap = cv2.VideoCapture(stream_url)
                
            time.sleep(1) # Give camera time to warm up
                
            if self.cap and self.cap.isOpened():
                # Set resolution properties (might not work for IP cameras, but good for USB)
                self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                self.display_message("Connection successful. Starting live preview...")
                self.instruction_label.config(text="Status: Connected")
                self.capture_button.config(state="normal")
                self.live_preview()
            else:
                self.display_message("ERROR: Could not open video stream. Check source or IP/URL format.")
                self.instruction_label.config(text="Status: Connection Failed")
                self.capture_button.config(state="disabled")
        except Exception as e:
            self.display_message(f"An unexpected error occurred during connection: {e}")
            self.instruction_label.config(text="Status: Connection Failed")
            self.capture_button.config(state="disabled")

    def live_preview(self):
        """Updates the live preview window."""
        if not self.cap or not self.cap.isOpened():
            return

        ret, frame = self.cap.read()
        if ret:
            # Undistort and display the frame if calibration is available
            if self.camera_matrix is not None and self.new_camera_matrix is not None:
                h, w = frame.shape[:2]
                mapx, mapy = cv2.initUndistortRectifyMap(self.camera_matrix, self.dist_coeffs, None, self.new_camera_matrix, (w, h), 5)
                frame = cv2.remap(frame, mapx, mapy, cv2.INTER_LINEAR)
                cv2.putText(frame, "UNDISTORTED VIEW", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            cv2.imshow('Live Stream (Press ESC to close)', frame)
            
            # Use ESC key to close the live window
            if cv2.waitKey(1) == 27: 
                cv2.destroyWindow('Live Stream (Press ESC to close)')
                return
        
        # Schedule the next update
        if cv2.getWindowProperty('Live Stream (Press ESC to close)', cv2.WND_PROP_VISIBLE) >= 1:
            self.root.after(30, self.live_preview)

    def capture_image(self):
        """Captures a frame and attempts to find the checkerboard pattern."""
        if not self.cap or not self.cap.isOpened():
            self.display_message("ERROR: Camera is not connected.")
            return

        ret, frame = self.cap.read()
        if not ret:
            self.display_message("ERROR: Could not read frame from stream.")
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Find the checkerboard corners
        ret_corners, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)

        if ret_corners:
            # Corner refinement
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

            self.img_points.append(corners2)
            self.obj_points.append(self.objp)

            # Draw and display the successful capture
            frame_success = cv2.drawChessboardCorners(frame.copy(), CHECKERBOARD, corners2, ret_corners)
            cv2.putText(frame_success, f"Captured: {len(self.img_points)}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.imshow('Capture Success', frame_success)
            cv2.waitKey(500)
            cv2.destroyWindow('Capture Success')
            
            self.display_message(f"Image captured successfully. Total: {len(self.img_points)}.")
            self.capture_button.config(text=f"Capture Image ({len(self.img_points)}/20)")
            
            if len(self.img_points) >= 10:
                self.calibrate_button.config(state="normal")

        else:
            self.display_message("Checkerboard pattern NOT found in the frame. Adjust position and lighting.")
            cv2.putText(frame, "NOT FOUND", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            cv2.imshow('Capture Failed', frame)
            cv2.waitKey(500)
            cv2.destroyWindow('Capture Failed')

    def calibrate(self):
        """Performs the camera calibration."""
        if len(self.img_points) < 10:
            self.display_message("ERROR: Need at least 10 successful captures to calibrate.")
            return
        
        self.display_message("Starting calibration process...")
        
        ret, frame = self.cap.read()
        if not ret:
             self.display_message("ERROR: Cannot read frame to get dimensions.")
             return
             
        h, w = frame.shape[:2] # Get image dimensions

        # The actual calibration function
        ret, self.camera_matrix, self.dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
            self.obj_points, self.img_points, (w, h), None, None
        )

        if ret:
            self.display_message("Calibration successful!")
            self.display_message(f"RMS Reprojection Error: {ret:.4f}")
            self.display_message(f"Camera Matrix:\n{self.camera_matrix}")
            self.display_message(f"Distortion Coefficients:\n{self.dist_coeffs}")
            
            # Calculate new camera matrix for undistortion
            self.new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(self.camera_matrix, self.dist_coeffs, (w, h), 1, (w, h))
            self.save_button.config(state="normal")
            
        else:
            self.display_message("Calibration FAILED. Try capturing more images or check your checkerboard size.")

    def save_calibration(self):
        """Saves the camera matrix and distortion coefficients to a PKL file."""
        if self.camera_matrix is None:
            self.display_message("ERROR: Please perform calibration first.")
            return

        filename = self.filename_var.get().strip()
        if not filename.lower().endswith('.pkl'):
            filename += '.pkl'
            
        calibration_data = {
            'camera_matrix': self.camera_matrix,
            'dist_coeffs': self.dist_coeffs,
            'new_camera_matrix': self.new_camera_matrix, # Saving for convenience
            'roi': None # Not strictly necessary, but can be added if needed
        }

        try:
            with open(filename, 'wb') as f:
                pickle.dump(calibration_data, f)
            self.display_message(f"SUCCESS: Calibration data saved to '{filename}'.")
            self.display_message(f"**Ready to calibrate the next camera. Click 'Cycle to Next Camera'.**")
        except Exception as e:
            self.display_message(f"ERROR: Could not save file. {e}")

    def reset_data(self):
        """Clears all captured data and disconnects the camera."""
        self.obj_points = []
        self.img_points = []
        self.camera_matrix = None
        self.dist_coeffs = None
        self.new_camera_matrix = None
        self.capture_button.config(text="Capture Image (0/20)", state="disabled")
        self.calibrate_button.config(state="disabled")
        self.save_button.config(state="disabled")

        if self.cap and self.cap.isOpened():
            self.cap.release()
            cv2.destroyAllWindows()
            self.cap = None
            self.instruction_label.config(text="Status: Disconnected")
            self.display_message("Camera disconnected and data cleared.")
        else:
            self.instruction_label.config(text="Status: Not Connected")
            self.display_message("All captured data cleared.")

    def cycle_to_next_cam(self):
        """Resets data and increments the IP and filename for the next camera."""
        current_source = self.source_var.get().strip()
        current_filename = self.filename_var.get().strip()
        
        # 1. Reset everything first
        self.reset_data() 
        
        # 2. Try to increment the IP address
        new_source = self._increment_ip_part(current_source)
        self.source_var.set(new_source)
        
        # 3. Try to increment the filename
        # We pass the number part of the new IP/source to ensure sync if the filename structure changes
        if new_source.isdigit():
             new_number = int(new_source)
        else:
            match = re.search(r'(\d+)$', new_source)
            new_number = int(match.group(1)) if match else IP_START_RANGE
        
        new_filename = self._set_filename_number(current_filename, new_number)
        self.filename_var.set(new_filename)
        
        self.display_message(f"\n--- Cycling to Next Camera ---")
        self.display_message(f"New Source: {new_source}")
        self.display_message(f"New Filename: {new_filename}")
        self.display_message(f"Click 'Connect & Start Live Stream' to begin calibration for this camera.")
        
    def _increment_ip_part(self, source_str):
        """Increments the number part of the IP address or index, cycling from 209 to 201."""
        if source_str.isdigit():
            # Handle local index cycling (if needed, but usually not capped)
            number = int(source_str)
            # We don't cap index for versatility, but you can add logic if you know the max index
            return str(number + 1)
        
        # Handle IP address cycling
        match = re.match(r'(.*)\.(\d+)$', source_str)
        if match:
            prefix = match.group(1)
            number = int(match.group(2))
            
            new_number = number + 1
            if new_number > IP_END_RANGE:
                new_number = IP_START_RANGE
                
            return f"{prefix}.{new_number}"
            
        return source_str # Return unchanged if it doesn't look like an IP

    def _set_filename_number(self, filename, new_number):
        """Sets the number in a filename (e.g., calib_XXX.pkl) to the new_number."""
        match = re.match(r'(.*?)(\d+)(\.pkl)$', filename, re.IGNORECASE)
        if match:
            prefix = match.group(1)
            suffix = match.group(3)
            return f"{prefix}{new_number}{suffix}"
            
        # If the filename format is not 'prefix_number.pkl', just try to append the number
        return f"{filename.replace('.pkl', '')}_{new_number}.pkl"

    def display_message(self, message):
        """Appends a message to the text area and scrolls to the bottom."""
        self.message_display.insert(tk.END, message + "\n")
        self.message_display.see(tk.END)

    # NOTE: The redundant and unreliable __del__ method was REMOVED from the class.

# -------------------------------------------
# --- Clean Exit Handler ---
# -------------------------------------------

def on_closing():
    """Ensures the camera is released and OpenCV windows are closed before Tkinter exits."""
    # Check if the camera object exists and is open
    if app.cap and app.cap.isOpened(): 
        app.cap.release()
    
    # Close any OpenCV windows (like the 'Live Stream' or 'Capture Success')
    cv2.destroyAllWindows() 
    
    # Close the main Tkinter window
    root.destroy()

if __name__ == "__main__":
    # --- IMPORTANT CHECKERBOARD INSTRUCTIONS ---
    print(f"Calibration starting. Please ensure your physical checkerboard has:")
    print(f"  - Inner Corners: {CHECKERBOARD[1]} x {CHECKERBOARD[0]} (e.g., a 7x10 board has 6x9 inner corners)")
    print(f"  - Square Size: {SQUARE_SIZE_MM} mm")
    print("These parameters must be accurate for correct pose estimation!")
    # -------------------------------------------
    
    root = tk.Tk()
    app = CameraCalibrationApp(root)
    
    # FIX: Register the custom on_closing function
    root.protocol("WM_DELETE_WINDOW", on_closing) 
    
    root.mainloop()
