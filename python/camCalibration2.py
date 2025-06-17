import tkinter as tk
from tkinter import ttk
import cv2
import numpy as np
import pickle  # For saving calibration data

class CameraCalibrationApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Camera Calibration")

        # Initialize camera
        self.cap = cv2.VideoCapture(0)  # Or specify camera index (e.g., 1, 2, etc.)
        if not self.cap.isOpened():
            self.display_message("Error: Could not open video stream. Check camera index.")
            # Optionally disable capture button if camera fails to open
            self.capture_button.config(state="disabled")

        # GUI elements
        self.instruction_label = ttk.Label(root, text="Place checkerboard in view and click 'Capture'")
        self.instruction_label.pack(pady=5)

        self.capture_button = ttk.Button(root, text="Capture Image", command=self.capture_image)
        self.capture_button.pack(pady=2)

        self.calibrate_button = ttk.Button(root, text="Calibrate Camera", command=self.calibrate)
        self.calibrate_button.pack(pady=2)
        self.calibrate_button.config(state="disabled")  # Disable until enough images

        self.progress_bar = ttk.Progressbar(root, mode="determinate")
        self.progress_bar.pack(pady=5, fill=tk.X, padx=10)

        self.message_display = tk.Text(root, height=4, width=60)
        self.message_display.pack(pady=5)

        self.save_button = ttk.Button(root, text="Save Calibration", command=self.save_calibration)
        self.save_button.pack(pady=2)
        self.save_button.config(state="disabled")

        self.load_button = ttk.Button(root, text="Load Calibration", command=self.load_calibration)
        self.load_button.pack(pady=2)
        
        self.canvas = tk.Canvas(root, width=640, height=480, bg="black")  # For displaying camera feed
        self.canvas.pack(pady=10)
        self.photo = None # Initialize photo to None

        self.update_camera_feed() # Start updating the camera feed

        # Calibration data storage
        self.images = []  # List to store captured images (frames with detected corners)
        self.objpoints = []  # 3D points of checkerboard corners in object coordinate system
        self.imgpoints = []  # 2D points of checkerboard corners in image plane
        self.calibration_data = {} # Dictionary to store calibration results (camera_matrix, distortion_coeffs)

        # Checkerboard parameters
        self.CHECKERBOARD_SIZE = (4, 6)  # Example: (rows, columns) of inner corners (e.g., 8x11 board has 7x10 inner corners)
        # You MUST define the actual size of one square on your checkerboard in a known unit (e.g., mm, inches)
        # This is critical for accurate 3D reconstruction.
        self.SQUARE_SIZE_MM = 25.0  # <--- IMPORTANT: MEASURE YOUR ACTUAL CHECKERBOARD SQUARE SIZE
        self.NUM_IMAGES_NEEDED = 5 # Minimum number of images required for calibration

    def update_camera_feed(self):
        ret, frame = self.cap.read()
        if ret:
            # Convert OpenCV BGR image to RGB for Tkinter
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            # Resize frame to fit canvas if necessary (optional, but good practice)
            # frame_resized = cv2.resize(frame_rgb, (self.canvas.winfo_width(), self.canvas.winfo_height()))
            
            # Convert image to PhotoImage format for Tkinter display
            img_bytes = cv2.imencode('.png', frame_rgb)[1].tobytes()
            self.photo = tk.PhotoImage(data=img_bytes)
            self.canvas.create_image(0, 0, image=self.photo, anchor=tk.NW)
        
        # Schedule the next update
        self.root.after(10, self.update_camera_feed)  # Update every 10 milliseconds (approx 100 FPS)

    def capture_image(self):
        ret, frame = self.cap.read()
        if not ret:
            self.display_message("Error capturing image. Camera might be disconnected or busy.")
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Find the chess board corners
        ret, corners = cv2.findChessboardCorners(gray, self.CHECKERBOARD_SIZE, None)

        # If found, add object points, image points (after refining them)
        if ret:
            # Refine corner locations for subpixel accuracy
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners2 = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
            
            self.images.append(frame) # Store the original colored frame
            
            # --- CORRECTED INDENTATION: This block now correctly belongs inside 'if ret:' ---
            # Prepare object points (3D coordinates of checkerboard corners)
            # Assuming checkerboard is on Z=0 plane
            objp = np.zeros((self.CHECKERBOARD_SIZE[0] * self.CHECKERBOARD_SIZE[1], 3), np.float32)
            objp[:, :2] = np.mgrid[0:self.CHECKERBOARD_SIZE[0], 0:self.CHECKERBOARD_SIZE[1]].T.reshape(-1, 2)
            objp[:, :2] *= self.SQUARE_SIZE_MM # Scale by actual square size
            self.objpoints.append(objp)

            # Store image points (2D coordinates of checkerboard corners)
            self.imgpoints.append(corners2) # Use refined corners
            # --- END CORRECTED INDENTATION ---

            self.display_message(f"Checkerboard detected. Image {len(self.images)} captured.")
            
            # Update progress bar
            self.progress_bar["maximum"] = self.NUM_IMAGES_NEEDED
            self.progress_bar["value"] = len(self.images)

            if len(self.images) >= self.NUM_IMAGES_NEEDED:
                self.calibrate_button.config(state="normal")  # Enable calibration
                self.display_message(f"Captured {len(self.images)} images. Click 'Calibrate Camera'")
            
            # Optionally draw and show corners for user feedback
            # frame_with_corners = cv2.drawChessboardCorners(frame.copy(), self.CHECKERBOARD_SIZE, corners2, ret)
            # cv2.imshow('Image Captured - Corners Detected', frame_with_corners)
            # cv2.waitKey(500) # Show for 0.5 seconds
            # cv2.destroyWindow('Image Captured - Corners Detected')
        else:
            self.display_message("Checkerboard not found. Try again.")
            self.progress_bar["value"] = len(self.images) # Still update progress if needed

    def calibrate(self):
        if len(self.images) < self.NUM_IMAGES_NEEDED:
            self.display_message(f"Need {self.NUM_IMAGES_NEEDED - len(self.images)} more images to calibrate.")
            return

        self.display_message("Calibrating camera... This may take a moment.")
        try:
            # Perform camera calibration
            ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
                self.objpoints, self.imgpoints, 
                (self.images[0].shape[1], self.images[0].shape[0]), # Image size (width, height)
                None, None # Optional initial camera matrix and distortion coeffs
            )
            
            if ret:
                self.calibration_data["camera_matrix"] = mtx
                self.calibration_data["dist_coeffs"] = dist
                
                self.display_message("Camera calibrated successfully!")
                self.display_message(f"Camera Matrix:\n{mtx}")
                self.display_message(f"Distortion Coefficients:\n{dist}")
                
                self.save_button.config(state="normal") # Enable save button
            else:
                self.display_message("Calibration failed. Try capturing more diverse images.")
        except Exception as e:
            self.display_message(f"Error during calibration: {e}")

    def save_calibration(self):
        if not self.calibration_data:
            self.display_message("No calibration data to save.")
            return
        try:
            filename = "camera_calibration_data.pkl"
            with open(filename, "wb") as f:
                pickle.dump(self.calibration_data, f)
            self.display_message(f"Calibration data saved to {filename}")
        except Exception as e:
            self.display_message(f"Error saving calibration data: {e}")

    def load_calibration(self):
        try:
            filename = "camera_calibration_data.pkl"
            with open(filename, "rb") as f:
                self.calibration_data = pickle.load(f)
            
            if "camera_matrix" in self.calibration_data and "distortion_coeffs" in self.calibration_data:
                self.display_message(f"Calibration data loaded from {filename}.")
                self.display_message(f"Loaded Camera Matrix:\n{self.calibration_data['camera_matrix']}")
                self.display_message(f"Loaded Distortion Coefficients:\n{self.calibration_data['distortion_coeffs']}")
                # If you want to use the loaded data immediately, you can enable other buttons here.
                # For example, if you had an "Undistort Image" button, you could enable it now.
            else:
                self.display_message("Loaded file does not contain valid calibration data.")
                self.calibration_data = {} # Clear invalid data
        except FileNotFoundError:
            self.display_message(f"Error: Calibration file '{filename}' not found.")
        except Exception as e:
            self.display_message(f"Error loading calibration data: {e}")
            
    def display_message(self, message):
        self.message_display.insert(tk.END, message + "\n")
        self.message_display.see(tk.END)  # Scroll to the bottom automatically

    def __del__(self):
        # Release the camera when the application closes
        if self.cap.isOpened():
            self.cap.release()

if __name__ == "__main__":
    root = tk.Tk()
    app = CameraCalibrationApp(root)
    root.mainloop()


