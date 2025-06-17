import cv2
import apriltag
import time
import numpy as np
import pickle # Import the pickle library
import socket # Import socket for communication

# --- Configuration ---
CAMERA_INDEX = 0
APRILTAG_FAMILY = 'tag36h11'
CALIBRATION_FILE = 'camera_calibration_data.pkl' # Path to your calibration file
TAG_SIZE_MM = 159 # !!! IMPORTANT: Replace with the actual physical size of your AprilTag in millimeters !!!

# --- Global Variables ---
tag_detector = None
camera_capture = None
camera_matrix = None
dist_coeffs = None
server_socket = None
client_connection = None

def initialize_camera():
    """Initializes the camera capture."""
    global camera_capture
    camera_capture = cv2.VideoCapture(CAMERA_INDEX)
    if not camera_capture.isOpened():
        raise IOError("Could not open camera.")
    return camera_capture

def initialize_apriltag_detector():
    """Initializes the AprilTag detector."""
    global tag_detector

    # 1. Initialize the Detector without arguments
    tag_detector = apriltag.Detector()

    # 2. Create an options object and set parameters
    # The apriltag library typically expects an apriltag.DetectorOptions object
    options = apriltag.DetectorOptions(
        families=APRILTAG_FAMILY,  # Pass your family string here
        nthreads=4,
        quad_decimate=1.0,
        #quad_sigma=0.0,
        refine_edges=1,
        #decode_sharpening=0.25,
        debug=0
    )

    # 3. Assign the options to the detector
    tag_detector.options = options # This is the crucial step

    print("AprilTag detector initialized.")
    return tag_detector
    return tag_detector

def load_camera_calibration(filepath):
    """Loads camera calibration data from a pickle file."""
    global camera_matrix, dist_coeffs
    try:
        with open(filepath, 'rb') as f:
            calibration_data = pickle.load(f)
            camera_matrix = calibration_data['camera_matrix']
            dist_coeffs = calibration_data['dist_coeffs']
        print("Camera calibration data loaded successfully.")
    except FileNotFoundError:
        print(f"Error: Calibration file not found at {filepath}")
        exit()
    except KeyError:
        print("Error: Calibration file does not contain 'camera_matrix' or 'dist_coeffs'.")
        exit()
    except Exception as e:
        print(f"Error loading calibration data: {e}")
        exit()

def setup_socket_server(host='127.0.0.1', port=65432):
    """Sets up a TCP socket server to send AprilTag data."""
    global server_socket, client_connection
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        server_socket.bind((host, port))
        server_socket.listen(1)
        print(f"Listening for connections on {host}:{port}")
        # Accept a connection (this will block until a client connects)
        client_connection, client_address = server_socket.accept()
        print(f"Accepted connection from {client_address}")
    except Exception as e:
        print(f"Error setting up socket server: {e}")
        server_socket = None
        client_connection = None


def process_apriltag_data(frame):
    """
    Detects AprilTags in the given frame, performs pose estimation,
    displays the camera feed, and sends data over socket.
    """
    if tag_detector is None:
        raise ValueError("AprilTag detector not initialized.")
    if camera_matrix is None or dist_coeffs is None:
        print("Warning: Camera calibration data not loaded. Distances will not be accurate.")

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    results = tag_detector.detect(gray)

    # Add the 'Press Q to quit' reminder
    cv2.putText(frame, "Press 'Q' to quit", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                (0, 0, 255), 2) # Red color for visibility

    if not results:
        # print("No AprilTags detected in this frame.")
        cv2.imshow("AprilTag Detection", frame)
        return

    # Define the 3D corners of the AprilTag in its own coordinate system (origin at center)
    # The order of corners is top-left, top-right, bottom-right, bottom-left
    half_size = TAG_SIZE_MM / 2.0
    object_points = np.array([
        [-half_size, half_size, 0],
        [half_size, half_size, 0],
        [half_size, -half_size, 0],
        [-half_size, -half_size, 0]
    ], dtype=np.float32)

    for r in results:
        tag_id = r.tag_id

        # Undistort corners if distortion coefficients are available
        # Although apriltag library handles some of this internally if camera params are passed,
        # it's good practice to ensure corners are undistorted for solvePnP if not doing it via apriltag.Detector
        # For simplicity, we assume the detector handles basic undistortion based on parameters
        # or that distortion is minimal for direct solvePnP on detected corners.
        # If camera_matrix and dist_coeffs are robust, cv2.undistortPoints could be used on r.corners first.

        # Use solvePnP to get rotation and translation vectors
        # If camera_matrix and dist_coeffs are valid, use them for accurate pose
        if camera_matrix is not None and dist_coeffs is not None:
            # AprilTag corners are typically in the order top-left, top-right, bottom-right, bottom-left
            # ensure image_points are float32 and correctly shaped for solvePnP
            image_points = r.corners.astype(np.float32)

            # Flags for solvePnP can be cv2.SOLVEPNP_ITERATIVE, cv2.SOLVEPNP_EPNP, cv2.SOLVEPNP_DLS, etc.
            # SOLVEPNP_IPPE is suitable for planar objects.
            success, rvec, tvec = cv2.solvePnP(object_points, image_points, camera_matrix, dist_coeffs, flags=cv2.SOLVEPNP_IPPE)

            if success:
                # tvec provides translation (X, Y, Z) in the same units as TAG_SIZE_MM
                x_mm = tvec[0][0]
                y_mm = tvec[1][0]
                z_mm = tvec[2][0] # Z is distance from camera along its optical axis

                # Calculate approximate yaw from rvec (rotation vector)
                # Convert rotation vector to rotation matrix
                rotation_matrix, _ = cv2.Rodrigues(rvec)
                # Extract Euler angles (e.g., Y-axis rotation for yaw)
                # This is a simplified approach; proper Euler angle extraction depends on convention
                # For yaw around Z-axis (camera looking down Z, X to right, Y down):
                # yaw = np.arctan2(rotation_matrix[1,0], rotation_matrix[0,0]) # if tag is aligned with XY plane
                # A common approach for tag yaw:
                # Yaw (rotation around Y-axis of the camera, looking at the tag)
                # Use a different row/column depending on your coordinate system and how you define yaw
                # For simple AprilTag in front of camera, Z is depth, X is right, Y is down
                # Yaw typically refers to rotation around the camera's Y-axis (up/down) or world Z-axis.
                # Here, we can derive it from the rotation of the tag's local axes relative to camera axes.
                # For a tag lying on the XY plane (z=0), its local X, Y, Z axes will be rotated.
                # We want the rotation around the camera's Z axis for planar rotation,
                # or rotation around the camera's Y axis for "yaw" (left/right turning).
                # A common method is to use the rotation matrix:
                sy = np.sqrt(rotation_matrix[0,0] * rotation_matrix[0,0] +  rotation_matrix[1,0] * rotation_matrix[1,0])
                singular = sy < 1e-6

                if not singular:
                    x_rot = np.arctan2(rotation_matrix[2,1], rotation_matrix[2,2]) # roll
                    y_rot = np.arctan2(-rotation_matrix[2,0], sy) # pitch
                    z_rot = np.arctan2(rotation_matrix[1,0], rotation_matrix[0,0]) # yaw
                else:
                    x_rot = np.arctan2(-rotation_matrix[1,2], rotation_matrix[1,1])
                    y_rot = np.arctan2(-rotation_matrix[2,0], sy)
                    z_rot = 0

                yaw_deg = np.degrees(z_rot) # Yaw in degrees (rotation around Z-axis of camera, if tag is in front)
                # Normalize yaw to be between 0 and 360
                yaw_deg = (yaw_deg + 360) % 360

                text = f"ID:{tag_id} X:{x_mm:.2f} Y:{y_mm:.2f} Z:{z_mm:.2f} Yaw:{yaw_deg:.2f} deg"
                print(text)

                global server_socket, client_connection

                # Send data over socket
                if client_connection:
                    try:
                        data_to_send = f"{tag_id},{x_mm:.2f},{y_mm:.2f},{z_mm:.2f},{yaw_deg:.2f}\n"
                        client_connection.sendall(data_to_send.encode('utf-8'))
                    except Exception as e:
                        print(f"Error sending data: {e}")
                        # Optionally try to re-establish connection if it broke
                        #global server_socket, client_connection
                        if server_socket:
                            server_socket.close()
                        server_socket = None
                        client_connection = None # Reset to None so it tries to reconnect next time
                        print("Attempting to re-establish socket connection...")
                        # In a real application, you might want to call setup_socket_server again here
                        # or have a more robust reconnection logic. For now, it will just stop sending.

            else:
                x_mm, y_mm, z_mm, yaw_deg = 0, 0, 0, 0
                text = f"ID:{tag_id} Pose estimation failed."
                print(text)
        else:
            # Fallback to pixel coordinates if calibration data is missing
            translation = r.center - np.array([frame.shape[1] / 2, frame.shape[0] / 2])
            x_mm = translation[0] # These are still pixel values, not real distances
            y_mm = translation[1]

            corner0 = r.corners[0].ravel()
            corner1 = r.corners[1].ravel()
            angle_rad = np.arctan2(corner1[1] - corner0[1], corner1[0] - corner0[0])
            yaw_deg = np.degrees(angle_rad)
            yaw_deg = (yaw_deg + 360) % 360

            text = f"ID:{tag_id} X_px:{x_mm:.2f} Y_px:{y_mm:.2f} Yaw_approx:{yaw_deg:.2f} deg (No calibration)"
            print(text)
            # No socket send for uncalibrated data, or send with a flag indicating it's uncalibrated

        try:
            corners = np.array(r.corners, dtype=np.int32).reshape((-1, 1, 2))
            cv2.polylines(frame, [corners], isClosed=True, color=(0, 255, 0), thickness=2)
        except Exception as e:
            print(f"Error drawing outline: {e}")
            print(f"Corners: {r.corners}")
            continue

        center_x, center_y = int(r.center[0]), int(r.center[1])
        cv2.putText(frame, text, (10, frame.shape[0] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                    (255, 255, 255), 2)


    cv2.imshow("AprilTag Detection", frame)


def main():
    """
    Main function to initialize components and run the AprilTag detection loop.
    """
    global camera_capture
    try:
        load_camera_calibration(CALIBRATION_FILE)
        camera_capture = initialize_camera()
        tag_detector = initialize_apriltag_detector()
        #setup_socket_server() # Setup the socket server

        while True:
            ret, frame = camera_capture.read()
            if not ret:
                print("Error: Could not read frame. Exiting.")
                break

            process_apriltag_data(frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            time.sleep(0.05) # Reduced sleep for smoother processing if possible

    except (IOError, ValueError) as e:
        print(f"Runtime Error: {e}")
    finally:
        if camera_capture:
            camera_capture.release()
            print("Camera released")
        if client_connection:
            client_connection.close()
            print("Client connection closed")
        if server_socket:
            server_socket.close()
            print("Server socket closed")
        cv2.destroyAllWindows()
        print("All windows destroyed")

if __name__ == "__main__":
    main()
