import cv2
import apriltag
import time
import numpy as np
import pickle
import socket
import math # Import the math module for trigonometric functions

# --- Configuration ---
CAMERA_INDEX = 0
APRILTAG_FAMILY = 'tag36h11'
CALIBRATION_FILE = 'camera_calibration_data.pkl'
TAG_SIZE_MM = 159 # !!! IMPORTANT: Replace with the actual physical size of your AprilTag in millimeters !!!

# NEW: Offsets from AprilTag's center to the robot's physical center
# These values should be determined by measuring the placement of the AprilTag on your robot.
# For example, if the robot's center is 50mm forward (along the tag's local Y-axis)
# and 10mm to the right (along the tag's local X-axis) from the AprilTag's center.
# The yaw offset is how much the robot's 'forward' direction differs from the AprilTag's 'forward'.
TAG_TO_ROBOT_CENTER_X_MM = 0.0  # Offset in mm along the AprilTag's local X-axis to the robot's center
TAG_TO_ROBOT_CENTER_Y_MM = 0.0  # Offset in mm along the AprilTag's local Y-axis to the robot's center
TAG_TO_ROBOT_CENTER_YAW_DEG = 0.0 # Offset in degrees from AprilTag's yaw to the robot's actual yaw

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
    options = apriltag.DetectorOptions(
        families=APRILTAG_FAMILY,
        nthreads=4,
        quad_decimate=1.0,
        refine_edges=1,
        debug=0
    )

    # 3. Assign the options to the detector
    tag_detector.options = options

    print("AprilTag detector initialized.")
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
    Applies configurable offsets from AprilTag center to robot's center.
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
                (0, 0, 255), 2)

    if not results:
        cv2.imshow("AprilTag Detection", frame)
        return

    half_size = TAG_SIZE_MM / 2.0
    object_points = np.array([
        [-half_size, half_size, 0],
        [half_size, half_size, 0],
        [half_size, -half_size, 0],
        [-half_size, -half_size, 0]
    ], dtype=np.float32)

    for r in results:
        tag_id = r.tag_id

        if camera_matrix is not None and dist_coeffs is not None:
            image_points = r.corners.astype(np.float32)

            success, rvec, tvec = cv2.solvePnP(object_points, image_points, camera_matrix, dist_coeffs, flags=cv2.SOLVEPNP_IPPE)

            if success:
                # --- Get Tag's Pose (raw from solvePnP) ---
                tag_x_mm = tvec[0][0]
                tag_y_mm = tvec[1][0]
                tag_z_mm = tvec[2][0]

                rotation_matrix, _ = cv2.Rodrigues(rvec)
                sy = np.sqrt(rotation_matrix[0,0] * rotation_matrix[0,0] +  rotation_matrix[1,0] * rotation_matrix[1,0])

                if sy < 1e-6: # Check for singular
                    tag_yaw_deg = np.degrees(np.arctan2(-rotation_matrix[1,2], rotation_matrix[1,1]))
                else:
                    tag_yaw_deg = np.degrees(np.arctan2(rotation_matrix[1,0], rotation_matrix[0,0]))

                # Normalize tag yaw to be between 0 and 360
                tag_yaw_deg = (tag_yaw_deg + 360) % 360

                # --- Apply Offsets to get Robot's Pose ---
                # Convert tag yaw to radians for rotation calculation
                tag_yaw_rad = math.radians(tag_yaw_deg)
                
                # Rotate the tag-to-robot offset vector by the tag's yaw
                # This transforms the offset from the tag's local frame to the camera's frame
                rotated_offset_x = TAG_TO_ROBOT_CENTER_X_MM * math.cos(tag_yaw_rad) - TAG_TO_ROBOT_CENTER_Y_MM * math.sin(tag_yaw_rad)
                rotated_offset_y = TAG_TO_ROBOT_CENTER_X_MM * math.sin(tag_yaw_rad) + TAG_TO_ROBOT_CENTER_Y_MM * math.cos(tag_yaw_rad)

                # Add the rotated offset to the tag's position to get the robot's position
                robot_x_mm = tag_x_mm + rotated_offset_x
                robot_y_mm = tag_y_mm + rotated_offset_y

                # Add the yaw offset directly
                robot_yaw_deg = (tag_yaw_deg + TAG_TO_ROBOT_CENTER_YAW_DEG) % 360

                text = f"ID:{tag_id} RX:{robot_x_mm:.2f} RY:{robot_y_mm:.2f} RZ:{tag_z_mm:.2f} RYaw:{robot_yaw_deg:.2f} deg"
                print(text)

                global server_socket, client_connection

                if client_connection:
                    try:
                        # Send the robot's estimated pose
                        data_to_send = f"{tag_id},{robot_x_mm:.2f},{robot_y_mm:.2f},{tag_z_mm:.2f},{robot_yaw_deg:.2f}\n"
                        client_connection.sendall(data_to_send.encode('utf-8'))
                    except Exception as e:
                        print(f"Error sending data: {e}")
                        if server_socket:
                            server_socket.close()
                        server_socket = None
                        client_connection = None
                        print("Attempting to re-establish socket connection...")

            else:
                robot_x_mm, robot_y_mm, tag_z_mm, robot_yaw_deg = 0, 0, 0, 0
                text = f"ID:{tag_id} Pose estimation failed."
                print(text)
        else:
            # Fallback to pixel coordinates if calibration data is missing (not ideal for robot control)
            translation = r.center - np.array([frame.shape[1] / 2, frame.shape[0] / 2])
            robot_x_mm = translation[0] # These are still pixel values, not real distances
            robot_y_mm = translation[1]

            corner0 = r.corners[0].ravel()
            corner1 = r.corners[1].ravel()
            angle_rad = np.arctan2(corner1[1] - corner0[1], corner1[0] - corner0[0])
            robot_yaw_deg = np.degrees(angle_rad)
            robot_yaw_deg = (robot_yaw_deg + 360) % 360

            text = f"ID:{tag_id} RX_px:{robot_x_mm:.2f} RY_px:{robot_y_mm:.2f} RYaw_approx:{robot_yaw_deg:.2f} deg (No calibration)"
            print(text)

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
        # Uncomment the line below to enable the socket server for sending data
        # setup_socket_server()

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
