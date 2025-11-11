import cv2
import apriltag
import time
import numpy as np
import pickle
import socket
import math # Import the math module for trigonometric functions
import json # Import the json module for data serialization

# --- Configuration ---
CAMERA_INDEX = 0
APRILTAG_FAMILY = 'tag36h11'
CALIBRATION_FILE = 'camera_calibration_data.pkl'
TAG_SIZE_MM = 159 # !!! IMPORTANT: Replace with the actual physical size of your AprilTag in millimeters !!!

# NEW: Offsets from AprilTag's center to the robot's physical center
# These values should be determined by measuring the placement of the AprilTag on your robot.
TAG_TO_ROBOT_CENTER_X_MM = 0.0
TAG_TO_ROBOT_CENTER_Y_MM = 0.0
TAG_TO_ROBOT_CENTER_YAW_DEG = -204.0

# --- Global Variables ---
tag_detector = None
camera_capture = None
camera_matrix = None
dist_coeffs = None
server_socket = None
client_connection = None
# New global flag for program exit
quit_program = False 

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

def setup_socket_server(host='127.0.0.1', port=65000):
    """
    Sets up a TCP socket server for listening, but does NOT block for a connection.
    Returns True if setup was successful, False otherwise.
    """
    global server_socket, client_connection
    if server_socket:
        # Server socket already exists
        return True

    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    try:
        server_socket.bind((host, port))
        server_socket.listen(1)
        # Set a small timeout for non-blocking client accept attempts later in the loop
        server_socket.settimeout(0.1)
        print(f"Socket server listening on {host}:{port}. Waiting for a client...")
        return True
    except Exception as e:
        print(f"Error setting up socket server or binding to port: {e}. Socket functionality disabled.")
        if server_socket:
            server_socket.close()
        server_socket = None
        client_connection = None
        return False

def attempt_client_accept():
    """
    Attempts to accept a client connection without blocking the main loop.
    Must only be called if server_socket is initialized but client_connection is None.
    """
    global server_socket, client_connection

    if server_socket is None or client_connection is not None:
        return

    try:
        # Attempt to accept. Since we set a timeout in setup_socket_server,
        # this will only block for a short time (0.1s) and raise a timeout error if no client connects.
        client_connection, client_address = server_socket.accept()
        print(f"Accepted connection from {client_address}")
        # Reset timeout to None to allow the established connection to behave normally.
        server_socket.settimeout(None)
    except socket.timeout:
        # This is the expected case when no client is waiting to connect.
        pass
    except Exception as e:
        # Handle unexpected errors during accept (e.g., connection reset)
        print(f"Error during client accept check: {e}")
        if server_socket:
            server_socket.settimeout(None)


def send_json_data(data):
    """
    Serializes a dictionary to JSON and sends it over the client socket connection.
    Handles disconnections gracefully and prints a message.
    """
    global server_socket, client_connection

    if not client_connection:
        # Gracefully skip sending if no client is connected
        return

    try:
        # Serialize the dictionary to a JSON string and add a newline delimiter
        json_data = json.dumps(data) + '\n'
        client_connection.sendall(json_data.encode('utf-8'))
    except socket.error as e:
        print(f"Socket error during send: {e}. Disconnecting client.")
        # Clean up resources associated with the failed connection
        if client_connection:
            try:
                client_connection.close()
            except:
                pass
            client_connection = None

    except Exception as e:
        print(f"General error during JSON send: {e}")


def process_apriltag_data(frame):
    """
    Detects AprilTags in the given frame, performs pose estimation,
    displays the camera feed, and sends data over socket.
    """
    if tag_detector is None:
        raise ValueError("AprilTag detector not initialized.")

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    results = tag_detector.detect(gray)

    # Prepare a list to hold all detected tag data for JSON
    all_tag_data = []

    # Add the 'Press Q to quit' reminder
    cv2.putText(frame, "Press 'Q' to quit", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                (0, 0, 255), 2)

    if not results:
        cv2.imshow("AprilTag Detection", frame)
        # Still send a message if no tags are detected (only if connected)
        send_json_data({"timestamp": time.time(), "detections": []})
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

        current_tag_data = {
            "id": int(tag_id),
            "pose_valid": False,
            "x_mm": 0.0,
            "y_mm": 0.0,
            "z_mm": 0.0,
            "yaw_deg": 0.0
        }

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
                tag_yaw_rad = math.radians(tag_yaw_deg)
                rotated_offset_x = TAG_TO_ROBOT_CENTER_X_MM * math.cos(tag_yaw_rad) - TAG_TO_ROBOT_CENTER_Y_MM * math.sin(tag_yaw_rad)
                rotated_offset_y = TAG_TO_ROBOT_CENTER_X_MM * math.sin(tag_yaw_rad) + TAG_TO_ROBOT_CENTER_Y_MM * math.cos(tag_yaw_rad)

                robot_x_mm = tag_x_mm + rotated_offset_x
                robot_y_mm = tag_y_mm + rotated_offset_y
                robot_yaw_deg = (tag_yaw_deg + TAG_TO_ROBOT_CENTER_YAW_DEG) % 360

                # --- Round to one decimal place before putting into JSON ---
                current_tag_data.update({
                    "pose_valid": True,
                    "x_mm": round(robot_x_mm, 1),
                    "y_mm": round(robot_y_mm, 1),
                    "z_mm": round(tag_z_mm, 1),
                    "yaw_deg": round(robot_yaw_deg, 1)
                })

                text = f"ID:{tag_id} RX:{robot_x_mm:.1f} RY:{robot_y_mm:.1f} RZ:{tag_z_mm:.1f} RYaw:{robot_yaw_deg:.1f} deg"
                print(text)

            else:
                robot_x_mm, robot_y_mm, tag_z_mm, robot_yaw_deg = 0, 0, 0, 0
                text = f"ID:{tag_id} Pose estimation failed."
                print(text)
        else:
            # Fallback to pixel coordinates if calibration data is missing
            translation = r.center - np.array([frame.shape[1] / 2, frame.shape[0] / 2])
            robot_x_mm = translation[0]
            robot_y_mm = translation[1]

            corner0 = r.corners[0].ravel()
            corner1 = r.corners[1].ravel()
            angle_rad = np.arctan2(corner1[1] - corner0[1], corner1[0] - corner0[0])
            robot_yaw_deg = np.degrees(angle_rad)
            robot_yaw_deg = (robot_yaw_deg + 360) % 360

            # Update the tag data dictionary (using _px for clarity)
            current_tag_data.update({
                "pose_valid": False,
                "x_px": round(float(robot_x_mm), 1),
                "y_px": round(float(robot_y_mm), 1),
                "yaw_approx_deg": round(float(robot_yaw_deg), 1)
            })

            text = f"ID:{tag_id} RX_px:{robot_x_mm:.1f} RY_px:{robot_y_mm:.1f} RYaw_approx:{robot_yaw_deg:.1f} deg (No calibration)"
            print(text)

        # Append the data for the current tag
        all_tag_data.append(current_tag_data)

        try:
            corners = np.array(r.corners, dtype=np.int32).reshape((-1, 1, 2))
            cv2.polylines(frame, [corners], isClosed=True, color=(0, 255, 0), thickness=2)
        except Exception as e:
            print(f"Error drawing outline: {e}")
            continue

        center_x, center_y = int(r.center[0]), int(r.center[1])


    cv2.putText(frame, text, (10, frame.shape[0] - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                (255, 255, 255), 2)

    cv2.imshow("AprilTag Detection", frame)

    # --- Send all detected tag data as a single JSON message ---
    full_payload = {
        "timestamp": time.time(),
        "detections": all_tag_data
    }
    send_json_data(full_payload)


def main():
    """
    Main function to initialize components and run the AprilTag detection loop.
    """
    global camera_capture, server_socket, client_connection, quit_program
    socket_enabled = False
    try:
        load_camera_calibration(CALIBRATION_FILE)
        camera_capture = initialize_camera()
        tag_detector = initialize_apriltag_detector()

        # 1. Setup the listening socket once.
        socket_enabled = setup_socket_server()

        while not quit_program: # Check the global quit flag
            # 2. Check for a client connection if the socket is enabled and not yet connected.
            if socket_enabled and client_connection is None:
                 attempt_client_accept()

            # --- High Priority Key Check ---
            # Check for 'q' immediately after non-blocking socket checks
            if cv2.waitKey(1) & 0xFF == ord('q'):
                quit_program = True
                continue

            ret, frame = camera_capture.read()
            if not ret:
                print("Error: Could not read frame. Exiting.")
                quit_program = True
                break

            process_apriltag_data(frame)

            time.sleep(0.01) # Small sleep to avoid maxing out CPU

    except (IOError, ValueError) as e:
        print(f"Runtime Error: {e}")
    finally:
        # Final cleanup for all global resources
        if camera_capture:
            camera_capture.release()
            print("Camera released")
        if client_connection:
            try:
                client_connection.close()
            except:
                pass
            print("Client connection closed")
        if server_socket:
            server_socket.close()
            print("Server socket closed")
        cv2.destroyAllWindows()
        print("All windows destroyed. Program terminated.")

if __name__ == "__main__":
    main()
