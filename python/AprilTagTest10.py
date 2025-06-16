import cv2
import apriltag
import time
import numpy as np

# --- Configuration ---
CAMERA_INDEX = 0
APRILTAG_FAMILY = 'tag36h11'

# --- Global Variables ---
tag_detector = None
camera_capture = None

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
    tag_detector = apriltag.Detector()
    return tag_detector

def process_apriltag_data(frame):
    """
    Detects AprilTags in the given frame and displays the camera feed.
    """
    if tag_detector is None:
        raise ValueError("AprilTag detector not initialized.")

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    results = tag_detector.detect(gray)

    if not results:
        print("No AprilTags detected in this frame.")
        cv2.imshow("AprilTag Detection", frame)
        return

    for r in results:
        tag_id = r.tag_id
        translation = r.center - np.array([frame.shape[1] / 2, frame.shape[0] / 2])

        try:
            corners = np.array(r.corners, dtype=np.int32).reshape((-1, 1, 2))
            cv2.polylines(frame, [corners], isClosed=True, color=(0, 255, 0), thickness=2)
        except Exception as e:
            print(f"Error drawing outline: {e}")
            print(f"Corners: {r.corners}")
            continue

        center_x, center_y = int(r.center[0]), int(r.center[1])

        # --- Approximate Yaw Calculation ---
        corner0 = r.corners[0].ravel()  # Get first corner as (x, y)
        corner1 = r.corners[1].ravel()  # Get second corner
        angle_rad = np.arctan2(corner1[1] - corner0[1], corner1[0] - corner0[0])
        angle_deg = np.degrees(angle_rad)
        # Normalize angle to be between 0 and 360
        angle_deg = (angle_deg + 360) % 360

        text = f"X:{translation[0]:.2f} Y:{translation[1]:.2f} Yaw:{angle_deg:.2f}"
        try:
            cv2.putText(frame, text, (10, frame.shape[0] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                        (255, 255, 255), 2)
        except Exception as e:
            print(f"Error drawing text: {e}")
            print(f"Text: {text}")

        print(f"Tag ID: {tag_id}, Translation: ({translation[0]:.4f}, {translation[1]:.4f}), Yaw: {angle_deg:.4f}")

    cv2.imshow("AprilTag Detection", frame)



def main():
    """
    Main function to initialize components and run the AprilTag detection loop.
    """
    global camera_capture
    try:
        camera_capture = initialize_camera()
        tag_detector = initialize_apriltag_detector()

        while True:
            ret, frame = camera_capture.read()
            if not ret:
                print("Error: Could not read frame.")
                break

            process_apriltag_data(frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            time.sleep(0.1)

    except (IOError, ValueError) as e:
        print(f"Error: {e}")
    finally:
        if camera_capture:
            camera_capture.release()
            print("Camera released")
        cv2.destroyAllWindows()
        print("AllWindows destroyed")

if __name__ == "__main__":
    main()

