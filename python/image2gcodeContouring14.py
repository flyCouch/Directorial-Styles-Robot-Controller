import cv2
import numpy as np

def image_to_gcode(image_path, gcode_file_path, robot_width, robot_height, max_segment_length,
                   origin_preference, gcode_header, gcode_footer,
                   laser_on_code="M03", laser_off_code="M05", initial_feed_rate=100,
                   movement_output_type="int"):
    """
    Converts a JPEG image to G-code, adaptively segmenting contours for controlled segment length.

    Args:
        image_path (str): Path to the JPEG image.
        gcode_file_path (str): Path to the output G-code file.
        robot_width (float): Width of the robot's workspace.
        robot_height (float): Height of the robot's workspace.
        max_segment_length (float): Maximum length of each G-code line segment in mm (or your units).
        origin_preference (str): "c" for camera/robot center origin, "bl" for printer bottom-left origin.
        gcode_header (list): A list of strings for G-code commands to be placed at the beginning.
        gcode_footer (list): A list of strings for G-code commands to be placed at the end.
        laser_on_code (str, optional): G-code to turn laser ON (can contain S-value like "S255").
        laser_off_code (str, optional): G-code to turn laser OFF (can contain S-value like "S0").
        initial_feed_rate (float, optional): Initial G-code movement speed set at the beginning.
        movement_output_type (str): "float" for floating-point coordinates, "int" for integer coordinates.
    """
    try:
        image = cv2.imread(image_path)
        if image is None:
            raise ValueError(f"Could not load image from {image_path}")

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        _, thresh = cv2.threshold(blurred, 127, 255, cv2.THRESH_BINARY)
        contours, _ = cv2.findContours(thresh, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        img_height, img_width = image.shape[:2]

        # --- Dynamic pixel_to_robot function based on preference ---
        if origin_preference == "c":
            print("Using CENTER origin for G-code generation.")
            def pixel_to_robot(x_pixel, y_pixel):
                """Convert pixel to robot coordinates with (0,0) at center."""
                x_robot = (x_pixel / img_width) * robot_width - (robot_width / 2)
                y_robot = (y_pixel / img_height) * robot_height - (robot_height / 2)
                return x_robot, y_robot
        elif origin_preference == "bl":
            print("Using BOTTOM-LEFT origin for G-code generation.")
            def pixel_to_robot(x_pixel, y_pixel):
                """Convert pixel to robot coordinates with (0,0) at bottom-left."""
                x_robot = (x_pixel / img_width) * robot_width
                y_robot = robot_height - (y_pixel / img_height) * robot_height # Invert Y for typical printer beds
                return x_robot, y_robot
        else:
            raise ValueError("Invalid origin_preference. Must be 'c' or 'bl'.")
        # --- End of dynamic pixel_to_robot ---


        def resample_contour(contour, max_segment_length_robot_units):
            """
            Resamples a contour to generate points with a maximum segment length in robot coordinates.

            Args:
                contour (np.ndarray):  OpenCV contour (list of points).
                max_segment_length_robot_units: Maximum segment length in robot units

            Returns:
                list: List of (x, y) robot coordinates, resampled.
            """
            resampled_points = []
            if len(contour) < 2:
                return []

            p1_pixel = contour[0][0]
            x1_robot, y1_robot = pixel_to_robot(p1_pixel[0], p1_pixel[1])
            resampled_points.append((x1_robot, y1_robot))

            dist_along_contour = 0.0
            i = 1
            while i < len(contour):
                p2_pixel = contour[i][0]
                x2_robot, y2_robot = pixel_to_robot(p2_pixel[0], p2_pixel[1])
                segment_length = np.sqrt((x2_robot - x1_robot)**2 + (y2_robot - y1_robot)**2)

                if (dist_along_contour + segment_length) >= max_segment_length_robot_units:
                    remaining_dist = max_segment_length_robot_units - dist_along_contour
                    ratio = remaining_dist / segment_length
                    x_new_robot = x1_robot + ratio * (x2_robot - x1_robot)
                    y_new_robot = y1_robot + ratio * (y2_robot - y1_robot)
                    resampled_points.append((x_new_robot, y_new_robot))
                    x1_robot, y1_robot = x_new_robot, y_new_robot
                    dist_along_contour = 0.0
                else:
                    dist_along_contour += segment_length
                    x1_robot, y1_robot = x2_robot, y2_robot
                    i += 1

            # Ensure the last point is added if it hasn't been already
            if not resampled_points or (resampled_points[-1] != (x2_robot, y2_robot) and len(contour) > 1):
                 resampled_points.append((x2_robot, y2_robot))
            return resampled_points


        def generate_gcode_path(resampled_contour):
            """Generate G-code commands for a resampled contour."""
            gcode_commands = []
            if resampled_contour:
                
                # Determine formatting based on movement_output_type
                if movement_output_type == "int":
                    coord_format = ".0f"  # Format for integers
                    x_start = int(round(resampled_contour[0][0]))
                    y_start = int(round(resampled_contour[0][1]))
                else: # Default to float
                    coord_format = ".2f"
                    x_start = resampled_contour[0][0]
                    y_start = resampled_contour[0][1]

                # Initial move to the start of the contour (laser off) - NO Z-AXIS
                gcode_commands.append(f"G0 X{x_start:{coord_format}} Y{y_start:{coord_format}}")
                gcode_commands.append(laser_on_code) # Turn laser ON (e.g., S255)
                for x, y in resampled_contour:
                    # Format based on type
                    if movement_output_type == "int":
                        x_coord = int(round(x))
                        y_coord = int(round(y))
                    else:
                        x_coord = x
                        y_coord = y
                    # G1 move - NO Z-AXIS
                    gcode_commands.append(f"G1 X{x_coord:{coord_format}} Y{y_coord:{coord_format}}")
                gcode_commands.append(laser_off_code) # Turn laser OFF (e.g., S0)
            return gcode_commands

        all_gcode_commands = []
        
        # --- Add G-code Header from variable ---
        all_gcode_commands.extend(gcode_header)
        
        # --- Set Feed Rate once at the top ---
        all_gcode_commands.append(f"F{initial_feed_rate:.1f}")

        for i, contour in enumerate(contours):
            print(f"Processing contour {i + 1} of {len(contours)}")
            resampled_contour = resample_contour(contour, max_segment_length)
            contour_gcode = generate_gcode_path(resampled_contour)
            all_gcode_commands.extend(contour_gcode)

        # --- Add G-code Footer from variable ---
        all_gcode_commands.extend(gcode_footer)

        with open(gcode_file_path, "w") as f:
            for command in all_gcode_commands:
                f.write(command + "\n")
        print(f"G-code written to {gcode_file_path}")

    except Exception as e:
        print(f"Error: {e}")
        return

def main():
    image_file = "/home/ron/Pictures/Gloria-Aura-Revista-H-Para-Hombres-MX-Julio-2013-18.jpg"
    gcode_file = "/home/ron/Pictures/output1.gcode"
    robot_width_mm = 200.0
    robot_height_mm = 200.0
    max_segment_length_mm = .1
    
    # --- Laser commands using S-values ---
    laser_on_command = "S255"
    laser_off_command = "S0"
    initial_feed_rate = 150.0

    # --- Define G-code Header and Footer as variables ---
    default_gcode_header = [
        "G90 (use absolute coordinates)",
        "S0 (laser off while travelling to first point)" # Added S0 command here
    ]
    default_gcode_footer = [
        "G0 X0 Y0 (move back to origin) (Note: Z-axis commands are omitted as per user request)"
    ]

    # --- Ask user for origin preference ---
    while True:
        preference = input("Choose origin for G-code (c for center / bl for bottom-left): ").lower().strip()
        if preference in ["c", "bl"]:
            break
        else:
            print("Invalid preference. Please enter 'c' or 'bl'.")

    # --- Ask user for movement output type ---
    while True:
        movement_type_choice = input("Choose movement output type (int for integers / float for floating-point): ").lower().strip()
        if movement_type_choice in ["int", "float"]:
            break
        else:
            print("Invalid choice. Please enter 'int' or 'float'.")

    image_to_gcode(
        image_path=image_file,
        gcode_file_path=gcode_file,
        robot_width=robot_width_mm,
        robot_height=robot_height_mm,
        max_segment_length=max_segment_length_mm,
        origin_preference=preference,
        gcode_header=default_gcode_header,
        gcode_footer=default_gcode_footer,
        laser_on_code=laser_on_command,
        laser_off_code=laser_off_command,
        initial_feed_rate=initial_feed_rate,
        movement_output_type=movement_type_choice
    )

if __name__ == "__main__":
    main()
