import cv2
import numpy as np
import math

def image_to_gcode(image_path, gcode_file_path, robot_width, robot_height, max_segment_length,
                   origin_preference, gcode_header, gcode_footer,
                   laser_on_code="M03", laser_off_code="M05", initial_feed_rate=100,
                   movement_output_type="int", num_decimal_places=2): # Added num_decimal_places
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
        movement_output_type (str, optional): "float" for floating-point coordinates, "int" for integer coordinates.
        num_decimal_places (int, optional): Number of decimal places for float output. Default to 2.
    """
    try:
        # Load the image in grayscale
        img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
        if img is None:
            raise FileNotFoundError(f"Image not found at {image_path}")

        # Invert colors (assuming black lines on white background, typical for contours)
        # If your image has white lines on black background, you might skip this or adjust thresholding
        img = cv2.bitwise_not(img)

        # Apply a binary threshold to get a black and white image
        # You might need to adjust the threshold value (e.g., 128, 200, etc.)
        _, binary_img = cv2.threshold(img, 128, 255, cv2.THRESH_BINARY)

        # Find contours
        # cv2.RETR_LIST retrieves all contours without any hierarchy.
        # cv2.CHAIN_APPROX_SIMPLE compresses horizontal, vertical, and diagonal segments.
        contours, _ = cv2.findContours(binary_img, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        # Get image dimensions
        img_height, img_width = img.shape

        # Scaling factors for X and Y coordinates
        scale_x = robot_width / img_width
        scale_y = robot_height / img_height

        all_gcode_commands = []

        # --- Add G-code Header from variable ---
        all_gcode_commands.extend(gcode_header)

        # Add initial feed rate command
        all_gcode_commands.append(f"G1 F{initial_feed_rate}")

        # Helper function to generate G-code for a path segment
        def generate_gcode_path(path_points):
            gcode_path = []
            
            # --- Determine coordinate format string based on output type and decimal places ---
            if movement_output_type == "float":
                # This format specifier ensures that the number of decimal places is always
                # maintained, including trailing zeros (e.g., 1.2 becomes 1.20 if num_decimal_places is 2).
                format_spec = f".{num_decimal_places}f"
                x_format = f"X{{:{format_spec}}}"
                y_format = f"Y{{:{format_spec}}}"
            else: # int
                x_format = "X{:.0f}"
                y_format = "Y{:.0f}"

            for i, point in enumerate(path_points):
                x_mm = point[0]
                y_mm = point[1]

                if i == 0:
                    # Move to the start of the contour with laser off (G0 for rapid move)
                    gcode_path.append(f"G0 {x_format.format(x_mm)} {y_format.format(y_mm)}")
                    gcode_path.append(laser_on_code) # Turn laser on
                else:
                    # Draw line segment with laser on (G1 for linear move)
                    gcode_path.append(f"G1 {x_format.format(x_mm)} {y_format.format(y_mm)}")
            return gcode_path

        # Helper function to resample a contour into segments of max_segment_length
        def resample_contour(contour, max_length):
            resampled_points = []
            if len(contour) < 2:
                return []

            # Start with the first point
            resampled_points.append(contour[0][0]) # contour is (num_points, 1, 2)
            
            for i in range(len(contour) - 1):
                p1 = contour[i][0]
                p2 = contour[i+1][0]
                
                # Calculate distance between current and next point
                segment_length = math.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)
                
                if segment_length > max_length:
                    # Determine how many new segments are needed
                    num_sub_segments = math.ceil(segment_length / max_length)
                    
                    for j in range(1, num_sub_segments + 1):
                        # Interpolate new points along the segment
                        t = j / num_sub_segments
                        interp_x = p1[0] + t * (p2[0] - p1[0])
                        interp_y = p1[1] + t * (p2[1] - p1[1])
                        resampled_points.append(np.array([interp_x, interp_y]))
                else:
                    # If segment is already short enough, just add the next point
                    resampled_points.append(p2)
            return resampled_points


        # Process each contour
        for i, contour in enumerate(contours):
            # Contour points are typically (x, y)
            # Apply scaling and adjust origin
            scaled_contour = []
            for point in contour:
                x_img, y_img = point[0]
                
                # Scale coordinates
                x_mm = x_img * scale_x
                y_mm = y_img * scale_y

                # Adjust origin if 'c' (center) is preferred
                if origin_preference == "c":
                    x_mm -= robot_width / 2
                    y_mm -= robot_height / 2
                # For 'bl' (bottom-left), y-axis needs to be inverted if image origin is top-left
                # OpenCV's Y-axis is top-to-bottom, G-code Y-axis is bottom-to-top (usually)
                y_mm = robot_height - y_mm # Invert Y for G-code plotter coordinate system

                scaled_contour.append([x_mm, y_mm])
            
            # Convert to numpy array for resample_contour
            scaled_contour_np = np.array(scaled_contour).reshape(-1, 1, 2)

            # Resample the contour
            resampled_contour = resample_contour(scaled_contour_np, max_segment_length)
            
            # Generate G-code for the current contour
            # Only add if the contour has points after resampling
            if resampled_contour:
                # Add laser off before moving to new contour's start
                if i > 0: # Don't add for the very first contour
                     all_gcode_commands.append(laser_off_code)
                
                contour_gcode = generate_gcode_path(resampled_contour)
                all_gcode_commands.extend(contour_gcode)
            
            all_gcode_commands.append(laser_off_code) # Turn laser off at the end of each contour


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
    gcode_file = "/home/ron/Pictures/output.gcode"
    robot_width_mm = 200.0
    robot_height_mm = 200.0
    max_segment_length_mm = 5.0
    
    # --- Laser commands using S-values ---
    laser_on_command = "S255"  # Laser ON (e.g., full power)
    laser_off_command = "S0"   # Laser OFF (e.g., zero power)

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

    # --- Prompt for decimal places if float is chosen ---
    num_decimal_places = 2 # Default value if 'int' is chosen or no input for 'float'
    if movement_type_choice == "float":
        while True:
            try:
                decimal_input = input("Enter desired number of decimal places (e.g., 2, 3, 4): ").strip()
                num_decimal_places = int(decimal_input)
                if num_decimal_places >= 0: # Ensure it's a non-negative integer
                    break
                else:
                    print("Please enter a non-negative integer.")
            except ValueError:
                print("Invalid input. Please enter a whole number.")
    # --- END Prompt ---

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
        initial_feed_rate=150,
        movement_output_type=movement_type_choice,
        num_decimal_places=num_decimal_places # PASS THE PARAMETER
    )

if __name__ == "__main__":
    main()
