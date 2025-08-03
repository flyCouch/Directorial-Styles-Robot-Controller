import cv2
import numpy as np
import math

def image_to_gcode(image_path, gcode_file_path, robot_width, robot_height, max_segment_length, min_segment_length,
                   origin_preference, gcode_header, gcode_footer,
                   laser_on_code="M03", laser_off_code="M05", initial_feed_rate=100,
                   movement_output_type="int", num_decimal_places=2):
    """
    Converts a JPEG image to G-code, adaptively segmenting contours for controlled segment length.

    Args:
        image_path (str): Path to the JPEG image.
        gcode_file_path (str): Path to the output G-code file.
        robot_width (float): Width of the robot's workspace.
        robot_height (float): Height of the robot's workspace.
        max_segment_length (float): Maximum length of each G-code line segment in mm (or your units).
        min_segment_length (float): Minimum length of each G-code line segment.
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

        # Invert colors (assuming black lines on white background)
        img = cv2.bitwise_not(img)

        # Apply a binary threshold
        _, binary_img = cv2.threshold(img, 128, 255, cv2.THRESH_BINARY)

        # Find contours
        contours, _ = cv2.findContours(binary_img, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        img_height, img_width = img.shape
        scale_x = robot_width / img_width
        scale_y = robot_height / img_height

        all_gcode_commands = []
        all_gcode_commands.extend(gcode_header)
        all_gcode_commands.append(f"G1 F{initial_feed_rate}")

        def generate_gcode_path(path_points):
            gcode_path = []
            if movement_output_type == "float":
                format_spec = f".{num_decimal_places}f"
                x_format = f"X{{:{format_spec}}}"
                y_format = f"Y{{:{format_spec}}}"
            else:
                x_format = "X{:.0f}"
                y_format = "Y{:.0f}"

            for i, point in enumerate(path_points):
                x_mm = point[0]
                y_mm = point[1]

                if i == 0:
                    gcode_path.append(f"G0 {x_format.format(x_mm)} {y_format.format(y_mm)}")
                    gcode_path.append(laser_on_code)
                else:
                    gcode_path.append(f"G1 {x_format.format(x_mm)} {y_format.format(y_mm)}")
            return gcode_path

        # --- MODIFIED: Resample contour function with min and max segment length ---
        def resample_contour(contour, max_len, min_len):
            resampled_points = []
            if len(contour) < 2:
                return []

            resampled_points.append(contour[0][0])
            current_point = contour[0][0]
            
            for i in range(1, len(contour)):
                next_point = contour[i][0]
                segment_length = math.sqrt((next_point[0] - current_point[0])**2 + (next_point[1] - current_point[1])**2)
                
                if segment_length < min_len and i < len(contour) - 1:
                    # If segment is too short, combine it with the next one
                    # We can't combine if it's the last point
                    continue
                
                if segment_length > max_len:
                    # If segment is too long, subdivide it
                    num_sub_segments = math.ceil(segment_length / max_len)
                    for j in range(1, num_sub_segments + 1):
                        t = j / num_sub_segments
                        interp_x = current_point[0] + t * (next_point[0] - current_point[0])
                        interp_y = current_point[1] + t * (next_point[1] - current_point[1])
                        resampled_points.append(np.array([interp_x, interp_y]))
                else:
                    # Segment length is within the desired range, add the point
                    resampled_points.append(next_point)
                
                # Update current_point for the next iteration
                current_point = next_point
            
            return resampled_points

        # Process each contour
        for i, contour in enumerate(contours):
            scaled_contour = []
            for point in contour:
                x_img, y_img = point[0]
                x_mm = x_img * scale_x
                y_mm = y_img * scale_y

                if origin_preference == "c":
                    x_mm -= robot_width / 2
                    y_mm -= robot_height / 2
                
                y_mm = robot_height - y_mm
                scaled_contour.append([x_mm, y_mm])
            
            scaled_contour_np = np.array(scaled_contour).reshape(-1, 1, 2)
            
            # Resample the contour using min and max lengths
            resampled_contour = resample_contour(scaled_contour_np, max_segment_length, min_segment_length)
            
            if resampled_contour:
                if i > 0:
                    all_gcode_commands.append(laser_off_code)
                
                contour_gcode = generate_gcode_path(resampled_contour)
                all_gcode_commands.extend(contour_gcode)
            
            all_gcode_commands.append(laser_off_code)

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
    
    # --- New variables for min/max segment length ---
    max_segment_length_mm = 10.0
    min_segment_length_mm = 1.0 # New minimum segment length
    
    laser_on_command = "S255"
    laser_off_command = "S0"
    
    default_gcode_header = [
        "G90 (use absolute coordinates)",
        "S0 (laser off while travelling to first point)"
    ]
    default_gcode_footer = [
        "G0 X0 Y0 (move back to origin) (Note: Z-axis commands are omitted as per user request)"
    ]

    while True:
        preference = input("Choose origin for G-code (c for center / bl for bottom-left): ").lower().strip()
        if preference in ["c", "bl"]:
            break
        else:
            print("Invalid preference. Please enter 'c' or 'bl'.")

    while True:
        movement_type_choice = input("Choose movement output type (int for integers / float for floating-point): ").lower().strip()
        if movement_type_choice in ["int", "float"]:
            break
        else:
            print("Invalid choice. Please enter 'int' or 'float'.")

    num_decimal_places = 2
    if movement_type_choice == "float":
        while True:
            try:
                decimal_input = input("Enter desired number of decimal places (e.g., 2, 3, 4): ").strip()
                num_decimal_places = int(decimal_input)
                if num_decimal_places >= 0:
                    break
                else:
                    print("Please enter a non-negative integer.")
            except ValueError:
                print("Invalid input. Please enter a whole number.")

    image_to_gcode(
        image_path=image_file,
        gcode_file_path=gcode_file,
        robot_width=robot_width_mm,
        robot_height=robot_height_mm,
        max_segment_length=max_segment_length_mm,
        min_segment_length=min_segment_length_mm, # Pass the new parameter
        origin_preference=preference,
        gcode_header=default_gcode_header,
        gcode_footer=default_gcode_footer,
        laser_on_code=laser_on_command,
        laser_off_code=laser_off_command,
        initial_feed_rate=150,
        movement_output_type=movement_type_choice,
        num_decimal_places=num_decimal_places
    )

if __name__ == "__main__":
    main()
