from PIL import Image, ImageDraw
import os # Import os module to handle file paths

def generate_and_save_checkerboard(square_mm, grid_width, grid_height, dpi=300, output_dir="."):
    """
    Generates a checkerboard image and saves it as a JPEG file locally.

    Args:
        square_mm (float): The desired size of each square in millimeters.
        grid_width (int): Number of squares horizontally.
        grid_height (int): Number of squares vertically.
        dpi (int): Dots per inch for the output image. (Default: 300 DPI for print quality)
        output_dir (str): Directory where the image file will be saved. (Default: current directory)

    Returns:
        str: The full path to the saved image file, or None if an error occurred.
    """
    mm_per_inch = 25
    pixels_per_mm = dpi / mm_per_inch

    # Calculate pixel size for each square and total image dimensions
    # Using round() to ensure integer pixel dimensions for crisp lines
    square_pixel_size = int(round(square_mm * pixels_per_mm))
    
    img_width_px = grid_width * square_pixel_size
    img_height_px = grid_height * square_pixel_size

    # Create a new blank image with a white background
    img = Image.new('RGB', (img_width_px, img_height_px), color = 'white')
    draw = ImageDraw.Draw(img)

    # Draw the checkerboard pattern
    for row in range(grid_height):
        for col in range(grid_width):
            # Determine color based on checkerboard pattern (alternating black and white)
            color = (0, 0, 0) if (row + col) % 2 == 0 else (255, 255, 255) # (0,0) is black

            # Calculate pixel coordinates for the current square
            x0 = col * square_pixel_size
            y0 = row * square_pixel_size
            x1 = x0 + square_pixel_size
            y1 = y0 + square_pixel_size

            # Draw the filled rectangle for the current square
            draw.rectangle([x0, y0, x1, y1], fill=color)

    # Construct the output filename
    output_filename = f"checkerboard_{grid_width}x{grid_height}_{square_mm:.0f}mm_{dpi}dpi.jpg"
    output_filepath = os.path.join(output_dir, output_filename)

    try:
        # Save the image as a JPEG file
        img.save(output_filepath, format='JPEG', quality=95) # quality=95 for good quality JPEG
        print(f"Checkerboard image saved successfully to: {output_filepath}")
        return output_filepath
    except Exception as e:
        print(f"Error saving image: {e}")
        return None

# --- Configuration for your checkerboard ---
SQUARE_SIZE_MM = 25.0
GRID_WIDTH_SQUARES = 5
GRID_HEIGHT_SQUARES = 6
DPI_RESOLUTION = 300 # Standard resolution for good print quality

# --- Run the generator ---
if __name__ == '__main__':
    # Make sure 'Pillow' is installed: pip install Pillow
    saved_file_path = generate_and_save_checkerboard(
        SQUARE_SIZE_MM, GRID_WIDTH_SQUARES, GRID_HEIGHT_SQUARES, DPI_RESOLUTION
    )

    if saved_file_path:
        print("\nTo use this image:")
        print(f"1. Locate the file: {saved_file_path}")
        print("2. Open it with any image viewer/printer.")
        print("3. When printing, ensure your printer settings are set to 'Actual Size' or '100% Scale' ")
        print("   and that the DPI matches the image's DPI (300 DPI) for accurate square sizes.")
    else:
        print("\nFailed to generate the checkerboard image.")


