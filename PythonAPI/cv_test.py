import cv2
import numpy as np

def calculate_color_percentage(image_path, target_color, tolerance):
    img = cv2.imread(image_path)

    lower = np.array([target_color[0] - tolerance, target_color[1] - tolerance, target_color[2] - tolerance])
    upper = np.array([target_color[0] + tolerance, target_color[1] + tolerance, target_color[2] + tolerance])
    mask = cv2.inRange(img, lower, upper)

    total_pixels = img.size
    colored_pixels = cv2.countNonZero(mask)
    percentage = (colored_pixels / total_pixels) * 100

    return percentage

# Example usage
image_path = 'my_image.jpg'
target_color = (0, 255, 0)  # Green 
tolerance = 25

percentage = calculate_color_percentage(image_path, target_color, tolerance)
print("Percentage of the target color:", percentage, "%")
