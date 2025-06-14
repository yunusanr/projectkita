import numpy as np
import cv2

# Load the extracted regions
extracted_regions = np.load("extracted_regions.npy", allow_pickle=True)

# Display each extracted region
for i, region in enumerate(extracted_regions):
    print(f"Extracted Region {i+1}: {region}")
    # Optionally, you can visualize the extracted pixels
    # Create an image to visualize the extracted pixels
    visualized_region = np.zeros(
        (1, len(region)), dtype=np.uint8
    )  # Create a single row image
    visualized_region[0, :] = region  # Set the extracted pixels
    cv2.imshow(
        f"Extracted Region {i+1}", visualized_region * 255
    )  # Scale to 255 for display
    cv2.waitKey(0)  # Wait for a key press to show the next region

cv2.destroyAllWindows()
