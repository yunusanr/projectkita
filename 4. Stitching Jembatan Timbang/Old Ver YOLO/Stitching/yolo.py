import cv2
import numpy as np

# Create a blank image for the truck from a top-down view
height, width = 400, 800
background = np.ones((height, width, 3), dtype=np.uint8) * 255  # White background

# Frame 1: Detecting the Cab (top-down view)
frame1 = background.copy()
cv2.rectangle(
    frame1, (300, 100), (500, 200), (0, 0, 255), 2
)  # Red bounding box for cab
cv2.putText(frame1, "Cab", (400, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

# Frame 2: Detecting the Trailer (top-down view)
frame2 = background.copy()
cv2.rectangle(
    frame2, (300, 100), (500, 200), (0, 0, 255), 2
)  # Red bounding box for cab
cv2.putText(frame2, "Cab", (400, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
cv2.rectangle(
    frame2, (500, 150), (700, 250), (0, 255, 0), 2
)  # Green bounding box for trailer
cv2.putText(
    frame2, "Trailer", (600, 140), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2
)

# Frame 3: Detecting the Back (top-down view)
frame3 = background.copy()
cv2.rectangle(
    frame3, (300, 100), (500, 200), (0, 0, 255), 2
)  # Red bounding box for cab
cv2.putText(frame3, "Cab", (400, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
cv2.rectangle(
    frame3, (500, 150), (700, 250), (0, 255, 0), 2
)  # Green bounding box for trailer
cv2.putText(
    frame3, "Trailer", (600, 140), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2
)
cv2.rectangle(
    frame3, (700, 200), (800, 300), (255, 0, 0), 2
)  # Blue bounding box for back
cv2.putText(frame3, "Back", (750, 190), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)

# Final Stitched Result (top-down view)
final_frame = background.copy()
cv2.rectangle(
    final_frame, (300, 100), (500, 200), (0, 0, 255), 2
)  # Red bounding box for cab
cv2.putText(
    final_frame, "Cab", (400, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2
)
cv2.rectangle(
    final_frame, (500, 150), (700, 250), (0, 255, 0), 2
)  # Green bounding box for trailer
cv2.putText(
    final_frame, "Trailer", (600, 140), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2
)
cv2.rectangle(
    final_frame, (700, 200), (800, 300), (255, 0, 0), 2
)  # Blue bounding box for back
cv2.putText(
    final_frame, "Back", (750, 190), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2
)

# Display the images
cv2.imshow("Frame 1: Cab", frame1)
cv2.imshow("Frame 2: Cab and Trailer", frame2)
cv2.imshow("Frame 3: Cab, Trailer, and Back", frame3)
cv2.imshow("Final Stitched Result", final_frame)

# Wait for a key press and close the windows
cv2.waitKey(0)
cv2.destroyAllWindows()
