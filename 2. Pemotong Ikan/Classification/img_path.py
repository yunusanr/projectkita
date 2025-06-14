from ultralytics import YOLO
import cv2
from PIL import Image
import tkinter as tk
from tkinter import filedialog

# Model and other initialization
model = YOLO("Nila.pt")

classNames = ["Nila", "Head"]
fishLen = []

config = dict(single_cls=True)


# Function to calculate width in pixels
def getWidthPixel(x1, x2):
    wpix = x2 - x1
    wpix = abs(wpix)
    return wpix


# Function to map width to a different range (cm)
def mapWidth(wpix, from_min=0, from_max=640, to_min=0, to_max=45.5):
    percentage = (wpix - from_min) / (from_max - from_min)
    w = (percentage * (to_max - to_min)) + to_min
    return w


# Function to open a file dialog and load an image
def load_image():
    root = tk.Tk()
    root.withdraw()  # Hide the root window
    file_path = filedialog.askopenfilename(
        title="Select an image", filetypes=[("Image Files", "*.jpg;*.png;*.jpeg")]
    )
    if file_path:
        return cv2.imread(file_path)
    else:
        print("No file selected.")
        return None


# Load image using the file dialog
img = load_image()
if img is None:
    print("No image loaded, exiting.")
    exit()

# Process the image with YOLO model
results = model(img, stream=True)
fishLen = []
count = 0

for r in results:
    boxes = r.boxes
    for box in boxes:
        x1, y1, x2, y2 = box.xyxy[0]
        x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)

        # Draw the bounding box
        cv2.rectangle(img, (x1, y1), (x2, y2), (255, 0, 255), 3)
        print(x1, y1, x2, y2)
        # Calculate width in pixels and map to cm
        wpix = getWidthPixel(x1, x2)
        w = mapWidth(wpix)
        wcm = str(f"{w} cm")

        # Get the class of the object detected
        cls = int(box.cls[0])

        # Add text for width in cm
        org = [x1, y1 - 10]
        font = cv2.FONT_HERSHEY_SIMPLEX
        fontScale = 1
        color = (0, 250, 0)
        thickness = 2
        cv2.putText(img, wcm, org, font, fontScale, color, thickness)

        fishLen.append(w)

        count += 1
        if count == 2:
            count = 0
            break

    fishLen.sort()
    if x2 >= 500 and x2 <= 502:
        print(fishLen)

# Show the processed image with detections
cv2.imshow("Detection", img)

# Wait for a key press to close the image window
cv2.waitKey(0)
cv2.destroyAllWindows()
