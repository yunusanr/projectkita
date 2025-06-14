from ultralytics import YOLO
import cv2
import pandas as pd  # Import pandas for Excel saving
import serial

# Serial port configuration
# ser = serial.Serial(port="COM3", baudrate=115200, timeout=0.1)

# Video capture setup
cnn = cv2.VideoCapture(0)
cnn.set(3, 640)  # Set width
cnn.set(4, 480)  # Set height

# Variables
tolerance = 200
head_width_cm = 0.0
body_width_cm = 0.0

# Load YOLO model
model = YOLO("best.pt")

# Calibration for mapping width
calibration_data = [
    (20, 35)
]  # Example: detected width = 20 pixels, actual width = 35 cm
calib_factor = [actual / detected for actual, detected in calibration_data]
CALIBRATION_FACTOR = sum(calib_factor) / len(calib_factor)

# Excel logging setup
x_data = []  # List to store x1 and x2
output_file = "fish_detection_data.xlsx"  # Output Excel file


# Function to calculate width in pixels
def getWidthPixel(x1, x2):
    wpix = abs(x2 - x1)
    return wpix


# Function to map pixel width to centimeters
def mapWidth(wpix, from_min=0, from_max=640, to_min=0, to_max=45.5):
    to_max_calibrated = to_max * CALIBRATION_FACTOR
    percentage = (wpix - from_min) / (from_max - from_min)
    w = (percentage * (to_max_calibrated - to_min)) + to_min
    return w


while True:
    success, img = cnn.read()
    results = model(img, stream=True)
    frame_height = img.shape[0]
    frame_width = img.shape[1]
    center_x = frame_width // 2  # Center line for vertical comparison
    vertical_line_color = (0, 255, 0)  # Green line by default

    for r in results:
        boxes = r.boxes
        for box in boxes:
            x1, y1, x2, y2 = box.xyxy[0]
            x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)

            # Save x1 and x2 values to the list
            x_data.append({"x1": x1, "x2": x2})

            # Draw bounding box
            cv2.rectangle(img, (x1, y1), (x2, y2), (255, 0, 255), 3)

            # Calculate width in cm
            wpix = getWidthPixel(x1, x2)
            w = mapWidth(wpix)
            wcm = str(f"{w} cm")

            # Classify detected objects
            cls = int(box.cls[0])
            if cls == 0:  # "Head"
                head_width_cm = w
            elif cls == 1:  # "Body"
                body_width_cm = w
            command = f"W{head_width_cm}\n"

            # Display width on the image
            org = [x1, y1 - 10]
            font = cv2.FONT_HERSHEY_SIMPLEX
            fontScale = 1
            color = (0, 255, 0)
            thickness = 2
            cv2.putText(img, wcm, org, font, fontScale, color, thickness)

            # Change line color if bounding box is near the center
            if x2 <= center_x + tolerance and x2 >= center_x - tolerance:
                vertical_line_color = (255, 0, 0)  # Blue line
                print(f"Send {command}")
                # ser.write(command.encode())

    # Draw vertical line on the frame
    cv2.line(img, (center_x, 0), (center_x, frame_height), vertical_line_color, 2)

    # Periodically save x1 and x2 to Excel
    if len(x_data) >= 50:  # Save every 50 entries
        df = pd.DataFrame(x_data)  # Convert to DataFrame
        df.to_excel(output_file, index=False)  # Save to Excel
        x_data = []  # Clear the list after saving

    # Show the image with detections
    cv2.imshow("Detection", img)

    # Exit the loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

# Save remaining data before exiting
if x_data:
    df = pd.DataFrame(x_data)
    df.to_excel(output_file, index=False)

cnn.release()
cv2.destroyAllWindows()
