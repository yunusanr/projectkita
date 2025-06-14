from ultralytics import YOLO
import cv2
import serial

# Initialize model
model = YOLO("Nila.pt")
classNames = ["Nila", "Head"]

# Initialize webcam
cnn = cv2.VideoCapture(0)
cnn.set(3, 640)  # Set width
cnn.set(4, 480)  # Set height


# Function to calculate width in pixels
def getWidthPixel(x1, x2):
    return abs(x2 - x1)


# Function to map width in pixels to cm
def mapWidth(wpix, from_min=0, from_max=640, to_min=0, to_max=45.5):
    percentage = (wpix - from_min) / (from_max - from_min)
    return (percentage * (to_max - to_min)) + to_min


while True:
    success, img = cnn.read()
    if not success:
        print("Failed to capture image")
        break

    # Process the frame with YOLO model
    results = model(img, stream=True)

    # Initialize variables for storing detected widths in cm
    head_width_cm = None
    body_width_cm = None

    for r in results:
        boxes = r.boxes
        for box in boxes:
            x1, y1, x2, y2 = box.xyxy[0]
            x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)

            # Draw bounding box
            cv2.rectangle(img, (x1, y1), (x2, y2), (255, 0, 255), 3)

            # Calculate width in pixels and convert to cm
            wpix = getWidthPixel(x1, x2)
            w = mapWidth(wpix)
            wcm_text = f"{w:.1f} cm"

            # Identify detected class (0: "Nila" / body, 1: "Head")
            cls = int(box.cls[0])

            # Store width in cm based on class
            if cls == 0:  # "head" (body)
                head_width_cm = w
            elif cls == 1:  # "body"
                body_width_cm = w

            # Display width text
            cv2.putText(
                img,
                wcm_text,
                (x1, y1 - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (0, 250, 0),
                2,
            )

    # Calculate difference if both head and body are detected
    if body_width_cm is not None and head_width_cm is not None:
        body_minus_head_cm = round(abs(body_width_cm - head_width_cm), 1)
        print(f"Body width: {body_width_cm} cm")
        print(f"Head width: {head_width_cm} cm")
        print(f"Difference (body - head): {body_minus_head_cm} cm")

    # Display the video feed with detection boxes and labels
    cv2.imshow("Detection", img)

    # Break loop on 'q' key press
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

# Release resources
cnn.release()
cv2.destroyAllWindows()
