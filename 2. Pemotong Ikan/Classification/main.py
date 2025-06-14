from ultralytics import YOLO
import cv2
import serial

# esp = serial.Serial(port="COM3", baudrate=115200, timeout=2)

# Config model
model = YOLO("Nila.pt")
classNames = ["Nila", "Head"]

# Config webcam
cnn = cv2.VideoCapture(0)
cnn.set(3, 640)
cnn.set(4, 480)

# Utility
config = dict(single_cls=True)
fishLen = []
x2 = 0
tolerance = 10  # pixel

# Calibration
calibration_data = [
    (15, 25),
]
calib_factor = [actual / detected for actual, detected in calibration_data]
CALIBRATION_FACTOR = sum(calib_factor) / len(calib_factor)


# Calc width in pixel
def getWidthPixel(x1, x2):
    wpix = x2 - x1
    wpix = abs(wpix)
    return wpix


# Map pixel width to cm
def mapWidth(wpix, from_min=0, from_max=640, to_min=0, to_max=45.5):
    to_max_calibrated = to_max * CALIBRATION_FACTOR
    percentage = (wpix - from_min) / (from_max - from_min)
    w = (percentage * (to_max_calibrated - to_min)) + to_min
    return w


while True:
    success, img = cnn.read()
    if not success:
        print("Failed to capture image")
        break

    # Process the frame with YOLO model
    results = model(img, stream=True)
    count = 0
    head_width_cm = None

    # Vertical line
    frame_width = img.shape[1]
    frame_height = img.shape[0]
    center_x = frame_width // 2
    vertical_line_color = (0, 255, 0)  # Hijau

    for r in results:
        boxes = r.boxes

        for box in boxes:
            x1, y1, x2, y2 = box.xyxy[0]
            x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)

            cv2.rectangle(img, (x1, y1), (x2, y2), (255, 0, 255), 3)

            wpix = getWidthPixel(x1, x2)
            w = mapWidth(wpix)
            wcm = str(f"{w} cm")

            cls = int(box.cls[0])
            if cls == 0:  # "head"
                head_width_cm = round(w, 1)

            org = [x1, y1 - 10]
            font = cv2.FONT_HERSHEY_SIMPLEX
            fontScale = 1
            color = (0, 250, 0)
            thickness = 2

            cv2.putText(img, wcm, org, font, fontScale, color, thickness)

            if x2 <= center_x + tolerance and x2 >= center_x - tolerance:
                vertical_line_color = (
                    255,
                    0,
                    0,
                )  # Ubah warna menjadi biru jika sejajar

            fishLen.append(w)
            count += 1

            if count == 2:
                count = 0
                break

            # break

        fishLen.sort()
        if x2 >= 500 and x2 <= 502:
            print(fishLen)

        cv2.imshow("Detection", img)

        fishLen = []

    # ser.write([cls, w])
    # Gambar garis vertikal di tengah
    cv2.line(img, (center_x, 0), (center_x, frame_height), vertical_line_color, 2)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cnn.release()
cv2.destroyAllWindows()
