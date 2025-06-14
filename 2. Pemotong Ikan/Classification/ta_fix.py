from ultralytics import YOLO
import cv2
import tkinter as tk
from PIL import Image
import time

import serial

# ser = serial.Serial(port="COM3", baudrate=115200, timeout=0.1)
var = []

cnn = cv2.VideoCapture(1)
cnn.set(3, 640)
cnn.set(4, 480)
# Attempt to set exposure
cnn.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)  # 0.25 to turn off auto exposure
cnn.set(
    cv2.CAP_PROP_EXPOSURE,
    -5,
)  # Set to a negative value; higher values for less exposure


x2 = 0
tolerance = 200
head_width_cm = 0.0
body_width_cm = 0.0

model = YOLO("best.pt")

classNames = ["Nila", "Head"]
fishLen = []

config = dict(single_cls=True)

# Calibration
calibration_data = [
    (20, 35),
]
calib_factor = [actual / detected for actual, detected in calibration_data]
CALIBRATION_FACTOR = sum(calib_factor) / len(calib_factor)


def getWidthPixel(x1, x2):
    wpix = x2 - x1
    wpix = abs(wpix)

    return wpix


# def mapWidth(wpix, from_min=0, from_max=640, to_min=0, to_max=45.5):
#     percentage = (wpix - from_min) / (from_max - from_min)
#     w = (percentage * (to_max - to_min)) + to_min

#     return w


# Map pixel width to cm
def mapWidth(wpix, from_min=0, from_max=640, to_min=0, to_max=45.5):
    to_max_calibrated = to_max * CALIBRATION_FACTOR
    percentage = (wpix - from_min) / (from_max - from_min)
    w = (percentage * (to_max_calibrated - to_min)) + to_min
    return w


while True:
    success, img = cnn.read()
    results = model(img, stream=True)
    count = 0

    # Garis vertikal dan horizontal (tengah)
    frame_height = img.shape[0]
    frame_width = img.shape[1]
    center_x = frame_width // 2

    # Tentukan warna awal garis vertikal (hijau)
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
                head_width_cm = w
            elif cls == 1:  # "body"
                body_width_cm = w
            command = f"W{head_width_cm}\n"

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
                print(f"Send {command}")
                # ser.write(command.encode())

            fishLen.append(w)

            count += 1

            if count == 2:
                count = 0
                break

            # break
        cv2.line(img, (center_x, 0), (center_x, frame_height), vertical_line_color, 2)

        fishLen.sort()
        if x2 >= 500 and x2 <= 502:
            print(fishLen)

        cv2.imshow("Detection", img)

        fishLen = []

    # ser.write([cls, w])

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cnn.release()
# ser.close()
cv2.destroyAllWindows()
