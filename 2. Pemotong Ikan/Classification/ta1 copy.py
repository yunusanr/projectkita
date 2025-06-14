from ultralytics import YOLO
import cv2
import tkinter as tk
from PIL import Image

# import serial

# ser = serial.Serial("/dev/ttyACM0", 115200, timeout=1)
var = []

cnn = cv2.VideoCapture(1)
cnn.set(3, 640)
cnn.set(4, 480)

x2 = 0

model = YOLO("Nila.pt")

classNames = ["Nila", "Head"]
fishLen = []

config = dict(single_cls=True)


def getWidthPixel(x1, x2):
    wpix = x2 - x1
    wpix = abs(wpix)

    return wpix


def mapWidth(wpix, from_min=0, from_max=640, to_min=0, to_max=45.5):
    percentage = (wpix - from_min) / (from_max - from_min)
    w = (percentage * (to_max - to_min)) + to_min

    return w


while True:
    success, img = cnn.read()
    results = model(img, stream=True)
    count = 0

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

            # break

        fishLen.sort()
        if x2 >= 500 and x2 <= 502:
            print(fishLen)

        cv2.imshow("Detection", img)

        fishLen = []

    # ser.write([cls, w])

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cnn.release()
cv2.destroyAllWindows()
