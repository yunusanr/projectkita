from ultralytics import YOLO
import cv2
import tkinter as tk
from PIL import Image
import serial

# ser = serial.Serial("/dev/ttyACM0", 115200, timeout=1)
# esp = serial.Serial(port="COM3", baudrate=115200, timeout=0.5)

var = []
cnn = cv2.VideoCapture(1)
cnn.set(3, 640)
cnn.set(4, 480)

x2 = 0
model = YOLO("Nila.pt")
classNames = ["Nila", "Head"]
fishLen = []

config = dict(single_cls=True)


# Fungsi untuk menghitung lebar piksel
def getWidthPixel(x1, x2):
    wpix = x2 - x1
    wpix = abs(wpix)
    return wpix


CALIBRATION_FACTOR = 15 / 25


# # Fungsi untuk mapping lebar dalam satuan cm
# def mapWidth(wpix, from_min=0, from_max=640, to_min=0, to_max=45.5):
#     percentage = (wpix - from_min) / (from_max - from_min)
#     w = (percentage * (to_max - to_min)) + to_min
#     return w


# Function to map width in pixels to cm with calibration
def mapWidth(wpix, from_min=0, from_max=640, to_min=0, to_max=45.5):
    to_max_calibrated = to_max * CALIBRATION_FACTOR  # Apply calibration to `to_max`
    percentage = (wpix - from_min) / (from_max - from_min)
    return (percentage * (to_max_calibrated - to_min)) + to_min


# Toleransi untuk deteksi sejajar
tolerance = 10  # dalam piksel

while True:
    success, img = cnn.read()
    results = model(img, stream=True)
    count = 0

    # Garis vertikal dan horizontal (tengah)
    frame_width = img.shape[1]
    frame_height = img.shape[0]
    center_x = frame_width // 2
    center_y = frame_height // 2

    # Tentukan warna awal garis vertikal (hijau)
    vertical_line_color = (0, 255, 0)  # Hijau

    for r in results:
        boxes = r.boxes

        for box in boxes:
            x1, y1, x2, y2 = box.xyxy[0]
            x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)

            # Gambar box
            cv2.rectangle(img, (x1, y1), (x2, y2), (255, 0, 255), 3)

            # Hitung lebar objek dalam piksel dan ubah ke cm
            wpix = getWidthPixel(x1, x2)
            w = mapWidth(wpix)
            wcm = str(f"{w} cm")

            cls = int(box.cls[0])
            org = [x1, y1 - 10]
            font = cv2.FONT_HERSHEY_SIMPLEX
            fontScale = 1
            color = (0, 250, 0)
            thickness = 2

            # Tampilkan lebar objek dalam cm
            cv2.putText(img, wcm, org, font, fontScale, color, thickness)

            # Cek apakah garis vertikal sejajar dengan box
            # Toleransi untuk pergeseran posisi box dengan garis vertikal
            if (x1 <= center_x + tolerance and x1 >= center_x - tolerance) or (
                x2 <= center_x + tolerance and x2 >= center_x - tolerance
            ):
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

        fishLen.sort()
        if x2 >= 500 and x2 <= 502:
            print(fishLen)

    # Gambar garis vertikal di tengah
    cv2.line(img, (center_x, 0), (center_x, frame_height), vertical_line_color, 2)

    # Gambar garis horizontal di tengah
    # cv2.line(img, (0, center_y), (frame_width, center_y), (0, 255, 0), 2)

    # Tampilkan frame dengan deteksi
    cv2.imshow("Detection", img)

    # Reset fishLen untuk frame berikutnya
    fishLen = []

    # Keluar jika tombol 'q' ditekan
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cnn.release()
cv2.destroyAllWindows()
