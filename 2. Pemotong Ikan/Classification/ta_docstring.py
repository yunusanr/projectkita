from ultralytics import YOLO
import cv2
import tkinter as tk
from PIL import Image
import time
import serial

# Inisialisasi koneksi serial
ser = serial.Serial(port="COM3", baudrate=115200, timeout=0.1)

# Variabel global untuk menyimpan data sementara
var = []

# Inisialisasi kamera
cnn = cv2.VideoCapture(0)  # Kamera dengan ID 1
cnn.set(3, 640)  # Resolusi lebar
cnn.set(4, 480)  # Resolusi tinggi

# Variabel tambahan
x2 = 0  # Variabel untuk posisi horizontal objek
tolerance = 200  # Toleransi deteksi di sekitar garis tengah
head_width_cm = 0.0  # Lebar kepala dalam cm
body_width_cm = 0.0  # Lebar tubuh dalam cm

# Muat model YOLO untuk deteksi objek
model = YOLO("best.pt")

# Nama kelas yang digunakan dalam model YOLO
classNames = ["Nila", "Head"]

# List untuk menyimpan panjang ikan
fishLen = []

# Konfigurasi YOLO
config = dict(single_cls=True)

# Data kalibrasi: pasangan nilai (deteksi pixel, nilai sebenarnya dalam cm)
calibration_data = [(20, 35)]

# Hitung faktor kalibrasi berdasarkan data kalibrasi
calib_factor = [actual / detected for actual, detected in calibration_data]
CALIBRATION_FACTOR = sum(calib_factor) / len(calib_factor)


# Fungsi untuk menghitung lebar objek dalam pixel
def getWidthPixel(x1, x2):
    wpix = abs(x2 - x1)
    return wpix


# Fungsi untuk mengonversi lebar dalam pixel menjadi cm, dengan faktor kalibrasi
def mapWidth(wpix, from_min=0, from_max=640, to_min=0, to_max=45.5):
    to_max_calibrated = to_max * CALIBRATION_FACTOR  # Sesuaikan dengan faktor kalibrasi
    percentage = (wpix - from_min) / (from_max - from_min)
    w = (percentage * (to_max_calibrated - to_min)) + to_min
    return w


# Loop utama untuk membaca frame dari kamera dan mendeteksi objek
while True:
    success, img = cnn.read()  # Ambil frame dari kamera
    results = model(img, stream=True)  # Deteksi objek dengan YOLO
    count = 0  # Hitung jumlah deteksi

    # Garis tengah vertikal
    frame_height = img.shape[0]
    frame_width = img.shape[1]
    center_x = frame_width // 2

    # Warna awal garis vertikal (hijau)
    vertical_line_color = (0, 255, 0)

    for r in results:  # Iterasi hasil deteksi
        boxes = r.boxes  # Dapatkan bounding box

        for box in boxes:
            # Koordinat bounding box
            x1, y1, x2, y2 = box.xyxy[0]
            x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)

            # Gambar bounding box pada frame
            cv2.rectangle(img, (x1, y1), (x2, y2), (255, 0, 255), 3)

            # Hitung lebar objek dalam pixel dan konversi ke cm
            wpix = getWidthPixel(x1, x2)
            w = mapWidth(wpix)
            wcm = str(f"{w} cm")

            # Identifikasi kelas objek
            cls = int(box.cls[0])
            if cls == 0:  # Jika objek adalah kepala
                head_width_cm = w

            # Format perintah untuk dikirim via serial
            command = f"W{head_width_cm}\n"

            # Tampilkan ukuran pada gambar
            org = [x1, y1 - 10]
            font = cv2.FONT_HERSHEY_SIMPLEX
            fontScale = 1
            color = (0, 250, 0)
            thickness = 2
            cv2.putText(img, wcm, org, font, fontScale, color, thickness)

            # Jika bounding box berada di sekitar garis tengah
            if center_x - tolerance <= x2 <= center_x + tolerance:
                vertical_line_color = (255, 0, 0)  # Ubah warna garis ke biru
                print(f"Send {command}")
                ser.write(command.encode())  # Kirim data via serial

            # Simpan panjang ikan
            fishLen.append(w)

            count += 1
            if count == 2:  # Batasi hanya untuk dua deteksi
                break

        # Gambar garis vertikal
        cv2.line(img, (center_x, 0), (center_x, frame_height), vertical_line_color, 2)

        # Tampilkan panjang ikan jika berada dalam rentang tertentu
        fishLen.sort()
        if 500 <= x2 <= 502:
            print(fishLen)

        # Tampilkan gambar dengan bounding box
        cv2.imshow("Detection", img)

        fishLen = []  # Reset daftar panjang ikan

    # Keluar dari loop jika tombol "q" ditekan
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

# Lepaskan kamera dan tutup koneksi serial
cnn.release()
cv2.destroyAllWindows()
