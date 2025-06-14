import serial
import time
import random

# ser = serial.Serial("COM3", 115200)  # Ganti 'COM3' dengan port yang sesuai
ser = serial.Serial("/dev/ttyUSB0", 115200, timeout=2)
# time.sleep(2)  # Tunggu ESP32 terhubung

try:
    value = 0
    while True:
        # Baca respons dari ESP32
        # if ser.in_waiting > 0:
        #     response = ser.readline().decode().strip()
        #     print("Response from ESP32:", response)

        # Menghasilkan nilai float acak antara 10 dan 100
        # value = random.uniform(10, 100)
        value += 1.1
        print(f"Send {value}")
        command = f"W{value}\n"  # Format string dengan prefix "w" di depan float
        # Kirim perintah ke ESP32
        ser.write(command.encode())
        ser.flush()

        # Tambahkan delay jika diperlukan
        time.sleep(0.5)

        if value > 50:
            value = 0

except KeyboardInterrupt:
    print("Program stopped")
finally:
    ser.close()
