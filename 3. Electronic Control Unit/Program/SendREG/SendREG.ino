#include <ModbusRtu.h>

const uint8_t RE_PIN = 4;  // Receiver Enable
const uint8_t DE_PIN = 5;  // Driver Enable
const int LED = 13;        // LED untuk status

// Data array untuk menyimpan register Modbus
uint16_t modbus_array[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
// Buat objek Modbus slave
Modbus bus(1, Serial, DE_PIN); // Slave ID = 1, Serial port, DE_PIN untuk kontrol RS485

void setup() {
  pinMode(RE_PIN, OUTPUT);
  pinMode(DE_PIN, OUTPUT);
  pinMode(LED, OUTPUT);

  digitalWrite(RE_PIN, LOW);  // Default ke mode Receive
  digitalWrite(DE_PIN, LOW);  // Default ke mode Receive

  // Inisialisasi komunikasi Modbus
  bus.begin(9600); // Baud rate 9600 bps
}

void loop() {
  // Simulasikan data untuk register
  modbus_array[0] = random(0, 2);    // Register pertama (0 atau 1)
  modbus_array[1] = 42;              // Register kedua (konstan)
  modbus_array[2] = random(0, 100);  // Register ketiga (random 0–100)
  modbus_array[3] = random(0, 2);    // Register pertama (0 atau 1)
  modbus_array[4] = 42;              // Register kedua (konstan)
  modbus_array[5] = random(0, 100);  // Register ketiga (random 0–100)
  

  // Handle polling dari master
  bus.poll(modbus_array, sizeof(modbus_array) / sizeof(modbus_array[0]));

  // Kontrol LED berdasarkan register pertama
  if (modbus_array[6] == 1) {
    digitalWrite(LED, HIGH); // Nyalakan LED
  } else {
    digitalWrite(LED, LOW);  // Matikan LED
  }

  delay(1000); // Delay 1 detik
}
