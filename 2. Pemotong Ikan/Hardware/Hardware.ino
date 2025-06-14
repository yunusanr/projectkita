#include <AccelStepper.h>

// Definisi pin untuk stepper motor, solenoid, dan sensor proximity
const uint8_t PULSE_PIN = 27;
const uint8_t DIRECTION_PIN = 26;
const uint8_t ENABLE_PIN = 25;
const uint8_t SOLENOID_PIN = 32;

// Inisialisasi objek stepper motor dengan driver step/direction
AccelStepper stepper(1, PULSE_PIN, DIRECTION_PIN);

// Variabel global
int threshold = 25;            // Batas nilai untuk mendeteksi objek (tester)
bool trigger_stepper = false;  // Apakah stepper perlu bergerak
char cmd_receive;              // Karakter yang diterima melalui komunikasi serial
float rec_width = 2.0;         // Lebar yang diterima melalui serial (default 2 cm)
int steps;                     // Jumlah langkah untuk stepper motor

void setup() {
  Serial.begin(115200);  // Inisialisasi komunikasi serial
  delay(2000);           // Tunggu 2 detik untuk stabilisasi

  // Konfigurasi pin input/output
  pinMode(SOLENOID_PIN, OUTPUT);

  // Inisialisasi stepper motor
  init_stepper();
  Serial.println("Setup completed");
}

void loop() {
  // Jika stepper perlu digerakkan
  if (trigger_stepper) {
    move_to_cut_pos();        // Gerakkan stepper ke posisi pemotongan
    perform_cutting();        // Aktifkan solenoid untuk memotong
    homing();                 // Kembalikan stepper ke posisi awal
    trigger_stepper = false;  // Reset status stepper
  } else {
    check_serial();  // Periksa input dari serial
    Serial.print("received: ");
    Serial.println(rec_width);  // Cetak lebar yang diterima
  }
}

// Inisialisasi parameter stepper motor
void init_stepper() {
  stepper.setMaxSpeed(1600);      // Kecepatan maksimum stepper
  stepper.setAcceleration(1600);  // Akselerasi stepper
  stepper.setCurrentPosition(0);  // Set posisi awal ke 0
  stepper.disableOutputs();       // Matikan output stepper
  Serial.println("Initialize stepper done");
  delay(500);  // Tunggu setengah detik
}

// Konversi lebar (cm) menjadi jumlah langkah stepper motor
int width_to_steps(float width) {
  int one_cm_steps = 1700;           // Langkah untuk setiap cm
  return int(one_cm_steps * width);  // Hitung total langkah
}

// Gerakkan stepper ke posisi pemotongan berdasarkan lebar yang diterima
void move_to_cut_pos() {
  steps = width_to_steps(rec_width);  // Hitung langkah berdasarkan lebar
  Serial.print("Moved: ");
  Serial.println(rec_width);
  Serial.print("Steps to move: ");
  Serial.println(steps);

  stepper.setMaxSpeed(1600);  // Set kecepatan maksimum
  stepper.move(steps);        // Tentukan langkah yang harus ditempuh
  stepper.enableOutputs();    // Aktifkan output stepper

  // Jalankan stepper hingga mencapai posisi yang diinginkan
  while (stepper.distanceToGo() != 0) {
    stepper.run();
  }
}

// Kembalikan stepper ke posisi awal (homing)
void homing() {
  Serial.println("Homing");

  // Jika stepper sudah di posisi awal
  if (stepper.currentPosition() == 0) {
    Serial.println("We are at the home position.");
    stepper.disableOutputs();  // Matikan output stepper
  } else {
    // Gerakkan stepper ke posisi 0
    stepper.setMaxSpeed(1600);
    stepper.moveTo(0);
    while (stepper.distanceToGo() != 0) {
      stepper.run();
    }
  }
}

// Aktifkan solenoid untuk melakukan pemotongan
void perform_cutting() {
  delay(500);                        // Tunggu sebelum memulai
  digitalWrite(SOLENOID_PIN, HIGH);  // Aktifkan solenoid
  delay(1500);                       // Tunggu selama 1,5 detik
  digitalWrite(SOLENOID_PIN, LOW);   // Matikan solenoid
  delay(1500);                       // Tunggu sebelum diulang
  digitalWrite(SOLENOID_PIN, HIGH);
  delay(1500);
  digitalWrite(SOLENOID_PIN, LOW);
  delay(2000);  // Tunggu setelah selesai
}

// Periksa apakah ada objek yang terdeteksi oleh sensor proximity
void detect_object() {
  int is_detected = digitalRead(PROXIMITY_PIN);  // Baca status sensor

  // Jika sensor mendeteksi objek dan belum terdeteksi sebelumnya
  if (is_detected == LOW && !detected) {
    trigger_stepper = true;  // Aktifkan stepper
    detected = true;         // Tandai bahwa objek telah terdeteksi
  } else if (is_detected == HIGH && detected) {
    detected = false;  // Reset status deteksi jika objek tidak lagi terdeteksi
  }
}

// Periksa perintah yang diterima melalui komunikasi serial
void check_serial() {
  if (Serial.available() > 0) {   // Jika ada data yang masuk
    cmd_receive = Serial.read();  // Baca karakter perintah

    // Proses perintah yang diterima
    switch (cmd_receive) {
      case 'W':  // Jika perintah adalah 'W', baca lebar (float)
        rec_width = Serial.parseFloat();
        break;

      case 'R':                         // Jika perintah adalah 'R', reset posisi stepper ke 0
        stepper.disableOutputs();       // Matikan output stepper
        stepper.setCurrentPosition(0);  // Set posisi awal ke 0
        Serial.print("The current position is updated to: ");
        Serial.println(stepper.currentPosition());
        break;

      default:  // Abaikan perintah yang tidak dikenal
        break;
    }
  }
}
