void setup() {
  // Mengatur pin 2 hingga 7 sebagai OUTPUT
  for (int i = 2; i <= 7; i++) {
    pinMode(i, OUTPUT);
  }
}

void loop() {
  // Menghidupkan semua pin dari 2 hingga 7
  for (int i = 2; i <= 7; i++) {
    digitalWrite(i, HIGH);
  }
  delay(1000); // Tunggu selama 1 detik

  // Mematikan semua pin dari 2 hingga 7
  for (int i = 2; i <= 7; i++) {
    digitalWrite(i, LOW);
  }
  delay(1000); // Tunggu selama 1 detik sebelum mengulangi
}
