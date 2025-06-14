#include <Servo.h>

Servo myServo;
int servo = 3;
int pos = 0;

const int relay1 = A0;
const int relay2 = A1;
const int ledPin = 13;

bool isRunning = false;
bool isStopping = false;
unsigned long previousMillis = 0;
const unsigned long stepDelay = 2000;  // Jeda antar step (2 detik)
int stepIndex = 0;

struct Step {
  void (*action)();
};

void startStep1() { Serial.println("Menyalakan relay1"); digitalWrite(relay1, LOW); }
void startStep2() { Serial.println("Menyalakan relay2"); for (pos = 70; pos <= 125; pos += 1) {
        myServo.write(pos);
        delay(10);
      } }
void startStep3() { Serial.println("Menyalakan relay3"); digitalWrite(relay2, LOW); }
void stopStep1()  { Serial.println("Mematikan relay3"); digitalWrite(relay2, HIGH); }
void stopStep2()  { Serial.println("Mematikan relay2"); for (pos = 125; pos >= 70; pos -= 1) {
      myServo.write(pos);
      delay(10);
    } }
void stopStep3()  { Serial.println("Mematikan relay1"); digitalWrite(relay1, HIGH); }

Step startSequence[] = { {startStep1}, {startStep2}, {startStep3} };
Step stopSequence[]  = { {stopStep1}, {stopStep2}, {stopStep3} };

void setup() {
  Serial.begin(9600);
  Serial.println("Starting........");
  pinMode(relay1, OUTPUT);
  pinMode(relay2, OUTPUT);
  myServo.attach(servo);

  digitalWrite(relay1, HIGH);
  digitalWrite(relay2, HIGH);
}

void loop() {
  Serial.println("still looping....");
  delay(500);
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    if (command == "start" && !isRunning) {
      Serial.println("Memulai Sequence START...");
      isRunning = true;
      isStopping = false;
      stepIndex = 0;
      previousMillis = millis();
    }
    if (command == "stop" && !isStopping) {
      Serial.println("Memulai Sequence STOP...");
      isRunning = false;
      isStopping = true;
      stepIndex = 0;
      previousMillis = millis();
    }
  }

  if (isRunning && stepIndex < 3) {
    if (millis() - previousMillis >= stepDelay) {
      previousMillis = millis();
      startSequence[stepIndex].action();
      stepIndex++;
      if (stepIndex >= 3) isRunning = false;
    }
  }

  if (isStopping && stepIndex < 3) {
    if (millis() - previousMillis >= stepDelay) {
      previousMillis = millis();
      stopSequence[stepIndex].action();
      stepIndex++;
      if (stepIndex >= 3) isStopping = false;
    }
  }
}
