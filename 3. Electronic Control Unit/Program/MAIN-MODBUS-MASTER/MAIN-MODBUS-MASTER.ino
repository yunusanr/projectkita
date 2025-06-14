// -------------------------- Component Library -------------------------------- //
#include <ModbusRTUSlave.h>
#include <Adafruit_INA219.h>
#include <MAX6675_Thermocouple.h>
#include <Servo.h>
#include <EEPROM.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// ----------------------------- Fuel Sensor ---------------------------------- //
const uint8_t fuelPin = A7;

// -------------------------------- Servo ------------------------------------- //
Servo myServo;
int servo = 3;
int pos = 0;
int varDeg;
int currentDeg;

// -------------------------------- Relay ------------------------------------- //
int relay1 = A0;
int relay2 = A1;
int relay3 = A2;

// ------------------------------ MAX_RS485 ----------------------------------- //
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__) || defined(ARDUINO_SAM_DUE)
#define MODBUS_SERIAL Serial
#else
#define MODBUS_SERIAL Serial1
#endif
#define MODBUS_BAUD 9600
#define MODBUS_CONFIG SERIAL_8N1
#define MODBUS_UNIT_ID 1
const int8_t dePin = 7;
const int8_t rePin = 6;
ModbusRTUSlave modbus(MODBUS_SERIAL, dePin);
const uint8_t numHoldingRegisters = 14;
uint16_t holdingRegisters[numHoldingRegisters];
int startReg; //Reg for starting function when receive true value
int stopReg; //Reg for stopping function when receive true value
int mapDeg; //Reg for saving custom servo degree based on persentation (0% - 100%)
int changeDeg; //Move servo degree based on mapDeg when receive true value
int engineStart;
int engineStop;
bool isRunning = false;
bool isStopping = false;
bool hasStopped = true;
bool START = false;
bool STOP = false;
bool hasStarted = false;
int failCounter;
const unsigned long stepDelay = 3000;  // Jeda antar step (2 detik)
int stepIndex = 0;
struct Step {
  void (*action)();
};
void startStep1() {
  digitalWrite(relay1, LOW);
}
void startStep2() {
  for (pos = 125; pos >= 70; pos -= 1) {
    myServo.write(pos);
    delay(10);
  }
}
void startStep3() {
  digitalWrite(relay2, LOW);
}
void startStep4() {
  digitalWrite(relay2, HIGH);
}
void stopStep1()  {
  if (currentDeg != 0) {
    for (pos = currentDeg; pos <= 125; pos += 1) {
      myServo.write(pos);
      delay(50);
    }
  } else {
    for (pos = 70; pos <= 125; pos += 1) {
      myServo.write(pos);
      delay(50);
    }
  }
}
void stopStep2()  {
  digitalWrite(relay1, HIGH);
}

Step startSequence[] = { {startStep1}, {startStep2}, {startStep3}, {startStep4} };
Step stopSequence[]  = { {stopStep1}, {stopStep2}};

// ------------------------------ RPM Sensor ---------------------------------- //
const int IR_PIN = 2;
volatile unsigned int counter = 0;
unsigned long previousMillis = 0;
unsigned int rpm = 0;
void Interrupt() {
  counter++;
}
int address = 0;
int totalRun;
int odoH;
unsigned long startTime = 0;
bool isTiming = false;
unsigned long elapsedTime;
unsigned long elapsedTimeInHours;

// ------------------------------- MAX_6675 ---------------------------------- //
#define SCK_PIN 13
#define CS_PIN 9
#define CS_PIN2 10
#define SO_PIN 12
Thermocouple* thermocouple;
Thermocouple* thermocouple2;

// -------------------------------- INA_219 ---------------------------------- //
Adafruit_INA219 ina219(0x40);
float current_mA = 0;

// ----------------------------- Voltage Sensor ------------------------------ //
const uint8_t ANALOG_IN_PIN = A6;
float adc_voltage = 0.0;
float in_voltage = 0.0;
float R1 = 30000.0;
float R2 = 7500.0;
float ref_voltage = 5.0;
int adc_value = 0;

// -------------------------------- LCD_I2C --------------------------------- //
LiquidCrystal_I2C lcd(0x27, 20, 4);
unsigned long previousLCDMillis = 0;
const long lcdInterval = 3000;  // Update LCD setiap 1000 ms (1 detik)
byte fullBlock[8] = { 0b11111, 0b11111, 0b11111, 0b11111, 0b11111, 0b11111, 0b11111, 0b11111 };
byte emptyBlock[8] = { 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000 };
void bootScreen()
{
  lcd.setCursor(5, 1);
  lcd.print("ELECTRICAL");
  lcd.setCursor(4, 2);
  lcd.print("CONTROL UNIT");
  delay(1000);
  lcd.clear();
  lcd.createChar(0, fullBlock);
  lcd.createChar(1, emptyBlock);
  lcd.setCursor(1, 1);
  lcd.print("LOADING...");

  lcd.setCursor(0, 2);
  lcd.write(byte(1));
  for (int i = 1; i < 19; i++) {
    lcd.write(byte(1));
  }
  lcd.write(byte(1));

  for (int progress = 1; progress <= 18; progress++)
  {
    lcd.setCursor(progress, 2);
    lcd.write(byte(0));
    delay(150);
  }
  delay(1000);
  lcd.clear();
}

void setup() {
  // ----- RS485 ------ //
  pinMode(13, OUTPUT);
  pinMode(dePin, OUTPUT);
  pinMode(rePin, OUTPUT);
  digitalWrite(dePin, LOW);  // Nonaktifkan mode TX
  digitalWrite(rePin, LOW);  // Aktifkan mode RX
  modbus.configureHoldingRegisters(holdingRegisters, numHoldingRegisters);
  MODBUS_SERIAL.begin(MODBUS_BAUD, MODBUS_CONFIG);
  modbus.begin(MODBUS_UNIT_ID, MODBUS_BAUD, MODBUS_CONFIG);

  // ----- INA219 ----- //
  ina219.begin();

  // ----- Servo ------ //
  myServo.attach(servo);
  myServo.write(125);

  // ----- Relay ------ //
  pinMode(relay1, OUTPUT);
  pinMode(relay2, OUTPUT);
  pinMode(relay3, OUTPUT);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  digitalWrite(relay1, HIGH);
  digitalWrite(relay2, HIGH);
  digitalWrite(relay3, HIGH);

  // ------ RPM ------- //
  pinMode(IR_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(IR_PIN), Interrupt, FALLING);
  EEPROM.get(address, odoH);

  // ----- Fuel ------- //
  pinMode(fuelPin, INPUT_PULLUP);

  // -- Thermocouple -- //
  thermocouple = new MAX6675_Thermocouple(SCK_PIN, CS_PIN, SO_PIN);
  thermocouple2 = new MAX6675_Thermocouple(SCK_PIN, CS_PIN2, SO_PIN);

  // ----- Voltage ---- //
  pinMode(ANALOG_IN_PIN, INPUT);

  // ------ LCD ------- //
  lcd.begin();
  lcd.backlight();
  bootScreen();
  lcd.clear();
}

void loop() {
  unsigned long currentMillis = millis();

  // ----- Fuel ------- //
  int fuel = analogRead(fuelPin);
  fuel = map(fuel, 860, 1024, 100, 0);


  // ------ RPM ------- //
  if (currentMillis - previousMillis >= 1000) {
    previousMillis = currentMillis;
    rpm = (counter / 3) * 60;
    counter = 0;
  }
  if (rpm >= 120) {
    if (!isTiming) {
      startTime = millis();
      isTiming = true;
    }
    elapsedTime = (millis() - startTime) / 1000;
    elapsedTimeInHours = elapsedTime / 3600;
    totalRun = elapsedTimeInHours +  odoH;
    EEPROM.put(address, totalRun);
  } else {
    isTiming = false;
  }

  // -- Thermocouple -- //
  double celsius = thermocouple->readCelsius();
  double celsius2 = thermocouple2->readCelsius();
  if (isnan(celsius)) {
    celsius = 0;
  }
  if (isnan(celsius2)) {
    celsius2 = 0;
  }

  // ----- INA219 ----- //
  current_mA = ina219.getCurrent_mA();

  // ----- Voltage ---- //
  adc_value = analogRead(ANALOG_IN_PIN);
  adc_voltage = (adc_value * ref_voltage) / 1024.0;
  in_voltage = adc_voltage / (R2 / (R1 + R2)) ;

  // ------ LCD ------- //
  if (currentMillis - previousLCDMillis >= lcdInterval) {
    previousLCDMillis = currentMillis;
    lcd.clear();
    lcd.setCursor(8, 0);
    lcd.print("ECU");
    lcd.setCursor(0, 1);
    lcd.print("V:" + String(in_voltage) + "V");
    lcd.setCursor(0, 2);
    lcd.print("I:" + String(current_mA) + "A");
    lcd.setCursor(9, 1);
    lcd.print("tmp1:" + String(celsius) + "C");
    lcd.setCursor(9, 2);
    lcd.print("tmp2:" + String(celsius2) + "C");
    lcd.setCursor(0, 3);
    lcd.print("RPM:" + String(rpm) + "rpm" + " Fuel:" + String(fuel) + "%");
  }

  // ----- RS485 ------ //
  digitalWrite(dePin, HIGH);  // Aktifkan mode TX (transmit)
  digitalWrite(rePin, HIGH);  // Nonaktifkan mode RX
  holdingRegisters[0] = in_voltage;
  holdingRegisters[1] = current_mA;
  holdingRegisters[2] = celsius;
  holdingRegisters[3] = celsius2;
  holdingRegisters[4] = rpm;
  holdingRegisters[5] = fuel;
  holdingRegisters[6] = totalRun;
  holdingRegisters[7] = elapsedTimeInHours;
  delay(10);
  digitalWrite(dePin, LOW);
  digitalWrite(rePin, LOW);
  modbus.poll();
  startReg = holdingRegisters[8];
  stopReg = holdingRegisters[9];
  mapDeg = holdingRegisters[10];
  changeDeg = holdingRegisters[11];

  // ----- Check startReg and stopReg ----- //
  if (startReg == 1 && !isRunning && !isStopping && hasStopped && START == false) {
    isRunning = true;
    isStopping = false;
    hasStopped = false; // Mencegah start berulang sebelum stop dilakukan
    currentDeg = 0;
    stepIndex = 0;
    previousMillis = millis();
  }
  if (START == true && startReg == 0) {
    START = false;
  }

  varDeg = map(mapDeg, 0, 100, 69, 35);
  if (hasStarted == true) {
    if (currentDeg == 0) {
      for (pos = 70; pos <= varDeg; pos -= 1) {
        myServo.write(pos);
        delay(10);
      }
      currentDeg = varDeg;
    } else if (currentDeg < varDeg){
      for (pos = currentDeg; pos >= varDeg; pos += 1) {
        myServo.write(pos);
        delay(10);
      }
      currentDeg = varDeg;
    } else if (currentDeg > varDeg){
      for (pos = currentDeg; pos <= varDeg; pos -= 1) {
        myServo.write(pos);
        delay(10);
      }
    }
  }

  if (stopReg == 1 && !isStopping && !isRunning && STOP == false) {
    isRunning = false;
    isStopping = true;
    stepIndex = 0;
    previousMillis = millis();
  }
  if (STOP == true && stopReg == 0) {
    STOP = false;
  }

  if (isRunning && stepIndex < 3) {
    if (millis() - previousMillis >= stepDelay) {
      previousMillis = millis();
      startSequence[stepIndex].action();
      stepIndex++;
      if (stepIndex >= 4) {
        isRunning = false;
        START = true;
        hasStarted = true;
      }
    }
  }
  if (isStopping && stepIndex < 3) {
    if (millis() - previousMillis >= stepDelay) {
      previousMillis = millis();
      stopSequence[stepIndex].action();
      stepIndex++;
      if (stepIndex >= 2) {
        isStopping = false;
        hasStopped = true;
        hasStarted = false;
        STOP = true;
      }
    }
  }
}
