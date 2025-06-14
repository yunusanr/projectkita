// -------------------------- Component Library -------------------------------- //
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <ModbusMaster.h>
#include <Adafruit_INA219.h>
#include <MAX6675_Thermocouple.h>
#include <Servo.h>
#include <EEPROM.h>

// ----------------------------- Fuel Sensor ---------------------------------- //
int TankValue;
const uint8_t fuelPin = A7;

// -------------------------------- Servo ------------------------------------- //
Servo myServo;
int servo = 3;
int pos = 0;
int varDeg;

// -------------------------------- Relay ------------------------------------- //
int relay1 = A0;
int relay2 = A1;
int relay3 = A2;


// ------------------------------ MAX_RS485 ----------------------------------- //
const uint8_t MAX485_DE = 7;
const uint8_t MAX485_RE_NEG = 6;
ModbusMaster node;
int startReg; //Reg for starting function when receive true value
int stopReg; //Reg for stopping function when receive true value
int mapDeg; //Reg for saving custom servo degree based on persentation (0% - 100%)
int changeDeg; //Move servo degree based on mapDeg when receive true value
int function1; //function 1 - 4 for further use
int function2;
int function3;
int function4;

// ------------------------------ RPM Sensor ---------------------------------- //
const int IR_PIN = 2;
volatile unsigned int counter = 0;
unsigned long previousMillis = 0;
unsigned long rpm = 0;
void Interrupt() {
  counter++;
}
int address = 0;
int odoH;
unsigned long startTime = 0;
bool isTiming = false;
int lastValue;
unsigned long elapsedTime;

// ------------------------------- MAX_6675 ---------------------------------- //
#define SCK_PIN 13
#define CS_PIN 9
#define CS_PIN2 10
#define SO_PIN 12
Thermocouple* thermocouple;
Thermocouple* thermocouple2;
void preTransmission()            //Function for setting state of Pins DE & RE of RS-485
{
  digitalWrite(MAX485_RE_NEG, 1);
  digitalWrite(MAX485_DE, 1);
}
void postTransmission()
{
  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);
}

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
  Serial.begin(9600);
  pinMode(MAX485_RE_NEG, OUTPUT);
  pinMode(MAX485_DE, OUTPUT);
  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);
  node.begin(1, Serial);
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);

  // ----- INA219 ----- //
  if (!ina219.begin()) {
    Serial.println("Failed to find INA219 chip at address 0x40");
  }

  // ----- Servo ------ //
  myServo.attach(servo);
  myServo.write(100);

  // ----- Relay ------ //
  pinMode(relay1, OUTPUT);
  pinMode(relay2, OUTPUT);
  pinMode(relay3, OUTPUT);
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);
  digitalWrite(relay1, HIGH);
  digitalWrite(relay2, HIGH);
  digitalWrite(relay3, HIGH);

  // ------ RPM ------- //
  pinMode(IR_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(IR_PIN), Interrupt, FALLING);
  EEPROM.get(address, lastValue);
  Serial.print("Nilai terakhir yang disimpan: ");
  Serial.println(lastValue);

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
  //  Serial.println("SETUP DONE");
}

void loop() {
  uint8_t result;
  uint16_t data[6];
  unsigned long currentMillis = millis();

  // ----- Fuel ------- //
  int fuel = analogRead(fuelPin);
  fuel = map(fuel, 548, 1024, 100, 0);

  // ------ RPM ------- //
  if (currentMillis - previousMillis >= 1000) {
    previousMillis = currentMillis;
    rpm = (counter / 2) * 60;
    counter = 0;
  }
  if (rpm >= 9000) {
    if (!isTiming) {
      startTime = millis();
      isTiming = true;
    }
    elapsedTime = (millis() - startTime) / 1000;
    Serial.print("Waktu berjalan (detik): ");
    Serial.println(elapsedTime);
    EEPROM.put(address, elapsedTime);
  } else {
    isTiming = false;
  }

  // -- Thermocouple -- //
  const double celsius = thermocouple->readCelsius();
  const double celsius2 = thermocouple2->readCelsius();

  // ----- INA219 ----- //
  current_mA = ina219.getCurrent_mA();

  // ----- Voltage ---- //
  adc_value = analogRead(ANALOG_IN_PIN);
  adc_voltage = (adc_value * ref_voltage) / 1024.0;
  in_voltage = adc_voltage / (R2 / (R1 + R2)) ;

  // ------ LCD ------- //
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

  // ----- RS485 ------ //
  node.writeSingleRegister(0x40000, in_voltage);
  node.writeSingleRegister(0x40001, current_mA);
  node.writeSingleRegister(0x40002, celsius);
  node.writeSingleRegister(0x40003, celsius2);
  node.writeSingleRegister(0x40004, rpm);
  node.writeSingleRegister(0x40005, fuel);

  result = node.readHoldingRegisters(0x40006, 4);
  startReg = node.getResponseBuffer(0);
  stopReg = node.getResponseBuffer(1);
  mapDeg = node.getResponseBuffer(2);
  changeDeg = node.getResponseBuffer(3);

  Serial.println("data start : " + String(startReg));
  Serial.println("data stop : " + String(stopReg));
  Serial.println("mapDeg : " + String(mapDeg));
  Serial.println("changeDeg : " + String(changeDeg));

  // ----- Check startReg and stopReg ----- //
  if (startReg == 1) {
    Serial.println("starting");
    Serial.println("relay 1 on");
    digitalWrite(13, HIGH);
    digitalWrite(relay1, LOW);
    delay(4000);
    Serial.println("servo berputar ke posisi 1000 rpm");
    for (pos = 100; pos >= 35; pos -= 1) {
      myServo.write(pos);
      delay(50);
    }
    delay(4000);
    Serial.println("relay 2 on");
    digitalWrite(relay2, LOW);
  }

  varDeg = map(mapDeg, 0, 100, 32, 10);
  if (changeDeg == 1) {
    Serial.println("change servo to : " + String(varDeg));
    myServo.write(varDeg);
  }

  if (stopReg == 1) {
    Serial.println("stopping");
    Serial.println("relay 2 off");
    digitalWrite(relay2, HIGH);
    digitalWrite(13, LOW);
    delay(4000);
    Serial.println("servo balik ke posisi off");
    for (pos = 35; pos <= 100; pos += 1) {
      myServo.write(pos);
      delay(50);
    }
    delay(4000);
    Serial.println("relay 1 off");
    digitalWrite(relay1, HIGH);
  }

  lcd.clear();
  delay(1000);
}
