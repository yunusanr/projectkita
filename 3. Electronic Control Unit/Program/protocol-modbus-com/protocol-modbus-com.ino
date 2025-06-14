#include <ModbusRTUSlave.h>

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
int data1 = 0;
int data2 = 0;
int data3 = 0;
int data4 = 0;
int data5 = 0;

void setup() {
  // ----- RS485 ------ //
  pinMode(dePin, OUTPUT);
  pinMode(rePin, OUTPUT);
  digitalWrite(dePin, LOW);  // Nonaktifkan mode TX
  digitalWrite(rePin, LOW);  // Aktifkan mode RX

  modbus.configureHoldingRegisters(holdingRegisters, numHoldingRegisters);
  MODBUS_SERIAL.begin(MODBUS_BAUD, MODBUS_CONFIG);
  modbus.begin(MODBUS_UNIT_ID, MODBUS_BAUD, MODBUS_CONFIG);
}

void loop() {
  // ----- RS485 ------ //
  digitalWrite(dePin, HIGH);  // Aktifkan mode TX (transmit)
  digitalWrite(rePin, HIGH);  // Nonaktifkan mode RX

  // Update holding registers with sensor data
  holdingRegisters[0] = in_voltage;
  holdingRegisters[1] = current_mA;
  holdingRegisters[2] = celsius;
  holdingRegisters[3] = celsius2;
  holdingRegisters[4] = rpm;
  holdingRegisters[5] = fuel;
  holdingRegisters[6] = totalRun;
  holdingRegisters[7] = elapsedTime;

  delay(10);
  digitalWrite(dePin, LOW);
  digitalWrite(rePin, LOW);

  // Poll for Modbus requests
  modbus.poll();

  // Read values from holding registers
  startReg = holdingRegisters[8];
  stopReg = holdingRegisters[9];
  mapDeg = holdingRegisters[10];
  changeDeg = holdingRegisters[11];


}
