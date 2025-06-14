#include <EEPROM.h>  
int address = 0; 
int sensorValue;
int rpm = 9000;
unsigned long startTime = 0; 
bool isTiming = false;

void setup() {  
  Serial.begin(9600);
  int lastValue;  
  EEPROM.get(address, lastValue);  
  Serial.print("Nilai terakhir yang disimpan: ");  
  Serial.println(lastValue);  
}  
  
void loop() {  
    if (rpm >= 9000) {  
        if (!isTiming) {  
            startTime = millis();  
            isTiming = true;
        }  
        unsigned long elapsedTime = (millis() - startTime) / 1000;
        Serial.print("Waktu berjalan (detik): ");  
        Serial.println(elapsedTime);  
        EEPROM.put(address, elapsedTime);  
    } else {  
        isTiming = false;
    }  
}  
