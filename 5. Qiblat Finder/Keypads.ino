#define I2CADDR 0x20

const byte ROWS = 4;  //four rows
const byte COLS = 3;  //three columns

char keys[ROWS][COLS] = {
  { '1', '2', '3' },
  { '4', '5', '6' },
  { '7', '8', '9' },
  { '*', '0', '#' }
};

// Digitran keypad, bit numbers of PCF8574 i/o port
byte rowPins[ROWS] = { 6, 1, 2, 4 };
byte colPins[COLS] = { 5, 7, 3 };

TwoWire *jwire = &Wire;  //test passing pointer to keypad lib
Keypad_I2C kpd(makeKeymap(keys), rowPins, colPins, ROWS, COLS, I2CADDR, PCF8574, jwire);

void init_keypad() {
  jwire->begin();
  kpd.begin();
}

void update_keypad() {
  char key = kpd.getKey();

  if (key) {
    if (key == '#') {
      Serial.println("Pressed");
    }
  }
}