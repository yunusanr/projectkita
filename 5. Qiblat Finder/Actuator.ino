#define DIR_PIN 2
#define STP_PIN 3
#define INTERFACE_TYPE 1

AccelStepper stepper = AccelStepper(INTERFACE_TYPE, STP_PIN, DIR_PIN);

// STEPPER
const int acceleration = 400;  // steps/s^2
const int max_speed = 800;
const float step_per_rev = 6400.0;  // Step per revolution based on driver configuration
int current_step, calculated_step, homing_step, last_step = 0;

// EEPROM
const int max_allowed_writes = 80;
const int mem_base = 350;
int address_int;

void init_eeprom() {
  EEPROM.setMemPool(mem_base, EEPROMSizeUno);
  EEPROM.setMaxAllowedWrites(max_allowed_writes);
  delay(150);
  address_int = EEPROM.getAddress(sizeof(int));
}

void init_actuator() {
  stepper.setMaxSpeed(max_speed);
  stepper.setAcceleration(acceleration);
  stepper.disableOutputs();
  display_lcd();
  while (heading == 0.0) {
    update_compass();
    refresh_lcd();
  }

  Serial.print("Heading: ");
  Serial.println(heading);

  // homing();
  rotate_to_zero();

  current_step = stepper.currentPosition();
  Serial.print("Starting step: ");
  Serial.println(current_step);
}

void rotate_by_compass() {
  digitalWrite(relay, LOW);  // Turn off laser

  stepper.enableOutputs();
  calculated_step = step_per_rev - angle_to_step(heading);  // Steps needed to qibla angle

  stepper.moveTo(calculated_step);
  stepper.runToPosition();

  current_step = stepper.currentPosition();
  // EEPROM.updateInt(address_int, current_step);
  stepper.disableOutputs();
  delay(10);
}

void rotate_to_zero() {
  stepper.enableOutputs();
  stepper.setCurrentPosition(0);
  digitalWrite(relay, LOW);

  calculated_step = step_per_rev - angle_to_step(heading);

  stepper.moveTo(calculated_step);
  stepper.runToPosition();

  current_step = stepper.currentPosition();
  delay(10);
  stepper.disableOutputs();
}

void homing() {  // GAK DIPAKAI SETELAH ADA KOMPAS
  stepper.enableOutputs();
  digitalWrite(relay, LOW);

  last_step = EEPROM.readInt(address_int);
  delay(10);
  stepper.setCurrentPosition(last_step);
  if (last_step >= 3200) {
    homing_step = step_per_rev;
  } else {
    homing_step = 0;
  }

  stepper.moveTo(homing_step);
  stepper.runToPosition();
  stepper.setCurrentPosition(0);
  current_step = stepper.currentPosition();
  EEPROM.updateInt(address_int, current_step);
  delay(10);
  stepper.disableOutputs();
}

int angle_to_step(float target_angle) {
  if (target_angle >= 0.0) {
    float steps = round((target_angle / 360.0) * step_per_rev);
    return int(steps);
  }
  return 0;
}