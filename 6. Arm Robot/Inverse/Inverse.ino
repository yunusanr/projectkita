#include <Servo.h>
#include <math.h>

// create servo objects to control joint servos
Servo baseRot;
Servo shoulder;
Servo elbow;
Servo wrist;
Servo gripper;

//Constants for arm lenghts. Based on arm anatomy, measured in millimeters
const int baseArm = 68;         //lenght from floor level to shoulder joint (in millimeters)
const int upperArm = 119;       //lenght from shoulder to elbow (in millimeters)
const int foreArm = 126;        //lenght from elbow to wrist (in millimeters)
const int gripperArm = 84 + 2;  //lenght from wrist to the tip of gripper. Add two millimeters for some tolerance

//Constants for Position vs measured voltage model. Correspond to linear model equations.
const float shoulderSlope = 0.6963;
const float shoulderConstant = -72.491;
const float elbowSlope = 0.622;
const float elbowConstant = -52.279;
const float wristSlope = 0.5797;
const float wristConstant = -43.19;

String serialMessage = "";

//variables to store servo's real position
float shoulderTrimpotPos = 0;
float elbowTrimpotPos = 0;
float wristTrimpotPos = 0;

//constants and variables to smooth the readings from a sensor (trimpots in this case), by calculating an average of its values
const int numReadings = 100;
int shoulderReadings[numReadings];  // the readings from the analog input
int shoulderReadIndex = 0;          // the index of the current reading
int shoulderReadTotal = 0;          // the running total
int shoulderSmoothReading = 0;      // the average
int elbowReadings[numReadings];     // the readings from the analog input
int elbowReadIndex = 0;             // the index of the current reading
int elbowReadTotal = 0;             // the running total
int elbowSmoothReading = 0;         // the average
int wristReadings[numReadings];     // the readings from the analog input
int wristReadIndex = 0;             // the index of the current reading
int wristReadTotal = 0;             // the running total
int wristSmoothReading = 0;         // the average

int shoulderInputPin = A0;
int elbowInputPin = A1;
int wristInputPin = A2;

const int xEntendPositions[] = { 90, 115, 140, 150, 190, 225 };
int posCounter = 0;
unsigned long servoExtentTimer;
unsigned long servoDelay = 4000;

void setup() {
  Serial.begin(9600);

  // initialize the matrixes for analog readings. Sends all the readings to 0:
  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    shoulderReadings[thisReading] = 0;
    elbowReadings[thisReading] = 0;
    wristReadings[thisReading] = 0;
  }
  //Attaches all servos. Min and Max values for angles are based on experimental data
  baseRot.attach(5);
  shoulder.attach(6, 670, 2310);
  elbow.attach(7, 610, 2430);
  wrist.attach(8, 540, 2360);
  gripper.attach(10);

  //raises the arm to a start position
  ArmUp();
}

void loop() {

  //Timer to separate in time diferent arm positions. Until timer it's reached, a new position is written. Timer is reset after every ExtendArm1D call.
  if ((millis() - servoExtentTimer) >= servoDelay) {

    Serial.println(String("posCounter: ") + int(posCounter) + String("  sizeXPosition: ") + int(xEntendPositions[posCounter]));

    ExtendArm1D(xEntendPositions[posCounter]);
    posCounter++;                 //increase counter to new Arm position
    servoExtentTimer = millis();  // resets timer

    //resets the counter when it reaches the last arm position
    if (posCounter == sizeof(xEntendPositions) / sizeof(xEntendPositions[0])) {
      posCounter = 0;
    }
  }

  //Read servo's trimpot value with smooth method
  shoulderSmoothReading = SmoothAnalogReading(&shoulderReadTotal, shoulderReadings, &shoulderReadIndex, shoulderInputPin);
  elbowSmoothReading = SmoothAnalogReading(&elbowReadTotal, elbowReadings, &elbowReadIndex, elbowInputPin);
  wristSmoothReading = SmoothAnalogReading(&wristReadTotal, wristReadings, &wristReadIndex, wristInputPin);

  //converts servo's trimpot data to real position data
  shoulderTrimpotPos = ConvertTrimpotPos(shoulderSmoothReading, shoulderSlope, shoulderConstant);
  elbowTrimpotPos = ConvertTrimpotPos(elbowSmoothReading, elbowSlope, elbowConstant);
  wristTrimpotPos = ConvertTrimpotPos(wristSmoothReading, wristSlope, wristConstant);

  //***NEED TO FIX THIS PART, CORRECTION SHOULD ONLY HAPPEN AFTER REACHING FINAL POSITION, FINAL ANGLES HAVE TO BE  GLOBAL VARIABLES
  //  Serial.println(String("SHOULDER desired: ") + int(shoulderAngle) + String("  Readed: " )+ int(shoulderTrimpotPos));
  //  CorrectAngleError (shoulder, int(shoulderAngle), shoulderSlope, shoulderConstant, &shoulderReadTotal, shoulderReadings, &shoulderReadIndex, shoulderInputPin) ;
}

void ArmUp() {
  //Move the arm to a raised position
  //  MoveSomeServosComp (shoulder, 90, elbow, 90, wrist, 5);

  wrist.write(0);
  elbow.write(90);
  shoulder.write(90);

  delay(1000);
}

void ArmRotate(int RotationPos) {
  //Rotates the arm to the specified angle. Right is 0, left 180, center 90.
  baseRot.write(int(RotationPos));
  delay(1000);
}

void GripperPinch(int OpenPct) {
  //Opens-Closes the gripper to certain percentage:
  //100% is fully open, 0% is fully closed
  gripper.write(map(OpenPct, 100, 0, 35, 150));
  delay(1000);
}

void ExtendArm1D(int xExtension) {
  //Based on inverse kinematics equations calculates angles for joints
  //xExtension the arms extension in X axis. It should be a value in millimeters (97-235mm)

  //calculates distance from shoulder joint to wrist joint, used on kinematic equations
  float shoulderWristLenght = sqrt(square(xExtension) + square(gripperArm - baseArm));

  //calculate angle of elbow joint. Servo starts measuring angle from upper arm, so it's corrected
  float elbowAngleRad = acos((square(foreArm) + square(upperArm) - square(shoulderWristLenght)) / (2 * foreArm * upperArm));
  float elbowAngleDeg = elbowAngleRad * 180 / PI;  //acos returns radians, so it's changed to degrees
  elbowAngleDeg = 180 - elbowAngleDeg;             //Servo takes upperarm as 0 degrees, so it's changed

  //calculate angle of wrist joint. Servo takes perpendicular line to forearm as 0 degree
  float wristAngle = asin(xExtension / shoulderWristLenght) + asin(upperArm / shoulderWristLenght * sin(elbowAngleRad));
  wristAngle = wristAngle * 180 / PI - 90;  //returns radians, so it's changed to degress. Substracts 90 degrees due to servo reference

  //calculate angle of shoulder. Servo takes X axis as 0 degrees.
  float shoulderAngle = acos(xExtension / shoulderWristLenght) + asin(foreArm / shoulderWristLenght * sin(elbowAngleRad));
  shoulderAngle = shoulderAngle * 180 / PI;  //returns radians, so it's changed to degress.

  //write angles positions into servos
  wrist.write(int(wristAngle));
  elbow.write(int(elbowAngleDeg));
  shoulder.write(int(shoulderAngle));

  //waits for the servos to reach its position
  delay(4000);
  Serial.println(String("WRIST desired: ") + int(wristAngle) + String("  Readed: ") + int(wristTrimpotPos));
  Serial.println(String("ELBOW desired: ") + int(elbowAngleDeg) + String("  Readed: ") + int(elbowTrimpotPos));
  Serial.println(String("SHOULDER desired: ") + int(shoulderAngle) + String("  Readed: ") + int(shoulderTrimpotPos));

  CorrectAngleError(shoulder, int(shoulderAngle), shoulderSlope, shoulderConstant, &shoulderReadTotal, shoulderReadings, &shoulderReadIndex, shoulderInputPin);
}


//Converts an analog value (read as 0-1024) to its corresponding position value.
//Based on the Position equations found experimentally
int ConvertTrimpotPos(int value, float slope, float constant) {
  float positiom = round(slope * value + constant);
  return positiom;
}


//Corrects the current position of a servo if it is biased from the desired angle
//it's a single function with if-else logic
void CorrectAngleError(Servo myServo, int desiredAngle, float slope, float constant, int* total, int readings[], int* index, int inputPin) {
  const int tolerance = 5;
  int gotoAngle = desiredAngle;

  int smoothReading = SmoothAnalogReading(total, readings, index, inputPin);
  int currentPos = ConvertTrimpotPos(smoothReading, slope, constant);

  Serial.println(String("CORRECTION desired: ") + int(desiredAngle) + String("  Readed: ") + int(currentPos));

  while (currentPos < (desiredAngle - tolerance)) {
    Serial.println(String("CURRENT ANG MENOR  --- Desired: ") + int(desiredAngle) + String("  >  Readed: ") + int(currentPos));
    gotoAngle++;
    shoulder.write(int(gotoAngle));
    delay(100);

    smoothReading = SmoothAnalogReading(total, readings, index, inputPin);
    currentPos = ConvertTrimpotPos(smoothReading, slope, constant);
  }

  while (currentPos > (desiredAngle + tolerance)) {
    Serial.println(String("CURRENT ANG MAYOR --- Desired: ") + int(desiredAngle) + String("  <  Readed: ") + int(currentPos));
    gotoAngle--;
    shoulder.write(int(gotoAngle));
    delay(100);

    smoothReading = SmoothAnalogReading(total, readings, index, inputPin);
    currentPos = ConvertTrimpotPos(smoothReading, slope, constant);
  }
}

//Method to smooth a sensor reading by taking an average of the last n analog readings
//Returns the current average value
int SmoothAnalogReading(int* total, int readings[], int* index, int inputPin) {

  //****comment the following line (for) and the closing braket when placing the analog reading in the main loop (to always read the AI values)
  //  for (int thisReading = 0; thisReading < numReadings; thisReading++){

  // subtract the last reading:
  *total = *total - readings[*index];
  // read from the sensor:
  readings[*index] = analogRead(inputPin);
  // add the reading to the total:
  *total = *total + readings[*index];
  // advance to the next position in the array:
  *index = *index + 1;

  // if we're at the end of the array...
  if (*index >= numReadings)
    // ...wrap around to the beginning:
    *index = 0;
  //  }
  // calculate the average:
  int average = *total / numReadings;
  return average;
}