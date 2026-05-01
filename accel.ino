#include <Servo.h>
#include <Wire.h>

// ── Servos ────────────────────────────────────────────────
Servo leftDoor;
Servo rightDoor;
Servo liftServo;

const int averageDistance = 5;

const int liftServoPin = 2;
const int leftDoorPin  = 4;
const int rightDoorPin = 5;

// ── Motor driver ──────────────────────────────────────────
const int directionLeftPin  = 12;
const int pwmLeftPin        = 3;
const int brakeLeftPin      = 9;
const int directionRightPin = 13;
const int pwmRightPin       = 11;
const int brakeRightPin     = 8;

int powerWheel = 120;
int powerTurn  = 90;

// Compensates for mechanical differences between wheels
const float resWheel1  = 4.0;
const float resWheel2  = 3.7;
const float DIFFERENCE = resWheel2 / resWheel1;

// ── Ultrasonic sensor ─────────────────────────────────────
const int sensorTrigPin = 6;
const int echoPin       = 7;

float duration, echoDistance;

// ── Bucket mechanism ──────────────────────────────────────
int doorTime = 420;

// ── Magnetometer (AK8975 inside MPU-9150) ─────────────────
#define MPU    0x68
#define AK8975 0x0C

int16_t minX = -31, maxX = 49;
int16_t minY = -50, maxY = 29;
int16_t minZ =  87, maxZ = 114;

float offsetX, offsetY, offsetZ;
float scaleX,  scaleY,  scaleZ;

// ── Navigation ────────────────────────────────────────────
float netDirection, leftDirection, rightDirection, backDirection;

int margin    = 10;
int turnTime  = 110;
int turnDelay = 50;
int average   = 10;

// ── State ─────────────────────────────────────────────────
bool startFlag  = true;
int closeToWall = 25;


// ── Forward declarations ──────────────────────────────────
void writeByte(uint8_t addr, uint8_t reg, uint8_t data);
float getDistance();
float getDirection();
float angleDiff(float current, float target);
float microsecondsToCentimeters(float microseconds);
void driveMotors(bool leftForward, bool rightForward, int spd, int time);
void driveMotorsDistance(bool leftForward, bool rightForward, int spd, int distance);
void openDoors();
void closeDoors();
void goTo0();


// ═════════════════════════════════════════════════════════
//  SETUP & LOOP
// ═════════════════════════════════════════════════════════

void setup() {
  Serial.begin(9600);
  Serial.println("SETUP RUNNING");

  // Pin modes
  pinMode(directionLeftPin,  OUTPUT);
  pinMode(pwmLeftPin,        OUTPUT);
  pinMode(brakeLeftPin,      OUTPUT);
  pinMode(directionRightPin, OUTPUT);
  pinMode(pwmRightPin,       OUTPUT);
  pinMode(brakeRightPin,     OUTPUT);
  pinMode(sensorTrigPin,     OUTPUT);
  pinMode(echoPin,           INPUT);

  // Servos
  leftDoor.attach(leftDoorPin);
  rightDoor.attach(rightDoorPin);
  liftServo.attach(liftServoPin);
  goTo0();

  // I2C / Magnetometer
  Wire.begin();
  writeByte(MPU,    0x6B, 0x00); // Wake up MPU-9150
  delay(100);
  writeByte(MPU,    0x37, 0x02); // Enable bypass mode to access magnetometer directly
  delay(10);
  writeByte(AK8975, 0x0A, 0x00); // Power down magnetometer before configuring
  delay(10);

  // Magnetometer calibration
  offsetX = (maxX + minX) / 2.0;
  offsetY = (maxY + minY) / 2.0;
  offsetZ = (maxZ + minZ) / 2.0;

  scaleX = (maxX - minX) / 2.0;
  scaleY = (maxY - minY) / 2.0;
  scaleZ = (maxZ - minZ) / 2.0;

  float avgScale = (scaleX + scaleY + scaleZ) / 3.0;
  scaleX = avgScale / scaleX;
  scaleY = avgScale / scaleY;
  scaleZ = avgScale / scaleZ;

  // Capture starting direction
  netDirection   = getDirection();
  leftDirection  = fmod(netDirection + 90.0,  360.0);
  rightDirection = fmod(netDirection + 270.0, 360.0);
  backDirection  = fmod(netDirection + 180.0, 360.0);



  delay(5000);
}

void loop() {
  //delay(2000);
  if (startFlag) raceStart();

  delay(2000);
  faceDirection(rightDirection);
  while (getDistance() > 30) {
    driveMotors(true, true, powerWheel, 200);
  }

  //faceDirection(netDirection);
  // path1();
  // dumpOverNet();
  // path2();
  // dumpOverNet();
  // while(true){
  //   loopedPath1();
  //   dumpOverNet();
  //   loopedPath2();
  //   dumpOverNet();
  // }
}


// ═════════════════════════════════════════════════════════
//  I2C HELPER
// ═════════════════════════════════════════════════════════

// Writes a single byte to a register on an I2C device.
void writeByte(uint8_t addr, uint8_t reg, uint8_t data) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission(true);
}

// ═════════════════════════════════════════════════════════
//  DRIVING
// ═════════════════════════════════════════════════════════

// Drives both motors at a given speed for a given time (ms), then brakes.
// leftForward/rightForward control each side's direction independently.
void driveMotors(bool leftForward, bool rightForward, int spd, int time) {
  digitalWrite(directionLeftPin,  leftForward  ? LOW : HIGH);
  digitalWrite(directionRightPin, rightForward ? LOW : HIGH);

  digitalWrite(brakeLeftPin,  LOW);
  digitalWrite(brakeRightPin, LOW);

  analogWrite(pwmLeftPin,  spd);
  analogWrite(pwmRightPin, DIFFERENCE * spd);

  delay(time);

  digitalWrite(brakeLeftPin,  HIGH);
  digitalWrite(brakeRightPin, HIGH);
  analogWrite(pwmLeftPin,  0);
  analogWrite(pwmRightPin, 0);
}

// Drives motors until the sensor detects a wall closer than distance (cm), then brakes.
// Opens doors while driving and closes them when approaching the wall.
void driveMotorsDistance(bool leftForward, bool rightForward, int spd, int distance) {
  digitalWrite(directionLeftPin,  leftForward  ? LOW : HIGH);
  digitalWrite(directionRightPin, rightForward ? LOW : HIGH);

  if (!startFlag) openDoors();

  digitalWrite(brakeLeftPin,  LOW);
  digitalWrite(brakeRightPin, LOW);

  analogWrite(pwmLeftPin,  spd);
  analogWrite(pwmRightPin, DIFFERENCE * spd);

  while (getDistance() > distance) {
    if (!startFlag && getDistance() < distance + 10) closeDoors();
    delay(100);
  }

  digitalWrite(brakeLeftPin,  HIGH);
  digitalWrite(brakeRightPin, HIGH);
  analogWrite(pwmLeftPin,  0);
  analogWrite(pwmRightPin, 0);
}

// Drives the robot straight forward for the given duration (ms).
void driveForward(int time) {
  driveMotors(true, true, powerWheel, time);
}

// Drives forward until the sensor detects a wall closer than distance (cm).
void driveForwardDistance(int distance) {
  driveMotorsDistance(true, true, powerWheel, distance);
}

// Drives the robot straight backward for a given time (ms).
void driveBackward(int time) {
  driveMotors(false, false, powerWheel, time);
}

// Turns the robot left for the given time (ms).
void turnLeft(int time) {
  driveMotors(false, true, powerTurn, time);
}

// Turns the robot right for the given time (ms).
void turnRight(int time) {
  driveMotors(true, false, powerTurn, time);
}

// Turns to face the given absolute compass direction.
void faceDirection(float targetDirection) {
  float diff = angleDiff(getDirection(), targetDirection);

  while (abs(diff) > margin) {
    if (diff < 0) {
      turnLeft(turnTime);

    } else {
      turnRight(turnTime);

    }
    delay(turnDelay);
    diff = angleDiff(getDirection(), targetDirection);
  }
}

// ═════════════════════════════════════════════════════════
//  ULTRASONIC SENSOR
// ═════════════════════════════════════════════════════════

// Fires the ultrasonic sensor 3 times and returns the average distance in cm.
float getDistance() {
  float total = 0;
  float temp = 0;
  for (int i = 0; i < averageDistance; i++) {
    digitalWrite(sensorTrigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(sensorTrigPin, LOW);
    duration = pulseIn(echoPin, HIGH);
   temp = microsecondsToCentimeters(duration);
    if(temp > total) {
      total = temp;
    }

  }
  float avgDistance = total / float(averageDistance);

  return total;
}

// Converts a pulse duration (microseconds) to centimeters using the speed of sound.
float microsecondsToCentimeters(float microseconds) {
  return microseconds / 29 / 2;
}


// ═════════════════════════════════════════════════════════
//  BUCKET MECHANISM
// ═════════════════════════════════════════════════════════

void openDoors() {
  leftDoor.write(180);
  delay(100);
  rightDoor.write(35);
  delay(doorTime);
  leftDoor.write(90);
  delay(100);
  rightDoor.write(90);
}

void closeDoors() {
  rightDoor.write(154);
  delay(100);
  leftDoor.write(25);
  delay(doorTime);
  rightDoor.write(90);
  delay(100);
  leftDoor.write(90);
}

void goTo180() { liftServo.write(179); }
void goTo95()  { liftServo.write(95);  }
void goTo0()   { liftServo.write(0);   }

void dumpOverNet() {
  faceDirection(backDirection);
  driveBackward(1000);
  goTo180();
  delay(2000);
  goTo0();
  driveForward(500);
}


// ═════════════════════════════════════════════════════════ 
//  MAGNETOMETER
// ═════════════════════════════════════════════════════════

// Returns the average compass heading in degrees (0–360).
float getDirection() {
  float sumX = 0;
  float sumY = 0;

  for (int i = 0; i < average; i++) {
    writeByte(AK8975, 0x0A, 0x01);
    delay(10);

    Wire.beginTransmission(AK8975);
    Wire.write(0x02);
    Wire.endTransmission(false);
    Wire.requestFrom(AK8975, 1, true);
    byte st1 = Wire.read();

    if (st1 & 0x01) {
      Wire.beginTransmission(AK8975);
      Wire.write(0x03);
      Wire.endTransmission(false);
      Wire.requestFrom(AK8975, 6, true);

      int16_t mx = Wire.read() | Wire.read() << 8;
      int16_t my = Wire.read() | Wire.read() << 8;
      int16_t mz = Wire.read() | Wire.read() << 8;

      mx = (mx - offsetX) * scaleX;
      my = (my - offsetY) * scaleY;
      mz = (mz - offsetZ) * scaleZ;

      float heading = atan2(my, mx);
      sumX += cos(heading);
      sumY += sin(heading);
    }
  }

  float avgHeading = atan2(sumY, sumX) * 180.0 / PI;
  if (avgHeading < 0) avgHeading += 360;




  return avgHeading;
}

// Returns the signed angular difference between current and target (degrees).
// Negative = turn left, positive = turn right.
float angleDiff(float current, float target) {
  float diff = fmod((current - target + 180.0), 360.0);
  if (diff < 0) diff += 360.0;
  return diff - 180.0;
}

// Rotates the sensor for 20 seconds and prints min/max values for calibration.
void calibrateMagnetometer() {

  int16_t calMinX = 32767,  calMinY = 32767,  calMinZ = 32767;
  int16_t calMaxX = -32768, calMaxY = -32768, calMaxZ = -32768;

  unsigned long start = millis();
  while (millis() - start < 20000) {
    writeByte(AK8975, 0x0A, 0x01);
    delay(10);

    Wire.beginTransmission(AK8975);
    Wire.write(0x03);
    Wire.endTransmission(false);
    Wire.requestFrom(AK8975, 6, true);

    int16_t mx = Wire.read() | Wire.read() << 8;
    int16_t my = Wire.read() | Wire.read() << 8;
    int16_t mz = Wire.read() | Wire.read() << 8;

    calMinX = min(calMinX, mx); calMaxX = max(calMaxX, mx);
    calMinY = min(calMinY, my); calMaxY = max(calMaxY, my);
    calMinZ = min(calMinZ, mz); calMaxZ = max(calMaxZ, mz);

    delay(50);
  }
}
// ═════════════════════════════════════════════════════════
//  RACE
// ═════════════════════════════════════════════════════════

void raceStart() {

   driveMotors(true, true, powerWheel*0.9, 1200);


  while (getDistance() > 170) {


    driveMotors(true, true, powerWheel, 800);
  }
  startFlag = false;
}

void path1() {
  driveForwardDistance(closeToWall);
  faceDirection(leftDirection);
  driveForwardDistance(60);
  faceDirection(backDirection);
  driveForwardDistance(closeToWall);
  faceDirection(leftDirection);
  driveForwardDistance(closeToWall);
  faceDirection(netDirection);
  driveForwardDistance(closeToWall);
}

void path2() {
  faceDirection(rightDirection);
  driveForwardDistance(60);
  faceDirection(backDirection);
  driveForwardDistance(closeToWall);
  faceDirection(rightDirection);
  driveForwardDistance(closeToWall);
  faceDirection(netDirection);
  driveForwardDistance(closeToWall);
}

void loopedPath1() {
  faceDirection(leftDirection);
  driveForwardDistance(closeToWall);
  faceDirection(backDirection);
  driveForwardDistance(150);
  faceDirection(rightDirection);
  driveForwardDistance(closeToWall);
  faceDirection(backDirection);
  driveForwardDistance(100);
  faceDirection(leftDirection);
  driveForwardDistance(closeToWall);
  faceDirection(netDirection);
  driveForwardDistance(closeToWall);
}

void loopedPath2() {
  faceDirection(rightDirection);
  driveForwardDistance(closeToWall);
  faceDirection(backDirection);
  driveForwardDistance(150);
  faceDirection(leftDirection);
  driveForwardDistance(closeToWall);
  faceDirection(backDirection);
  driveForwardDistance(100);
  faceDirection(rightDirection);
  driveForwardDistance(closeToWall);
  faceDirection(netDirection);
  driveForwardDistance(closeToWall);
}
