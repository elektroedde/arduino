#include <Servo.h>


// Door servo assignment
Servo leftDoor;
Servo rightDoor;
Servo liftServo;


// Motor driver pin assignments
const int directionLeftPin  = 12;
const int pwmLeftPin        = 3;
const int brakeLeftPin      = 9;
const int directionRightPin = 13;
const int pwmRightPin       = 11;
const int brakeRightPin     = 8;


// Ultrasonic sensor pins
const int sensorTrigPin = 6;
const int echoPin1 = 7;


// Bucket pin assignment
const int liftServoPin = 2;
const int leftDoorPin = 4;
const int rightDoorPin = 5;






// Wheel circumference compensation (corrects for mechanical differences between wheels)
const float resWheel1  = 4.0;
const float resWheel2  = 3.6;
const float DIFFERENCE = resWheel2 / resWheel1; // Always < 1; scales right motor speed down




long duration, echoDistance;
bool startFlag;


// The lift and door-mechanism
int doorTime = 480;


// Magnetometer

#include <Wire.h>

#define MPU 0x68
#define AK8975 0x0C  // Magnetometer inside MPU-9150

void writeByte(uint8_t addr, uint8_t reg, uint8_t data) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission(true);
}


void setup() {
  Serial.begin(9600);

  pinMode(directionLeftPin,  OUTPUT);
  pinMode(pwmLeftPin,        OUTPUT);
  pinMode(brakeLeftPin,      OUTPUT);
  pinMode(directionRightPin, OUTPUT);
  pinMode(pwmRightPin,       OUTPUT);
  pinMode(brakeRightPin,     OUTPUT);


  pinMode(sensorTrigPin, OUTPUT);
  pinMode(echoPin1, INPUT);


  startFlag = false;
  echoDistance  = 4000;




  Serial.println("SETUP RUNNING");


  leftDoor.attach(leftDoorPin);
  rightDoor.attach(rightDoorPin);
  liftServo.attach(liftServoPin);
  goTo0();
  delay(1000);



  //Vinkel
  Wire.begin();

  // Wake up MPU-9150
  writeByte(MPU, 0x6B, 0x00);
  delay(100);

  // Enable bypass mode so we can talk directly to magnetometer
  writeByte(MPU, 0x37, 0x02);
  delay(10);

  // Power down magnetometer first
  writeByte(AK8975, 0x0A, 0x00);
  delay(10);

  Serial.println("Ready");
}


///////////////////// Driving ////////////////////
// Drives both motors in the given directions at a set speed for a given time (ms),
// then applies the brakes. leftForward/rightForward control each sides direction.
void driveMotors(bool leftForward, bool rightForward, int spd, int time) {
   digitalWrite(directionLeftPin,  leftForward  ? LOW : HIGH);
   digitalWrite(directionRightPin, rightForward ? LOW : HIGH);


   digitalWrite(brakeLeftPin,  LOW);
   digitalWrite(brakeRightPin, LOW);


   analogWrite(pwmLeftPin,  spd);
   analogWrite(pwmRightPin, DIFFERENCE * spd); // Compensate for wheel resistance difference


   delay(time);


   digitalWrite(brakeLeftPin,  HIGH);
   digitalWrite(brakeRightPin, HIGH);
   analogWrite(pwmLeftPin,  0);
   analogWrite(pwmRightPin, 0);
}


// Drives the robot straight forward for the given duration (ms).
void driveForward(int time) {
   digitalWrite(LED_BUILTIN, HIGH);
   driveMotors(true, true, 100, time);
}


// Drives the robot straight backward for a given time (ms).
void driveBackward(int time) {
   digitalWrite(LED_BUILTIN, LOW);
   driveMotors(false, false, 100, time);
}


// Turns the robot left for the given time (ms).
void turnLeft(int time) {
   driveMotors(false, true, 90, time);
}


// Turns the robot right for the given time (ms).
void turnRight(int time) {
   driveMotors(true, false, 100, time);
}
//////////////////////////////////////////////////


////////////////// The sensor //////////////////////
// Fires the ultrasonic sensor and drives forward if the path is clear (> 100 cm).
// Intended as a startup/gating routine before the main loop begins.
void start() {
   digitalWrite(sensorTrigPin, HIGH);
   delayMicroseconds(10);
   digitalWrite(sensorTrigPin, LOW);


   duration = pulseIn(echoPin1, HIGH);
   echoDistance = microsecondsToCentimeters(duration);


   Serial.print("Duration: ");
   Serial.println(duration);
   Serial.println(echoDistance);


   if (echoDistance > 100) {
       driveForward(300);
   }


   delay(5000);
}


// Converts a pulse duration (microseconds) to centimeters using the speed of sound.
long microsecondsToCentimeters(long microseconds) {
   return microseconds / 29 / 2;
}


/////////////////////////////////////////////////////////


////////////// The bucket /////////////////////
void openDoors(int time){
  leftDoor.write(180);   // Rotate outwards
  rightDoor.write(0);
  delay(time);
  leftDoor.write(90);  // is still
  rightDoor.write(90);
}


void closeDoors(int time){
  leftDoor.write(0); // Rotate inwards
  rightDoor.write(180);
  delay(time);
  leftDoor.write(90);  // is still
  rightDoor.write(90);
}


void goTo180() {
 liftServo.write(179);   // safer starting point for HS-422
}
void goTo95() {
 liftServo.write(95);   // safer starting point for HS-422
}
void goTo0() {
 liftServo.write(0);
}
///////////////////////////////////////////////////////


void magnetometerLoop() {
  // --- Magnetometer ---
  writeByte(AK8975, 0x0A, 0x01); // Trigger single measurement
  delay(10);

  Wire.beginTransmission(AK8975);
  Wire.write(0x02);
  Wire.endTransmission(false);
  Wire.requestFrom(AK8975, 1, true);
  byte st1 = Wire.read();

  float heading = 0;

  if (st1 & 0x01) {
    Wire.beginTransmission(AK8975);
    Wire.write(0x03);
    Wire.endTransmission(false);
    Wire.requestFrom(AK8975, 6, true);

    int16_t mx = Wire.read() | Wire.read() << 8;
    int16_t my = Wire.read() | Wire.read() << 8;
    int16_t mz = Wire.read() | Wire.read() << 8; // not used

    // Heading in degrees, 0-360
    heading = atan2(my, mx) * 180.0 / PI;
    if (heading < 0) heading += 360.0;
  }

  Serial.print("Heading (deg): ");
  Serial.println(heading);

  delay(500);
}

// Currently the loop turns the robot left with delay
void loop() {
   // Turn left in 30 short increments with brief pauses between each step
   for (int i = 0; i < 30; i++) {
       turnLeft(50);
       delay(50);
   }
   delay(1000);

   magnetometerLoop(); 

   //delay(1000);
}
