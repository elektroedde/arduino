// RÖD -> 3.3V
// SVART -> GND
// ORANGE -> Pin 21
// BLÅ -> Pin 20

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
  Wire.begin();
  Serial.begin(9600);

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

void loop() {
  // --- Accel + Gyro ---
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 14, true);

  int16_t ax = Wire.read() << 8 | Wire.read();
  int16_t ay = Wire.read() << 8 | Wire.read();
  int16_t az = Wire.read() << 8 | Wire.read();
  int16_t tmp = Wire.read() << 8 | Wire.read();
  int16_t gx = Wire.read() << 8 | Wire.read();
  int16_t gy = Wire.read() << 8 | Wire.read();
  int16_t gz = Wire.read() << 8 | Wire.read();

  // --- Magnetometer ---
  // Trigger single measurement
  writeByte(AK8975, 0x0A, 0x01);
  delay(10);

  // Wait for data ready
  Wire.beginTransmission(AK8975);
  Wire.write(0x02); // ST1 status register
  Wire.endTransmission(false);
  Wire.requestFrom(AK8975, 1, true);
  byte st1 = Wire.read();

  int16_t mx = 0, my = 0, mz = 0;
  if (st1 & 0x01) { // Data ready
    Wire.beginTransmission(AK8975);
    Wire.write(0x03); // Start of mag data
    Wire.endTransmission(false);
    Wire.requestFrom(AK8975, 6, true);

    mx = Wire.read() | Wire.read() << 8;
    my = Wire.read() | Wire.read() << 8;
    mz = Wire.read() | Wire.read() << 8;
  }

  // --- Print ---
  //Serial.print("Accel (g): ");
  //Serial.print(ax/16384.0); Serial.print(", ");
  //Serial.print(ay/16384.0); Serial.print(", ");
  //Serial.println(az/16384.0);
//
  //Serial.print("Gyro (dps): ");
  //Serial.print(gx/131.0); Serial.print(", ");
  //Serial.print(gy/131.0); Serial.print(", ");
  //Serial.println(gz/131.0);

  Serial.print("Mag (uT):  ");
  Serial.print(mx * 0.3); Serial.print(", ");
  Serial.print(my * 0.3); Serial.print(", ");
  Serial.println(mz * 0.3);

  Serial.println("---");
  delay(1500);
}
