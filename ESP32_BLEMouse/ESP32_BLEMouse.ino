#include "I2Cdev.h"
#include <Wire.h>
#include <MPU6050.h>
#include <BleMouse.h>
#include "Kalman.h"
Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;
BleMouse bleMouse;
MPU6050 imu;
int16_t accX, accY, accZ;
int16_t gyroX, gyroY, gyroZ;

double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double compAngleX, compAngleY; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY;
int buttonL = 0; // IO0 button
int buttonR = 4;
int buttonLstate = HIGH; 
int buttonRstate = HIGH; 
uint32_t timer;
uint8_t i2cData[14];

#define RESTRICT_PITCH
void setup() {
  Serial.begin(115200);
  Wire.begin();
  pinMode(buttonL, INPUT_PULLUP);
  pinMode(buttonR, INPUT_PULLUP);
#if ARDUINO >= 157
  Wire.setClock(400000UL); // Set I2C frequency to 400kHz
#else
  TWBR = ((F_CPU / 400000UL) - 16) / 2; // Set I2C frequency to 400kHz
#endif  
  Serial.print("MPU6050 initializing");
  imu.initialize();
  while (!imu.testConnection()) { Serial.print("."); }
  Serial.println();  
  Serial.println("BLE Mouse starting !");
  bleMouse.begin();
  imu.getAcceleration(&accX, &accY, &accZ);
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  kalmanX.setAngle(roll); // Set starting angle
  kalmanY.setAngle(pitch);
  gyroXangle = roll;
  gyroYangle = pitch;
  compAngleX = roll;
  compAngleY = pitch;

  timer = micros();
}

void loop() {
  imu.getMotion6(&accX, &accY, &accZ, &gyroX, &gyroY, &gyroZ);
  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();
  #ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  double gyroXrate = gyroX / 131.0; // Convert to deg/s
  double gyroYrate = gyroY / 131.0; // Convert to deg/s

#ifdef RESTRICT_PITCH
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
    kalmanX.setAngle(roll);
    compAngleX = roll;
    kalAngleX = roll;
    gyroXangle = roll;
  } else
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleX) > 90)
    gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
#else
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
    kalmanY.setAngle(pitch);
    compAngleY = pitch;
    kalAngleY = pitch;
    gyroYangle = pitch;
  } else
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleY) > 90)
    gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
#endif

  gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
  gyroYangle += gyroYrate * dt;
  //gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
  //gyroYangle += kalmanY.getRate() * dt;

  compAngleX = 0.5 * (compAngleX + gyroXrate * dt) + 0.1 * roll; // Calculate the angle using a Complimentary filter
  compAngleY = 0.5 * (compAngleY + gyroYrate * dt) + 0.1 * pitch;

  // Reset the gyro angle when it has drifted too much
  if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = kalAngleX;
  if (gyroYangle < -180 || gyroYangle > 180)
    gyroYangle = kalAngleY;

  
  Serial.print("gx = ");   Serial.print(accX);
  Serial.print(", gy = "); Serial.print(accY);
  Serial.print("\t");
  Serial.print("X = ");    Serial.print(compAngleX);
  Serial.print(", Y = ");  Serial.println(compAngleY);
  
  bleMouse.move(compAngleX, compAngleY);
    
  buttonLstate = digitalRead(buttonL);
  buttonRstate = digitalRead(buttonR);  
  
  if (buttonLstate == LOW) { // press button to Ground
    bleMouse.click(MOUSE_LEFT);
    delay(100);
  } 
  else if(buttonRstate == LOW) { // press button to Ground
    bleMouse.click(MOUSE_RIGHT);
    delay(100);
  }
  delay(100);
}
