#include <Wire.h>
#include <MPU9250.h>
#include "lpf.h"
#include "util.h"

// Create an MPU9250 object using I2C
MPU9250 IMU(Wire);

// Calibrating parameters
Vector accBias;
Vector gyroBias;
Vector accScale(1, 1, 1);

void setupIMU() {
  print("Setup IMU\n");

  // Initialize the I2C communication
  Wire.begin(); // Initialize Wire library for I2C

  IMU.begin();  // Initialize the MPU9250 sensor

  configureIMU();  // Configure sensor settings
}

void configureIMU() {
  // Set accelerometer range to 4G
  IMU.setAccelRange(IMU.ACCEL_RANGE_4G);
  // Set gyroscope range to 2000 degrees per second
  IMU.setGyroRange(IMU.GYRO_RANGE_2000DPS);
  // Set digital low pass filter
  IMU.setDLPF(IMU.DLPF_MAX);
  // Set data rate to approximately 1kHz
  IMU.setRate(IMU.RATE_1KHZ_APPROX);
}

void readIMU() {
  IMU.waitForData();  // Wait for new data from the IMU
  IMU.getGyro(gyro.x, gyro.y, gyro.z);  // Get gyro data
  IMU.getAccel(acc.x, acc.y, acc.z);  // Get accelerometer data
  
  calibrateGyroOnce();  // Calibrate the gyroscope once
  // Apply scale and bias for accelerometer and gyroscope
  acc = (acc - accBias) / accScale;
  gyro = gyro - gyroBias;
  
  // Rotate the accelerometer and gyroscope data
  rotateIMU(acc);
  rotateIMU(gyro);
}

void rotateIMU(Vector& data) {
  // Rotate from LFD to FLU
  // Axes orientation adjustment for various boards
  data = Vector(data.y, data.x, data.z);
}

void calibrateGyroOnce() {
  static float landedTime = 0;
  landedTime = landed ? landedTime + dt : 0;
  if (landedTime < 2) return;  // Only calibrate if definitely stationary

  static LowPassFilter<Vector> gyroCalibrationFilter(0.001);
  gyroBias = gyroCalibrationFilter.update(gyro);  // Update gyro bias with low-pass filter
}

void calibrateAccel() {
  print("Calibrating accelerometer\n");
  IMU.setAccelRange(IMU.ACCEL_RANGE_2G);  // Use the most sensitive mode

  print("Place level [8 sec]\n");
  pause(8);
  calibrateAccelOnce();
  print("Place nose up [8 sec]\n");
  pause(8);
  calibrateAccelOnce();
  print("Place nose down [8 sec]\n");
  pause(8);
  calibrateAccelOnce();
  print("Place on right side [8 sec]\n");
  pause(8);
  calibrateAccelOnce();
  print("Place on left side [8 sec]\n");
  pause(8);
  calibrateAccelOnce();
  print("Place upside down [8 sec]\n");
  pause(8);
  calibrateAccelOnce();

  printIMUCal();
  print("✓ Calibration done!\n");
  configureIMU();
}

void calibrateAccelOnce() {
  const int samples = 1000;
  static Vector accMax(-INFINITY, -INFINITY, -INFINITY);
  static Vector accMin(INFINITY, INFINITY, INFINITY);

  // Compute the average of the accelerometer readings
  acc = Vector(0, 0, 0);
  for (int i = 0; i < samples; i++) {
    IMU.waitForData();
    Vector sample;
    IMU.getAccel(sample.x, sample.y, sample.z);
    acc = acc + sample;
  }
  acc = acc / samples;

  // Update the maximum and minimum values
  if (acc.x > accMax.x) accMax.x = acc.x;
  if (acc.y > accMax.y) accMax.y = acc.y;
  if (acc.z > accMax.z) accMax.z = acc.z;
  if (acc.x < accMin.x) accMin.x = acc.x;
  if (acc.y < accMin.y) accMin.y = acc.y;
  if (acc.z < accMin.z) accMin.z = acc.z;

  // Compute scale and bias
  accScale = (accMax - accMin) / 2 / ONE_G;
  accBias = (accMax + accMin) / 2;
}

void printIMUCal() {
  print("gyro bias: %f %f %f\n", gyroBias.x, gyroBias.y, gyroBias.z);
  print("accel bias: %f %f %f\n", accBias.x, accBias.y, accBias.z);
  print("accel scale: %f %f %f\n", accScale.x, accScale.y, accScale.z);
}

void printIMUInfo() {
  IMU.status() ? print("status: ERROR %d\n", IMU.status()) : print("status: OK\n");
  print("model: %s\n", IMU.getModel());
  print("who am I: 0x%02X\n", IMU.whoAmI());
}
