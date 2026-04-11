#include <Arduino.h>
#include "imu.h"

#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>

#include "vector.h"
#include "utils.h"

Adafruit_MPU6050 imu;

const short CALIB_SAMPLES = 2000;

Vector gyro;  // gyroscope output, rad/s
Vector gyroDeg;  // gyroscope output, deg/s
Vector gyroBias;

Vector acc;  // accelerometer output, m/s/s
Vector accG;  // accelerometer output, g
Vector accBias;


void setupIMU() {
  Serial.println("Setup IMU");
  if (!imu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

  configureIMU();
}

void configureIMU() {
  imu.setAccelerometerRange(MPU6050_RANGE_4_G);
  imu.setGyroRange(MPU6050_RANGE_2000_DEG);
  imu.setFilterBandwidth(MPU6050_BAND_94_HZ);
  // TODO

  printAccelerometerRange();
  printGyroRange();
  printFilterBandwidth();
}

void readIMU() {
  sensors_event_t a, g, temp;
  imu.getEvent(&a, &g, &temp);

  acc.x = a.acceleration.x;
  acc.y = a.acceleration.y;
  acc.z = a.acceleration.z;
  acc -= accBias;

  gyro.x = g.gyro.x;
  gyro.y = g.gyro.y;
  gyro.z = g.gyro.z;
  gyro -= gyroBias;

  gyroDeg = gyro * RAD_TO_DEG;

  // g
  accG.x = (float) acc.x / gConst;
  accG.y = (float) acc.y / gConst;
  accG.z = (float) acc.z / gConst;
}

void calibrateAsync() {
  Serial.println("Не двигай датчик! Калибровка начинается...");

  Vector accOffset;
  Vector gyroOffset;

  for (int i = 0; i < CALIB_SAMPLES; i++) {
    sensors_event_t a, g, t;
    imu.getEvent(&a, &g, &t);

    accOffset += Vector(a.acceleration.x, a.acceleration.y, a.acceleration.z);
    gyroOffset += Vector(g.gyro.x, g.gyro.y, g.gyro.z);

    delay(2);
  }

  accOffset.x /= CALIB_SAMPLES;
  accOffset.y /= CALIB_SAMPLES;
  accOffset.z = (accOffset.z / CALIB_SAMPLES) - 9.80665;  // гравитация

  gyroOffset.x /= CALIB_SAMPLES;
  gyroOffset.y /= CALIB_SAMPLES;
  gyroOffset.z /= CALIB_SAMPLES;

  /*Serial.println("\n=== ОФФСЕТЫ ===");
  Serial.print("AX: "); Serial.println(ax_off);
  Serial.print("AY: "); Serial.println(ay_off);
  Serial.print("AZ: "); Serial.println(az_off);

  Serial.print("GX: "); Serial.println(gx_off);
  Serial.print("GY: "); Serial.println(gy_off);
  Serial.print("GZ: "); Serial.println(gz_off);

  Serial.println("\nСохрани и вычитай их из измерений");*/

  accBias = accOffset;
  gyroBias = gyroOffset;
}

void printAccelerometerRange() {
  Serial.print("Accelerometer range: ");

  switch (imu.getAccelerometerRange()) {
    case MPU6050_RANGE_2_G:
      Serial.println("+-2G");
      break;
    case MPU6050_RANGE_4_G:
      Serial.println("+-4G");
      break;
    case MPU6050_RANGE_8_G:
      Serial.println("+-8G");
      break;
    case MPU6050_RANGE_16_G:
      Serial.println("+-16G");
      break;
  }
}

void printGyroRange() {
  Serial.print("Gyro range: ");
  
  switch (imu.getGyroRange()) {
    case MPU6050_RANGE_250_DEG:
      Serial.println("+- 250 deg/s");
      break;
    case MPU6050_RANGE_500_DEG:
      Serial.println("+- 500 deg/s");
      break;
    case MPU6050_RANGE_1000_DEG:
      Serial.println("+- 1000 deg/s");
      break;
    case MPU6050_RANGE_2000_DEG:
      Serial.println("+- 2000 deg/s");
      break;
  }
}

void printFilterBandwidth() {
  Serial.print("Filter bandwidth: ");

  switch (imu.getFilterBandwidth()) {
    case MPU6050_BAND_260_HZ:
      Serial.println("260 Hz");
      break;
    case MPU6050_BAND_184_HZ:
      Serial.println("184 Hz");
      break;
    case MPU6050_BAND_94_HZ:
      Serial.println("94 Hz");
      break;
    case MPU6050_BAND_44_HZ:
      Serial.println("44 Hz");
      break;
    case MPU6050_BAND_21_HZ:
      Serial.println("21 Hz");
      break;
    case MPU6050_BAND_10_HZ:
      Serial.println("10 Hz");
      break;
    case MPU6050_BAND_5_HZ:
      Serial.println("5 Hz");
      break;
    }
}
