#pragma once

#include "MPU6050.h"
#include "QMC5883L.h"
#include "vector.h"

extern const float G_CONST;

extern Vector gyroRad;  // gyroscope output, rad/s

extern Vector acc;  // accelerometer output, m/s/s
extern Vector accNorm;  // accelerometer output, normalized acc
extern Vector accG;  // accelerometer output, g

extern MPU6050 accgyro;
extern MPU6050Data accgyroData;

extern QMC5883L magnetometer;
extern QMC5883LData magData;

extern bool imu_updated;  // true - значит imu обновился.

void initIMU();
void readIMU();  // Читает сырые данные и составляет из них вектора
void calibrateIMU();
