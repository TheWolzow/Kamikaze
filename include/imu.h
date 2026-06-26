#pragma once

#include "vector.h"

extern Vector gyroRad;  // gyroscope output, rad/s
extern Vector gyroBias;

extern Vector acc;  // accelerometer output, m/s/s
extern Vector accNorm;  // accelerometer output, normalized acc
extern Vector accG;  // accelerometer output, g
extern Vector accBias;

extern bool imu_updated;  // true - значит imu обновился.

void initIMU();
void mpu6050_read_raw();  // Читает и записывает в указазанные переменные сырые данные используя Wire
void readIMU();  // Читает сырые данные и составляет из них вектора
void calibrateIMU();
