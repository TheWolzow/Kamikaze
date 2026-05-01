#pragma once

#include "vector.h"

extern Vector gyroRad;  // gyroscope output, rad/s
extern Vector gyroBias;

extern Vector accG;  // accelerometer output, g
extern Vector accBias;

void initIMU();
void mpu6050_read_raw();  // Читает и записывает в указазанные переменные сырые данные используя Wire
void readIMU();  // Читает сырые данные и составляет из них вектора
void calibrateAsync();
