#pragma once

#include "vector.h"

extern Vector gyro;  // gyroscope output, rad/s
extern Vector gyroDeg;  // gyroscope output, deg/s
extern Vector gyroBias;

extern Vector acc;  // accelerometer output, m/s/s
extern Vector accG;  // accelerometer output, g
extern Vector accBias;

void setupIMU();
void configureIMU();
void readIMU();
void calibrateAsync();
void printAccelerometerRange();
void printGyroRange();
void printFilterBandwidth();
