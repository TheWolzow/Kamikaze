// roll - крен
// yaw - рыскание
// pitch - тангаж

// Положительный крен (ось X) - наклон вправо.
// Положительный тангаж (ось Y) - наклон вверх.

#include <Arduino.h>
#include "estimate.h"

#include "imu.h"
#include "step.h"
#include "vector.h"
#include "quaternion.h"

// ПИ-регулятор фильтра Махони
#define MAHONY_Kp 6.5
#define MAHONY_Ki 0  //0.0005


Vector rotation;  // Ориентация дрона в градусах
Vector rotationRad;  // Ориентация дрона в радианах

Quaternion attitude;  // Ориентация дрона в кватернионах

static Vector integral;


void estimate() {
  Vector gravity;

  gravity.x =
    2.0f * (attitude.x * attitude.z -
            attitude.w * attitude.y);

  gravity.y =
    2.0f * (attitude.w * attitude.x +
            attitude.y * attitude.z);

  gravity.z =
    attitude.w * attitude.w -
    attitude.x * attitude.x -
    attitude.y * attitude.y +
    attitude.z * attitude.z;

  Vector error = Vector::cross(accNorm, gravity);

  integral += error * MAHONY_Ki * dt;

  Vector correctedGyro =
    gyroRad +
    error * MAHONY_Kp +
    integral;

  Quaternion omega(
    0,
    correctedGyro.x,
    correctedGyro.y,
    correctedGyro.z
  );
  
  Quaternion qDot =
    (attitude * omega) * 0.5f;

  // Интегрируем гироскоп
  attitude = attitude + qDot * dt;
  attitude.normalize();

  rotationRad = attitude.toEuler();
  rotation = rotationRad * RAD_TO_DEG;

  // Serial.print(error.x);
  // Serial.print(" ");
  // Serial.print(error.y);
  // Serial.print(" ");
  // Serial.println(error.z);

  // Serial.println(rotation);
}
