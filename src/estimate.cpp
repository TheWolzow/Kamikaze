// roll - крен
// yaw - рыскание
// pitch - тангаж

// Отрицательный тангаж - наклон вниз.
// Отрицательный крен - наклон вправо.

#include <Arduino.h>
#include "estimate.h"

#include "imu.h"
#include "step.h"
#include "vector.h"

Vector rotation;  // Ориентация дрона в градусах
Vector rotationRad;  // Ориентация дрона в радианах

static float complementary_alpha = 0.77;


void estimate() {
  // Градусы
  float roll_acc = atan2(accG.y, accG.z);
  float pitch_acc = atan2(accG.x, sqrt(accG.y * accG.y + accG.z * accG.z));

  // Комплементарный фильтр
  rotationRad.x = complementary_alpha * (rotationRad.x  + gyroRad.x * dt) + (1 - complementary_alpha) * roll_acc;
  rotationRad.y = complementary_alpha * (rotationRad.y + gyroRad.y * dt) + (1 - complementary_alpha) * pitch_acc;
  rotationRad.z = rotationRad.z + gyroRad.z * dt;

  rotation = rotationRad * RAD_TO_DEG;

  //Serial.println(rotation);
}
