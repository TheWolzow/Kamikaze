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

static float complementary_alpha = 0.77;


void estimate() {
  // Градусы
  float roll_acc = atan2(accG.y, accG.z) * RAD_TO_DEG;
  float pitch_acc = atan2(accG.x, sqrt(accG.y * accG.y + accG.z * accG.z)) * RAD_TO_DEG;

  // Комплементарный фильтр
  rotation.x = complementary_alpha * (rotation.x  + gyroDeg.x * dt) + (1 - complementary_alpha) * roll_acc;
  rotation.y = complementary_alpha * (rotation.y + gyroDeg.y * dt) + (1 - complementary_alpha) * pitch_acc;
  rotation.z = rotation.z + gyroDeg.z * dt;

  // Serial.println(rotation);
}
