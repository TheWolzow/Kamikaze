#include <Arduino.h>
#include "motors.h"

#include "utils.h"

float motors[4];  // Нормализованная тяга моторов в пределах [0..1]

// ===== НАСТРОЙКИ =====
static const int motorPins[4] = {16, 17, 18, 19};
static const int motorChannels[4] = {0, 1, 2, 3};

static const int PWM_FREQ = 400;       // 50 или 400 Гц для ESC
static const int PWM_RESOLUTION = 10;  // 10 или 12 бит

// диапазон ESC
static const int MIN_US = 800;
static const int MAX_US = 1800;

const int MOTOR_FRONT_LEFT = 0;
const int MOTOR_FRONT_RIGHT = 1;
const int MOTOR_REAR_RIGHT = 2;
const int MOTOR_REAR_LEFT = 3;


// Пересчёт микросекунд в duty
uint32_t usToDuty(int us) {
  // максимальное число, которое можно записать в N бит
  uint32_t maxDuty = (1 << PWM_RESOLUTION) - 1;
  /*
  Если PWM_RESOLUTION = 10:
  1        = 0000000001
  1 << 10  = 10000000000

  это число:

  2^10 = 1024

  при 10 битах максимум:

  1111111111 (10 единиц) = 1023

  Итог:
  maxDuty = 2^PWM_RESOLUTION - 1
  Биты	maxDuty
  8	255
  10	1023
  12	4095
  */

  // шаг 2: период сигнала
  float period_us = 1000000.0 / PWM_FREQ; // 20 000 us

  // шаг 3: перевод
  return (us / period_us) * maxDuty;

  /*
  пример:
  если us = 1250:
  1250 / 2500 = 0.5
  при 10 бит:
  maxDuty = 1023
   итог:
  duty ≈ 511
  то есть:
  "сигнал включён 50% времени"
  */
}

// ===== INIT =====
void attachMotors() {
  for (int i = 0; i < 4; i++) {
    ledcSetup(motorChannels[i], PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(motorPins[i], motorChannels[i]);
  }

  writeAllMotors(0.0);
  delay(6000); // арминга ESC
}

// ===== ЗАПИСЬ =====
void writeMotors() {
  for (int i = 0; i < 4; i++) {
    motors[i] = constrain(motors[i], 0.0, 1.0);

    int us = MIN_US + motors[i] * (MAX_US - MIN_US);
    uint32_t duty = usToDuty(us);

    ledcWrite(motorChannels[i], duty);
  }
}

void writeAllMotors(float throttle) {
  for (int i = 0; i < 4; i++) {
    motors[i] = throttle;
  }
  writeMotors();
}

bool motorsActive() {
	return motors[0] != 0 || motors[1] != 0 || motors[2] != 0 || motors[3] != 0;
}
