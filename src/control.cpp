// Программа контроля полёта.
// Контроллирует моторы дрона и состояние арминга.

#include "HardwareSerial.h"

#include <Arduino.h>
#include "control.h"

#include "estimate.h"
#include "motors.h"
#include "rc.h"
#include "pid.h"
#include "imu.h"
#include "utils.h"
#include "quaternion.h"

#define PITCHRATE_P 0.05
#define PITCHRATE_I 0.01
#define PITCHRATE_D 0
#define PITCHRATE_I_LIM 0.3
#define ROLLRATE_P PITCHRATE_P
#define ROLLRATE_I PITCHRATE_I
#define ROLLRATE_D PITCHRATE_D
#define ROLLRATE_I_LIM PITCHRATE_I_LIM
#define YAWRATE_P 0.02
#define YAWRATE_I 0.0
#define YAWRATE_D 0.0
#define YAWRATE_I_LIM 0.3

#define ROLL_P 1
#define ROLL_I 0
#define ROLL_D 0
#define PITCH_P ROLL_P
#define PITCH_I ROLL_I
#define PITCH_D ROLL_D
#define YAW_P ROLL_P
#define YAW_I ROLL_I
#define YAW_D ROLL_D

#define PITCHRATE_MAX radians(200)
#define ROLLRATE_MAX radians(200)
#define YAWRATE_MAX radians(150)
#define RATES_D_LPF_ALPHA 0.2  // cutoff frequency ~ 40 Hz

PID rollRatePID(ROLLRATE_P, ROLLRATE_I, ROLLRATE_D, ROLLRATE_I_LIM, RATES_D_LPF_ALPHA);
PID pitchRatePID(PITCHRATE_P, PITCHRATE_I, PITCHRATE_D, PITCHRATE_I_LIM, RATES_D_LPF_ALPHA);
PID yawRatePID(YAWRATE_P, YAWRATE_I, YAWRATE_D);
PID rollPID(ROLL_P, ROLL_I, ROLL_D);
PID pitchPID(PITCH_P, PITCH_I, PITCH_D);
PID yawPID(YAW_P, YAW_I, YAW_D);

Vector maxRateRad(ROLLRATE_MAX, PITCHRATE_MAX, YAWRATE_MAX);

bool armed = true;  // Если арм, то моторы могут крутиться.

FlightMode flightMode = ANGLE;
FlightMode prevFlightMode = flightMode;

Quaternion attitudeTarget;  // Целевая ориентация дрона в виде кватерниона

Vector ratesTarget;  // Целевая угловая скорость (рад/с)
Vector ratesExtra;

float outRoll, outPitch, outYaw;


void control() {
  controlArming();
  
  controlModes();
  controlTorque();
}

void controlModes() {
  if (flightMode == ANGLE) {
    float yawTarget = attitudeTarget.getYaw();

    // Удержание курса при нейтральном стике и команда вращения при отклонённом стике
    if (!armed || isnan(yawTarget) || yaw_channel != 0) {
      yawTarget = attitude.getYaw();
    }

		attitudeTarget = Quaternion::fromEuler(Vector(
      roll_channel * DEG_TO_RAD,
      pitch_channel * DEG_TO_RAD,
      yawTarget * DEG_TO_RAD
    ));

    ratesExtra.z = yaw_channel * DEG_TO_RAD;
  }

  if (flightMode == ACRO) {
    attitudeTarget.invalidate();

    ratesTarget = Vector(
      roll_channel * DEG_TO_RAD,
      pitch_channel * DEG_TO_RAD,
      yaw_channel * DEG_TO_RAD
    );
    ratesTarget.constrainVector(maxRateRad);
  }
}

void controlTorque() {
  if (!armed || throttle_channel == 0) {
    writeAllMotors(0);

    resetControlPIDs();

    return;
  }

  // Если режим полёта был изменён
  if (flightMode != prevFlightMode) {
    resetControlPIDs();  // Сбрасываем накопления в ПИД
  }

  // Внешний контур
  controlAttitude();

  // Внутренний контур
  controlRates();

  // Микшер
  controlMixer();

  prevFlightMode = flightMode;
}

// Внешний контур. Вычисляет rates (целевую угловую скорость)
void controlAttitude() {
  // Если внешний контур отключен или если газ слишком мал - пропускаем
  if (attitudeTarget.invalid() || throttle_channel < 0.07) return;

  // Ошибка между настоящей вертикалью и целевой
	const Vector up(0, 0, 1);
	Vector upActual = Quaternion::rotateVector(up, attitude);
	Vector upTarget = Quaternion::rotateVector(up, attitudeTarget);

  // В радианах (потому что внутри используются acos и др.)
	Vector error = Vector::rotationVectorBetween(upTarget, upActual);

  // ПИД. Высчитываем целевую угловую скорость
	ratesTarget.x = rollPID.update(error.x) + ratesExtra.x;
	ratesTarget.y = pitchPID.update(error.y) + ratesExtra.y;

  // Ошибка курса
	float yawError = wrapAngle(attitudeTarget.getYaw() - attitude.getYaw());
	ratesTarget.z = yawPID.update(yawError) + ratesExtra.z;

  // Serial.println(ratesTarget);

  // Serial.println(ratesTarget);
}

// Внутренний контур. Rate регулятор
void controlRates() {
  Vector rateError = ratesTarget - gyroRad;
  // Serial.println(rateError);

  // ПИД моторов
  outRoll  = rollRatePID.update(rateError.x);
  outPitch = pitchRatePID.update(rateError.y);
  outYaw   = yawRatePID.update(rateError.z);
}

/*void controlHorizon() {
  // Берём выход ACRO
  controlAcro();
  float acroRollRateError  = rollRateError;
  float acroPitchRateError = pitchRateError;
  float acroYawRateError   = yawRateError;

  // Берём выход ANGLE
  controlAngle();
  float angleRollRateError  = rollRateError;
  float anglePitchRateError = pitchRateError;
  float angleYawRateError   = yawRateError;

  // Считаем силу преобладания ANGLE для всех осей
  float roll_horizon_strength = calculateHorizonStrength(roll_channel / 90.0f, rotation.x);
  float pitch_horizon_strength = calculateHorizonStrength(pitch_channel / 90.0f, rotation.y);
  float yaw_horizon_strength = 0;

  // Смешиваем по весу
  rollRateError = acroRollRateError * (1.0f - roll_horizon_strength) + angleRollRateError * roll_horizon_strength;
  pitchRateError = acroPitchRateError * (1.0f - pitch_horizon_strength) + anglePitchRateError * pitch_horizon_strength;
  yawRateError = acroYawRateError * (1.0f - yaw_horizon_strength) + angleYawRateError * yaw_horizon_strength;
}*/

void controlMixer() {
  // Микшер
  motors[MOTOR_FRONT_LEFT]  = constrain(
      throttle_channel + outRoll - outPitch + outYaw,
      0, 1
  );

  motors[MOTOR_FRONT_RIGHT] = constrain(
      throttle_channel - outRoll - outPitch - outYaw,
      0, 1
  );

  motors[MOTOR_REAR_RIGHT]  = constrain(
      throttle_channel - outRoll + outPitch + outYaw,
      0, 1
  );

  motors[MOTOR_REAR_LEFT]   = constrain(
      throttle_channel + outRoll + outPitch - outYaw,
      0, 1
  );

  // Serial.print(motors[MOTOR_FRONT_LEFT]);
  // Serial.print(" ");
  // Serial.print(motors[MOTOR_FRONT_RIGHT]);
  // Serial.print(" ");
  // Serial.print(motors[MOTOR_REAR_RIGHT]);
  // Serial.print(" ");
  // Serial.println(motors[MOTOR_REAR_LEFT]);


  //Serial.println(String(outRoll) + ", " + String(outPitch) + ", " + String(outYaw));
}

void controlArming() {
  // Если сигнал есть и моторы на минимальном газе, то: делаем арминг.
  if (motor_switch_channel == 1 && throttle_channel <= 0.03) {
    armed = true;
  }

  // Если нет радиосигнала, то дизарм.
  if (radioLost || motor_switch_channel == 0) {
    armed = false;
  }
}

// Сбрасывает ПИДы (например, интеграл)
void resetControlPIDs() {
  rollPID.reset();
  pitchPID.reset();

  rollRatePID.reset();
  pitchRatePID.reset();
  yawRatePID.reset();
}

/*
  Десатурация моторов.
  Чтобы даже при максимальном газе разница в управляющих сигналах была одинаковой, сохраняя полётный профиль
*/
void desaturate() {
	float max_ = -INFINITY, min_ = INFINITY;
	float correction = 0;

	// Находим максимум и минимум газа
	for (float m : motors) {
		if (m > max_) max_ = m;
		if (m < min_) min_ = m;
	}

	if (max_ > 1) {  // Газ превышает максимум
		correction = max_ - 1;
    
	} else if (min_ < 0) {  // Газ ниже минимума
		correction = min_;
	}

	for (float &m : motors) {
		m -= correction;
	}
}

// Преобладание режима (сила) ANGLE над режимом ACRO в режиме HORIZON.
float calculateHorizonStrength(
    float stickNormalized,       // -1.0 ... +1.0
    float angleDeg     // текущий угол наклона
) {
    constexpr float STICK_LIMIT = 0.75f;
    constexpr float ANGLE_LIMIT = 135.0f;

    // Удаление от центра стика уменьшает силу
    float stickFactor = 1.0f - constrain(abs(stickNormalized) / STICK_LIMIT, 0.0f, 1.0f);

    // При наклоненном дроне сила становится меньше
    float angleFactor = constrain(1.0f - abs(angleDeg) / ANGLE_LIMIT, 0.0f, 1.0f);

    return stickFactor * angleFactor;
}
