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

#define PITCHRATE_P 0.01
#define PITCHRATE_I 0
#define PITCHRATE_D 0.01
#define PITCHRATE_I_LIM 0.3
#define ROLLRATE_P PITCHRATE_P
#define ROLLRATE_I PITCHRATE_I
#define ROLLRATE_D PITCHRATE_D
#define ROLLRATE_I_LIM PITCHRATE_I_LIM
#define YAWRATE_P 0.0
#define YAWRATE_I 0.0
#define YAWRATE_D 0.0
#define YAWRATE_I_LIM 0.3
#define ROLL_P 0
#define ROLL_I 0
#define ROLL_D 0
#define PITCH_P ROLL_P
#define PITCH_I ROLL_I
#define PITCH_D ROLL_D
#define PITCHRATE_MAX radians(200)
#define ROLLRATE_MAX radians(200)
#define YAWRATE_MAX radians(150)
#define RATES_D_LPF_ALPHA 0.2  // cutoff frequency ~ 40 Hz

PID rollRatePID(ROLLRATE_P, ROLLRATE_I, ROLLRATE_D, ROLLRATE_I_LIM, RATES_D_LPF_ALPHA);
PID pitchRatePID(PITCHRATE_P, PITCHRATE_I, PITCHRATE_D, PITCHRATE_I_LIM, RATES_D_LPF_ALPHA);
PID yawRatePID(YAWRATE_P, YAWRATE_I, YAWRATE_D);
PID rollPID(ROLL_P, ROLL_I, ROLL_D);
PID pitchPID(PITCH_P, PITCH_I, PITCH_D);
Vector maxRate(ROLLRATE_MAX, PITCHRATE_MAX, YAWRATE_MAX);

bool armed = true;  // Если арм, то моторы могут крутиться.


void control() {
  controlTorque();
  controlArming();
}

void controlTorque() {
  if (!armed || throttle_channel == 0) {
    writeAllMotors(0);

    rollPID.reset();
    pitchPID.reset();

    rollRatePID.reset();
    pitchRatePID.reset();
    yawRatePID.reset();

    return;
  }

  // Ошибка угла в радианах
  float rollError = roll_channel * DEG_TO_RAD - rotationRad.x;
  float pitchError = pitch_channel * DEG_TO_RAD - rotationRad.y;

  //Serial.println(String(roll_channel) + ", " + String(pitch_channel));
  //Serial.println(String(rollError) + ", " + String(pitchError));

  // ПИД целевой угловой скорости
  float targetRollRate = rollPID.update(rollError);
  float targetPitchRate = pitchPID.update(pitchError);
  // Ограничиваем максимальную скорость вращения
  targetRollRate = constrain(targetRollRate, -maxRate.x, maxRate.x);
  targetPitchRate = constrain(targetPitchRate, -maxRate.y, maxRate.y);

  float targetYawRate = yaw_channel * DEG_TO_RAD * maxRate.z;
  
  // ПИД угловой скорости в радианах
  float rollRateError  = targetRollRate  - gyroRad.x;
  float pitchRateError = targetPitchRate - gyroRad.y;
  float yawRateError   = targetYawRate   - gyroRad.z;
  
  // ПИД моторов
  float outRoll  = rollRatePID.update(rollRateError);
  float outPitch = pitchRatePID.update(pitchRateError);
  float outYaw   = yawRatePID.update(yawRateError);

  // Serial.print(rollRateError);
  // Serial.print(", ");
  // Serial.print(targetRollRate);
  // Serial.print(", ");
  // Serial.print(gyroRad);
  // Serial.print(", ");
  // Serial.println(outRoll);

  // Микшер
  motors[MOTOR_FRONT_LEFT]  = constrain(throttle_channel + outRoll + outPitch - outYaw, 0, 1);
  motors[MOTOR_FRONT_RIGHT] = constrain(throttle_channel - outRoll + outPitch + outYaw, 0, 1);
  motors[MOTOR_REAR_RIGHT]  = constrain(throttle_channel - outRoll - outPitch - outYaw, 0, 1);
  motors[MOTOR_REAR_LEFT]   = constrain(throttle_channel + outRoll - outPitch + outYaw, 0, 1);

  //Serial.println(String(outRoll) + ", " + String(outPitch) + ", " + String(outYaw));
  //Serial.println(String(throttle_channel));

  //Serial.println(String(roll_channel) + ", " + String(pitch_channel));

  // Serial.print(motors[MOTOR_FRONT_LEFT]);
  // Serial.print(", ");
  // Serial.print(motors[MOTOR_FRONT_RIGHT]);
  // Serial.print(", ");
  // Serial.print(motors[MOTOR_REAR_RIGHT]);
  // Serial.print(", ");
  // Serial.println(motors[MOTOR_REAR_LEFT]);
}

void controlArming() {
  // Если сигнал есть, а моторы на минимальном газе, то: делаем арминг.
  if (motor_switch_channel == 1 && throttle_channel <= MIN_MOTOR + 50) {
    armed = true;
  }

  // Если нет радиосигнала, то дизарм.
  if (radioLost || motor_switch_channel == 0) {
    armed = false;
  }
  
  if (!armed) {
		return;
	}
}
