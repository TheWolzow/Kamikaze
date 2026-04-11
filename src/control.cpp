#include "HardwareSerial.h"

#include <Arduino.h>
#include "control.h"

#include "estimate.h"
#include "motors.h"
#include "rc.h"
#include "pid.h"


#define ROLL_P 0.2
#define ROLL_I 0.01
#define ROLL_D 0
#define PITCH_P ROLL_P
#define PITCH_I ROLL_I
#define PITCH_D ROLL_D
//#define YAW_P 3

PID rollPID(ROLL_P, ROLL_I, ROLL_D, 0.3);
PID pitchPID(PITCH_P, PITCH_I, PITCH_D, 0.3);
//PID yawPID(YAW_P, 0, 0);

bool armed = true;  // Если арм, то моторы могут крутиться.


void control() {
  controlTorque();
  controlArming();
}

void controlTorque() {
  if (!armed) {
    return;
  }

  float rollError = roll_channel - rotation.x;
  float pitchError = pitch_channel - rotation.y;

  // Serial.println(String(rollError) + ", " + String(pitchError));

  float outRoll = rollPID.update(rollError);
  float outPitch = pitchPID.update(pitchError);

  Serial.println(String(outRoll) + ", " + String(outPitch));

  motors[MOTOR_FRONT_LEFT] = constrain(throttle_channel + outRoll + outPitch, 0, 1);
  motors[MOTOR_FRONT_RIGHT] = constrain(throttle_channel - outRoll + outPitch, 0, 1);
  motors[MOTOR_REAR_RIGHT] = constrain(throttle_channel - outRoll - outPitch, 0, 1);
  motors[MOTOR_REAR_LEFT] = constrain(throttle_channel + outRoll - outPitch, 0, 1);
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
		memset(motors, 0, sizeof(motors));  // Останавливаем моторы при дизарме
		return;
	}
}
