#pragma once

#define MIN_MOTOR 800
#define MAX_MOTOR 1800

extern float motors[4];

extern const int MOTOR_FRONT_LEFT;
extern const int MOTOR_FRONT_RIGHT;
extern const int MOTOR_REAR_RIGHT;
extern const int MOTOR_REAR_LEFT;

void attachMotors();
void writeAllMotors(float throttle);
void writeMotors();

bool motorsActive();
void applyMicrosecondsToAll(int value);
