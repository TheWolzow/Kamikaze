#pragma once

#include "lpf.h"
#include "step.h"

class PID {
public:
  	float p, i, d;
	float windup;
	float dtMax;

	float derivative = 0;
	float integral = 0;

	LowPassFilter<float> lpf;  // Фильтр нижних частот для D компоненты

	PID(float p, float i, float d, float windup = 0, float dAlpha = 1, float dtMax = 0.1) :
		p(p), i(i), d(d), windup(windup), lpf(dAlpha), dtMax(dtMax) {}

	float update(float error) {
		if (dt > 0 && dt < dtMax && throttle_channel > 0.05f) {
			// Защита от первого вызова после сброса
			if (isnan(prevError)) {
				prevError = error;
				derivative = 0;
			} else {
				integral += error * dt;
				derivative = lpf.update((error - prevError) / dt);
			}
		} else {
			integral = 0;
			derivative = 0;
			// Опционально: prevError = error; чтобы при следующем входе производная считалась корректно
			prevError = error;
		}
		prevError = error;   // обновляется в любом случае

		return p * error + constrain(i * integral, -windup, windup) + d * derivative;
	}

	void reset() {
		prevError = NAN;
		integral = 0;
		derivative = 0;
		lpf.reset();
	}

private:
	float prevError = NAN;
	float prevTime = NAN;
};
