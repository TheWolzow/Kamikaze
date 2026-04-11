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
		if (dt > 0 && dt < dtMax) {
			integral += error * dt;
      // Высчитываем производную и применяем фильтр нижних частот (lpf)
			derivative = lpf.update((error - prevError) / dt);
		
    } else {
			integral = 0;
			derivative = 0;
		}

		prevError = error;

    	// PID
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
