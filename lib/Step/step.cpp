#include <Arduino.h>
#include "step.h"

// Время сейчас, сек
float t;

// Дельта времени с предыдущей итерации, сек
float dt;

void step() {
	float now = micros() / 1000000.0;
	dt = now - t;
	t = now;

	if (!(dt > 0)) {
    // Предполагается что дельта равна 0 на первой итерации или после ресета
		dt = 0;
	}
}
