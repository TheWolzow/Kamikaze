#pragma once

inline float mapf(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Wrap angle to [-PI, PI)
inline float wrapAngle(float angle) {
	angle = fmodf(angle, 2 * PI);
  
	if (angle > PI) {
		angle -= 2 * PI;
	} else if (angle < -PI) {
		angle += 2 * PI;
	}
  
	return angle;
}

// Disable reset on low voltage
// void disableBrownOut() {
// 	REG_CLR_BIT(RTC_CNTL_BROWN_OUT_REG, RTC_CNTL_BROWN_OUT_ENA);
// }
