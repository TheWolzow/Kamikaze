#include <Arduino.h>
#include "led.h"

#define LED_R_PIN 12
#define LED_G_PIN 13
#define LED_B_PIN 14

static RGB setupLEDsColor = {0, 255, 0};


void setupLEDs() {
  pinMode(LED_R_PIN, OUTPUT);
  pinMode(LED_G_PIN, OUTPUT);
  pinMode(LED_B_PIN, OUTPUT);

  writeRGB(setupLEDsColor);
}

void writeRGB(RGB rgbValue) {
  analogWrite(LED_R_PIN, rgbValue.R);
  analogWrite(LED_G_PIN, rgbValue.G);
  analogWrite(LED_B_PIN, rgbValue.B);
}
