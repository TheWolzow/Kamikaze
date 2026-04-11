/*

Программа полётного контроллера.

Эта учебная программа создана по мотивам Flix
https://github.com/okalachev/flix/tree/master/flix

*/

#include <Arduino.h>
#include "step.h"
#include "rc.h"
#include "led.h"
#include "imu.h"
#include "motors.h"
#include "control.h"
#include "estimate.h"


void setup() {
  Serial.begin(115200);

  pinMode(2, OUTPUT);

  attachMotors();
  beginSBUS();
  setupLEDs();
  setupIMU();
  //delay(6000);
  
  calibrateAsync();
}

void loop() {
  readIMU();
  step();
  readRC();
  estimate();
  control();
  writeMotors();
  //handleInput();  // cli
  //logData();  // log
  //syncParameters();  // params
}