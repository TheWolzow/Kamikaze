/*

Программа полётного контроллера.

Вес дрона с аккумулятором: 550г
Аккум: 90г

Полезные ресурсы:
 Flix: https://github.com/okalachev/flix/tree/master/flix

TODO лист:

3. При резком изменении режима (когда сделаю) сбрасывать ПИДы (PID::reset)

Например:

смена Acro → Angle
включение стабилизации

*/

#include <Arduino.h>
#include <Wire.h>
#include "step.h"
#include "rc.h"
#include "led.h"
#include "imu.h"
#include "motors.h"
#include "control.h"
#include "estimate.h"

// LEDStrip strip(13, 10);  // Пин 13, n светодиодов


void setup() {
  Serial.begin(115200);

  Wire.begin(21, 22);  // SDA=21, SCL=22
  Wire.setClock(400000);

  // strip.begin();
  
  // // Тест
  // strip.setPixel(0, 255, 0, 0);
  // strip.setPixel(1, 255, 0, 0);

  // strip.setPixel(5, 0, 0, 255);
  // strip.setPixel(6, 0, 0, 255);
  // strip.show();
  
  delay(1000);

  attachMotors();

  /*
  Если нужна калибровка моторов, стоит попробовать это:
  calibrateMotors();
  return;
  */

  beginSBUS();
  initIMU();
  //delay(6000);
  
  calibrateIMU();
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