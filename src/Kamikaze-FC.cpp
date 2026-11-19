/*

Программа полётного контроллера.
Гитхаб: https://github.com/TheWolzow/Kamikaze

Вес дрона с аккумулятором: 550г
Аккум: 90г

Полезные ресурсы:
 Flix: https://github.com/okalachev/flix/tree/master/flix
 Mixer theory: https://cookierobotics.com/066/

TODO:

  Но есть один нюанс: если у твоего rollPID и pitchPID внутри используется интегральная составляющая, 
  вызов controlAngle() даже при нулевом весе ANGLE продолжит обновлять интегратор. 
  Для базовой реализации это не обязательно критично, но поведение лучше продумать: 
  например, не обновлять внешний Angle PID, когда его влияние не требуется, 
  либо корректно сбрасывать/ограничивать интегратор.

*/

#include <Arduino.h>
#include <Wire.h>

#include "rc.h"
#include "cli.h"
#include "led.h"
#include "imu.h"
#include "step.h"
#include "motors.h"
#include "logger.h"
#include "control.h"
#include "espwifi.h"
#include "estimate.h"

//LEDStrip strip(4, 10);  // Пин 4, n светодиодов



void setup() {
  Serial.begin(115200);

  Wire.begin(21, 22);  // SDA=21, SCL=22
  Wire.setClock(400000);
  
  delay(1000);

  attachMotors();

  beginSBUS();
  
  initIMU();
  calibrateIMU();

  setupWiFi();
}

void loop() {
  readIMU();

  if (imu_updated) {
    step();
    readRC();
    estimate();
    control();
    writeMotors();

    handleLogging();  // Логгирование
  }

  handleCLI();  // Интерфейс командной строки
}