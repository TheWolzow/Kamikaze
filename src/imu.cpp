#include <Arduino.h>
#include "imu.h"

#include <Wire.h>
#include "QMC5883L.h"
#include "MPU6050.h"
#include "vector.h"
#include "utils.h"

bool imu_updated = false;

unsigned long last_time = 0;

const float G_CONST = 9.8;  //  м/с^2

Vector acc;  // accelerometer output, m/s/s
Vector accNorm;  // accelerometer output, normalized acc
Vector accG;  // accelerometer output, g

Vector gyroRad;  // gyroscope output, rad/s

MPU6050 accgyro;
MPU6050Data accgyroData;
MPU6050Data accgyroBias;  // raw acc-gyro bias

QMC5883L magnetometer;
QMC5883LData magData;


void initIMU() {
  // Акселерометр-гироскоп
  accgyro.init();

  // Магнитометр
  magnetometer.init();

  last_time = micros();

  Serial.println("Инициализация IMU: успешно");
}

// Читает сырые данные и составляет из них вектора
void readIMU() {
  unsigned long now = micros();
  
  if (now - last_time >= accgyro.getPeriod()) {
    last_time += accgyro.getPeriod();  // не now, чтобы избежать накопления дрейфа

    // Читаем сырые данные акселерометра-гироскопа
    bool success = accgyro.readRaw(accgyroData);
    if (!success) {
      imu_updated = false;

      Serial.println("accgyro.readRaw failed");
    }

    // Serial.println(accgyroData);

    // Применяем оффсеты
    accgyroData -= accgyroBias;

    // Записываем вектора. Формируем accG и gyroRad через accgyroData
    accgyro.writeVectors(accgyroData, accG, gyroRad);

    acc = accG * G_CONST;
    accNorm = acc.normalize();

    // Читаем сырые данные магнитометра
    //magnetometer.read(magData);

   

    // Serial.println(acc);
    //Serial.println(magData);

    imu_updated = true;

  } else {
    imu_updated = false;
  }
}

void calibrateIMU() {
  Serial.println("Не двигайте дрон! Калибровка начинается...");

  accgyro.calibrate(accgyroBias);

  Serial.println("Калибровка завершена.");
  Serial.println(accgyroBias);
}
