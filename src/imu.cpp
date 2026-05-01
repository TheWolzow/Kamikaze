#include <Arduino.h>
#include "imu.h"

#include <Wire.h>
#include "vector.h"
#include "utils.h"

#define MPU_ADDR 0x68

const float ACCEL_LSB_PER_G = 16384.0;  // +-2g
const float GYRO_LSB_PER_DPS = 131.0;   // +-250 градусов/с

const short CALIB_SAMPLES = 2000;

const unsigned long period = 2000;  // микросекунды

unsigned long last_time = 0;

Vector gyroRad;  // gyroscope output, rad/s
Vector gyroDeg;  // gyroscope output, deg/s
Vector gyroBias;  // raw gyro bias

Vector acc;  // accelerometer output, m/s/s
Vector accG;  // accelerometer output, g
Vector accBias;  // raw acc bias


void initIMU() {
  Serial.println("Initializing IMU");
  
  Wire.begin(21, 22);               // SDA=21, SCL=22 (настройте под свою плату)
  Wire.setClock(400000);            // 400 кГц

  // 1. Выход из спящего режима, выбор внутреннего тактового генератора
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);                 // PWR_MGMT_1
  Wire.write(0x00);                 // всё по нулям – пробуждение
  Wire.endTransmission();

  // 2. Настройка гироскопа (+-250 гр/с, без самодиагностики)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B);                 // GYRO_CONFIG
  Wire.write(0x00);
  Wire.endTransmission();

  // 3. Настройка акселерометра (+-2g, без самодиагностики)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C);                 // ACCEL_CONFIG
  Wire.write(0x00);
  Wire.endTransmission();

  // Опционально: проверить WHO_AM_I
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x75);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 1);
  if (Wire.read() == 0x68) {
    Serial.println("MPU6050 найден.");
  } else {
    Serial.println("Ошибка: датчик не обнаружен!");
  }

  last_time = micros();
}

// Читает и записывает в указазанные переменные сырые данные используя Wire
void mpu6050_read_raw(int16_t &ax, int16_t &ay, int16_t &az,
                      int16_t &gx, int16_t &gy, int16_t &gz) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);                // стартовый адрес ACCEL_XOUT_H
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, (uint8_t) 14);

  ax = (Wire.read() << 8) | Wire.read();
  ay = (Wire.read() << 8) | Wire.read();
  az = (Wire.read() << 8) | Wire.read();
  // Пропускаем температуру (2 байта)
  Wire.read(); Wire.read();
  gx = (Wire.read() << 8) | Wire.read();
  gy = (Wire.read() << 8) | Wire.read();
  gz = (Wire.read() << 8) | Wire.read();
}

// Читает сырые данные и составляет из них вектора
void readIMU() {
  // Фиксированный период 2000 мкс -> 500 Гц
  unsigned long now = micros();
  
  if (now - last_time >= period) {
    last_time += period;  // не now, чтобы избежать накопления дрейфа

    int16_t ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw;
    mpu6050_read_raw(ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw);

    // Применяем оффсеты
    ax_raw -= accBias.x;
    ay_raw -= accBias.y;
    az_raw -= accBias.z;
    gx_raw -= gyroBias.x;
    gy_raw -= gyroBias.y;
    gz_raw -= gyroBias.z;

    accG.x = ax_raw / ACCEL_LSB_PER_G;
    accG.y = ay_raw / ACCEL_LSB_PER_G;
    accG.z = az_raw / ACCEL_LSB_PER_G;

    gyroRad.x = gx_raw / GYRO_LSB_PER_DPS * DEG_TO_RAD;
    gyroRad.y = gy_raw / GYRO_LSB_PER_DPS * DEG_TO_RAD;
    gyroRad.z = gz_raw / GYRO_LSB_PER_DPS * DEG_TO_RAD;
  }
}

void calibrateAsync() {
  Serial.println("Не двигай датчик! Калибровка начинается...");

  Vector accOffset;
  Vector gyroOffset;

  for (int i = 0; i < CALIB_SAMPLES; i++) {
    int16_t ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw;
    mpu6050_read_raw(ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw);

    accOffset += Vector(ax_raw, ay_raw, az_raw);
    gyroOffset += Vector(gx_raw, gy_raw, gz_raw);

    delay(2);  // небольшая пауза, чтобы набрать статистику
  }

  accOffset.x /= CALIB_SAMPLES;
  accOffset.y /= CALIB_SAMPLES;
  accOffset.z = (accOffset.z / CALIB_SAMPLES) - ACCEL_LSB_PER_G;  // гравитация

  gyroOffset.x /= CALIB_SAMPLES;
  gyroOffset.y /= CALIB_SAMPLES;
  gyroOffset.z /= CALIB_SAMPLES;

  //Serial.println("\nСохрани и вычитай их из измерений");*/

  accBias = accOffset;
  gyroBias = gyroOffset;
}
