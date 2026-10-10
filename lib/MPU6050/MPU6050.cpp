#include <Arduino.h>
#include <Wire.h>

#include "MPU6050.h"
#include "register.h"


// Настройки
const float ACCEL_LSB_PER_G = 16384.0;  // +-2g
const float GYRO_LSB_PER_DPS = 131.0;   // +-250 градусов/с
const uint32_t MPU6050_CALIB_SAMPLES = 2000;  // Единицы

// Фиксированный период 1000 мкс -> 1000 Гц
const uint32_t PERIOD = 1000;  // Микросекунды

// Адреса
const uint8_t MPU_ADDR = 0x68;
const uint8_t WHO_AM_I = 0x75;
const uint8_t PWR_MGMT_1 = 0x6B;
const uint8_t ACCEL_CONFIG = 0x1C;
const uint8_t GYRO_CONFIG = 0x1B;
const uint8_t ACCEL_XOUT_H = 0x3B;
const uint8_t SMPLRT_DIV = 0x19;
const uint8_t CONFIG = 0x1A;


void MPU6050::init() {
  // 1. Выход из спящего режима, выбор внутреннего тактового генератора
  // Всё по нулям – пробуждение
  writeRegister(MPU_ADDR, PWR_MGMT_1, 0x00);

  // 2. Настройка гироскопа (+-250 гр/с, без самодиагностики)
  writeRegister(MPU_ADDR, GYRO_CONFIG, 0x00);

  // 3. Настройка акселерометра (+-2g, без самодиагностики)
  writeRegister(MPU_ADDR, ACCEL_CONFIG, 0x00);

  // Проверяем WHO_AM_I
  if (readRegister(MPU_ADDR, WHO_AM_I) == MPU_ADDR) {
    Serial.println("MPU6050 найден.");
  } else {
    Serial.println("Ошибка: датчик не обнаружен!");
  }
}

// Читает MPU6050 и записывает в указазанные переменные сырые данные используя Wire
bool MPU6050::readRaw(MPU6050Data &data) {
    Wire.beginTransmission(MPU_ADDR);
    // Стартовый адрес ACCEL_XOUT_H
    Wire.write(ACCEL_XOUT_H);
    if (Wire.endTransmission(false) != 0) {
        return false;
    }

    if (Wire.requestFrom(MPU_ADDR, (uint8_t)14) != 14) {
      return false;
    }

    uint8_t axH = Wire.read();
    uint8_t axL = Wire.read();

    uint8_t ayH = Wire.read();
    uint8_t ayL = Wire.read();

    uint8_t azH = Wire.read();
    uint8_t azL = Wire.read();

    uint8_t tempH = Wire.read();
    uint8_t tempL = Wire.read();

    uint8_t gxH = Wire.read();
    uint8_t gxL = Wire.read();

    uint8_t gyH = Wire.read();
    uint8_t gyL = Wire.read();

    uint8_t gzH = Wire.read();
    uint8_t gzL = Wire.read();

    data.ax = (int16_t)((axH << 8) | axL);
    data.ay = (int16_t)((ayH << 8) | ayL);
    data.az = (int16_t)((azH << 8) | azL);

    data.gx = (int16_t)((gxH << 8) | gxL);
    data.gy = (int16_t)((gyH << 8) | gyL);
    data.gz = (int16_t)((gzH << 8) | gzL);

    // Serial.printf(
    //     "AX: %02X %02X -> %d | "
    //     "AY: %02X %02X -> %d | "
    //     "AZ: %02X %02X -> %d\n",
    //     axH, axL, data.ax,
    //     ayH, ayL, data.ay,
    //     azH, azL, data.az
    // );

    return true;
}

void MPU6050::writeVectors(MPU6050Data &accgyroData, Vector &accG, Vector &gyroRad) {
    accG.x = accgyroData.ax / ACCEL_LSB_PER_G;
    accG.y = accgyroData.ay / ACCEL_LSB_PER_G;
    accG.z = accgyroData.az / ACCEL_LSB_PER_G;

    gyroRad.x = accgyroData.gx / GYRO_LSB_PER_DPS * DEG_TO_RAD;
    gyroRad.y = accgyroData.gy / GYRO_LSB_PER_DPS * DEG_TO_RAD;
    gyroRad.z = accgyroData.gz / GYRO_LSB_PER_DPS * DEG_TO_RAD;
}

void MPU6050::calibrate(MPU6050Data &accgyroBias) {
    MPU6050Data bias{};

    uint32_t validSamples = 0;

    for (uint32_t i = 0; i < MPU6050_CALIB_SAMPLES; i++) {
        MPU6050Data raw{};

        if (readRaw(raw)) {
            bias += raw;
            validSamples++;
        }

        delay(2);
    }

    if (validSamples == 0) {
        accgyroBias = {};
        return;
    }

    bias.ax /= validSamples;
    bias.ay /= validSamples;
    bias.az /= validSamples;

    bias.gx /= validSamples;
    bias.gy /= validSamples;
    bias.gz /= validSamples;

    // Убираем 1g с оси Z
    bias.az -= ACCEL_LSB_PER_G;

    accgyroBias = bias;
}

uint16_t MPU6050::getPeriod() {
    return PERIOD;
}
