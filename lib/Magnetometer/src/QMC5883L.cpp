// Даташит: https://static.chipdip.ru/lib/490/DOC042490192.pdf

#include <Arduino.h>
#include <Wire.h>

#include "QMC5883L.h"
#include "register.h"


#define QMC5883L_ADDR 0x0D

// Configurations
#define REG_CONTROL1 0x09
#define REG_CONTROL2 0x0A

#define REG_DATA_X_LSB 0x00
#define REG_STATUS 0x06

// Mode
#define MODE_STANDBY 0b00
#define MODE_CONTINUOUS 0b01

// Output data rate
#define ODR_200Hz 0b11

// Range
#define RNG_8G 0b01  // Gauss

// Over sample ratio
#define OSR_512 0b00


void QMC5883L::init() {
    uint8_t config =
      (OSR_512  << 6)
    | (RNG_8G   << 4)
    | (ODR_200Hz  << 2)
    | MODE_CONTINUOUS;

    writeRegister(QMC5883L_ADDR, REG_CONTROL1, config);
}

bool QMC5883L::read(QMC5883LData &data) {
    if (!isDataReady())
        return false;

    // Говорим QMC, что хотим читать начиная с регистра 0x00
    Wire.beginTransmission(QMC5883L_ADDR);
    Wire.write(REG_DATA_X_LSB);
    Wire.endTransmission(false);

    // Запрашиваем 6 байт:
    // X_LSB, X_MSB, Y_LSB, Y_MSB, Z_LSB, Z_MSB
    if (Wire.requestFrom(QMC5883L_ADDR, 6) != 6)
        return false;

    uint8_t xLSB = Wire.read();
    uint8_t xMSB = Wire.read();

    uint8_t yLSB = Wire.read();
    uint8_t yMSB = Wire.read();

    uint8_t zLSB = Wire.read();
    uint8_t zMSB = Wire.read();

    data.x = (int16_t) ((xMSB << 8) | xLSB);
    data.y = (int16_t) ((yMSB << 8) | yLSB);
    data.z = (int16_t) ((zMSB << 8) | zLSB);

    return true;
}

bool QMC5883L::isDataReady() {
    uint8_t status = readRegister(QMC5883L_ADDR, REG_STATUS);

    return status & 0b00000001;
}
