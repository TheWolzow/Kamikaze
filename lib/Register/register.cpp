#include <Wire.h>

#include "register.h"


void writeRegister(uint8_t addr, uint8_t reg, uint8_t value) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
}

uint8_t readRegister(uint8_t addr, uint8_t reg) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.endTransmission(false);

    Wire.requestFrom(addr, 1);

    if (Wire.available()) {
        return Wire.read();
    }

    return 0;
}
