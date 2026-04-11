#pragma once

struct RGB {
  uint8_t R, G, B;
};

void writeRGB(RGB rgbValue);
void setupLEDs();
