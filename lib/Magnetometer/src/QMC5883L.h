#pragma once

#include <Arduino.h>


class QMC5883LData : public Printable {
public:
    int16_t x, y, z;

    QMC5883LData(): x(0), y(0), z(0) {};

    QMC5883LData(int16_t x, int16_t y, int16_t z): x(x), y(y), z(z) {};

    size_t printTo(Print& p) const {
		return
			p.print(x) + p.print(" ") +
			p.print(y) + p.print(" ") +
			p.print(z);
	}
};

class QMC5883L {
public:
	void init();

    bool read(QMC5883LData& data);

    bool isDataReady();
};
