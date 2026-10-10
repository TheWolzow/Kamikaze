#pragma once

#include <Arduino.h>
#include "vector.h"


class MPU6050Data : public Printable {
public:
    int64_t ax, ay, az, gx, gy, gz;

    MPU6050Data(): ax(0), ay(0), az(0), gx(0), gy(0), gz(0) {};

    MPU6050Data(
        int64_t ax, int64_t ay, int64_t az,
        int64_t gx, int64_t gy, int64_t gz
    ): ax(ax), ay(ay), az(az), gx(gx), gy(gy), gz(gz) {};

    MPU6050Data operator + (const MPU6050Data& b) const {
		return MPU6050Data(
            ax + b.ax, ay + b.ay, az + b.az,
            gx + b.gx, gy + b.gy, gz + b.gz
        );
	}

	MPU6050Data operator - (const MPU6050Data& b) const {
		return MPU6050Data(
            ax - b.ax, ay - b.ay, az - b.az,
            gx - b.gx, gy - b.gy, gz - b.gz
        );
	}

	MPU6050Data& operator += (const MPU6050Data& b) {
		return *this = *this + b;
	}

	MPU6050Data& operator -= (const MPU6050Data& b) {
		return *this = *this - b;
	}

    size_t printTo(Print& p) const {
		return
			p.print(ax) + p.print(" ") +
			p.print(ay) + p.print(" ") +
            p.print(az) + p.print(" ") +
            p.print(gx) + p.print(" ") +
            p.print(gy) + p.print(" ") +
			p.print(gz);
	}
};

class MPU6050 {
public:
	void init();
    void calibrate(MPU6050Data &accgyroBias);

    bool readRaw(MPU6050Data& data);
    void writeVectors(MPU6050Data &accgyroData, Vector &accG, Vector &gyroRad);

    uint16_t getPeriod();
};
