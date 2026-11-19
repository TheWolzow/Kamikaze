#pragma once

#include <math.h>
#include "vector.h"

class Quaternion : public Printable {
public:
    float w, x, y, z;

    Quaternion() : w(1), x(0), y(0), z(0) {}
    Quaternion(float w_, float x_, float y_, float z_) : w(w_), x(x_), y(y_), z(z_) {}

    static Quaternion fromEuler(const Vector &euler) {
		float cx = cosf(euler.x / 2);
		float cy = cosf(euler.y / 2);
		float cz = cosf(euler.z / 2);
		float sx = sinf(euler.x / 2);
		float sy = sinf(euler.y / 2);
		float sz = sinf(euler.z / 2);

		return Quaternion(
			cx * cy * cz + sx * sy * sz,
			sx * cy * cz - cx * sy * sz,
			cx * sy * cz + sx * cy * sz,
			cx * cy * sz - sx * sy * cz
        );
	}

    static Vector rotateVector(const Vector& v, const Quaternion& q) {
        return q.conjugateInversed(v);
    }

    bool finite() const {
		return isfinite(w) && isfinite(x) && isfinite(y) && isfinite(z);
	}

	bool valid() const {
		return finite();
	}

	bool invalid() const {
		return !valid();
	}

    float getRoll() const {
		return toEuler().x;
	}

	float getPitch() const {
		return toEuler().y;
	}

	float getYaw() const {
		return toEuler().z;
	}

	void invalidate() {
		w = NAN;
		x = NAN;
		y = NAN;
		z = NAN;
	}

    void normalize() {
        float mag = sqrtf(
            w*w +
            x*x +
            y*y +
            z*z
        );

        if (mag == 0) return;

        w /= mag;
        x /= mag;
        y /= mag;
        z /= mag;
    }

    float dot(const Quaternion &q) const {
        return w*q.w + x*q.x + y*q.y + z*q.z;
    }

    Quaternion conjugate() const {
        return Quaternion(w, -x, -y, -z);
    }

    Quaternion inverse() const {
        float normSq = w*w + x*x + y*y + z*z;

        if (normSq == 0.0f) return Quaternion();

        return conjugate() * (1.0f / normSq);
    }

    Vector conjugateInversed(const Vector& v) const {
        Quaternion vectorQuat(0, v.x, v.y, v.z);

        Quaternion result = inverse() * vectorQuat * (*this);

        return Vector(result.x, result.y, result.z);
    }

    // Возвращает вектор вращения в радианах из кватерниона
    Vector toEuler() const {
        Vector euler;

        euler.x = atan2f(
            2*(w*x + y*z),
            1 - 2*(x*x + y*y)
        );

        euler.y = asinf(
            constrain(2*(w*y - z*x), -1.0f, 1.0f)
        );

        euler.z = atan2f(
            2*(w*z + x*y),
            1 - 2*(y*y + z*z)
        );

        return euler;
    }

    Quaternion operator * (const Quaternion &q) const {
        return Quaternion(
            w*q.w - x*q.x - y*q.y - z*q.z,

            w*q.x + x*q.w + y*q.z - z*q.y,

            w*q.y - x*q.z + y*q.w + z*q.x,

            w*q.z + x*q.y - y*q.x + z*q.w
        );
    }

    Quaternion operator * (float scalar) const {
        return Quaternion(w * scalar, x * scalar, y * scalar, z * scalar);
    }

    Quaternion operator + (const Quaternion& q) const {
        return Quaternion(w + q.w, x + q.x, y + q.y, z + q.z);
    }

    Quaternion operator - (const Quaternion& q) const {
        return Quaternion(w - q.w, x - q.x, y - q.y, z - q.z);
    }

    size_t printTo(Print& p) const {
		return
            p.print(w, 5) + p.print(" ") +
			p.print(x, 5) + p.print(" ") +
			p.print(y, 5) + p.print(" ") +
			p.print(z, 5);
	}
};
