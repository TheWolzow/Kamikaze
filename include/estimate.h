#pragma once

#include "vector.h"
#include "quaternion.h"

extern Vector rotation;
extern Vector rotationRad;

extern Quaternion attitude;

void estimate();
