#pragma once

extern bool radioLost;

// Каналы
extern float throttle_channel;  // Газ
extern float yaw_channel;  // Рыскание в градусах
extern float roll_channel;  // Крен в градусах
extern float pitch_channel;  // Тангаж в градусах

extern bool motor_switch_channel;  // 0 - Моторы выключены. 1 - Моторы включены.

void beginSBUS();
void readRC();
void readSBUS();
void decodeSBUS();
void parseSBUS();
