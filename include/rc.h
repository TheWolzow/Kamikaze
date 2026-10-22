#pragma once

extern bool radioLost;

// Каналы
extern float throttle_channel;  // Газ [0; 1]
extern float yaw_channel;  // Рыскание в градусах
extern float roll_channel;  // Крен в градусах
extern float pitch_channel;  // Тангаж в градусах

extern bool motor_switch_channel;  // 0 - Моторы выключены. 1 - Моторы включены.
extern bool log_state_channel;  // 0 - Логгирование выключено. 1 - Логгирование включено
extern bool log_send_channel;  // 0 - Дампинг логов отключён. 1 - Дампинг логов включён.

void beginSBUS();
void readRC();
void readSBUS();
void decodeSBUS();
void parseSBUS();
