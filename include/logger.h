#pragma once

#include <stdint.h>

#pragma pack(push, 1)

struct LogHeader
{
    char magic[4];          // "DRON"
    uint16_t version;       // Версия формата
    uint16_t frameSize;     // sizeof(LogFrame)
    uint32_t frameCount;    // Сколько кадров записано
};

struct LogFrame
{
    uint32_t t;

    int16_t  gx;
    int16_t  gy;
    int16_t  gz;

    int16_t  ax;
    int16_t  ay;
    int16_t  az;

    int16_t  roll;
    int16_t  pitch;
    int16_t  yaw;
};

#pragma pack(pop)

extern LogFrame logBuffer[];
extern uint32_t logIndex;

void startLogging();  // Начать логгирование
void stopLogging();  // Остановить логгирование
void handleLogging();  // Управление состояния логгирования
void logData();  // Итерация логгирования
void sendLog();  // Передать логи по Serial
