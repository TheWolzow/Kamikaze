#include <Arduino.h>

#include "rc.h"
#include "imu.h"
#include "logger.h"
#include "espwifi.h"
#include "estimate.h"

#define MAX_LOG_FRAMES 2000

enum LogType {
    WIFI_LOGTYPE
};
LogType currentLogType = WIFI_LOGTYPE;

LogFrame logBuffer[MAX_LOG_FRAMES];

bool sendingLog = false;
bool logging = false;
uint32_t logIndex = 0;


void startLogging() {
    //Serial.println("Логгирование начато");

    logIndex = 0;
    logging = true;
}

void stopLogging() {
    //Serial.println("Логгирование завершено");

    logging = false;
}

void handleLogging() {
    if (log_state_channel == 1) {
        if (!logging && logIndex < MAX_LOG_FRAMES) startLogging();

    } else {
        if (logging) stopLogging();
    }

    logData();

    if (log_send_channel == 1) {
        if (!logging && !sendingLog) sendLog();
        
    } else {
        sendingLog = false;
    }
}

void logData() {
    if (!logging) return;

    if (logIndex >= MAX_LOG_FRAMES) {
        stopLogging();
        return;
    }

    LogFrame &f = logBuffer[logIndex++];

    f.t = micros();

    f.gx = gyroRad.x * 100;
    f.gy = gyroRad.y * 100;
    f.gz = gyroRad.z * 100;

    f.ax = acc.x * 100;
    f.ay = acc.y * 100;
    f.az = acc.z * 100;

    f.roll  = rotationRad.x * 100;
    f.pitch = rotationRad.y * 100;
    f.yaw   = rotationRad.z * 100;
}

void sendLog() {
    // Serial.println("sendLog");
    sendingLog = true;

    if (currentLogType == WIFI_LOGTYPE) {
        sendLogWiFi();
    }
}
