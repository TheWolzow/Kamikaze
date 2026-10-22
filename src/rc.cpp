#include <Arduino.h>
#include "rc.h"

#include <HardwareSerial.h>
#include "driver/uart.h"
#include "control.h"
#include "motors.h"
#include "utils.h"

HardwareSerial SBUS(2);  // UART2

uint8_t sbusFrame[25];
uint8_t sbusIndex = 0;

uint16_t sbusChannels[16];
uint32_t lastSbusTime = 0;

const uint8_t RX_SBUS_PIN = 23;

static const uint8_t channelMin[10] = {
  192,
  179,
  185,
  172,
  172,
  172,
  172,
  172,
  172,
  172
};

static const uint16_t channelMax[10] = {
  1811,
  1811,
  1811,
  1811,
  1811,
  1811,
  1811,
  1811,
  1811,
  1811
};

bool sbusReady;
bool sbusFailsafe;  // Когда радиосигнал с пульта потерян
bool sbusLostFrame;

bool radioLost;

// Каналы
float throttle_channel = 0;  // Газ [0; 1]
float yaw_channel = 0;  // Рыскание в градусах
float roll_channel = 0;  // Крен в градусах
float pitch_channel = 0;  // Тангаж в градусах

bool motor_switch_channel = 0;  // 0 - Моторы выключены. 1 - Моторы включены.
bool log_state_channel = 0;  // 0 - Логгирование выключено. 1 - Логгирование включено.
bool log_send_channel = 0;  // 0 - Дампинг логов отключён. 1 - Дампинг логов включён.


void beginSBUS() {
  SBUS.begin(
    100000,            // скорость SBUS
    SERIAL_8E2,         // формат
    RX_SBUS_PIN,                 // RX (SBUS)
    -1                  // TX не нужен
  );

  uart_set_line_inverse(UART_NUM_2, UART_SIGNAL_RXD_INV);
}

void readRC() {
  // Данные с приёмника
  readSBUS();
  parseSBUS();

  radioLost = sbusFailsafe || (millis() - lastSbusTime > 100); 
}

void readSBUS() {
  static uint32_t lastByteTime = 0;

  while (SBUS.available()) {
    uint8_t b = SBUS.read();
    uint32_t now = micros();

    // если большой разрыв — новый кадр
    if (now - lastByteTime > 3000) {
      sbusIndex = 0;
      //Serial.println("Time resync");
    }
    lastByteTime = now;

    if (sbusIndex == 0) {
      if (b != 0x0F) continue;
    }

    sbusFrame[sbusIndex++] = b;

    if (sbusIndex == 25) {
      if (sbusFrame[24] == 0x00) {
        sbusReady = true;
        lastSbusTime = millis();
        //Serial.println("OK frame");
      } else {
        Serial.println("Bad frame");
      }

      sbusIndex = 0;
    }
  }
}

void decodeSBUS() {
  // Serial.println("Decoding SBUS...");

  sbusChannels[0]  = (sbusFrame[1]       | sbusFrame[2]  << 8) & 0x07FF;
  sbusChannels[1]  = (sbusFrame[2] >> 3  | sbusFrame[3]  << 5) & 0x07FF;
  sbusChannels[2]  = (sbusFrame[3] >> 6  | sbusFrame[4]  << 2 | sbusFrame[5]  << 10) & 0x07FF;
  sbusChannels[3]  = (sbusFrame[5] >> 1  | sbusFrame[6]  << 7) & 0x07FF;
  sbusChannels[4]  = (sbusFrame[6] >> 4  | sbusFrame[7]  << 4) & 0x07FF;
  sbusChannels[5]  = (sbusFrame[7] >> 7  | sbusFrame[8]  << 1 | sbusFrame[9]  << 9) & 0x07FF;
  sbusChannels[6]  = (sbusFrame[9] >> 2  | sbusFrame[10] << 6) & 0x07FF;
  sbusChannels[7]  = (sbusFrame[10] >> 5 | sbusFrame[11] << 3) & 0x07FF;
  sbusChannels[8]  = (sbusFrame[12]      | sbusFrame[13] << 8) & 0x07FF;
  sbusChannels[9]  = (sbusFrame[13] >> 3 | sbusFrame[14] << 5) & 0x07FF;
  sbusChannels[10] = (sbusFrame[14] >> 6 | sbusFrame[15] << 2 | sbusFrame[16] << 10) & 0x07FF;
  sbusChannels[11] = (sbusFrame[16] >> 1 | sbusFrame[17] << 7) & 0x07FF;
  sbusChannels[12] = (sbusFrame[17] >> 4 | sbusFrame[18] << 4) & 0x07FF;
  sbusChannels[13] = (sbusFrame[18] >> 7 | sbusFrame[19] << 1 | sbusFrame[20] << 9) & 0x07FF;
  sbusChannels[14] = (sbusFrame[20] >> 2 | sbusFrame[21] << 6) & 0x07FF;
  sbusChannels[15] = (sbusFrame[21] >> 5 | sbusFrame[22] << 3) & 0x07FF;

  sbusLostFrame = sbusFrame[23] & (1 << 2) != 0;
  sbusFailsafe  = sbusFrame[23] & (1 << 3) != 0;

  // Serial.print("Failsafe: ");
  // Serial.print(sbusFailsafe);
  // Serial.print(" LostFrame: ");
  // Serial.println(sbusLostFrame);
}

void parseSBUS() {
  if (sbusReady) {
    sbusReady = false;

    decodeSBUS();

    for (int i = 0; i < 16; i++) {
      if (sbusChannels[i] < 150 || sbusChannels[i] > 1900) {
        Serial.println("Invalid channel detected!");
        return; // мусорный кадр
      }
    }

    // Пример: печать первых n каналов
    // for (int i = 0; i < 16; i++) {
    //   Serial.print(i);
    //   Serial.print(":");
    //   Serial.print(sbusChannels[i]);
    //   Serial.print(" ");
    // }
    // Serial.println();
    /*
      Каналы:
      0 - Газ (больше число - вверх ручка)
      1 - Крен (больше число - вправо ручка)
      2 - Тангаж (больше число - вверх ручка)
      3 - Рыскание (больше число - вправо ручка)
      4 - Арминг
      5 - В минимальном положении стоп, в максимальном старт логгирования
      6 - Отправить логи по Wi-Fi
      7 - Крутилка левая
      8 - Крутилка правая
      9 - Режимы полёта: в самый низ - Acro, по середине - Angle.
    */

    throttle_channel = mapf(sbusChannels[0], channelMin[0], channelMax[0], 0.0, 1.0);
    throttle_channel = constrain(throttle_channel, 0, 1);

    roll_channel = mapf(sbusChannels[1], channelMin[1], channelMax[1], -90, 90);
    pitch_channel = mapf(sbusChannels[2], channelMin[2], channelMax[2], -90, 90);
    yaw_channel = mapf(sbusChannels[3], channelMin[3], channelMax[3], -90, 90);

    roll_channel = abs(roll_channel) < 0.45 ? 0 : roll_channel;
    pitch_channel = abs(pitch_channel) < 0.45 ? 0 : pitch_channel;
    yaw_channel = abs(yaw_channel) < 0.45 ? 0 : yaw_channel;

    motor_switch_channel = sbusChannels[4] > 1000;

    if (sbusChannels[9] == channelMin[9]) {
      prevFlightMode = flightMode;
      flightMode = ACRO;

    } else if (sbusChannels[9] < 1000) {
      
      flightMode = ANGLE;
    }

    if (sbusChannels[5] == channelMin[5]) {
      log_state_channel = 0;
    } else if (sbusChannels[5] == channelMax[5]) {
      log_state_channel = 1;
    }

    if (sbusChannels[6] == channelMin[6]) {
      log_send_channel = 0;
    } else if (sbusChannels[6] == channelMax[6]) {
      log_send_channel = 1;
    }

    // Serial.print(throttle_channel);
    // Serial.print(", ");
    // Serial.print(roll_channel);
    // Serial.print(", ");
    // Serial.print(pitch_channel);
    // Serial.println();
  }
}

