#include <Arduino.h>
#include "rc.h"

#include <HardwareSerial.h>
#include "driver/uart.h"
#include "motors.h"
#include "utils.h"

HardwareSerial SBUS(2);  // UART2

uint8_t sbusFrame[25];
uint8_t sbusIndex = 0;

uint16_t sbusChannels[16];
uint32_t lastSbusTime = 0;

bool sbusReady;
bool sbusFailsafe;  // Когда радиосигнал с пульта потерян
bool sbusLostFrame;

bool radioLost;

// Каналы
float throttle_channel = MIN_MOTOR;  // Газ
float yaw_channel = 0;  // Рыскание в градусах
float roll_channel = 0;  // Крен в градусах
float pitch_channel = 0;  // Тангаж в градусах

bool motor_switch_channel = 0;  // 0 - Моторы выключены. 1 - Моторы включены.


void beginSBUS() {
  SBUS.begin(
    100000,            // скорость SBUS
    SERIAL_8E2,         // формат
    23,                 // RX (SBUS)
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

  sbusLostFrame = sbusFrame[23] & (1 << 2);
  sbusFailsafe  = sbusFrame[23] & (1 << 3);

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

    // пример: печать первых n каналов
    /*for (int i = 0; i < 16; i++) {
      Serial.print(i);
      Serial.print(":");
      Serial.print(sbusChannels[i]);
      Serial.print(" ");
    }
    Serial.println();*/
/*
      Каналы:
      0 - Газ (больше число - вверх ручка)
      1 - Крен (больше число - вправо ручка)
      2 - Тангаж (больше число - вверх ручка)
      3 - Рыскание (больше число - вправо ручка)
      4 - 
    */

    //  Нормализация: float norm = (channel - 992.0f) / 820.0f;  // -1 … +1

    throttle_channel = mapf((float) sbusChannels[0], 172.0, 1811.0, 0.0, 1.0);
    throttle_channel = constrain(throttle_channel, 0, 1);

    roll_channel = ((float) sbusChannels[1] - 992.0f) / 820.0f * 90.0f;
    pitch_channel = ((float) sbusChannels[2] - 992.0f) / 820.0f * -90.0f;
    yaw_channel = ((float) sbusChannels[3] - 992.0f) / 820.0f * 90.0f;
    
    // float pid_channel = sbusChannels[6];
    // float k = fmap(sbusChannels[7], 172, 1811, 0.0, 5.0);

    // if (pid_channel == 1811) {
    //   pidRoll.Kp = k;
    //   pidPitch.Kp = k;
    // } else if (pid_channel == 992) {
    //   pidRoll.Ki = k;
    //   pidPitch.Ki = k;

    // } else if (pid_channel == 172) {
    //   pidRoll.Kd = k;
    //   pidPitch.Kd = k;
    // }
    

    // Serial.print(pidRoll.Kp);
    // Serial.print(", ");
    // Serial.print(pidRoll.Ki);
    // Serial.print(", ");
    // Serial.print(pidRoll.Kd);
    // Serial.println();

    motor_switch_channel = sbusChannels[4] == 172 ? 0 : 1;

    // Serial.print(throttle_channel);
    // Serial.print(", ");
    // Serial.print(roll_channel);
    // Serial.print(", ");
    // Serial.print(pitch_channel);
    // Serial.print(", ");
    // Serial.print(motor_switch_channel);
    // Serial.println();

    // delay(200);
  }
}

