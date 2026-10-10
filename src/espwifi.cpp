#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>

#include "espwifi.h"
#include "logger.h"


const char* ssid = "DroneBlackbox";
const char* password = "12345678";

WiFiUDP udp;
IPAddress pcIP(192, 168, 4, 2); // ПК в сети ESP
const int pcPort = 5005;


void setupWiFi() {
    WiFi.mode(WIFI_AP);
    WiFi.softAP(ssid, password);

    udp.begin(pcPort);
}

static void sendHeaderUDP() {
    LogHeader header;

    memcpy(header.magic, "DRON", 4);
    header.version = 1;
    header.frameSize = sizeof(LogFrame);
    header.frameCount = logIndex;

    udp.beginPacket(pcIP, pcPort);
    udp.write((uint8_t*)&header, sizeof(header));
    udp.endPacket();
}

void sendLogWiFi() {
    sendHeaderUDP();

    const uint8_t* data = (uint8_t*) logBuffer;
    size_t size = logIndex * sizeof(LogFrame);

    size_t offset = 0;
    
    const size_t chunkSize = 512;

    Serial.printf("Sending %u bytes\n", size);

    while (offset < size) {
        size_t len = min(chunkSize, size - offset);

        while (true) {
            udp.beginPacket(pcIP, pcPort);
            udp.write(data + offset, len);

            if (udp.endPacket()) {
                delay(1);
                break;
            }

            delay(5);
        }

        offset += len;
    }

    Serial.printf("Finished, offset=%u\n", offset);
}