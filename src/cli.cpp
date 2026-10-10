#include <Arduino.h>

#include "cli.h"
#include "logger.h"


void handleCLI() {
    while (Serial.available()) {
        char input = Serial.read();
        
        switch (toupper(input)) {
        // Начать передачу логов по USB
        case 'D':
            sendLog();
            break;
        
        // Начать логгирование
        case 'L':
            startLogging();
            break;

        default:
            break;
        }
    }
}
