#pragma once

enum FlightMode {
    /*
    ACRO (АКРО режим)
        Стики на пульте радиоуправления изменяют угловую скорость дрона.
    */
    ACRO,

    /*
    ANGLE (СТАБ режим)
        Стики на пульте радиоуправления задают угол поворота дрона.
        Стабилизация в горизонт при отпущенных стиках. 
    */
    ANGLE,

    /*
    HORIZON (ГОРИЗОНТ режим)
        Работают как ANGLE в центре стиков, но отклоняя их от центра переходит
        в режим ACRO
    */
   HORIZON
};

extern bool armed;
extern FlightMode flightMode;
extern FlightMode prevFlightMode;

void control();
void controlModes();
void controlTorque();
void controlAttitude();
void controlRates();
void controlMixer();
void controlArming();

void desaturate();
void resetControlPIDs();
float calculateHorizonStrength(float stickNormalized, float angleDeg);
