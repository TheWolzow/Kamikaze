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
};

extern bool armed;
extern FlightMode flightMode;

void control();
void controlTorque();
void controlMixer();
void controlAcro();
void controlAngle();
void controlArming();
