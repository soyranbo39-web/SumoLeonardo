#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>
#include "Definiciones.h"

class Motor {
    int pinA1, pinA2, pinPWM;
public:
    Motor(int a1, int a2, int pwm);
    void avanzar(int velocidad);
    void retroceder(int velocidad);
    void detener();
};

class Motores {
    Motor motorIzq, motorDer;
public:
    Motores();
    void adelante(int velocidad);
    void retroceder(int velocidad);
    void detener();
    void derecha(int velocidad);
    void izquierda(int velocidad);
    void curvaDerecha(int velocidad);
    void curvaIzquierda(int velocidad);
    Motor& getIzquierdo();
    Motor& getDerecho();
};

#endif
