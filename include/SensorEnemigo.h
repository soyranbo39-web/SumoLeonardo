#ifndef SENSORENEMIGO_H
#define SENSORENEMIGO_H

#include <Arduino.h>
#include "ISensor.h"
#include "Definiciones.h"

class SensorEnemigo : public ISensor {
    int pin;
public:
    SensorEnemigo(int pin);
    bool detectar() override;
};

#endif // SENSORENEMIGO_H
