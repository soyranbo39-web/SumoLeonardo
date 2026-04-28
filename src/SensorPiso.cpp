#include "SensorPiso.h"

SensorPiso::SensorPiso(int pin, int umbral) : pin(pin), umbral(umbral) {}

bool SensorPiso::detectar() {
    int lectura = analogRead(pin);
    return lectura <= umbral;
}
