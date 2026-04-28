#include "SensorEnemigo.h"

SensorEnemigo::SensorEnemigo(int pin) : pin(pin) {}

bool SensorEnemigo::detectar() {
    // Filtro rapido por mayoria para reducir lecturas espurias.
    uint8_t activaciones = 0;
    activaciones += (digitalRead(pin) == HIGH) ? 1 : 0;
    activaciones += (digitalRead(pin) == HIGH) ? 1 : 0;
    activaciones += (digitalRead(pin) == HIGH) ? 1 : 0;
    return activaciones >= 2;
}
