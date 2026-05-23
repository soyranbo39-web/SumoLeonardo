#ifndef TELEMETRIA_H
#define TELEMETRIA_H

#include <Arduino.h>

class Telemetria {
public:
    static void logEstadoSensores(bool pisoIzq, bool pisoDer, bool enemigoFrontal, bool enemigoIzq, bool enemigoDer, bool enemigoLatIzq, bool enemigoLatDer);
    static void logAccion(const char* accion);
};

#endif // TELEMETRIA_H
