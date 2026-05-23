#include "Telemetria.h"
#include <Arduino.h>

void Telemetria::logEstadoSensores(bool pisoIzq, bool pisoDer, bool enemigoFrontal, bool enemigoIzq, bool enemigoDer, bool enemigoLatIzq, bool enemigoLatDer) {
    Serial.print("PisoIzq:"); Serial.print(pisoIzq);
    Serial.print(" PisoDer:"); Serial.print(pisoDer);
    Serial.print(" EFrontal:"); Serial.print(enemigoFrontal);
    Serial.print(" EIzq:"); Serial.print(enemigoIzq);
    Serial.print(" EDer:"); Serial.print(enemigoDer);
    Serial.print(" ELatIzq:"); Serial.print(enemigoLatIzq);
    Serial.print(" ELatDer:"); Serial.println(enemigoLatDer);
}

void Telemetria::logAccion(const char* accion) {
    Serial.print("Accion: ");
    Serial.println(accion);
}
