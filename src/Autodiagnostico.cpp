#include "Autodiagnostico.h"
#include "SensorPiso.h"
#include "SensorEnemigo.h"
#include "Motor.h"

Autodiagnostico::Autodiagnostico() : falloSensor(false), falloMotor(false) {}

void Autodiagnostico::checarSensores() {
    // Ejemplo: si algún sensor siempre da el mismo valor, marcar fallo
    // Aquí deberías leer los sensores reales
    // falloSensor = ...
}

void Autodiagnostico::checarMotores() {
    // Ejemplo: si el motor no responde, marcar fallo
    // falloMotor = ...
}

bool Autodiagnostico::hayFallo() const {
    return falloSensor || falloMotor;
}
