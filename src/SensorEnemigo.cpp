#include "SensorEnemigo.h"
#include "FiltroDeKalma.h"

SensorEnemigo::SensorEnemigo(int pin) : pin(pin) {}

bool SensorEnemigo::detectar() {
    // Leer el valor del sensor (asumiendo HIGH=1, LOW=0)
    float lectura = (digitalRead(pin) == HIGH) ? 1.0f : 0.0f;
    // Aplicar el filtro de Kalman para suavizar la señal
    float suavizado = filtroEnemigo.actualizar(lectura);
    // Umbral ajustable para decidir si hay detección de enemigo
    return suavizado > 0.5f;
}
