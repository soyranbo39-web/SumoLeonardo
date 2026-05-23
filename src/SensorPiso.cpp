#include "SensorPiso.h"
#include "FiltroDeKalma.h"

SensorPiso::SensorPiso(int pin, int umbral) : pin(pin), umbral(umbral) {}

bool SensorPiso::detectar() {
    int lectura = analogRead(pin);
    // Aplicar el filtro de Kalman para suavizar la señal del sensor de piso
    float lecturaSuavizada = filtroPiso.actualizar((float)lectura);
    return lecturaSuavizada <= umbral;
}
