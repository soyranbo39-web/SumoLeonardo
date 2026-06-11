#include "SensorPiso.h"

SensorPiso::SensorPiso(int pin, int umbral)
    : pin(pin),
      umbral(umbral),
      bordeDetectado(false),
      filtro(0.2f, 0.01f),
      sumaCalibracion(0),
      muestrasCalibracion(0),
      calibrado(!Auto_calibrar_piso),
      lecturaCruda(0),
      lecturaSuavizada(0.0f) {}

bool SensorPiso::detectar() {
    lecturaCruda = analogRead(pin);

    if (!calibrado) {
        sumaCalibracion += lecturaCruda;
        muestrasCalibracion++;
        if (muestrasCalibracion >= Muestras_calibracion_piso) {
            const int base = (int)(sumaCalibracion / muestrasCalibracion);
            umbral = Sensor_piso_activo_bajo ? (base - Margen_calibracion_piso) : (base + Margen_calibracion_piso);
            calibrado = true;
        }
    }

    lecturaSuavizada = filtro.actualizar((float)lecturaCruda);

    if (Sensor_piso_activo_bajo) {
        if (!bordeDetectado && lecturaSuavizada <= (umbral - Histeresis_piso)) {
            bordeDetectado = true;
        } else if (bordeDetectado && lecturaSuavizada >= (umbral + Histeresis_piso)) {
            bordeDetectado = false;
        }
    } else {
        if (!bordeDetectado && lecturaSuavizada >= (umbral + Histeresis_piso)) {
            bordeDetectado = true;
        } else if (bordeDetectado && lecturaSuavizada <= (umbral - Histeresis_piso)) {
            bordeDetectado = false;
        }
    }

    return calibrado && bordeDetectado;
}

int SensorPiso::obtenerLecturaCruda() const {
    return lecturaCruda;
}

float SensorPiso::obtenerLecturaSuavizada() const {
    return lecturaSuavizada;
}

int SensorPiso::obtenerUmbral() const {
    return umbral;
}
