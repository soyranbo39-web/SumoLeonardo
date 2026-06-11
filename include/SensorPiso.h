#ifndef SENSORPISO_H
#define SENSORPISO_H

#include <Arduino.h>
#include "ISensor.h"
#include "Definiciones.h"
#include "FiltroDeKalma.h"

class SensorPiso : public ISensor {
    int pin;
    int umbral;
    bool bordeDetectado;
    FiltroKalma filtro;
    long sumaCalibracion;
    int muestrasCalibracion;
    bool calibrado;
    int lecturaCruda;
    float lecturaSuavizada;
public:
    SensorPiso(int pin, int umbral);
    bool detectar() override;
    int obtenerLecturaCruda() const;
    float obtenerLecturaSuavizada() const;
    int obtenerUmbral() const;
};

#endif // SENSORPISO_H
