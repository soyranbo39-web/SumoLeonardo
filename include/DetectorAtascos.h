#ifndef DETECTOR_ATASCOS_H
#define DETECTOR_ATASCOS_H

#include <Arduino.h>

class DetectorAtascos {
public:
    DetectorAtascos();
    void actualizar(bool movimiento, bool enemigoDetectado, bool bordeDetectado);
    bool estaAtascado() const;
    void resolverAtasco();
private:
    int ciclosSinCambio;
    bool ultimoMovimiento;
};

#endif // DETECTOR_ATASCOS_H
