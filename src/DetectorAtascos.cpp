
#include "DetectorAtascos.h"
#include "Motor.h"
#include "Definiciones.h"

extern Motores motores;

DetectorAtascos::DetectorAtascos() : ciclosSinCambio(0), ultimoMovimiento(true) {}

void DetectorAtascos::actualizar(bool movimiento, bool enemigoDetectado, bool bordeDetectado) {
    if (!movimiento && !enemigoDetectado && !bordeDetectado) {
        ciclosSinCambio++;
    } else {
        ciclosSinCambio = 0;
    }
    ultimoMovimiento = movimiento;
}

bool DetectorAtascos::estaAtascado() const {
    return ciclosSinCambio > 8;
}

void DetectorAtascos::resolverAtasco() {
    motores.retroceder(Velocidad_normal);
    delay(200);
    motores.derecha(Velocidad_normal);
    delay(150);
    motores.detener();
}
