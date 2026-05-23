#ifndef MEJORAS_ROBOT_H
#define MEJORAS_ROBOT_H

#include <Arduino.h>

// Estrategias de búsqueda avanzadas
class BusquedaAvanzada {
public:
    enum Patron { GIRO, ZIGZAG, ESPIRAL };
    BusquedaAvanzada();
    void ejecutar();
    void cambiarPatron();
    Patron getPatronActual() const;
private:
    Patron patronActual;
    int ciclosSinEnemigo;
};

// Detección y resolución de atascos
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

// Ataque adaptativo
class AtaqueAdaptativo {
public:
    AtaqueAdaptativo();
    void atacar(bool frontal, bool izq, bool der, bool latIzq, bool latDer);
};

// Telemetría
class Telemetria {
public:
    static void logEstadoSensores(bool pisoIzq, bool pisoDer, bool enemigoFrontal, bool enemigoIzq, bool enemigoDer, bool enemigoLatIzq, bool enemigoLatDer);
    static void logAccion(const char* accion);
};

#endif // MEJORAS_ROBOT_H
