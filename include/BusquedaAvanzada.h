#ifndef BUSQUEDA_AVANZADA_H
#define BUSQUEDA_AVANZADA_H

#include <Arduino.h>

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

#endif // BUSQUEDA_AVANZADA_H
