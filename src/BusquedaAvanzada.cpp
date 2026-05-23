
#include "BusquedaAvanzada.h"
#include "Motor.h"
#include "Definiciones.h"

extern Motores motores;

BusquedaAvanzada::BusquedaAvanzada() : patronActual(GIRO), ciclosSinEnemigo(0) {}

void BusquedaAvanzada::ejecutar() {
    switch (patronActual) {
        case GIRO:
            motores.derecha(Velocidad_normal);
            delay(80);
            motores.detener();
            break;
        case ZIGZAG:
            motores.curvaIzquierda(Velocidad_normal);
            delay(60);
            motores.curvaDerecha(Velocidad_normal);
            delay(60);
            motores.detener();
            break;
        case ESPIRAL:
            motores.adelante(Velocidad_normal);
            delay(100 + ciclosSinEnemigo * 10);
            motores.derecha(Velocidad_normal);
            delay(40);
            motores.detener();
            break;
    }
    ciclosSinEnemigo++;
    if (ciclosSinEnemigo > 10) cambiarPatron();
}

void BusquedaAvanzada::cambiarPatron() {
    patronActual = static_cast<Patron>((patronActual + 1) % 3);
    ciclosSinEnemigo = 0;
}

BusquedaAvanzada::Patron BusquedaAvanzada::getPatronActual() const {
    return patronActual;
}
