#include "AprendizajeSimple.h"

AprendizajeSimple::AprendizajeSimple() : victorias(0), derrotas(0) {}

void AprendizajeSimple::registrarResultado(bool victoria) {
    if (victoria) victorias++;
    else derrotas++;
}

void AprendizajeSimple::ajustarParametros() {
    // Aquí podrías ajustar velocidades, umbrales, etc. según el desempeño
    // Ejemplo: si muchas derrotas, bajar velocidad máxima
}

int AprendizajeSimple::getVictorias() const { return victorias; }
int AprendizajeSimple::getDerrotas() const { return derrotas; }
