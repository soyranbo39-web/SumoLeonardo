#include "EnganoSumo.h"
#include "Motor.h"
#include "Definiciones.h"

extern Motores motores;

EnganoSumo::EnganoSumo() {}

void EnganoSumo::fintarRetroceso() {
    motores.retroceder(Velocidad_normal);
    delay(60);
    motores.adelante(Velocidad_maxima);
    delay(80);
    motores.detener();
}

void EnganoSumo::fintarGiro() {
    motores.derecha(Velocidad_normal);
    delay(50);
    motores.izquierda(Velocidad_maxima);
    delay(70);
    motores.detener();
}
