#include "EstrategiasDinamicas.h"
#include "Motor.h"
#include "Definiciones.h"

extern Motores motores;

EstrategiasDinamicas::EstrategiasDinamicas() : tipoRival(DESCONOCIDO), contadorAtaques(0), contadorEscapes(0) {}

void EstrategiasDinamicas::actualizar(bool ataqueRecibido, bool enemigoEscapa, bool enemigoAtaca) {
    if (ataqueRecibido) contadorAtaques++;
    if (enemigoEscapa) contadorEscapes++;
    if (contadorAtaques > 3) tipoRival = AGRESIVO;
    else if (contadorEscapes > 3) tipoRival = EVASIVO;
    else if (contadorAtaques == 0 && contadorEscapes == 0) tipoRival = DEFENSIVO;
    else tipoRival = DESCONOCIDO;
}

EstrategiasDinamicas::TipoRival EstrategiasDinamicas::getTipoRival() const {
    return tipoRival;
}

void EstrategiasDinamicas::aplicarEstrategia() {
    switch (tipoRival) {
        case AGRESIVO:
            motores.retroceder(Velocidad_normal);
            delay(80);
            motores.curvaIzquierda(Velocidad_normal);
            delay(60);
            motores.detener();
            break;
        case EVASIVO:
            motores.adelante(Velocidad_maxima);
            delay(100);
            motores.detener();
            break;
        case DEFENSIVO:
            motores.derecha(Velocidad_normal);
            delay(80);
            motores.detener();
            break;
        default:
            // Estrategia por defecto
            motores.adelante(Velocidad_normal);
            delay(60);
            motores.detener();
            break;
    }
}
