#include "ReconocimientoPatrones.h"

ReconocimientoPatrones::ReconocimientoPatrones() : indice(0) {
    for (int i = 0; i < 10; ++i) historial[i] = -1;
}

void ReconocimientoPatrones::registrarMovimientoEnemigo(int direccion) {
    historial[indice] = direccion;
    indice = (indice + 1) % 10;
}

int ReconocimientoPatrones::predecirSiguienteMovimiento() {
    // Simple: retorna el movimiento más frecuente
    int conteo[3] = {0,0,0};
    for (int i = 0; i < 10; ++i) {
        if (historial[i] >= 0 && historial[i] < 3) conteo[historial[i]]++;
    }
    int maxDir = 0;
    for (int i = 1; i < 3; ++i) {
        if (conteo[i] > conteo[maxDir]) maxDir = i;
    }
    return maxDir;
}
