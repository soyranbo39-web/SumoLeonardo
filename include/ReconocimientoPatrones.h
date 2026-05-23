#ifndef RECONOCIMIENTO_PATRONES_H
#define RECONOCIMIENTO_PATRONES_H

#include <Arduino.h>

class ReconocimientoPatrones {
public:
    ReconocimientoPatrones();
    void registrarMovimientoEnemigo(int direccion); // 0=izq, 1=frente, 2=der
    int predecirSiguienteMovimiento();
private:
    int historial[10];
    int indice;
};

#endif // RECONOCIMIENTO_PATRONES_H
