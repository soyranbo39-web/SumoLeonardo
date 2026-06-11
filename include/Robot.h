#ifndef ROBOT_H
#define ROBOT_H

#include <Arduino.h>
#include "Definiciones.h"
#include "Motor.h"
#include "SensorPiso.h"
#include "SensorEnemigo.h"

class Robot {
    Motores motores;
    SensorPiso sensorPisoIzq, sensorPisoDer;
    SensorEnemigo sensorFrontal, sensorFrontalIzq, sensorFrontalDer, sensorLateralIzq, sensorLateralDer;
    bool leerPiso(bool &pisoIzq, bool &pisoDer);
    bool esperarConPrioridadPiso(unsigned long duracionMs);
    bool enemigoVistoRapido();
    bool esperarConPrioridadPisoYEnemigo(unsigned long duracionMs);
    void retrocesoSeguro(unsigned long duracionMs);
    void giroEscapeSeguro(bool haciaDerecha, unsigned long duracionMs);
    void avanceEscapeSeguro(unsigned long duracionMs);
    bool reingresoCombateSeguro(bool ultimoGiroDerecha, unsigned long avanceMs, unsigned long correccionMs);
    void ejecutarBusquedaCompacta(bool haciaDerecha, unsigned long tiempoEnCiclo, unsigned long avanceMs);
    void rutinaInicialBordeMediaVuelta();
public:
    Robot();
    void setup();
    void detenerse();
    void ataqueEnemigo();
    void moverAdelante();
    void retroceder();
    void moverDerecha();
    void moverIzquierda();
    void sensoresPiso(bool pisoIzq, bool pisoDer);
    void sensoresFrontales(bool central, bool derecho, bool izquierdo);
    void sensoresLaterales(bool sensorIzquierdo, bool sensorDerecho);
    void loop();
};

#endif // ROBOT_H
