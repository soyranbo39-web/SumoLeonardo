// filtro de kalma para suavizar las lecturas de los sensores y evitar falsas detecciones
#ifndef FILTRO_DE_KALMA_H
#define FILTRO_DE_KALMA_H

#include <Arduino.h>


class FiltroKalma {
    float valorEstimado;
    float errorEstimado;
    float errorMedicion;
    float errorProceso;
    float gananciaKalman;
public:
    FiltroKalma(float errorMedicionInicial, float errorProcesoInicial = 0.01);
    float actualizar(float medicion);
    void setErrorMedicion(float nuevoErrorMedicion);
    void setErrorProceso(float nuevoErrorProceso);
    float getValorEstimado() const;
};

#include "Definiciones.h"


// Permite acceso global a la instancia del filtro para sensores de piso
extern FiltroKalma filtroPiso;
// Permite acceso global a la instancia del filtro para sensores de enemigo
extern FiltroKalma filtroEnemigo;

#endif // FILTRO_DE_KALMA_H