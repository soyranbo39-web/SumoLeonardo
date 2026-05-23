// filtro de kalman para sensores de piso y enemigo
#include "FiltroDeKalma.h"


// Implementación mejorada del filtro de Kalman
FiltroKalma::FiltroKalma(float errorMedicionInicial, float errorProcesoInicial) {
    valorEstimado = 0;
    errorEstimado = 1;
    errorMedicion = errorMedicionInicial;
    errorProceso = errorProcesoInicial;
    gananciaKalman = 0;
}

float FiltroKalma::actualizar(float medicion) {
    // Predicción: se asume que el valor estimado no cambia, pero el error sí aumenta por el proceso
    errorEstimado += errorProceso;
    // Actualización de la ganancia de Kalman
    gananciaKalman = errorEstimado / (errorEstimado + errorMedicion);
    // Actualización del valor estimado
    valorEstimado = valorEstimado + gananciaKalman * (medicion - valorEstimado);
    // Actualización del error estimado
    errorEstimado = (1 - gananciaKalman) * errorEstimado;
    return valorEstimado;
}

void FiltroKalma::setErrorMedicion(float nuevoErrorMedicion) {
    errorMedicion = nuevoErrorMedicion;
}

void FiltroKalma::setErrorProceso(float nuevoErrorProceso) {
    errorProceso = nuevoErrorProceso;
}

float FiltroKalma::getValorEstimado() const {
    return valorEstimado;
}


// Instancia para sensores de piso (ajusta los valores según el ruido de tus sensores)
FiltroKalma filtroPiso(0.2, 0.01);

// Instancia para sensores de enemigo
FiltroKalma filtroEnemigo(0.5, 0.01);

// Ejemplo de uso:
// float lecturaSuavizadaPiso = filtroPiso.actualizar(lecturaSensorPiso);
// float lecturaSuavizadaEnemigo = filtroEnemigo.actualizar(lecturaSensorEnemigo);