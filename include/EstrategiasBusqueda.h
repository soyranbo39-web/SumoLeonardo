// estrategias para la búsqueda de oponentes en el sumo robot
#ifndef ESTRATEGIAS_BUSQUEDA_H
#define ESTRATEGIAS_BUSQUEDA_H

#include <Arduino.h>
#include "Definiciones.h"

class EstrategiasBusqueda {
public:
    void busquedaEnemigo();
    void busquedaPiso();
};

#endif // ESTRATEGIAS_BUSQUEDA_H
#include "Definiciones.h"
