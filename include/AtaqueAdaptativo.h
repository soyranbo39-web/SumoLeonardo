#ifndef ATAQUE_ADAPTATIVO_H
#define ATAQUE_ADAPTATIVO_H

#include <Arduino.h>

class AtaqueAdaptativo {
public:
    AtaqueAdaptativo();
    void atacar(bool frontal, bool izq, bool der, bool latIzq, bool latDer);
};

#endif // ATAQUE_ADAPTATIVO_H
