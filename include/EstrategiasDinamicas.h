#ifndef ESTRATEGIAS_DINAMICAS_H
#define ESTRATEGIAS_DINAMICAS_H

#include <Arduino.h>

class EstrategiasDinamicas {
public:
    enum TipoRival { AGRESIVO, DEFENSIVO, EVASIVO, DESCONOCIDO };
    EstrategiasDinamicas();
    void actualizar(bool ataqueRecibido, bool enemigoEscapa, bool enemigoAtaca);
    TipoRival getTipoRival() const;
    void aplicarEstrategia();
private:
    TipoRival tipoRival;
    int contadorAtaques;
    int contadorEscapes;
};

#endif // ESTRATEGIAS_DINAMICAS_H
