#ifndef APRENDIZAJE_SIMPLE_H
#define APRENDIZAJE_SIMPLE_H

#include <Arduino.h>

class AprendizajeSimple {
public:
    AprendizajeSimple();
    void registrarResultado(bool victoria);
    void ajustarParametros();
    int getVictorias() const;
    int getDerrotas() const;
private:
    int victorias;
    int derrotas;
};

#endif // APRENDIZAJE_SIMPLE_H
