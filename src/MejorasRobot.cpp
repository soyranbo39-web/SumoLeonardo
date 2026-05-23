#include "MejorasRobot.h"
#include "Motor.h"
#include "Definiciones.h"

BusquedaAvanzada::BusquedaAvanzada() : patronActual(GIRO), ciclosSinEnemigo(0) {}

void BusquedaAvanzada::ejecutar() {
    // Ejemplo de patrones
    switch (patronActual) {
        case GIRO:
            motores.derecha(Velocidad_normal);
            delay(80);
            motores.detener();
            break;
        case ZIGZAG:
            motores.curvaIzquierda(Velocidad_normal);
            delay(60);
            motores.curvaDerecha(Velocidad_normal);
            delay(60);
            motores.detener();
            break;
        case ESPIRAL:
            motores.adelante(Velocidad_normal);
            delay(100 + ciclosSinEnemigo * 10); // Espiral creciente
            motores.derecha(Velocidad_normal);
            delay(40);
            motores.detener();
            break;
    }
    ciclosSinEnemigo++;
    if (ciclosSinEnemigo > 10) cambiarPatron();
}

void BusquedaAvanzada::cambiarPatron() {
    patronActual = static_cast<Patron>((patronActual + 1) % 3);
    ciclosSinEnemigo = 0;
}

BusquedaAvanzada::Patron BusquedaAvanzada::getPatronActual() const {
    return patronActual;
}

DetectorAtascos::DetectorAtascos() : ciclosSinCambio(0), ultimoMovimiento(true) {}

void DetectorAtascos::actualizar(bool movimiento, bool enemigoDetectado, bool bordeDetectado) {
    if (!movimiento && !enemigoDetectado && !bordeDetectado) {
        ciclosSinCambio++;
    } else {
        ciclosSinCambio = 0;
    }
    ultimoMovimiento = movimiento;
}

bool DetectorAtascos::estaAtascado() const {
    return ciclosSinCambio > 8;
}

void DetectorAtascos::resolverAtasco() {
    motores.retroceder(Velocidad_normal);
    delay(200);
    motores.derecha(Velocidad_normal);
    delay(150);
    motores.detener();
}

AtaqueAdaptativo::AtaqueAdaptativo() {}

void AtaqueAdaptativo::atacar(bool frontal, bool izq, bool der, bool latIzq, bool latDer) {
    if (frontal) {
        motores.adelante(Velocidad_maxima);
        Telemetria::logAccion("Ataque frontal");
    } else if (izq) {
        motores.curvaIzquierda(Velocidad_estandar);
        Telemetria::logAccion("Ataque curva izquierda");
    } else if (der) {
        motores.curvaDerecha(Velocidad_estandar);
        Telemetria::logAccion("Ataque curva derecha");
    } else if (latIzq) {
        motores.izquierda(Velocidad_normal);
        Telemetria::logAccion("Ataque lateral izquierdo");
    } else if (latDer) {
        motores.derecha(Velocidad_normal);
        Telemetria::logAccion("Ataque lateral derecho");
    } else {
        motores.detener();
        Telemetria::logAccion("Sin enemigo");
    }
    delay(120);
    motores.detener();
}

void Telemetria::logEstadoSensores(bool pisoIzq, bool pisoDer, bool enemigoFrontal, bool enemigoIzq, bool enemigoDer, bool enemigoLatIzq, bool enemigoLatDer) {
    Serial.print("PisoIzq:"); Serial.print(pisoIzq);
    Serial.print(" PisoDer:"); Serial.print(pisoDer);
    Serial.print(" EFrontal:"); Serial.print(enemigoFrontal);
    Serial.print(" EIzq:"); Serial.print(enemigoIzq);
    Serial.print(" EDer:"); Serial.print(enemigoDer);
    Serial.print(" ELatIzq:"); Serial.print(enemigoLatIzq);
    Serial.print(" ELatDer:"); Serial.println(enemigoLatDer);
}

void Telemetria::logAccion(const char* accion) {
    Serial.print("Accion: ");
    Serial.println(accion);
}
