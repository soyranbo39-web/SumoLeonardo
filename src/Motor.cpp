#include "Motor.h"
#include "Definiciones.h"
#include "SensorPiso.h"

extern SensorPiso sensorPisoIzq;
extern SensorPiso sensorPisoDer;

// indicador de movimientos
namespace {
void setLedsMovimiento(bool txOn, bool rxOn) {
#if defined(TXLED0) && defined(TXLED1) && defined(RXLED0) && defined(RXLED1)
    if (txOn) {
        TXLED0;
    } else {
        TXLED1;
    }

    if (rxOn) {
        RXLED0;
    } else {
        RXLED1;
    }
#endif
}
} 

// movimientos basicos de un motor
Motor::Motor(int a1, int a2, int pwm) : pinA1(a1), pinA2(a2), pinPWM(pwm) {}

void Motor::avanzar(int velocidad) {
    digitalWrite(pinA1, HIGH);
    digitalWrite(pinA2, LOW);
    analogWrite(pinPWM, velocidad);
}

void Motor::retroceder(int velocidad) {
    digitalWrite(pinA1, LOW);
    digitalWrite(pinA2, HIGH);
    analogWrite(pinPWM, velocidad);
}

void Motor::detener() {
    digitalWrite(pinA1, LOW);
    digitalWrite(pinA2, LOW);
    analogWrite(pinPWM, 0);
}

// Implementación de Motores
Motores::Motores() :
    motorIzq(MA1A, MA2A, PWMA),
    motorDer(MA1B, MA2B, PWMB) {}

bool Motores::bordeBloqueaMovimiento() {
    if (!Validar_borde_en_movimientos) {
        return false;
    }

    if (sensorPisoIzq.detectar() || sensorPisoDer.detectar()) {
        detener();
        return true;
    }

    return false;
}

void Motores::adelante(int velocidad) {
    if (bordeBloqueaMovimiento()) {
        return;
    }
    motorIzq.avanzar(velocidad);
    motorDer.avanzar(velocidad);
    setLedsMovimiento(true, false);
}

void Motores::retroceder(int velocidad) {
    motorIzq.retroceder(velocidad);
    motorDer.retroceder(velocidad);
    setLedsMovimiento(false, true);
}

void Motores::detener() {
    motorIzq.detener();
    motorDer.detener();
    setLedsMovimiento(false, false);
}

void Motores::derecha(int velocidad) {
    if (bordeBloqueaMovimiento()) {
        return;
    }
    motorIzq.avanzar(velocidad);
    motorDer.retroceder(velocidad);
    setLedsMovimiento(true, true);
}

void Motores::izquierda(int velocidad) {
    if (bordeBloqueaMovimiento()) {
        return;
    }
    motorIzq.retroceder(velocidad);
    motorDer.avanzar(velocidad);
    setLedsMovimiento(false, true);
}

void Motores::curvaDerecha(int velocidad) {
    if (bordeBloqueaMovimiento()) {
        return;
    }
    motorIzq.avanzar(velocidad);
    motorDer.detener();
    setLedsMovimiento(true, false);
}

void Motores::curvaIzquierda(int velocidad) {
    if (bordeBloqueaMovimiento()) {
        return;
    }
    motorIzq.detener();
    motorDer.avanzar(velocidad);
    setLedsMovimiento(false, true);
}

Motor& Motores::getIzquierdo() {
    return motorIzq;
}

Motor& Motores::getDerecho() {
    return motorDer;
}
