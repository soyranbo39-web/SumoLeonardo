#include "Robot.h"

namespace {
const bool TELEMETRIA_ACTIVA = false;
}


Robot::Robot() :
    motores(),
    sensorPisoIzq(SENSOR_DE_PISO_IZQUIERDO, BLANCO),
    sensorPisoDer(SENSOR_DE_PISO_DERECHO, BLANCO),
    sensorFrontal(SENSOR_FRONTAL_CENTRA),
    sensorFrontalIzq(SENSOR_FRONTAL_IZQUIERDO),
    sensorFrontalDer(SENSOR_FRONTAL_DERECHO),
    sensorLateralIzq(SENSOR_LATERAL_IZQUIERDO),
    sensorLateralDer(SENSOR_LATERAL_DERECHO)
{}

bool Robot::leerPiso(bool &pisoIzq, bool &pisoDer) {
    pisoIzq = sensorPisoIzq.detectar();
    pisoDer = sensorPisoDer.detectar();
    return pisoIzq || pisoDer;
}

bool Robot::esperarConPrioridadPiso(unsigned long duracionMs) {
    unsigned long inicio = millis();
    while (millis() - inicio < duracionMs) {
        bool pisoIzq = false;
        bool pisoDer = false;
        if (leerPiso(pisoIzq, pisoDer)) {
            
            return true;
        }
        delay(5);
    }
    return false;
}

bool Robot::enemigoVistoRapido() {
    return sensorFrontalDer.detectar() ||
           sensorFrontalIzq.detectar() ||
           sensorFrontal.detectar() ||
           sensorLateralDer.detectar() ||
           sensorLateralIzq.detectar();
}

bool Robot::esperarConPrioridadPisoYEnemigo(unsigned long duracionMs) {
    unsigned long inicio = millis();
    while (millis() - inicio < duracionMs) {
        bool pisoIzq = false;
        bool pisoDer = false;
        if (leerPiso(pisoIzq, pisoDer)) {
            return true;
        }

        // Si vuelve a ver enemigo, corta la espera para reevaluar de inmediato.
        if (enemigoVistoRapido()) {
            return false;
        }
        delay(3);
    }
    return false;
}

void Robot::retrocesoSeguro(unsigned long duracionMs) {
    unsigned long inicio = millis();
    while (millis() - inicio < duracionMs) {
        retroceder();
        delay(5);
    }
}

void Robot::giroEscapeSeguro(bool haciaDerecha, unsigned long duracionMs) {
    unsigned long inicio = millis();
    while (millis() - inicio < duracionMs) {
        bool pisoIzq = false;
        bool pisoDer = false;
        bool enBorde = leerPiso(pisoIzq, pisoDer);

        if (haciaDerecha) {
            moverDerecha();
        } else {
            moverIzquierda();
        }

        // Mantiene al menos un giro corto y sale cuando ya no detecta borde.
        if (!enBorde && (millis() - inicio) > 60) {
            return;
        }

        delay(5);
    }
}

void Robot::setup() {
    Serial.begin(9600);
    pinMode(SENSOR_DE_PISO_IZQUIERDO, INPUT);
    pinMode(SENSOR_DE_PISO_DERECHO, INPUT);
    pinMode(SENSOR_FRONTAL_DERECHO, INPUT);
    pinMode(SENSOR_FRONTAL_CENTRA, INPUT);
    pinMode(SENSOR_FRONTAL_IZQUIERDO, INPUT);
    pinMode(SENSOR_LATERAL_IZQUIERDO, INPUT);
    pinMode(SENSOR_LATERAL_DERECHO, INPUT);
    pinMode(MA2A, OUTPUT);
    pinMode(MA1A, OUTPUT);
    pinMode(PWMA, OUTPUT);
    pinMode(MA2B, OUTPUT);
    pinMode(MA1B, OUTPUT);
    pinMode(PWMB, OUTPUT);
}

void Robot::detenerse() {
    motores.detener();
}

void Robot::ataqueEnemigo() {
    motores.adelante(Velocidad_maxima);
}

void Robot::moverAdelante() {
    motores.adelante(Velocidad_movimiento_seguir);
}

void Robot::retroceder() {
    motores.retroceder(Velocidad_estandar);
}

void Robot::moverDerecha() {
    motores.derecha(Velocidad_maxima);
}

void Robot::moverIzquierda() {
    motores.izquierda(Velocidad_maxima);
}

void Robot::sensoresPiso(bool pisoIzq, bool pisoDer) {
    static bool giroAlternadoDerecha = true;

    if (pisoIzq && pisoDer) {
        retrocesoSeguro(180); // 120 el tiempo anterior 
        giroEscapeSeguro(giroAlternadoDerecha, 280);// 220 el tiempo anterior
        giroAlternadoDerecha = !giroAlternadoDerecha;
    } else if (pisoDer) {
        retrocesoSeguro(180);
        giroEscapeSeguro(false, 280);
    } else if (pisoIzq) {
        retrocesoSeguro(180);
        giroEscapeSeguro(true, 280);
    }
}

void Robot::sensoresFrontales(bool central, bool derecho, bool izquierdo) {
    if (central || (derecho && izquierdo)) {
        ataqueEnemigo();
    } else if (derecho) {
        moverDerecha();
        if (esperarConPrioridadPisoYEnemigo(55)) {
            return;
        }
        motores.adelante(Velocidad_maxima);
        esperarConPrioridadPisoYEnemigo(55);
    } else if (izquierdo) {
        moverIzquierda();
        if (esperarConPrioridadPisoYEnemigo(55)) {
            return;
        }
        motores.adelante(Velocidad_maxima);
        esperarConPrioridadPisoYEnemigo(55);
    }
}

void Robot::sensoresLaterales(bool sensorIzquierdo, bool sensorDerecho) {
    if (sensorIzquierdo && sensorDerecho) return;

    bool pisoIzq = false, pisoDer = false;
    leerPiso(pisoIzq, pisoDer);
    if (pisoIzq || pisoDer) return;

    // Gira comprometido 120ms hacia el lado del enemigo.
    // Verifica piso cada 5ms para abortar si hay borde.
    if (sensorIzquierdo) {
        motores.curvaIzquierda(Velocidad_maxima);  // Solo rueda derecha gira
        esperarConPrioridadPisoYEnemigo(110);
    } else if (sensorDerecho) {
        motores.curvaDerecha(Velocidad_maxima);    // Solo rueda izquierda gira
        esperarConPrioridadPisoYEnemigo(110);
    }
}

void Robot::loop() {
    static unsigned long ultimoContacto = 0;
    static bool busquedaDerecha = true;
    static unsigned long inicioPulsoBusqueda = 0;
    static unsigned long ultimoFrontalDerMs = 0;
    static unsigned long ultimoFrontalIzqMs = 0;
    static unsigned long ultimoFrontalCentralMs = 0;
    static unsigned long ultimoLateralDerMs = 0;
    static unsigned long ultimoLateralIzqMs = 0;

    const unsigned long ahora = millis();

    bool PisoIzq = false;
    bool PisoDer = false;
    int valorPisoIzq = analogRead(SENSOR_DE_PISO_IZQUIERDO);
    int valorPisoDer = analogRead(SENSOR_DE_PISO_DERECHO);
    PisoIzq = (valorPisoIzq <= BLANCO);
    PisoDer = (valorPisoDer <= BLANCO);
    bool FrontalDerRaw = sensorFrontalDer.detectar();
    bool FrontalIzqRaw = sensorFrontalIzq.detectar();
    bool FrontalCentralRaw = sensorFrontal.detectar();
    bool LateralDerRaw = sensorLateralDer.detectar();
    bool LateralIzqRaw = sensorLateralIzq.detectar();

    if (FrontalDerRaw) ultimoFrontalDerMs = ahora;
    if (FrontalIzqRaw) ultimoFrontalIzqMs = ahora;
    if (FrontalCentralRaw) ultimoFrontalCentralMs = ahora;
    if (LateralDerRaw) ultimoLateralDerMs = ahora;
    if (LateralIzqRaw) ultimoLateralIzqMs = ahora;

    // Persistencia corta para no perder al enemigo por parpadeo de lectura.
    const unsigned long persistenciaFrontalMs = 70;
    const unsigned long persistenciaLateralMs = 90;
    bool FrontalDer = FrontalDerRaw || ((ahora - ultimoFrontalDerMs) <= persistenciaFrontalMs);
    bool FrontalIzq = FrontalIzqRaw || ((ahora - ultimoFrontalIzqMs) <= persistenciaFrontalMs);
    bool FrontalCentral = FrontalCentralRaw || ((ahora - ultimoFrontalCentralMs) <= persistenciaFrontalMs);
    bool LateralDer = LateralDerRaw || ((ahora - ultimoLateralDerMs) <= persistenciaLateralMs);
    bool LateralIzq = LateralIzqRaw || ((ahora - ultimoLateralIzqMs) <= persistenciaLateralMs);
    
    // Debug: Mostrar estado de sensores laterales
    static unsigned long ultimoReporteLateral = 0;
    if (TELEMETRIA_ACTIVA && (ahora - ultimoReporteLateral >= 300)) {
        ultimoReporteLateral = ahora;
        if (LateralIzq || LateralDer) {
            Serial.print("LATERAL DETECTADO - Izq: ");
            Serial.print(LateralIzq ? "SI" : "NO");
            Serial.print(" | Der: ");
            Serial.println(LateralDer ? "SI" : "NO");
        }
    }

    const bool enemigoDetectado = FrontalDer || FrontalIzq || FrontalCentral || LateralDer || LateralIzq;
    if (enemigoDetectado) {
        ultimoContacto = ahora;
    }

    static unsigned long ultimoReporte = 0;
    const unsigned long intervaloReporteMs = 120;
    if (TELEMETRIA_ACTIVA && (ahora - ultimoReporte >= intervaloReporteMs)) {
        ultimoReporte = ahora;

        Serial.print("PISO IZQ: ");
        Serial.print(valorPisoIzq);
        Serial.print(" -> ");
        Serial.print(PisoIzq ? "BLANCO" : "PISTA");
        Serial.print(" | PISO DER: ");
        Serial.print(valorPisoDer);
        Serial.print(" -> ");
        Serial.println(PisoDer ? "BLANCO" : "PISTA");
    }

    if (PisoIzq || PisoDer) {
        inicioPulsoBusqueda = 0;
        sensoresPiso(PisoIzq, PisoDer);
    } else if (LateralDer || LateralIzq) {
        inicioPulsoBusqueda = 0;
        sensoresLaterales(LateralIzq, LateralDer);
    } else if (FrontalDer || FrontalIzq || FrontalCentral) {
        inicioPulsoBusqueda = 0;
        sensoresFrontales(FrontalCentral, FrontalDer, FrontalIzq);
    } else {
        unsigned long tiempoSinContacto = ahora - ultimoContacto;

        // Si acaba de perder al oponente, empuja hacia adelante para intentar reconexion rapida.
        if (tiempoSinContacto < 250) {
            inicioPulsoBusqueda = 0;
                motores.adelante(Velocidad_maxima);
        } else {
            // Busqueda por pulsos: avance corto seguido de un giro corto.
            const unsigned long duracionPulsoAvance = 170;
            const unsigned long duracionPulsoGiro = 110;
            const unsigned long duracionCicloBusqueda = duracionPulsoAvance + duracionPulsoGiro;

            if (inicioPulsoBusqueda == 0) {
                inicioPulsoBusqueda = ahora;
            }

            unsigned long tiempoEnBusqueda = ahora - inicioPulsoBusqueda;
            unsigned long tiempoEnCiclo = tiempoEnBusqueda % duracionCicloBusqueda;

            if (tiempoEnBusqueda >= (duracionCicloBusqueda * 2UL)) {
                busquedaDerecha = !busquedaDerecha;
                inicioPulsoBusqueda = ahora;
                tiempoEnCiclo = 0;
            }

            if (tiempoEnCiclo < duracionPulsoAvance) {
                moverAdelante();
            } else if (busquedaDerecha) {
                moverDerecha();
            } else {
                moverIzquierda();
            }
        }
    }
}
