#include "Robot.h"

namespace {
const bool TELEMETRIA_ACTIVA = false;
const unsigned long RETROCESO_BORDE_SIMPLE_MS = 260;
const unsigned long RETROCESO_BORDE_DOBLE_MS = 320;
const unsigned long GIRO_ESCAPE_BORDE_SIMPLE_MS = 220;
const unsigned long GIRO_ESCAPE_BORDE_DOBLE_MS = 250;
const unsigned long AVANCE_INTERIOR_SIMPLE_MS = 140;
const unsigned long AVANCE_INTERIOR_DOBLE_MS = 170;
}

// ------------------ Control remoto (nivel) ------------------
bool robot_encendido = false;
unsigned long ultimo_cambio_boton = 0;
const unsigned long debounce_delay = 300;

int estado_control_anterior = LOW;
// Si tu receptor es activo-alto (START=1, STOP=0), true.
// Si lo vieras invertido, pon false.
const bool REMOTE_ACTIVE_HIGH = true;


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
    const unsigned long giroMinimoMs = 90;
    const unsigned long giroCorreccionMs = 40;
    unsigned long inicio = millis();
    bool bordeLiberado = false;

    while (millis() - inicio < duracionMs) {
        bool pisoIzq = false;
        bool pisoDer = false;
        bool enBorde = leerPiso(pisoIzq, pisoDer);

        if (haciaDerecha) {
            moverDerecha();
        } else {
            moverIzquierda();
        }

        unsigned long tiempoGirando = millis() - inicio;

        // Obliga un giro corto para despegarse del borde, pero no deja que siga rotando de mas.
        if (tiempoGirando < giroMinimoMs) {
            delay(5);
            continue;
        }

        if (!enBorde) {
            if (bordeLiberado || tiempoGirando >= (giroMinimoMs + giroCorreccionMs)) {
                return;
            }
            bordeLiberado = true;
        } else {
            bordeLiberado = false;
        }

        if (tiempoGirando >= duracionMs) {
            return;
        }

        delay(5);
    }
}

void Robot::avanceEscapeSeguro(unsigned long duracionMs) {
    unsigned long inicio = millis();
    while (millis() - inicio < duracionMs) {
        bool pisoIzq = false;
        bool pisoDer = false;
        if (leerPiso(pisoIzq, pisoDer)) {
            return;
        }

        moverAdelante();
        delay(5);
    }
}

void Robot::ejecutarBusquedaCompacta(bool haciaDerecha, unsigned long tiempoEnCiclo, unsigned long avanceMs) {
    if (tiempoEnCiclo < avanceMs) {
        moverAdelante();
        return;
    }

    if (haciaDerecha) {
        motores.curvaDerecha(Velocidad_maxima);
    } else {
        motores.curvaIzquierda(Velocidad_maxima);
    }
}

void Robot::setup() {
    Serial.begin(9600);
    pinMode(Pin_Control_Remoto, INPUT);
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

    // Estado inicial segun nivel del control remoto.
    int lecturaInicial = digitalRead(Pin_Control_Remoto);
    estado_control_anterior = lecturaInicial;
    bool controlActivo = REMOTE_ACTIVE_HIGH ? (lecturaInicial == HIGH) : (lecturaInicial == LOW);
    robot_encendido = controlActivo;
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
        retrocesoSeguro(RETROCESO_BORDE_DOBLE_MS);
        giroEscapeSeguro(giroAlternadoDerecha, GIRO_ESCAPE_BORDE_DOBLE_MS);
        avanceEscapeSeguro(AVANCE_INTERIOR_DOBLE_MS);
        giroAlternadoDerecha = !giroAlternadoDerecha;
    } else if (pisoDer) {
        retrocesoSeguro(RETROCESO_BORDE_SIMPLE_MS);
        giroEscapeSeguro(false, GIRO_ESCAPE_BORDE_SIMPLE_MS);
        avanceEscapeSeguro(AVANCE_INTERIOR_SIMPLE_MS);
    } else if (pisoIzq) {
        retrocesoSeguro(RETROCESO_BORDE_SIMPLE_MS);
        giroEscapeSeguro(true, GIRO_ESCAPE_BORDE_SIMPLE_MS);
        avanceEscapeSeguro(AVANCE_INTERIOR_SIMPLE_MS);
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
    // Importante: no se corta por "enemigo visto" porque en lateral el sensor
    // puede permanecer activo durante todo el giro y abortarlo demasiado pronto.
    if (sensorIzquierdo) {
        motores.curvaIzquierda(Velocidad_maxima);  // Solo rueda derecha gira
        esperarConPrioridadPiso(110);
    } else if (sensorDerecho) {
        motores.curvaDerecha(Velocidad_maxima);    // Solo rueda izquierda gira
        esperarConPrioridadPiso(110);
    }
}

void Robot::loop() {
    // ------------------ Control remoto (nivel + debounce) ------------------
    const unsigned long ahoraControl = millis();
    int lecturaControl = digitalRead(Pin_Control_Remoto);

    if (lecturaControl != estado_control_anterior) {
        estado_control_anterior = lecturaControl;
        ultimo_cambio_boton = ahoraControl;

        // Enciende inmediatamente al detectar Start, sin esperar debounce.
        bool ahora_activo = REMOTE_ACTIVE_HIGH ? (lecturaControl == HIGH) : (lecturaControl == LOW);
        if (ahora_activo) {
            robot_encendido = true;
        }
    }

    // Solo aplica debounce para apagar, evita cortes por ruido en la señal.
    if ((ahoraControl - ultimo_cambio_boton) >= debounce_delay) {
        bool controlActivo = REMOTE_ACTIVE_HIGH ? (lecturaControl == HIGH) : (lecturaControl == LOW);
        robot_encendido = controlActivo;
    }

    if (!robot_encendido) {
        detenerse();
        return;
    }

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
        if (tiempoSinContacto < 220) {
            inicioPulsoBusqueda = 0;
            motores.adelante(Velocidad_maxima);
        } else {
            // En un dojo de 70x70 conviene barrer compacto: avances cortos y giros en curva.
            const bool busquedaExtendida = tiempoSinContacto > 1200;
            const unsigned long duracionPulsoAvance = busquedaExtendida ? 140 : 105;
            const unsigned long duracionPulsoGiro = busquedaExtendida ? 95 : 80;
            const unsigned long duracionCicloBusqueda = duracionPulsoAvance + duracionPulsoGiro;
            const unsigned long ciclosAntesDeCambiar = busquedaExtendida ? 3UL : 4UL;

            if (inicioPulsoBusqueda == 0) {
                inicioPulsoBusqueda = ahora;
            }

            unsigned long tiempoEnBusqueda = ahora - inicioPulsoBusqueda;
            unsigned long tiempoEnCiclo = tiempoEnBusqueda % duracionCicloBusqueda;

            if (tiempoEnBusqueda >= (duracionCicloBusqueda * ciclosAntesDeCambiar)) {
                busquedaDerecha = !busquedaDerecha;
                inicioPulsoBusqueda = ahora;
                tiempoEnCiclo = 0;
            }

            ejecutarBusquedaCompacta(busquedaDerecha, tiempoEnCiclo, duracionPulsoAvance);
        }
    }
}
