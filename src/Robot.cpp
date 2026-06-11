#include "Robot.h"

namespace {
const bool TELEMETRIA_ACTIVA = false;
const unsigned long RETROCESO_BORDE_SIMPLE_MS = 350;
const unsigned long RETROCESO_BORDE_DOBLE_MS = 400;
const unsigned long GIRO_ESCAPE_BORDE_SIMPLE_MS = 760;
const unsigned long GIRO_ESCAPE_BORDE_DOBLE_MS = 840;
const unsigned long AVANCE_INTERIOR_SIMPLE_MS = 230;
const unsigned long AVANCE_INTERIOR_DOBLE_MS = 280;
const unsigned long GIRO_REUBICACION_MS = 720;
const unsigned long RETROCESO_INICIO_MS = 180;
const unsigned long MEDIA_VUELTA_INICIO_MS = 760;
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

bool Robot::puedePerseguirDuranteEvasion() {
    // Requiere piso estable por varias muestras para evitar falsos "seguro"
    // cuando aun esta cerca del borde.
    const uint8_t muestrasSeguras = 8;
    bool enemigoDetectado = false;

    for (uint8_t i = 0; i < muestrasSeguras; ++i) {
        bool pisoIzq = false;
        bool pisoDer = false;
        if (leerPiso(pisoIzq, pisoDer)) {
            return false;
        }

        if (enemigoVistoRapido()) {
            enemigoDetectado = true;
        }

        delay(3);
    }

    return enemigoDetectado;
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

bool Robot::retrocesoSeguro(unsigned long duracionMs) {
    unsigned long inicio = millis();
    while (millis() - inicio < duracionMs) {
        if (enemigoVistoRapido()) {
            return true;
        }

        retroceder();

        delay(5);
    }

    return false;
}

bool Robot::giroEscapeSeguro(bool haciaDerecha, unsigned long duracionMs) {
    unsigned long inicio = millis();

    while (millis() - inicio < duracionMs) {
        if (enemigoVistoRapido()) {
            return true;
        }

        if (haciaDerecha) {
            moverDerecha();
        } else {
            moverIzquierda();
        }

        delay(5);
    }

    return false;
}

bool Robot::avanceEscapeSeguro(unsigned long duracionMs) {
    const unsigned long avanceMinimoMs = 140;
    unsigned long inicio = millis();
    while (millis() - inicio < duracionMs) {
        if (enemigoVistoRapido()) {
            return true;
        }

        bool pisoIzq = false;
        bool pisoDer = false;
        if (leerPiso(pisoIzq, pisoDer)) {
            return false;
        }

        if ((millis() - inicio) >= avanceMinimoMs && puedePerseguirDuranteEvasion()) {
            return true;
        }

        moverAdelante();
        delay(5);
    }

    return false;
}

bool Robot::giroReubicacionSeguro(bool haciaDerecha, unsigned long duracionMs) {
    const unsigned long giroMinimoMs = 120;
    unsigned long inicio = millis();
    while (millis() - inicio < duracionMs) {
        if (enemigoVistoRapido()) {
            return true;
        }

        bool pisoIzq = false;
        bool pisoDer = false;
        bool enBorde = leerPiso(pisoIzq, pisoDer);

        if (!enBorde && (millis() - inicio) >= giroMinimoMs && puedePerseguirDuranteEvasion()) {
            return true;
        }

        if (haciaDerecha) {
            moverDerecha();
        } else {
            moverIzquierda();
        }
        delay(5);
    }

    return false;
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
    pinMode(Pin_Control_Remoto, INPUT_PULLUP);
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

    // Sin arrancador externo, el pull-up interno deja el robot listo para arrancar.
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

void Robot::ataqueSeguroPrioridadPiso() {
    bool pisoIzq = false;
    bool pisoDer = false;
    if (!leerPiso(pisoIzq, pisoDer)) {
        ataqueEnemigo();
        return;
    }

    // Si aun esta en borde, prioriza reingresar al dojo antes de empujar.
    unsigned long inicio = millis();
    while (millis() - inicio < 110) {
        retroceder();
        delay(5);
    }

    inicio = millis();
    while (millis() - inicio < 130) {
        bool pIzq = false;
        bool pDer = false;
        if (!leerPiso(pIzq, pDer)) {
            return;
        }

        if (pisoIzq && !pisoDer) {
            moverDerecha();
        } else if (pisoDer && !pisoIzq) {
            moverIzquierda();
        } else {
            moverDerecha();
        }
        delay(5);
    }
}

void Robot::moverAdelante() {
    motores.adelante(Velocidad_movimiento_seguir);
}

void Robot::retroceder() {
    motores.retroceder(Velocidad_estandar);
}

void Robot::moverDerecha() {
    motores.curvaDerecha(Velocidad_maxima);
}

void Robot::moverIzquierda() {
    motores.curvaIzquierda(Velocidad_maxima);
}

void Robot::sensoresPiso(bool pisoIzq, bool pisoDer) {
    static bool giroAlternadoDerecha = true;

    if (pisoIzq && pisoDer) {
        if (retrocesoSeguro(RETROCESO_BORDE_DOBLE_MS)) { ataqueSeguroPrioridadPiso(); return; }
        if (giroEscapeSeguro(giroAlternadoDerecha, GIRO_ESCAPE_BORDE_DOBLE_MS)) { ataqueSeguroPrioridadPiso(); return; }
        if (avanceEscapeSeguro(AVANCE_INTERIOR_DOBLE_MS)) { ataqueSeguroPrioridadPiso(); return; }
        if (giroReubicacionSeguro(giroAlternadoDerecha, GIRO_REUBICACION_MS)) { ataqueSeguroPrioridadPiso(); return; }
        giroAlternadoDerecha = !giroAlternadoDerecha;
    } else if (pisoDer) {
        if (retrocesoSeguro(RETROCESO_BORDE_SIMPLE_MS)) { ataqueSeguroPrioridadPiso(); return; }
        if (giroEscapeSeguro(false, GIRO_ESCAPE_BORDE_SIMPLE_MS)) { ataqueSeguroPrioridadPiso(); return; }
        if (avanceEscapeSeguro(AVANCE_INTERIOR_SIMPLE_MS)) { ataqueSeguroPrioridadPiso(); return; }
        if (giroReubicacionSeguro(true, GIRO_REUBICACION_MS)) { ataqueSeguroPrioridadPiso(); return; }
    } else if (pisoIzq) {
        if (retrocesoSeguro(RETROCESO_BORDE_SIMPLE_MS)) { ataqueSeguroPrioridadPiso(); return; }
        if (giroEscapeSeguro(true, GIRO_ESCAPE_BORDE_SIMPLE_MS)) { ataqueSeguroPrioridadPiso(); return; }
        if (avanceEscapeSeguro(AVANCE_INTERIOR_SIMPLE_MS)) { ataqueSeguroPrioridadPiso(); return; }
        if (giroReubicacionSeguro(false, GIRO_REUBICACION_MS)) { ataqueSeguroPrioridadPiso(); return; }
    }
}

void Robot::sensoresFrontales(bool central, bool derecho, bool izquierdo) {
    if (central || (derecho && izquierdo)) {
        ataqueEnemigo();
    } else if (derecho) {
        moverDerecha();
        if (esperarConPrioridadPisoYEnemigo(90)) {
            return;
        }
        motores.adelante(Velocidad_maxima);
        esperarConPrioridadPisoYEnemigo(90);
    } else if (izquierdo) {
        moverIzquierda();
        if (esperarConPrioridadPisoYEnemigo(90)) {
            return;
        }
        motores.adelante(Velocidad_maxima);
        esperarConPrioridadPisoYEnemigo(90);
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
        esperarConPrioridadPiso(160);
    } else if (sensorDerecho) {
        motores.curvaDerecha(Velocidad_maxima);    // Solo rueda izquierda gira
        esperarConPrioridadPiso(160);
    }
}

void Robot::loop() {
    // ------------------ Control remoto (nivel + debounce) ------------------
    static bool estadoAnteriorEncendido = false;
    static bool busquedaDerecha = true;
    static uint8_t faseBusqueda = 0;
    static unsigned long inicioFaseBusqueda = 0;
    static bool busquedaActiva = false;
    static unsigned long ultimoFrontalDerMs = 0;
    static unsigned long ultimoFrontalIzqMs = 0;
    static unsigned long ultimoFrontalCentralMs = 0;
    static unsigned long ultimoLateralDerMs = 0;
    static unsigned long ultimoLateralIzqMs = 0;
    static bool rutinaInicioPendiente = false;
    static uint8_t faseRutinaInicio = 0;
    static unsigned long inicioFaseRutinaInicio = 0;

    const unsigned long ahoraControl = millis();
    int lecturaControl = digitalRead(Pin_Control_Remoto);

    if (lecturaControl != estado_control_anterior) {
        estado_control_anterior = lecturaControl;
        ultimo_cambio_boton = ahoraControl;

        // Enciende inmediatamente al detectar Start.
        // El apagado se valida con debounce para evitar paradas por ruido.
        bool ahora_activo = REMOTE_ACTIVE_HIGH ? (lecturaControl == HIGH) : (lecturaControl == LOW);
        if (ahora_activo) {
            robot_encendido = true;

            // Cada START fuerza la rutina de llegar a la linea.
            faseBusqueda = 0;
            inicioFaseBusqueda = 0;
            busquedaDerecha = true;
            ultimoFrontalDerMs = 0;
            ultimoFrontalIzqMs = 0;
            ultimoFrontalCentralMs = 0;
            ultimoLateralDerMs = 0;
            ultimoLateralIzqMs = 0;
            busquedaActiva = false;
            rutinaInicioPendiente = true;
            faseRutinaInicio = 0;
            inicioFaseRutinaInicio = 0;
        }
    }

    // Solo aplica debounce para apagar, evita cortes por ruido en la señal.
    if ((ahoraControl - ultimo_cambio_boton) >= debounce_delay) {
        bool controlActivo = REMOTE_ACTIVE_HIGH ? (lecturaControl == HIGH) : (lecturaControl == LOW);
        robot_encendido = controlActivo;
    }

    const bool recienEncendido = (!estadoAnteriorEncendido && robot_encendido);

    if (!robot_encendido) {
        // Cada apagado reinicia estados para que al volver a ON siempre ejecute rutina inicial.
        busquedaDerecha = true;
        faseBusqueda = 0;
        inicioFaseBusqueda = 0;
        busquedaActiva = false;
        rutinaInicioPendiente = false;
        faseRutinaInicio = 0;
        inicioFaseRutinaInicio = 0;
        ultimoFrontalDerMs = 0;
        ultimoFrontalIzqMs = 0;
        ultimoFrontalCentralMs = 0;
        ultimoLateralDerMs = 0;
        ultimoLateralIzqMs = 0;

        detenerse();
        estadoAnteriorEncendido = false;
        return;
    }

    estadoAnteriorEncendido = true;

    if (recienEncendido) {
        // Arranque limpio de busqueda y sensores al pasar de OFF->ON.
        faseBusqueda = 0;
        inicioFaseBusqueda = 0;
        busquedaDerecha = true;
        ultimoFrontalDerMs = 0;
        ultimoFrontalIzqMs = 0;
        ultimoFrontalCentralMs = 0;
        ultimoLateralDerMs = 0;
        ultimoLateralIzqMs = 0;
        busquedaActiva = false;
        rutinaInicioPendiente = true;
        faseRutinaInicio = 0;
        inicioFaseRutinaInicio = 0;
    }

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
    (void)enemigoDetectado;

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

    // Rutina obligatoria de inicio: avanzar recto hasta tocar borde.
    // Mientras esta activa, no ataca ni ejecuta busqueda.
    if (rutinaInicioPendiente) {
        if (faseRutinaInicio == 0) {
            if (PisoIzq || PisoDer) {
                faseRutinaInicio = 1;
                inicioFaseRutinaInicio = ahora;
            } else {
                moverAdelante();
                return;
            }
        }

        if (faseRutinaInicio == 1) {
            if ((ahora - inicioFaseRutinaInicio) < RETROCESO_INICIO_MS) {
                retroceder();
                return;
            }
            faseRutinaInicio = 2;
            inicioFaseRutinaInicio = ahora;
        }

        if (faseRutinaInicio == 2) {
            if ((ahora - inicioFaseRutinaInicio) < MEDIA_VUELTA_INICIO_MS) {
                moverDerecha();
                return;
            }

            rutinaInicioPendiente = false;
            faseRutinaInicio = 0;
            inicioFaseRutinaInicio = 0;
            faseBusqueda = 0;
            inicioFaseBusqueda = 0;
            busquedaActiva = false;
        }
    }

    if (PisoIzq || PisoDer) {
        inicioFaseBusqueda = 0;
        faseBusqueda = 0;
        busquedaActiva = false;
        sensoresPiso(PisoIzq, PisoDer);
    } else if (LateralDer || LateralIzq) {
        inicioFaseBusqueda = 0;
        faseBusqueda = 0;
        busquedaActiva = false;
        sensoresLaterales(LateralIzq, LateralDer);
    } else if (FrontalDer || FrontalIzq || FrontalCentral) {
        inicioFaseBusqueda = 0;
        faseBusqueda = 0;
        busquedaActiva = false;
        sensoresFrontales(FrontalCentral, FrontalDer, FrontalIzq);
    } else {
        // Prioridad 3 (BAJA): busqueda sin enemigo.
        // Estrategia unica (sin modos): barrido trasero + lateral + empuje frontal corto.
        const int margenSeguridadPiso = 50;
        const bool cercaBordeIzq = valorPisoIzq <= (BLANCO + margenSeguridadPiso);
        const bool cercaBordeDer = valorPisoDer <= (BLANCO + margenSeguridadPiso);

        if (!busquedaActiva) {
            busquedaActiva = true;
            faseBusqueda = 0;
            inicioFaseBusqueda = ahora;
        }

        const unsigned long duraciones[6] = {
            1150, // Fase 0: giro largo para barrer espalda.
            300, // Fase 1: avance corto.
            1150, // Fase 2: giro largo contrario.
            300, // Fase 3: avance corto.
            760, // Fase 4: giro medio para cubrir lateral/frente.
            200  // Fase 5: empuje frontal corto.
        };

        if ((ahora - inicioFaseBusqueda) >= duraciones[faseBusqueda]) {
            inicioFaseBusqueda = ahora;
            faseBusqueda = (faseBusqueda + 1) % 6;
            if (faseBusqueda == 0) {
                busquedaDerecha = !busquedaDerecha;
            }
        }

        // Si detecta cercania de borde durante la busqueda, recentra primero.
        if (cercaBordeIzq || cercaBordeDer) {
            if (cercaBordeIzq && !cercaBordeDer) {
                moverDerecha();
            } else if (cercaBordeDer && !cercaBordeIzq) {
                moverIzquierda();
            } else if (busquedaDerecha) {
                moverDerecha();
            } else {
                moverIzquierda();
            }
            faseBusqueda = 0;
            inicioFaseBusqueda = ahora;
            return;
        }

        if (faseBusqueda == 0) {
            if (busquedaDerecha) moverDerecha(); else moverIzquierda();
        } else if (faseBusqueda == 1) {
            motores.adelante(Velocidad_maxima);
        } else if (faseBusqueda == 2) {
            if (busquedaDerecha) moverIzquierda(); else moverDerecha();
        } else if (faseBusqueda == 3) {
            motores.adelante(Velocidad_maxima);
        } else if (faseBusqueda == 4) {
            if (busquedaDerecha) moverDerecha(); else moverIzquierda();
        } else {
            motores.adelante(Velocidad_maxima);
        }
    }
}
