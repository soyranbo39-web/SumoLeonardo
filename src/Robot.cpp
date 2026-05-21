#include "Robot.h"
#include <EEPROM.h>

namespace {
const unsigned long RETROCESO_BORDE_SIMPLE_MS = 260;
const unsigned long RETROCESO_BORDE_DOBLE_MS = 320;
const unsigned long GIRO_ESCAPE_BORDE_SIMPLE_MS = 220;
const unsigned long GIRO_ESCAPE_BORDE_DOBLE_MS = 250;
const unsigned long AVANCE_INTERIOR_SIMPLE_MS = 140;
const unsigned long AVANCE_INTERIOR_DOBLE_MS = 170;

const uint16_t QTABLE_EEPROM_MAGIC = 0x534C; // "SL"
const uint8_t QTABLE_EEPROM_VERSION = 2;
const int QTABLE_STATES = 32;
const int QTABLE_ACTIONS = 6;
const uint8_t QTABLE_PROFILE_COUNT = 3;
const unsigned long QTABLE_GUARDADO_INTERVALO_MS = 15000;

enum QTableProfile : uint8_t {
    PROFILE_GENERAL = 0,
    PROFILE_FRONTAL = 1,
    PROFILE_LATERAL = 2
};

const unsigned long PERFIL_MUESTREO_MS = 1200;
const int PERFIL_MARGEN_DECISION = 6;

struct PerfilControl {
    float kp;
    float ki;
    float kd;
    uint8_t epsilonInicial;
    unsigned long duracionesBusqueda[6];
};

const PerfilControl PERFIL_CONTROL[QTABLE_PROFILE_COUNT] = {
    // GENERAL
    {50.0f, 2.0f, 4.0f, 10, {780, 500, 780, 500, 520, 300}},
    // FRONTAL
    {56.0f, 2.5f, 4.5f, 8,  {700, 420, 700, 420, 470, 260}},
    // LATERAL
    {44.0f, 1.6f, 3.6f, 14, {860, 560, 860, 560, 590, 340}}
};

struct QTableHeader {
    uint16_t magic;
    uint8_t version;
    uint8_t profileCount;
};

int qtableEepromHeaderAddress() {
    return 0;
}

int qtableEepromDataAddress() {
    return (int)sizeof(QTableHeader);
}

int qtableEepromProfileAddress(uint8_t profile) {
    return qtableEepromDataAddress() + (int)profile * (int)sizeof(int8_t[QTABLE_STATES][QTABLE_ACTIONS]);
}

bool qtableHeaderValido(const QTableHeader &h) {
    return h.magic == QTABLE_EEPROM_MAGIC &&
           h.version == QTABLE_EEPROM_VERSION &&
           h.profileCount == QTABLE_PROFILE_COUNT;
}

bool cargarQTableDesdeEEPROM(uint8_t profile, int8_t (&qtable)[QTABLE_STATES][QTABLE_ACTIONS]) {
    if (profile >= QTABLE_PROFILE_COUNT) {
        return false;
    }

    QTableHeader h;
    EEPROM.get(qtableEepromHeaderAddress(), h);
    if (!qtableHeaderValido(h)) {
        return false;
    }

    EEPROM.get(qtableEepromProfileAddress(profile), qtable);
    return true;
}

void guardarQTableEnEEPROM(uint8_t profile, const int8_t (&qtable)[QTABLE_STATES][QTABLE_ACTIONS]) {
    if (profile >= QTABLE_PROFILE_COUNT) {
        return;
    }

    const QTableHeader h = {QTABLE_EEPROM_MAGIC, QTABLE_EEPROM_VERSION, QTABLE_PROFILE_COUNT};
    EEPROM.put(qtableEepromHeaderAddress(), h);
    EEPROM.put(qtableEepromProfileAddress(profile), qtable);
}

void inicializarPerfilesQTableEnEEPROM(const int8_t (&qtableDefaults)[QTABLE_STATES][QTABLE_ACTIONS]) {
    const QTableHeader h = {QTABLE_EEPROM_MAGIC, QTABLE_EEPROM_VERSION, QTABLE_PROFILE_COUNT};
    EEPROM.put(qtableEepromHeaderAddress(), h);
    for (uint8_t p = 0; p < QTABLE_PROFILE_COUNT; p++) {
        EEPROM.put(qtableEepromProfileAddress(p), qtableDefaults);
    }
}

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

    // Ya no se usa control remoto para encendido.
    estado_control_anterior = LOW;
    robot_encendido = true;
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

// Valores iniciales de la Q-tabla. Se usan para restaurar al arrancar.
static const int8_t QTABLE_DEFAULTS[32][6] = {
    /* s00 */ { 0,  0,  0,  0,  0, 20},
    /* s01 */ { 5, 10,  0, 20,  0,  0},
    /* s02 */ { 5,  0, 10,  0, 20,  0},
    /* s03 */ {20,  0,  0,  5,  5,  0},
    /* s04 */ {20,  0,  0,  0,  0,  0},
    /* s05 */ {20,  0,  0, 10,  0,  0},
    /* s06 */ {20,  0,  0,  0, 10,  0},
    /* s07 */ {20,  0,  0,  0,  0,  0},
    /* s08 */ { 0, 20,  0, 10,  0,  0},
    /* s09 */ {10,  5,  0, 20,  0,  0},
    /* s10 */ {20,  0,  0,  5,  0,  0},
    /* s11 */ {20,  0,  0, 10,  0,  0},
    /* s12 */ {20,  0,  0,  5,  0,  0},
    /* s13 */ {20,  0,  0, 10,  0,  0},
    /* s14 */ {10,  5,  0, 20,  0,  0},
    /* s15 */ {20,  0,  0,  5,  0,  0},
    /* s16 */ { 0,  0, 20,  0, 10,  0},
    /* s17 */ {20,  0,  0,  0,  0,  0},
    /* s18 */ {10,  0,  5,  0, 20,  0},
    /* s19 */ {20,  0,  0,  0,  5,  0},
    /* s20 */ {20,  0,  0,  0,  5,  0},
    /* s21 */ {10,  0,  5,  0, 20,  0},
    /* s22 */ {20,  0,  0,  0, 10,  0},
    /* s23 */ {20,  0,  0,  0,  5,  0},
    /* s24 */ {20,  0,  0,  0,  0,  0},
    /* s25 */ {20,  0,  0, 10,  0,  0},
    /* s26 */ {20,  0,  0,  0, 10,  0},
    /* s27 */ {20,  0,  0,  0,  0,  0},
    /* s28 */ {20,  0,  0,  0,  0,  0},
    /* s29 */ {20,  0,  0,  5,  0,  0},
    /* s30 */ {20,  0,  0,  0,  5,  0},
    /* s31 */ {20,  0,  0,  0,  0,  0},
};

void Robot::loop() {
    // ------------------ Control remoto (nivel + debounce) ------------------
    static bool qtableSucia = false;
    static unsigned long ultimoGuardadoQMs = 0;
    static uint8_t perfilQActivo = PROFILE_GENERAL;
    static float kpPidActual = 50.0f;
    static float kiPidActual = 2.0f;
    static float kdPidActual = 4.0f;
    static uint8_t epsilonPct = 10;
    static unsigned long ultimoDecayEpsilonMs = 0;
    static unsigned long duracionesBusqueda[6] = {780, 500, 780, 500, 520, 300};
    static bool busquedaDerecha = true;
    static uint8_t faseBusqueda = 0;
    static unsigned long inicioFaseBusqueda = 0;
    static bool busquedaActiva = false;
    // Kalman 1D para sensores de piso (analogicos)
    static float kfPisoIzqX = 500.0f;
    static float kfPisoIzqP = 50.0f;
    static float kfPisoDerX = 500.0f;
    static float kfPisoDerP = 50.0f;
    // Filtro de confianza para sensores de enemigo (digitales)
    static int8_t confFrontalDer     = 0;
    static int8_t confFrontalIzq     = 0;
    static int8_t confFrontalCentral = 0;
    static int8_t confLateralDer     = 0;
    static int8_t confLateralIzq     = 0;
    // Q-tabla [32 estados x 6 acciones] — se actualiza online durante el combate.
    // Estado = bitmask: bit0=FrDer, bit1=FrIzq, bit2=FrCen, bit3=LatDer, bit4=LatIzq
    // Acciones: 0=ATAQUE(PID), 1=GIRO_DER, 2=GIRO_IZQ, 3=CURVA_DER, 4=CURVA_IZQ, 5=BUSCAR
    static int8_t qtable[32][6];
    static bool qtableIniciada = false;
    if (!qtableIniciada) {
        QTableHeader h;
        EEPROM.get(qtableEepromHeaderAddress(), h);
        if (!qtableHeaderValido(h)) {
            inicializarPerfilesQTableEnEEPROM(QTABLE_DEFAULTS);
        }

        if (!cargarQTableDesdeEEPROM(PROFILE_GENERAL, qtable)) {
            memcpy(qtable, QTABLE_DEFAULTS, sizeof(qtable));
            guardarQTableEnEEPROM(PROFILE_GENERAL, qtable);
        }

        perfilQActivo = PROFILE_GENERAL;
        kpPidActual = PERFIL_CONTROL[perfilQActivo].kp;
        kiPidActual = PERFIL_CONTROL[perfilQActivo].ki;
        kdPidActual = PERFIL_CONTROL[perfilQActivo].kd;
        epsilonPct = PERFIL_CONTROL[perfilQActivo].epsilonInicial;
        memcpy(duracionesBusqueda, PERFIL_CONTROL[perfilQActivo].duracionesBusqueda, sizeof(duracionesBusqueda));
        qtableSucia = false;
        ultimoGuardadoQMs = millis();
        ultimoDecayEpsilonMs = millis();
        qtableIniciada = true;
    }
    static uint8_t estadoPrev = 0;
    static uint8_t accionPrev = 5;
    static uint8_t rachaSinEnemigo = 0;
    // PID angular: error = posicion angular del enemigo [-4,+4], salida = diferencial de motores
    static float pidIntegral  = 0.0f;
    static float pidPrevError = 0.0f;
    static float pidSalida    = 0.0f;
    static unsigned long pidTmsAnt = 0;
    // Prediccion lineal de primer orden: estima angulo del enemigo en ~80ms
    static float anguloAnterior   = 0.0f;
    static float velAngular       = 0.0f;
    static unsigned long tAngPrev = 0;

    const unsigned long ahora = millis();
    // Leer sensores de piso para rutina inicial y lógica general
    int valorPisoIzq = analogRead(SENSOR_DE_PISO_IZQUIERDO);
    int valorPisoDer = analogRead(SENSOR_DE_PISO_DERECHO);
    bool PisoIzq = (valorPisoIzq <= (float)BLANCO);
    bool PisoDer = (valorPisoDer <= (float)BLANCO);

    // Rutina inicial siempre se ejecuta al arranque
    static bool rutinaInicialCompletada = false;
    static bool rutinaInicialEnCurso = true;
    static bool rutinaDetectoBorde = false;
    static unsigned long rutinaEvasionMs = 0;
    static bool rutinaInicializada = false;
    if (!rutinaInicializada) {
        // Inicializa variables de rutina inicial
        faseBusqueda = 0;
        inicioFaseBusqueda = 0;
        busquedaDerecha = true;
        kfPisoIzqX = 500.0f; kfPisoIzqP = 50.0f;
        kfPisoDerX = 500.0f; kfPisoDerP = 50.0f;
        confFrontalDer     = 0;
        confFrontalIzq     = 0;
        confFrontalCentral = 0;
        confLateralDer     = 0;
        confLateralIzq     = 0;
        pidIntegral  = 0.0f;
        pidPrevError = 0.0f;
        pidSalida    = 0.0f;
        pidTmsAnt    = 0;
        anguloAnterior = 0.0f;
        velAngular     = 0.0f;
        tAngPrev       = 0;
        estadoPrev = 0;
        accionPrev = 5;
        rachaSinEnemigo = 0;
        busquedaActiva = false;
        rutinaInicialCompletada = false;
        rutinaInicialEnCurso = true;
        rutinaDetectoBorde = false;
        rutinaEvasionMs = 0;
        rutinaInicializada = true;
    }

    // Rutina inicial reforzada: avanzar lento hasta tocar borde, luego retroceder y girar con tiempos notorios
    if (!rutinaInicialCompletada && rutinaInicialEnCurso) {
        if (!rutinaDetectoBorde) {
            // Avanza muy lento hasta detectar borde
            motores.adelante(Velocidad_baja); // Usa velocidad baja para mayor control
            if (PisoIzq || PisoDer) {
                rutinaDetectoBorde = true;
                rutinaEvasionMs = ahora;
                // Retrocede fuerte al detectar borde
                motores.retroceder(Velocidad_estandar);
            }
            return; // Bloquea toda la lógica normal
        } else {
            // Evasión: retrocede 700ms, luego gira 600ms
            if (ahora - rutinaEvasionMs < 700) {
                motores.retroceder(Velocidad_estandar);
                return;
            } else if (ahora - rutinaEvasionMs < 1300) {
                motores.derecha(Velocidad_estandar);
                return;
            } else {
                motores.detener();
                rutinaInicialCompletada = true;
                rutinaInicialEnCurso = false;
                // Espera un ciclo antes de activar lógica normal
                return;
            }
        }
    }

    // (Eliminada segunda declaración redundante de 'ahora')

    // --- Kalman 1D para sensores de piso (analogicos) ---
    // Suaviza ruido electrico y vibracion para evitar falsas detecciones de borde.
    const float KF_Q = 8.0f;   // ruido de proceso
    const float KF_R = 25.0f;  // ruido de medicion

    kfPisoIzqP += KF_Q;
    float K_izq = kfPisoIzqP / (kfPisoIzqP + KF_R);
    kfPisoIzqX += K_izq * ((float)valorPisoIzq - kfPisoIzqX);
    kfPisoIzqP *= (1.0f - K_izq);

    kfPisoDerP += KF_Q;
    float K_der = kfPisoDerP / (kfPisoDerP + KF_R);
    kfPisoDerX += K_der * ((float)valorPisoDer - kfPisoDerX);
    kfPisoDerP *= (1.0f - K_der);

    // PisoIzq y PisoDer ya calculados arriba para rutina inicial

    // --- Filtro de confianza para sensores de enemigo (digitales) ---
    // +2 al detectar (respuesta rapida), -1 sin deteccion (histeresis).
    // Activo si contador >= 2. Rango [0, 4].
    confFrontalDer     = (int8_t)constrain(confFrontalDer     + (sensorFrontalDer.detectar() ? 2 : -1), 0, 4);
    confFrontalIzq     = (int8_t)constrain(confFrontalIzq     + (sensorFrontalIzq.detectar() ? 2 : -1), 0, 4);
    confFrontalCentral = (int8_t)constrain(confFrontalCentral + (sensorFrontal.detectar()    ? 2 : -1), 0, 4);
    confLateralDer     = (int8_t)constrain(confLateralDer     + (sensorLateralDer.detectar() ? 2 : -1), 0, 4);
    confLateralIzq     = (int8_t)constrain(confLateralIzq     + (sensorLateralIzq.detectar() ? 2 : -1), 0, 4);

    bool FrontalDer     = (confFrontalDer     >= 2);
    bool FrontalIzq     = (confFrontalIzq     >= 2);
    bool FrontalCentral = (confFrontalCentral >= 2);
    bool LateralDer     = (confLateralDer     >= 2);
    bool LateralIzq     = (confLateralIzq     >= 2);

    // ...existing code...

    // --- Posicion angular del enemigo estimada desde valores de confianza [-4, +4] ---
    float totalConf = (float)(confFrontalDer + confFrontalIzq + confFrontalCentral +
                               confLateralDer + confLateralIzq);
    float angulo = 0.0f;
    if (totalConf > 0.0f) {
        angulo = ((float)confFrontalDer  *  2.0f - (float)confFrontalIzq  *  2.0f +
                  (float)confLateralDer  *  4.0f - (float)confLateralIzq  *  4.0f)
                 / totalConf;
    }

    // --- Prediccion lineal a 80ms: adelanta la reaccion ante enemigos rapidos ---
    {
        unsigned long dtAngMs = ahora - tAngPrev;
        if (dtAngMs > 0 && dtAngMs < 100) {
            velAngular = (angulo - anguloAnterior) / (float)dtAngMs;
        }
        anguloAnterior = angulo;
        tAngPrev = ahora;
    }
    float anguloPred = angulo + velAngular * 80.0f;

    // --- PID angular: diferencial de velocidad para centrar al enemigo ---
    {
        float dtPidMs = (float)(ahora - pidTmsAnt);
        if (dtPidMs > 0.0f && dtPidMs < 100.0f) {
            float dtS = dtPidMs * 0.001f;
            pidIntegral += anguloPred * dtS;
            if (pidIntegral >  3.0f) pidIntegral =  3.0f;
            if (pidIntegral < -3.0f) pidIntegral = -3.0f;
            float deriv = (anguloPred - pidPrevError) / dtS;
            pidSalida = kpPidActual * anguloPred + kiPidActual * pidIntegral + kdPidActual * deriv;
        }
        pidPrevError = anguloPred;
        pidTmsAnt    = ahora;
    }

    // --- Q-tabla: construye estado y selecciona accion optima ---
    uint8_t estadoQ = 0;
    if (FrontalDer)     estadoQ |= 0x01;
    if (FrontalIzq)     estadoQ |= 0x02;
    if (FrontalCentral) estadoQ |= 0x04;
    if (LateralDer)     estadoQ |= 0x08;
    if (LateralIzq)     estadoQ |= 0x10;

    // Persistencia de ataque: sigue atacando hasta 1200 ms después de perder al enemigo
    static unsigned long ultimoEnemigoDetectadoMs = 0;
    static bool persistirAtaque = false;
    const unsigned long persistenciaAtaqueMs = 1200;

    const bool enemigoDetectadoAhora = (estadoQ != 0);
    if (enemigoDetectadoAhora) {
        ultimoEnemigoDetectadoMs = ahora;
        persistirAtaque = true;
    } else if (persistirAtaque && (ahora - ultimoEnemigoDetectadoMs > persistenciaAtaqueMs)) {
        persistirAtaque = false;
    }

    // Si está en persistencia de ataque, fuerza estadoQ distinto de cero
    uint8_t estadoQ_persistente = estadoQ;
    if (persistirAtaque && estadoQ == 0) {
        // Mantiene el último estado de ataque (por defecto, frontal)
        estadoQ_persistente = 0x04; // FrontalCentral
    }

    if (estadoQ == 0) {
        if (rachaSinEnemigo < 250) rachaSinEnemigo++;
    } else {
        rachaSinEnemigo = 0;
    }

    // Actualiza Q-tabla con resultado de la iteracion anterior (Bellman / online).
    {
        int recompensa = 0;
        if (PisoIzq || PisoDer) {
            recompensa = -25;
        } else {
            recompensa += (estadoQ != 0) ? 3 : -1;

            const float absAnguloPred = fabs(anguloPred);
            if (estadoQ != 0 && absAnguloPred <= 0.8f) {
                recompensa += 3;
            } else if (estadoQ != 0 && absAnguloPred <= 1.8f) {
                recompensa += 1;
            }

            recompensa -= (int)(rachaSinEnemigo / 6);
            if (accionPrev >= 1 && accionPrev <= 4 && rachaSinEnemigo > 8) {
                recompensa -= 2;
            }
        }

        if (recompensa > 20) recompensa = 20;
        if (recompensa < -30) recompensa = -30;

        int8_t maxQNuevo  = qtable[estadoQ][0];
        for (uint8_t a = 1; a < 6; a++) {
            if (qtable[estadoQ][a] > maxQNuevo) maxQNuevo = qtable[estadoQ][a];
        }
        const int8_t qAnterior = qtable[estadoPrev][accionPrev];
        float qUpd = (float)qtable[estadoPrev][accionPrev]
                   + 0.15f * ((float)recompensa + 0.9f * (float)maxQNuevo
                              - (float)qtable[estadoPrev][accionPrev]);
        int qC = (int)qUpd;
        if (qC >  120) qC =  120;
        if (qC < -100) qC = -100;
        qtable[estadoPrev][accionPrev] = (int8_t)qC;
        if (qtable[estadoPrev][accionPrev] != qAnterior) {
            qtableSucia = true;
        }
    }

    // Selecciona la accion con mayor Q-valor.
    uint8_t accionQ = 0;
    {
        int8_t bestQ = qtable[estadoQ][0];
        for (uint8_t a = 1; a < 6; a++) {
            if (qtable[estadoQ][a] > bestQ) {
                bestQ   = qtable[estadoQ][a];
                accionQ = a;
            }
        }
    }

    // Exploracion epsilon-greedy (solo cuando hay enemigo detectado o persistencia de ataque).
    if (estadoQ_persistente != 0 && epsilonPct > 0) {
        if ((uint8_t)random(100) < epsilonPct) {
            accionQ = (uint8_t)random(5);
        }
    }

    // Decaimiento suave de epsilon para estabilizar estrategia durante el combate.
    if ((ahora - ultimoDecayEpsilonMs) >= 250) {
        ultimoDecayEpsilonMs = ahora;
        if (epsilonPct > 2) {
            epsilonPct--;
        }
    }

    // Sin enemigo detectado ni persistencia, ejecuta búsqueda; si hay persistencia, sigue atacando
    if (estadoQ_persistente == 0) {
        accionQ = 5;
    }

    estadoPrev = estadoQ_persistente;
    accionPrev = accionQ;

    // Guardado periodico para reducir perdida de aprendizaje sin castigar EEPROM.
    if (qtableSucia && (ahora - ultimoGuardadoQMs >= QTABLE_GUARDADO_INTERVALO_MS)) {
        guardarQTableEnEEPROM(perfilQActivo, qtable);
        qtableSucia = false;
        ultimoGuardadoQMs = ahora;
    }

    // --- Ejecucion: piso tiene prioridad absoluta (seguridad), Q-tabla gestiona el resto ---
    if (PisoIzq || PisoDer) {
        faseBusqueda = 0;
        inicioFaseBusqueda = 0;
        busquedaActiva = false;
        pidIntegral = 0.0f;
        sensoresPiso(PisoIzq, PisoDer);
    } else {
        switch (accionQ) {
            case 0: {
                // Ataque con PID diferencial: centra al enemigo ajustando velocidades.
                float out = pidSalida;
                if (out >  (float)Velocidad_maxima) out =  (float)Velocidad_maxima;
                if (out < -(float)Velocidad_maxima) out = -(float)Velocidad_maxima;
                int velI = (int)((float)Velocidad_maxima + out);
                int velD = (int)((float)Velocidad_maxima - out);
                if (velI > Velocidad_maxima) velI = Velocidad_maxima;
                if (velI < 60) velI = 60;
                if (velD > Velocidad_maxima) velD = Velocidad_maxima;
                if (velD < 60) velD = 60;
                motores.getIzquierdo().avanzar(velI);
                motores.getDerecho().avanzar(velD);
                faseBusqueda = 0;
                busquedaActiva = false;
                break;
            }
            case 1:
                moverDerecha();
                faseBusqueda = 0;
                busquedaActiva = false;
                break;
            case 2:
                moverIzquierda();
                faseBusqueda = 0;
                busquedaActiva = false;
                break;
            case 3:
                motores.curvaDerecha(Velocidad_maxima);
                faseBusqueda = 0;
                busquedaActiva = false;
                break;
            case 4:
                motores.curvaIzquierda(Velocidad_maxima);
                faseBusqueda = 0;
                busquedaActiva = false;
                break;
            default: {
                // BUSCAR: patron por fases (barrido trasero + lateral + empuje frontal).
                const int margenSeguridadPiso = 35;
                const bool cercaBordeIzq = kfPisoIzqX <= (float)(BLANCO + margenSeguridadPiso);
                const bool cercaBordeDer = kfPisoDerX <= (float)(BLANCO + margenSeguridadPiso);

                if (!busquedaActiva) {
                    busquedaActiva = true;
                    faseBusqueda = 0;
                    inicioFaseBusqueda = ahora;
                }

                if ((ahora - inicioFaseBusqueda) >= duracionesBusqueda[faseBusqueda]) {
                    inicioFaseBusqueda = ahora;
                    faseBusqueda = (faseBusqueda + 1) % 6;
                    if (faseBusqueda == 0) busquedaDerecha = !busquedaDerecha;
                }

                if (cercaBordeIzq || cercaBordeDer) {
                    if      (cercaBordeIzq && !cercaBordeDer) moverDerecha();
                    else if (cercaBordeDer && !cercaBordeIzq) moverIzquierda();
                    else if (busquedaDerecha)                 moverDerecha();
                    else                                      moverIzquierda();
                    faseBusqueda = 0;
                    inicioFaseBusqueda = ahora;
                    return;
                }

                if      (faseBusqueda == 0) { if (busquedaDerecha) moverDerecha();   else moverIzquierda(); }
                else if (faseBusqueda == 1) { motores.adelante(Velocidad_maxima); }
                else if (faseBusqueda == 2) { if (busquedaDerecha) moverIzquierda(); else moverDerecha();   }
                else if (faseBusqueda == 3) { motores.adelante(Velocidad_maxima); }
                else if (faseBusqueda == 4) { if (busquedaDerecha) moverDerecha();   else moverIzquierda(); }
                else                        { motores.adelante(Velocidad_maxima); }
                break;
            }
        }
    }
}
