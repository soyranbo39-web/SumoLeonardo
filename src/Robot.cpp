#include "Robot.h"

namespace {
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
    static bool estadoAnteriorEncendido = false;
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
        memcpy(qtable, QTABLE_DEFAULTS, sizeof(qtable));
        qtableIniciada = true;
    }
    static uint8_t estadoPrev = 0;
    static uint8_t accionPrev = 5;
    // PID angular: error = posicion angular del enemigo [-4,+4], salida = diferencial de motores
    static float pidIntegral  = 0.0f;
    static float pidPrevError = 0.0f;
    static float pidSalida    = 0.0f;
    static unsigned long pidTmsAnt = 0;
    // Prediccion lineal de primer orden: estima angulo del enemigo en ~80ms
    static float anguloAnterior   = 0.0f;
    static float velAngular       = 0.0f;
    static unsigned long tAngPrev = 0;

    const unsigned long ahoraControl = millis();
    int lecturaControl = digitalRead(Pin_Control_Remoto);

    if (lecturaControl != estado_control_anterior) {
        estado_control_anterior = lecturaControl;
        ultimo_cambio_boton = ahoraControl;

        // Cambia de estado inmediatamente en flancos del control remoto.
        bool ahora_activo = REMOTE_ACTIVE_HIGH ? (lecturaControl == HIGH) : (lecturaControl == LOW);
        robot_encendido = ahora_activo;
    }

    // Solo aplica debounce para apagar, evita cortes por ruido en la señal.
    if ((ahoraControl - ultimo_cambio_boton) >= debounce_delay) {
        bool controlActivo = REMOTE_ACTIVE_HIGH ? (lecturaControl == HIGH) : (lecturaControl == LOW);
        robot_encendido = controlActivo;
    }

    const bool recienEncendido = (!estadoAnteriorEncendido && robot_encendido);

    if (!robot_encendido) {
        // Reset inmediato de busqueda al apagar, para que arranque limpio.
        busquedaDerecha = true;
        faseBusqueda = 0;
        inicioFaseBusqueda = 0;
        busquedaActiva = false;

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
        memcpy(qtable, QTABLE_DEFAULTS, sizeof(qtable));
        busquedaActiva = false;
    }

    const unsigned long ahora = millis();

    // --- Kalman 1D para sensores de piso (analogicos) ---
    // Suaviza ruido electrico y vibracion para evitar falsas detecciones de borde.
    const float KF_Q = 8.0f;   // ruido de proceso
    const float KF_R = 25.0f;  // ruido de medicion

    int valorPisoIzq = analogRead(SENSOR_DE_PISO_IZQUIERDO);
    int valorPisoDer = analogRead(SENSOR_DE_PISO_DERECHO);

    kfPisoIzqP += KF_Q;
    float K_izq = kfPisoIzqP / (kfPisoIzqP + KF_R);
    kfPisoIzqX += K_izq * ((float)valorPisoIzq - kfPisoIzqX);
    kfPisoIzqP *= (1.0f - K_izq);

    kfPisoDerP += KF_Q;
    float K_der = kfPisoDerP / (kfPisoDerP + KF_R);
    kfPisoDerX += K_der * ((float)valorPisoDer - kfPisoDerX);
    kfPisoDerP *= (1.0f - K_der);

    bool PisoIzq = (kfPisoIzqX <= (float)BLANCO);
    bool PisoDer = (kfPisoDerX <= (float)BLANCO);

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

    const bool enemigoDetectado = FrontalDer || FrontalIzq || FrontalCentral || LateralDer || LateralIzq;
    (void)enemigoDetectado;

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
    const float KP_PID = 50.0f;
    const float KI_PID =  2.0f;
    const float KD_PID =  4.0f;
    {
        float dtPidMs = (float)(ahora - pidTmsAnt);
        if (dtPidMs > 0.0f && dtPidMs < 100.0f) {
            float dtS = dtPidMs * 0.001f;
            pidIntegral += anguloPred * dtS;
            if (pidIntegral >  3.0f) pidIntegral =  3.0f;
            if (pidIntegral < -3.0f) pidIntegral = -3.0f;
            float deriv = (anguloPred - pidPrevError) / dtS;
            pidSalida = KP_PID * anguloPred + KI_PID * pidIntegral + KD_PID * deriv;
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

    // Actualiza Q-tabla con resultado de la iteracion anterior (Bellman / online).
    {
        int8_t recompensa = (PisoIzq || PisoDer) ? -20 : (estadoQ != 0 ? 5 : 0);
        int8_t maxQNuevo  = qtable[estadoQ][0];
        for (uint8_t a = 1; a < 6; a++) {
            if (qtable[estadoQ][a] > maxQNuevo) maxQNuevo = qtable[estadoQ][a];
        }
        float qUpd = (float)qtable[estadoPrev][accionPrev]
                   + 0.15f * ((float)recompensa + 0.9f * (float)maxQNuevo
                              - (float)qtable[estadoPrev][accionPrev]);
        int qC = (int)qUpd;
        if (qC >  120) qC =  120;
        if (qC < -100) qC = -100;
        qtable[estadoPrev][accionPrev] = (int8_t)qC;
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

    // Sin enemigo detectado, ejecuta siempre la busqueda clasica por fases.
    if (estadoQ == 0) {
        accionQ = 5;
    }

    estadoPrev = estadoQ;
    accionPrev = accionQ;

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

                const unsigned long duraciones[6] = {
                    780, 500, 780, 500, 520, 300
                };

                if ((ahora - inicioFaseBusqueda) >= duraciones[faseBusqueda]) {
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
