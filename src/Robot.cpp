#include "Robot.h"

namespace {
const unsigned long RETROCESO_BORDE_SIMPLE_MS = 280;
const unsigned long RETROCESO_BORDE_DOBLE_MS = 340;
const unsigned long GIRO_ESCAPE_BORDE_SIMPLE_MS = 680;
const unsigned long GIRO_ESCAPE_BORDE_DOBLE_MS = 750;
const unsigned long RETROCESO_BORDE_MINIMO_MS = 95;
const unsigned long CONFIRMACION_LIBRE_BORDE_MS = 26;
const unsigned long AVANCE_INTERIOR_SIMPLE_MS = 140;
const unsigned long AVANCE_INTERIOR_DOBLE_MS = 170;
const unsigned long CORRECCION_REINGRESO_SIMPLE_MS = 90;
const unsigned long CORRECCION_REINGRESO_DOBLE_MS = 120;
const unsigned long RETROCESO_PREVENTIVO_BORDE_MS = 200;
const unsigned long BLOQUEO_POST_BORDE_MS = 280;
const int VELOCIDAD_RETROCESO_BORDE = 240;
const int VELOCIDAD_MANIOBRA = 200;
const int VELOCIDAD_MANIOBRA_ATAQUE = 210;
const int VELOCIDAD_REINGRESO_COMBATE = 188;
const int VELOCIDAD_BUSQUEDA = 175;
const int VELOCIDAD_BUSQUEDA_AGRESIVA = 198;
const int DIFERENCIAL_ARCO_BUSQUEDA = 58;
const int VELOCIDAD_LATERAL = 160;
const int VELOCIDAD_RETROCESO_NORMAL = 150;
const unsigned long MEMORIA_OBJETIVO_MS = 260;
const unsigned long PIVOTE_LATERAL_MS = 120;
const unsigned long EMPUJE_TRAS_PIVOTE_MS = 90;
const unsigned long RETROCESO_BORDE_DOBLE_EXTRA_MS = 140;
const unsigned long GIRO_ESCAPE_BORDE_DOBLE_EXTRA_MS = 90;
const int VELOCIDAD_ATAQUE_FRENTE = 255;
const int VELOCIDAD_ATAQUE_CRUCERO = 230;
const int BONO_PULSO_ATAQUE = 25;
const int BONO_PULSO_ATAQUE_CENTRO = 15;
const float ANGULO_MAX_PULSO_ATAQUE = 1.4f;
const unsigned long ATAQUE_PULSO_ON_MS = 70;
const unsigned long ATAQUE_PULSO_OFF_MS = 35;
const unsigned long MOV_PULSO_ON_MS = 60;
const unsigned long MOV_PULSO_OFF_MS = 35;
const int BONO_PULSO_TRACCION = 18;
const int VELOCIDAD_MIN_TRACCION = 65;
const int VELOCIDAD_AVANCE_NORMAL = 100;
const unsigned long RUTINA_INICIO_GIRO_MEDIA_VUELTA_MS = 470;
const unsigned long RUTINA_INICIO_ASENTAR_MS = 80;
const unsigned long RUTINA_INICIO_RETROCESO_MS = 280;
const int VELOCIDAD_RUTINA_INICIO = 68;
const int VELOCIDAD_RUTINA_INICIO_LENTA = 48;
const int VELOCIDAD_RUTINA_INICIO_RETROCESO = 150;
const int MARGEN_PREVENTIVO_BORDE = 42;
const int MARGEN_DETECCION_BORDE_CERCA = 130;
const int MARGEN_DETECCION_BORDE_INICIO = 80;
const uint8_t MUESTRAS_BORDE_CONFIRMACION = 4;
const unsigned long RUTINA_INICIO_PULSO_AVANCE_MS = 10;
const unsigned long RUTINA_INICIO_PULSO_FRENO_MS = 14;
const int CONF_ENEMIGO_INC = 2;
const int CONF_ENEMIGO_DEC = 1;
const int CONF_ENEMIGO_MAX = 6;
const int CONF_ENEMIGO_ACTIVO = 3;
const float ANGULO_EMA_BETA = 0.34f;
const float VEL_ANGULAR_MAX = 0.10f;
const float PREDICCION_ANTICIPO_MS = 75.0f;
const float ANGULO_PRED_MAX = 4.5f;
const float KP_PID = 44.0f;
const float KI_PID = 3.2f;
const float KD_PID = 2.8f;
const float PID_DERIV_EMA_BETA = 0.42f;
const float PID_INTEGRAL_MAX = 2.4f;
const float PID_OUT_MAX = 140.0f;
const float PID_ZONA_MUERTA = 0.10f;
const float Q_ALPHA = 0.12f;
const float Q_GAMMA = 0.88f;
const int8_t RECOMPENSA_ENEMIGO = 8;
const int8_t PENALIZACION_SIN_ENEMIGO = -1;
const int8_t PENALIZACION_BORDE = -25;
const int8_t PENALIZACION_CAMBIO_ACCION = -2;
const unsigned long MIN_TIEMPO_ACCION_MS = 70;
const uint8_t EXPLORACION_PORCENTAJE = 8;
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
    unsigned long libreDesde = 0;
    while (millis() - inicio < duracionMs) {
        bool pisoIzq = false;
        bool pisoDer = false;
        bool enBorde = leerPiso(pisoIzq, pisoDer);

        motores.retroceder(VELOCIDAD_RETROCESO_BORDE);

        const unsigned long t = millis() - inicio;
        if (!enBorde && t >= RETROCESO_BORDE_MINIMO_MS) {
            if (libreDesde == 0) {
                libreDesde = millis();
            }
            if ((millis() - libreDesde) >= CONFIRMACION_LIBRE_BORDE_MS) {
                return;
            }
        } else {
            libreDesde = 0;
        }

        delay(5);
    }
}

void Robot::giroEscapeSeguro(bool haciaDerecha, unsigned long duracionMs) {
    unsigned long inicio = millis();

    // Giro completo sin interrupcion: completa la duracion especificada.
    while (millis() - inicio < duracionMs) {
        if (haciaDerecha) {
            moverDerecha();
        } else {
            moverIzquierda();
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

bool Robot::reingresoCombateSeguro(bool ultimoGiroDerecha, unsigned long avanceMs, unsigned long correccionMs) {
    // Sale del borde con un empuje fuerte y luego corrige rumbo para quedar orientado al combate.
    unsigned long inicioAvance = millis();
    while ((millis() - inicioAvance) < avanceMs) {
        bool pisoIzq = false;
        bool pisoDer = false;
        if (leerPiso(pisoIzq, pisoDer)) {
            return false;
        }

        if (enemigoVistoRapido()) {
            motores.adelante(VELOCIDAD_ATAQUE_CRUCERO);
            esperarConPrioridadPisoYEnemigo(65);
            return true;
        }

        motores.adelante(VELOCIDAD_REINGRESO_COMBATE);
        delay(5);
    }

    unsigned long inicioCorreccion = millis();
    while ((millis() - inicioCorreccion) < correccionMs) {
        bool pisoIzq = false;
        bool pisoDer = false;
        if (leerPiso(pisoIzq, pisoDer)) {
            return false;
        }

        if (enemigoVistoRapido()) {
            motores.adelante(VELOCIDAD_ATAQUE_CRUCERO);
            esperarConPrioridadPisoYEnemigo(55);
            return true;
        }

        if (ultimoGiroDerecha) {
            motores.curvaIzquierda(VELOCIDAD_MANIOBRA_ATAQUE);
        } else {
            motores.curvaDerecha(VELOCIDAD_MANIOBRA_ATAQUE);
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
        motores.curvaDerecha(VELOCIDAD_BUSQUEDA);
    } else {
        motores.curvaIzquierda(VELOCIDAD_BUSQUEDA);
    }
}

void Robot::rutinaInicialBordeMediaVuelta() {
    auto bordeCercaInicio = []() {
        const int pisoIzq = analogRead(SENSOR_DE_PISO_IZQUIERDO);
        const int pisoDer = analogRead(SENSOR_DE_PISO_DERECHO);
        return (pisoIzq <= (BLANCO + MARGEN_DETECCION_BORDE_CERCA)) ||
               (pisoDer <= (BLANCO + MARGEN_DETECCION_BORDE_CERCA));
    };

    auto bordeDetectadoInicio = []() {
        const int pisoIzq = analogRead(SENSOR_DE_PISO_IZQUIERDO);
        const int pisoDer = analogRead(SENSOR_DE_PISO_DERECHO);
        return (pisoIzq <= (BLANCO + MARGEN_DETECCION_BORDE_INICIO)) ||
               (pisoDer <= (BLANCO + MARGEN_DETECCION_BORDE_INICIO));
    };

    uint8_t muestrasConBorde = 0;
    bool zonaCercana = false;

    // Avanza obligatoriamente hasta confirmar borde; durante esta rutina no se procesa enemigo.
    while (true) {
        zonaCercana = zonaCercana || bordeCercaInicio();

        if (bordeDetectadoInicio()) {
            if (muestrasConBorde < 255) {
                muestrasConBorde++;
            }
            if (muestrasConBorde >= MUESTRAS_BORDE_CONFIRMACION) {
                break;
            }
        } else {
            muestrasConBorde = 0;
        }

        // Avance por micropulsos para reducir inercia y poder frenar antes de salir.
        if (zonaCercana) {
            motores.adelante(VELOCIDAD_RUTINA_INICIO_LENTA);
        } else {
            motores.adelante(VELOCIDAD_RUTINA_INICIO);
        }
        delay(RUTINA_INICIO_PULSO_AVANCE_MS);
        motores.detener();
        delay(RUTINA_INICIO_PULSO_FRENO_MS);
    }

    motores.detener();
    delay(RUTINA_INICIO_ASENTAR_MS);

    // Retrocede mientras siga viendo borde y despues agrega un margen extra al interior.
    while (bordeDetectadoInicio()) {
        motores.retroceder(VELOCIDAD_RUTINA_INICIO_RETROCESO);
        delay(5);
    }

    const unsigned long inicioRetrocesoExtra = millis();
    while ((millis() - inicioRetrocesoExtra) < RUTINA_INICIO_RETROCESO_MS) {
        motores.retroceder(VELOCIDAD_RUTINA_INICIO_RETROCESO);
        delay(5);
    }

    motores.detener();
    delay(RUTINA_INICIO_ASENTAR_MS);

    // Media vuelta para quedar orientado hacia el interior del dohyo.
    const unsigned long inicioGiro = millis();
    while ((millis() - inicioGiro) < RUTINA_INICIO_GIRO_MEDIA_VUELTA_MS) {
        moverDerecha();
        delay(5);
    }

    motores.detener();
}

void Robot::setup() {
    #if USAR_ARRANCADOR
    pinMode(Pin_Control_Remoto, INPUT);
    #endif
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

    #if USAR_ARRANCADOR
    // Estado inicial segun nivel del control remoto.
    int lecturaInicial = digitalRead(Pin_Control_Remoto);
    estado_control_anterior = lecturaInicial;
    bool controlActivo = REMOTE_ACTIVE_HIGH ? (lecturaInicial == HIGH) : (lecturaInicial == LOW);
    robot_encendido = controlActivo;
    #else
    // Sin arrancador remoto: el robot queda habilitado al energizarse.
    estado_control_anterior = REMOTE_ACTIVE_HIGH ? HIGH : LOW;
    robot_encendido = true;
    #endif
}

void Robot::detenerse() {
    motores.detener();
}

void Robot::ataqueEnemigo() {
    motores.adelante(VELOCIDAD_ATAQUE_FRENTE);
}

void Robot::moverAdelante() {
    motores.adelante(VELOCIDAD_AVANCE_NORMAL);
}

void Robot::retroceder() {
    motores.retroceder(VELOCIDAD_RETROCESO_NORMAL);
}

void Robot::moverDerecha() {
    motores.derecha(VELOCIDAD_MANIOBRA);
}

void Robot::moverIzquierda() {
    motores.izquierda(VELOCIDAD_MANIOBRA);
}

void Robot::sensoresPiso(bool pisoIzq, bool pisoDer) {
    static bool giroAlternadoDerecha = true;

    if (pisoIzq && pisoDer) {
        // Caso mas critico: ambos sensores en borde. Prioriza alejarse y revalidar.
        retrocesoSeguro(RETROCESO_BORDE_DOBLE_MS + RETROCESO_BORDE_DOBLE_EXTRA_MS);
        giroEscapeSeguro(giroAlternadoDerecha, GIRO_ESCAPE_BORDE_DOBLE_MS + GIRO_ESCAPE_BORDE_DOBLE_EXTRA_MS);

        bool pisoPostIzq = false;
        bool pisoPostDer = false;
        if (leerPiso(pisoPostIzq, pisoPostDer)) {
            retrocesoSeguro(RETROCESO_BORDE_SIMPLE_MS);
            giroEscapeSeguro(!giroAlternadoDerecha, GIRO_ESCAPE_BORDE_SIMPLE_MS);
        }

        // En doble borde prioriza reingreso al centro y reacquisicion temprana de enemigo.
        reingresoCombateSeguro(giroAlternadoDerecha, AVANCE_INTERIOR_DOBLE_MS + 40, CORRECCION_REINGRESO_DOBLE_MS);
        giroAlternadoDerecha = !giroAlternadoDerecha;
    } else if (pisoDer) {
        retrocesoSeguro(RETROCESO_BORDE_SIMPLE_MS);
        giroEscapeSeguro(false, GIRO_ESCAPE_BORDE_SIMPLE_MS);
        reingresoCombateSeguro(false, AVANCE_INTERIOR_SIMPLE_MS + 45, CORRECCION_REINGRESO_SIMPLE_MS);
    } else if (pisoIzq) {
        retrocesoSeguro(RETROCESO_BORDE_SIMPLE_MS);
        giroEscapeSeguro(true, GIRO_ESCAPE_BORDE_SIMPLE_MS);
        reingresoCombateSeguro(true, AVANCE_INTERIOR_SIMPLE_MS + 45, CORRECCION_REINGRESO_SIMPLE_MS);
    }
}

void Robot::sensoresFrontales(bool central, bool derecho, bool izquierdo) {
    if (central || (derecho && izquierdo)) {
        ataqueEnemigo();
    } else if (derecho) {
        motores.derecha(VELOCIDAD_MANIOBRA_ATAQUE);
        if (esperarConPrioridadPisoYEnemigo(55)) {
            return;
        }
        motores.adelante(VELOCIDAD_ATAQUE_FRENTE);
        esperarConPrioridadPisoYEnemigo(55);
    } else if (izquierdo) {
        motores.izquierda(VELOCIDAD_MANIOBRA_ATAQUE);
        if (esperarConPrioridadPisoYEnemigo(55)) {
            return;
        }
        motores.adelante(VELOCIDAD_ATAQUE_FRENTE);
        esperarConPrioridadPisoYEnemigo(55);
    }
}

void Robot::sensoresLaterales(bool sensorIzquierdo, bool sensorDerecho) {
    if (sensorIzquierdo && sensorDerecho) return;

    bool pisoIzq = false, pisoDer = false;
    leerPiso(pisoIzq, pisoDer);
    if (pisoIzq || pisoDer) return;

    // Pivote con una sola llanta para orientar el frontal hacia el enemigo lateral.
    // Luego aplica un empuje corto para reenganchar con sensores frontales.
    if (sensorIzquierdo) {
        motores.curvaIzquierda(VELOCIDAD_LATERAL);  // Solo rueda derecha gira
        if (esperarConPrioridadPiso(PIVOTE_LATERAL_MS)) return;
        motores.adelante(VELOCIDAD_LATERAL);
        esperarConPrioridadPisoYEnemigo(EMPUJE_TRAS_PIVOTE_MS);
    } else if (sensorDerecho) {
        motores.curvaDerecha(VELOCIDAD_LATERAL);    // Solo rueda izquierda gira
        if (esperarConPrioridadPiso(PIVOTE_LATERAL_MS)) return;
        motores.adelante(VELOCIDAD_LATERAL);
        esperarConPrioridadPisoYEnemigo(EMPUJE_TRAS_PIVOTE_MS);
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
    static unsigned long ultimoAvistamientoMs = 0;
    static int8_t ultimoLadoEnemigo = 0; // -1 izquierda, +1 derecha, 0 centro/indefinido
    // PID angular: error = posicion angular del enemigo [-4,+4], salida = diferencial de motores
    static float pidIntegral  = 0.0f;
    static float pidPrevError = 0.0f;
    static float pidSalida    = 0.0f;
    static float pidDerivFiltrada = 0.0f;
    static unsigned long pidTmsAnt = 0;
    static bool ataquePulsoOn = true;
    static unsigned long ataquePulsoRefMs = 0;
    // Prediccion lineal de primer orden: estima angulo del enemigo en ~80ms
    static float anguloFiltrado  = 0.0f;
    static float anguloAnterior   = 0.0f;
    static float velAngular       = 0.0f;
    static unsigned long tAngPrev = 0;
    static uint8_t accionAplicada = 5;
    static unsigned long tAccionAplicadaMs = 0;
    static uint32_t lcgEstado = 0xC0FFEE11UL;
    static unsigned long bloqueoPostBordeHastaMs = 0;
    static int8_t ultimoLadoBorde = 0; // -1 borde izquierdo, +1 borde derecho

    #if USAR_ARRANCADOR
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
    #else
    robot_encendido = true;
    #endif

    const bool recienEncendido = (!estadoAnteriorEncendido && robot_encendido);

    if (!robot_encendido) {
        // Reset inmediato de busqueda al apagar, para que arranque limpio.
        busquedaDerecha = true;
        faseBusqueda = 0;
        inicioFaseBusqueda = 0;
        busquedaActiva = false;
        bloqueoPostBordeHastaMs = 0;
        ultimoLadoBorde = 0;

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
        pidDerivFiltrada = 0.0f;
        pidTmsAnt    = 0;
        ataquePulsoOn = true;
        ataquePulsoRefMs = 0;
        anguloFiltrado = 0.0f;
        anguloAnterior = 0.0f;
        velAngular     = 0.0f;
        tAngPrev       = 0;
        accionAplicada = 5;
        tAccionAplicadaMs = 0;
        lcgEstado = 0xC0FFEE11UL;
        estadoPrev = 0;
        accionPrev = 5;
        ultimoAvistamientoMs = 0;
        ultimoLadoEnemigo = 0;
        bloqueoPostBordeHastaMs = 0;
        ultimoLadoBorde = 0;
        memcpy(qtable, QTABLE_DEFAULTS, sizeof(qtable));
        busquedaActiva = false;

        // Rutina obligatoria de inicio: ir al borde y hacer media vuelta.
        rutinaInicialBordeMediaVuelta();
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
    const bool riesgoBordeIzq = (kfPisoIzqX <= (float)(BLANCO + MARGEN_PREVENTIVO_BORDE));
    const bool riesgoBordeDer = (kfPisoDerX <= (float)(BLANCO + MARGEN_PREVENTIVO_BORDE));

    // --- Filtro de confianza para sensores de enemigo (digitales) ---
    // Filtro de confianza con histéresis más estable para reducir falsos picos.
    confFrontalDer     = (int8_t)constrain(confFrontalDer     + (sensorFrontalDer.detectar() ? CONF_ENEMIGO_INC : -CONF_ENEMIGO_DEC), 0, CONF_ENEMIGO_MAX);
    confFrontalIzq     = (int8_t)constrain(confFrontalIzq     + (sensorFrontalIzq.detectar() ? CONF_ENEMIGO_INC : -CONF_ENEMIGO_DEC), 0, CONF_ENEMIGO_MAX);
    confFrontalCentral = (int8_t)constrain(confFrontalCentral + (sensorFrontal.detectar()    ? CONF_ENEMIGO_INC : -CONF_ENEMIGO_DEC), 0, CONF_ENEMIGO_MAX);
    confLateralDer     = (int8_t)constrain(confLateralDer     + (sensorLateralDer.detectar() ? CONF_ENEMIGO_INC : -CONF_ENEMIGO_DEC), 0, CONF_ENEMIGO_MAX);
    confLateralIzq     = (int8_t)constrain(confLateralIzq     + (sensorLateralIzq.detectar() ? CONF_ENEMIGO_INC : -CONF_ENEMIGO_DEC), 0, CONF_ENEMIGO_MAX);

    bool FrontalDer     = (confFrontalDer     >= CONF_ENEMIGO_ACTIVO);
    bool FrontalIzq     = (confFrontalIzq     >= CONF_ENEMIGO_ACTIVO);
    bool FrontalCentral = (confFrontalCentral >= CONF_ENEMIGO_ACTIVO);
    bool LateralDer     = (confLateralDer     >= CONF_ENEMIGO_ACTIVO);
    bool LateralIzq     = (confLateralIzq     >= CONF_ENEMIGO_ACTIVO);

    const bool enemigoDetectado = FrontalDer || FrontalIzq || FrontalCentral || LateralDer || LateralIzq;

    // --- Posicion angular del enemigo estimada desde valores de confianza [-4, +4] ---
    float totalConf = (float)(confFrontalDer + confFrontalIzq + confFrontalCentral +
                               confLateralDer + confLateralIzq);
    float anguloInst = 0.0f;
    if (totalConf > 0.0f) {
        anguloInst = ((float)confFrontalDer  *  2.0f - (float)confFrontalIzq  *  2.0f +
                      (float)confLateralDer  *  4.0f - (float)confLateralIzq  *  4.0f)
                     / totalConf;
    }
    anguloFiltrado += ANGULO_EMA_BETA * (anguloInst - anguloFiltrado);

    // --- Prediccion lineal a 80ms: adelanta la reaccion ante enemigos rapidos ---
    {
        unsigned long dtAngMs = ahora - tAngPrev;
        if (dtAngMs > 0 && dtAngMs < 100) {
            velAngular = (anguloFiltrado - anguloAnterior) / (float)dtAngMs;
            if (velAngular >  VEL_ANGULAR_MAX) velAngular =  VEL_ANGULAR_MAX;
            if (velAngular < -VEL_ANGULAR_MAX) velAngular = -VEL_ANGULAR_MAX;
        }
        anguloAnterior = anguloFiltrado;
        tAngPrev = ahora;
    }
    float anguloPred = anguloFiltrado + velAngular * PREDICCION_ANTICIPO_MS;
    if (anguloPred >  ANGULO_PRED_MAX) anguloPred =  ANGULO_PRED_MAX;
    if (anguloPred < -ANGULO_PRED_MAX) anguloPred = -ANGULO_PRED_MAX;

    if (enemigoDetectado) {
        ultimoAvistamientoMs = ahora;
        if (LateralIzq && !LateralDer) {
            ultimoLadoEnemigo = -1;
        } else if (LateralDer && !LateralIzq) {
            ultimoLadoEnemigo = 1;
        } else if (FrontalIzq && !FrontalDer) {
            ultimoLadoEnemigo = -1;
        } else if (FrontalDer && !FrontalIzq) {
            ultimoLadoEnemigo = 1;
        } else {
            ultimoLadoEnemigo = 0;
        }
    }

    // --- PID angular robusto: zona muerta, derivada filtrada y anti-windup ---
    {
        float dtPidMs = (float)(ahora - pidTmsAnt);
        if (dtPidMs > 0.0f && dtPidMs < 100.0f) {
            float dtS = dtPidMs * 0.001f;
            float errorPid = anguloPred;
            if (errorPid < PID_ZONA_MUERTA && errorPid > -PID_ZONA_MUERTA) {
                errorPid = 0.0f;
            }

            if (enemigoDetectado) {
                pidIntegral += errorPid * dtS;
            } else {
                pidIntegral *= 0.85f;
            }
            if (pidIntegral >  PID_INTEGRAL_MAX) pidIntegral =  PID_INTEGRAL_MAX;
            if (pidIntegral < -PID_INTEGRAL_MAX) pidIntegral = -PID_INTEGRAL_MAX;

            float derivRaw = (errorPid - pidPrevError) / dtS;
            pidDerivFiltrada += PID_DERIV_EMA_BETA * (derivRaw - pidDerivFiltrada);
            pidSalida = KP_PID * errorPid + KI_PID * pidIntegral + KD_PID * pidDerivFiltrada;
            if (pidSalida >  PID_OUT_MAX) pidSalida =  PID_OUT_MAX;
            if (pidSalida < -PID_OUT_MAX) pidSalida = -PID_OUT_MAX;
            pidPrevError = errorPid;
        }
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
        int8_t recompensa = (PisoIzq || PisoDer) ? PENALIZACION_BORDE : (estadoQ != 0 ? RECOMPENSA_ENEMIGO : PENALIZACION_SIN_ENEMIGO);
        if (accionPrev != accionAplicada) {
            recompensa += PENALIZACION_CAMBIO_ACCION;
        }
        int8_t maxQNuevo  = qtable[estadoQ][0];
        for (uint8_t a = 1; a < 6; a++) {
            if (qtable[estadoQ][a] > maxQNuevo) maxQNuevo = qtable[estadoQ][a];
        }
        float qUpd = (float)qtable[estadoPrev][accionPrev]
                   + Q_ALPHA * ((float)recompensa + Q_GAMMA * (float)maxQNuevo
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

    // Sin enemigo detectado, usa memoria corta para no perder seguimiento.
    if (estadoQ == 0) {
        if ((ahora - ultimoAvistamientoMs) <= MEMORIA_OBJETIVO_MS) {
            if (ultimoLadoEnemigo > 0) {
                accionQ = 3;
            } else if (ultimoLadoEnemigo < 0) {
                accionQ = 4;
            } else {
                accionQ = 0;
            }
        } else {
            accionQ = 5;
        }
    }

    // Exploracion pequeña y controlada para evitar estancamiento del aprendizaje.
    if (estadoQ != 0) {
        lcgEstado = 1664525UL * lcgEstado + 1013904223UL;
        uint8_t r100 = (uint8_t)((lcgEstado >> 24) % 100U);
        if (r100 < EXPLORACION_PORCENTAJE) {
            accionQ = (uint8_t)((lcgEstado >> 16) % 5U);
        }
    }

    // Evita cambios de accion nerviosos: mantiene una accion minima mientras hay enemigo.
    if (tAccionAplicadaMs == 0) {
        accionAplicada = accionQ;
        tAccionAplicadaMs = ahora;
    }
    if (estadoQ != 0 && accionQ != accionAplicada && (ahora - tAccionAplicadaMs) < MIN_TIEMPO_ACCION_MS) {
        accionQ = accionAplicada;
    }
    if (accionQ != accionAplicada) {
        accionAplicada = accionQ;
        tAccionAplicadaMs = ahora;
    }

    estadoPrev = estadoQ;
    accionPrev = accionQ;

    // --- Ejecucion: piso tiene prioridad absoluta (seguridad), Q-tabla gestiona el resto ---
    if (PisoIzq || PisoDer) {
        if (PisoIzq && !PisoDer) {
            ultimoLadoBorde = -1;
        } else if (PisoDer && !PisoIzq) {
            ultimoLadoBorde = 1;
        } else if (kfPisoIzqX < kfPisoDerX) {
            ultimoLadoBorde = -1;
        } else {
            ultimoLadoBorde = 1;
        }
        bloqueoPostBordeHastaMs = ahora + BLOQUEO_POST_BORDE_MS;
        faseBusqueda = 0;
        inicioFaseBusqueda = 0;
        busquedaActiva = false;
        pidIntegral = 0.0f;
        pidDerivFiltrada = 0.0f;
        ataquePulsoOn = true;
        ataquePulsoRefMs = 0;
        accionAplicada = 5;
        tAccionAplicadaMs = 0;
        sensoresPiso(PisoIzq, PisoDer);
    } else if (riesgoBordeIzq || riesgoBordeDer) {
        // Evasion preventiva con giro completo: retrocede y luego da vuelta de 180 grados.
        if (riesgoBordeIzq && !riesgoBordeDer) {
            ultimoLadoBorde = -1;
        } else if (riesgoBordeDer && !riesgoBordeIzq) {
            ultimoLadoBorde = 1;
        } else if (kfPisoIzqX < kfPisoDerX) {
            ultimoLadoBorde = -1;
        } else {
            ultimoLadoBorde = 1;
        }
        bloqueoPostBordeHastaMs = ahora + BLOQUEO_POST_BORDE_MS;
        faseBusqueda = 0;
        inicioFaseBusqueda = 0;
        busquedaActiva = false;
        pidIntegral = 0.0f;
        pidDerivFiltrada = 0.0f;
        ataquePulsoOn = true;
        ataquePulsoRefMs = 0;
        accionAplicada = 5;
        tAccionAplicadaMs = 0;
        retrocesoSeguro(RETROCESO_PREVENTIVO_BORDE_MS);
        bool giroDerecha = (ultimoLadoBorde > 0);
        giroEscapeSeguro(giroDerecha, GIRO_ESCAPE_BORDE_SIMPLE_MS);
        return;
    } else if (ahora < bloqueoPostBordeHastaMs) {
        // Ventana post-borde: primero orienta al interior y luego empuja al centro para no quedar orbitando.
        ataquePulsoOn = true;
        ataquePulsoRefMs = 0;
        const unsigned long restanteBloqueoMs = bloqueoPostBordeHastaMs - ahora;

        if (FrontalCentral && !riesgoBordeIzq && !riesgoBordeDer) {
            bloqueoPostBordeHastaMs = 0;
        }

        if (restanteBloqueoMs <= 120) {
            motores.adelante(VELOCIDAD_REINGRESO_COMBATE);
            return;
        }

        if (ultimoLadoBorde < 0) {
            motores.curvaDerecha(VELOCIDAD_MANIOBRA_ATAQUE);
        } else if (ultimoLadoBorde > 0) {
            motores.curvaIzquierda(VELOCIDAD_MANIOBRA_ATAQUE);
        } else {
            motores.adelante(VELOCIDAD_REINGRESO_COMBATE);
        }
        return;
    } else {
        const bool lateralSoloIzq = LateralIzq && !LateralDer && !FrontalCentral && !FrontalIzq && !FrontalDer;
        const bool lateralSoloDer = LateralDer && !LateralIzq && !FrontalCentral && !FrontalIzq && !FrontalDer;
        if (lateralSoloIzq || lateralSoloDer) {
            faseBusqueda = 0;
            inicioFaseBusqueda = 0;
            busquedaActiva = false;
            sensoresLaterales(lateralSoloIzq, lateralSoloDer);
            return;
        }

        auto aplicarMovimientoPidPulso = [&](int baseIzq, int baseDer, bool habilitarPulso, int bonoPulso, float escalaPid) {
            if (ataquePulsoRefMs == 0) {
                ataquePulsoRefMs = ahora;
            }

            const unsigned long ventanaPulsoMs = ataquePulsoOn ? MOV_PULSO_ON_MS : MOV_PULSO_OFF_MS;
            if ((ahora - ataquePulsoRefMs) >= ventanaPulsoMs) {
                ataquePulsoOn = !ataquePulsoOn;
                ataquePulsoRefMs = ahora;
            }

            int velBaseIzq = baseIzq;
            int velBaseDer = baseDer;
            if (habilitarPulso && ataquePulsoOn) {
                if (velBaseIzq > 0) velBaseIzq += bonoPulso;
                if (velBaseIzq < 0) velBaseIzq -= bonoPulso;
                if (velBaseDer > 0) velBaseDer += bonoPulso;
                if (velBaseDer < 0) velBaseDer -= bonoPulso;
            }

            float out = pidSalida * escalaPid;
            if (out >  PID_OUT_MAX) out =  PID_OUT_MAX;
            if (out < -PID_OUT_MAX) out = -PID_OUT_MAX;

            int cmdIzq = velBaseIzq + (int)out;
            int cmdDer = velBaseDer - (int)out;

            if (cmdIzq > VELOCIDAD_ATAQUE_FRENTE) cmdIzq = VELOCIDAD_ATAQUE_FRENTE;
            if (cmdIzq < -VELOCIDAD_ATAQUE_FRENTE) cmdIzq = -VELOCIDAD_ATAQUE_FRENTE;
            if (cmdDer > VELOCIDAD_ATAQUE_FRENTE) cmdDer = VELOCIDAD_ATAQUE_FRENTE;
            if (cmdDer < -VELOCIDAD_ATAQUE_FRENTE) cmdDer = -VELOCIDAD_ATAQUE_FRENTE;

            if (cmdIzq > 0 && cmdIzq < VELOCIDAD_MIN_TRACCION) cmdIzq = VELOCIDAD_MIN_TRACCION;
            if (cmdIzq < 0 && cmdIzq > -VELOCIDAD_MIN_TRACCION) cmdIzq = -VELOCIDAD_MIN_TRACCION;
            if (cmdDer > 0 && cmdDer < VELOCIDAD_MIN_TRACCION) cmdDer = VELOCIDAD_MIN_TRACCION;
            if (cmdDer < 0 && cmdDer > -VELOCIDAD_MIN_TRACCION) cmdDer = -VELOCIDAD_MIN_TRACCION;

            if (cmdIzq > 0)      motores.getIzquierdo().avanzar(cmdIzq);
            else if (cmdIzq < 0) motores.getIzquierdo().retroceder(-cmdIzq);
            else                 motores.getIzquierdo().detener();

            if (cmdDer > 0)      motores.getDerecho().avanzar(cmdDer);
            else if (cmdDer < 0) motores.getDerecho().retroceder(-cmdDer);
            else                 motores.getDerecho().detener();
        };

        switch (accionQ) {
            case 0: {
                // Ataque frontal con control unificado de traccion (PID + pulsos).
                float absAnguloPred = anguloPred;
                if (absAnguloPred < 0.0f) absAnguloPred = -absAnguloPred;
                const bool frenteFuerte = FrontalCentral && (FrontalDer || FrontalIzq || (confFrontalCentral >= (CONF_ENEMIGO_MAX - 1)));
                const bool habilitarPulso = enemigoDetectado && (absAnguloPred <= ANGULO_MAX_PULSO_ATAQUE);
                int baseAtaque = VELOCIDAD_ATAQUE_CRUCERO;
                int bonoPulso = BONO_PULSO_ATAQUE;
                if (frenteFuerte) {
                    bonoPulso += BONO_PULSO_ATAQUE_CENTRO;
                }
                aplicarMovimientoPidPulso(baseAtaque, baseAtaque, habilitarPulso, bonoPulso, 1.0f);
                faseBusqueda = 0;
                busquedaActiva = false;
                break;
            }
            case 1:
                aplicarMovimientoPidPulso(VELOCIDAD_MANIOBRA_ATAQUE, -VELOCIDAD_MANIOBRA_ATAQUE, true, BONO_PULSO_TRACCION, 0.55f);
                faseBusqueda = 0;
                busquedaActiva = false;
                break;
            case 2:
                aplicarMovimientoPidPulso(-VELOCIDAD_MANIOBRA_ATAQUE, VELOCIDAD_MANIOBRA_ATAQUE, true, BONO_PULSO_TRACCION, 0.55f);
                faseBusqueda = 0;
                busquedaActiva = false;
                break;
            case 3:
                aplicarMovimientoPidPulso(VELOCIDAD_MANIOBRA_ATAQUE, 95, true, BONO_PULSO_TRACCION, 0.70f);
                faseBusqueda = 0;
                busquedaActiva = false;
                break;
            case 4:
                aplicarMovimientoPidPulso(95, VELOCIDAD_MANIOBRA_ATAQUE, true, BONO_PULSO_TRACCION, 0.70f);
                faseBusqueda = 0;
                busquedaActiva = false;
                break;
            default: {
                // BUSCAR: avance agresivo + barrido en arco amplio para cubrir mas area con menos giros.
                const int margenSeguridadPiso = 48;
                const bool cercaBordeIzq = kfPisoIzqX <= (float)(BLANCO + margenSeguridadPiso);
                const bool cercaBordeDer = kfPisoDerX <= (float)(BLANCO + margenSeguridadPiso);

                if (!busquedaActiva) {
                    busquedaActiva = true;
                    faseBusqueda = 0;
                    inicioFaseBusqueda = ahora;
                }

                const unsigned long duraciones[3] = {
                    360, 300, 220
                };

                if ((ahora - inicioFaseBusqueda) >= duraciones[faseBusqueda]) {
                    inicioFaseBusqueda = ahora;
                    faseBusqueda = (faseBusqueda + 1) % 3;
                    if (faseBusqueda == 0) busquedaDerecha = !busquedaDerecha;
                }

                if (cercaBordeIzq || cercaBordeDer) {
                    // Retroceso corto para quitar inercia y luego curva de correccion.
                    retrocesoSeguro(RETROCESO_PREVENTIVO_BORDE_MS);
                    if      (cercaBordeIzq && !cercaBordeDer) motores.curvaDerecha(VELOCIDAD_MANIOBRA_ATAQUE);
                    else if (cercaBordeDer && !cercaBordeIzq) motores.curvaIzquierda(VELOCIDAD_MANIOBRA_ATAQUE);
                    else if (busquedaDerecha)                 motores.curvaDerecha(VELOCIDAD_MANIOBRA_ATAQUE);
                    else                                      motores.curvaIzquierda(VELOCIDAD_MANIOBRA_ATAQUE);
                    faseBusqueda = 0;
                    inicioFaseBusqueda = ahora;
                    return;
                }

                const int velExterior = VELOCIDAD_BUSQUEDA_AGRESIVA;
                const int velInterior = VELOCIDAD_BUSQUEDA_AGRESIVA - DIFERENCIAL_ARCO_BUSQUEDA;

                if (faseBusqueda == 0 || faseBusqueda == 2) {
                    aplicarMovimientoPidPulso(velExterior, velExterior, true, BONO_PULSO_TRACCION, 0.45f);
                } else {
                    // Arco con ambas ruedas: barre mas area y evita pivotes bruscos.
                    if (busquedaDerecha) {
                        aplicarMovimientoPidPulso(velExterior, velInterior, true, BONO_PULSO_TRACCION, 0.50f);
                    } else {
                        aplicarMovimientoPidPulso(velInterior, velExterior, true, BONO_PULSO_TRACCION, 0.50f);
                    }
                }
                break;
            }
        }
    }
}
