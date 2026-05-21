



    // Variables estáticas para la rutina de búsqueda
    static bool busquedaDerecha = true;
    static unsigned char faseBusqueda = 0;
    static unsigned long inicioFase = 0;
#include <Arduino.h>
#include <avr/wdt.h>
#include "Robot.h"
#include "Definiciones.h"

// ------------------ Control remoto (nivel) ------------------
bool robot_encendido          = false;

Robot::Robot() :
    motores(),
    sensorPisoIzq(SENSOR_DE_PISO_IZQUIERDO, BLANCO),
    sensorPisoDer(SENSOR_DE_PISO_DERECHO, BLANCO),
    sensorFrontal(SENSOR_FRONTAL_CENTRA),
    sensorFrontalIzq(SENSOR_FRONTAL_IZQUIERDO),
    sensorFrontalDer(SENSOR_FRONTAL_DERECHO),
    sensorLateralIzq(SENSOR_LATERAL_IZQUIERDO),
    sensorLateralDer(SENSOR_LATERAL_DERECHO)
{
}
// Variable global para el estado del borde
bool bordeDetectado = false;

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

void Robot::retrocesoSeguro(unsigned long duracionMs) {
    unsigned long inicio = millis();
    unsigned long pistaEstableDesde = 0;
    const unsigned long retrocesoMinimoMs = 90;
    while (millis() - inicio < duracionMs) {
        retroceder();

        bool pisoIzq = false;
        bool pisoDer = false;
        bool enBorde = leerPiso(pisoIzq, pisoDer);

        if (!enBorde) {
            if (pistaEstableDesde == 0) {
                pistaEstableDesde = millis();
            }
            if ((millis() - inicio) > retrocesoMinimoMs && (millis() - pistaEstableDesde) > 25) {
                return;
            }
        } else {
            pistaEstableDesde = 0;
        }
    }
}

void Robot::giroEscapeSeguro(bool haciaDerecha, unsigned long duracionMs) {
    unsigned long inicio = millis();
    unsigned long pistaEstableDesde = 0;
    while (millis() - inicio < duracionMs) {
        bool pisoIzq = false;
        bool pisoDer = false;
        bool enBorde = leerPiso(pisoIzq, pisoDer);

        if (haciaDerecha) {
            moverDerecha();
        } else {
            moverIzquierda();
        }

        // Requiere una pequeña ventana estable fuera del borde para terminar el giro.
        if (!enBorde) {
            if (pistaEstableDesde == 0) {
                pistaEstableDesde = millis();
            }
            if ((millis() - inicio) > 60 && (millis() - pistaEstableDesde) > 30) {
                return;
            }
        } else {
            pistaEstableDesde = 0;
        }
    }
}

void Robot::giroEscapeCompleto(bool haciaDerecha, unsigned long duracionMs) {
    unsigned long inicio = millis();
    unsigned long pistaEstableDesde = 0;
    const unsigned long giroMinimoMs = 130;
    while (millis() - inicio < duracionMs) {
        bool pisoIzq = false;
        bool pisoDer = false;
        bool enBorde = leerPiso(pisoIzq, pisoDer);

        if (haciaDerecha) {
            moverDerecha();
        } else {
            moverIzquierda();
        }

        if (!enBorde) {
            if (pistaEstableDesde == 0) {
                pistaEstableDesde = millis();
            }
            if ((millis() - inicio) > giroMinimoMs && (millis() - pistaEstableDesde) > 45) {
                return;
            }
        } else {
            pistaEstableDesde = 0;
        }
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
    pinMode(Pin_Control_Remoto, INPUT);
}

void Robot::detenerse() {
    motores.detener();
}

void Robot::ataqueEnemigo() {
    motores.adelante(Velocidad_maxima_Ataque);
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
        retrocesoSeguro(260);
        giroEscapeSeguro(giroAlternadoDerecha, 190);
        // Solo avanza si ya no detecta borde
        bool pisoIzq2 = false, pisoDer2 = false;
        if (!leerPiso(pisoIzq2, pisoDer2)) {
            motores.adelante(Velocidad_movimiento_seguir);
            delay(180);
        }
        giroAlternadoDerecha = !giroAlternadoDerecha;
    } else if (pisoDer) {
        retrocesoSeguro(300);
        giroEscapeCompleto(false, 330);
        bool pisoIzq2 = false, pisoDer2 = false;
        if (!leerPiso(pisoIzq2, pisoDer2)) {
            motores.adelante(Velocidad_movimiento_seguir);
            delay(180);
        }
    } else if (pisoIzq) {
        retrocesoSeguro(300);
        giroEscapeCompleto(true, 330);
        bool pisoIzq2 = false, pisoDer2 = false;
        if (!leerPiso(pisoIzq2, pisoDer2)) {
            motores.adelante(Velocidad_movimiento_seguir);
            delay(180);
        }
    }
}

void Robot::sensoresFrontales(bool central, bool derecho, bool izquierdo) {
    // Aquí solo llega si es frontal asimétrico (no central), así que ataca hacia ese lado
      
    if (derecho && !izquierdo) {
        moverDerecha();
        if (esperarConPrioridadPiso(70)) return;
        motores.adelante(Velocidad_maxima);
        esperarConPrioridadPiso(70);
    } else if (izquierdo && !derecho) {
        moverIzquierda();
        if (esperarConPrioridadPiso(70)) return;
        motores.adelante(Velocidad_maxima);
        esperarConPrioridadPiso(70);
    }
}

void Robot::sensoresLaterales(bool sensorIzquierdo, bool sensorDerecho) {
    if (sensorIzquierdo && sensorDerecho) return;

    bool pisoIzq = false, pisoDer = false;
    leerPiso(pisoIzq, pisoDer);
    if (pisoIzq || pisoDer) return;

    // Posicionamiento rápido: solo una llanta gira y la otra queda detenida.
    // Verifica piso cada 5ms para abortar si hay borde.
    if (sensorIzquierdo) {
        motores.getIzquierdo().detener();
        motores.getDerecho().avanzar(Velocidad_maxima_Ataque);
        esperarConPrioridadPiso(120);
    } else if (sensorDerecho) {
        motores.getDerecho().detener();
        motores.getIzquierdo().avanzar(Velocidad_maxima_Ataque);
        esperarConPrioridadPiso(120);
    }
}

void Robot::loop() {
    // --- Robot siempre encendido, sin control remoto ---
    static bool estadoAnterior = false;
    const bool recienEncendido = !estadoAnterior;
    estadoAnterior = true;

    // LECTURA DE SENSORES
    bool PisoIzq = false, PisoDer = false;
    int valorPisoIzq = analogRead(SENSOR_DE_PISO_IZQUIERDO);
    int valorPisoDer = analogRead(SENSOR_DE_PISO_DERECHO);
    PisoIzq = (valorPisoIzq <= BLANCO);
    PisoDer = (valorPisoDer <= BLANCO);
    
    bool FrontalDer = sensorFrontalDer.detectar();
    bool FrontalIzq = sensorFrontalIzq.detectar();
    bool FrontalCentral = sensorFrontal.detectar();
    bool LateralDer = sensorLateralDer.detectar();
    bool LateralIzq = sensorLateralIzq.detectar();
    const bool enemigoDetectado = FrontalDer || FrontalIzq || FrontalCentral || LateralDer || LateralIzq;


    // --- NUEVA LÓGICA: avanzar hasta detectar borde antes de rutina normal ---

    // ...existing code...

    if (!bordeDetectado) {
        // Avanza hacia el borde a velocidad reducida
        motores.adelante(Velocidad_borde);
        if (PisoIzq || PisoDer) {
            // Al detectar el borde, retrocede para no salirse
            bordeDetectado = true;
            retrocesoSeguro(350); // Aumenta el tiempo si es necesario
            sensoresPiso(PisoIzq, PisoDer);
        }
        return;
    }

    // SISTEMA DE PRIORIDADES
    // Prioridad 0 (MÁXIMA): Escape del borde
    if (PisoIzq || PisoDer) {
        sensoresPiso(PisoIzq, PisoDer);
        return;
    }

    // Prioridad 1 (ALTA): Enemigo frontal
    if (FrontalCentral || (FrontalDer && FrontalIzq)) {
        ataqueEnemigo();
        return;
    }

    // Prioridad 1B: Frontal asimétrico
    if (FrontalDer || FrontalIzq) {
        sensoresFrontales(false, FrontalDer, FrontalIzq);
        return;
    }

    // Prioridad 2 (MEDIA): Enemigo lateral (solo después de detectar el borde)
    if (LateralDer || LateralIzq) {
        sensoresLaterales(LateralIzq, LateralDer);
        return;
    }

    static unsigned long ultimoContacto = 0;
    if (enemigoDetectado) {
        ultimoContacto = millis();
    }

    // Prioridad 3 (BAJA): Búsqueda sin enemigo
    // Patrón: barrido de arco ~180° + paso corto, cubre trasero, laterales y frente.
    //   Fase 0: giro ~180° hacia busquedaDerecha       (~350 ms)
    //   Fase 1: avance corto hacia el interior          ( ~80 ms)
    //   Fase 2: giro ~180° en dirección contraria       (~350 ms)
    //   Fase 3: avance corto hacia el interior          ( ~80 ms)
    // Al completar el ciclo alterna la dirección inicial para no repetir el mismo patrón.
    static bool busquedaDerecha = true;
    static uint8_t faseBusqueda = 0;
    static unsigned long inicioFase = 0;

    if (recienEncendido) {
        faseBusqueda = 0;
        inicioFase = 0;
    }

    const unsigned long ahora = millis();
    const unsigned long tiempoSinContacto = ahora - ultimoContacto;
    const int margenSeguridadPiso = 35;
    const bool cercaBordeIzq = valorPisoIzq <= (BLANCO + margenSeguridadPiso);
    const bool cercaBordeDer = valorPisoDer <= (BLANCO + margenSeguridadPiso);

    // Si acaba de perder contacto, gira de inmediato para re-encontrar al enemigo.
    // No avanza: si el enemigo está detrás, avanzar lo alejaría.
    if (tiempoSinContacto < 130) {
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
        } else {
            // Gira en lugar de avanzar: cubre el ángulo trasero cuanto antes
            if (busquedaDerecha) moverDerecha(); else moverIzquierda();
        }
        faseBusqueda = 0;
        inicioFase = ahora;
        return;
    }

    if (inicioFase == 0) {
        inicioFase = ahora;
    }

    // Duraciones de cada fase del barrido 360° (2 arcos de ~180° con avance entre medias)
    const unsigned long duraciones[4] = {350, 80, 350, 80};

    if ((ahora - inicioFase) >= duraciones[faseBusqueda]) {
        inicioFase = ahora;
        faseBusqueda = (faseBusqueda + 1) % 4;
        // Al completar las 4 fases (vuelta completa), invierte dirección inicial
        if (faseBusqueda == 0) {
            busquedaDerecha = !busquedaDerecha;
        }
    }

    // Si hay borde cerca durante la búsqueda, re-centrar antes de seguir girando.
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
        inicioFase = ahora;
        return;
    }

    // Fase 0: primer arco ~180°
    if (faseBusqueda == 0) {
        if (busquedaDerecha) moverDerecha(); else moverIzquierda();
    }
    // Fase 1: avance hacia el centro
    else if (faseBusqueda == 1) {
        motores.adelante(Velocidad_maxima);
    }
    // Fase 2: segundo arco ~180° en sentido contrario (completa los 360°)
    else if (faseBusqueda == 2) {
        if (busquedaDerecha) moverIzquierda(); else moverDerecha();
    }
    // Fase 3: avance hacia el centro
    else {
        motores.adelante(Velocidad_maxima);
    }
}



// ...existing code...

// const uint16_t QTABLE_EEPROM_MAGIC = 0x534C; // "SL"
// const uint8_t QTABLE_EEPROM_VERSION = 2;
// const int QTABLE_STATES = 32;
// const int QTABLE_ACTIONS = 6;
// const uint8_t QTABLE_PROFILE_COUNT = 3;
// const unsigned long QTABLE_GUARDADO_INTERVALO_MS = 15000;

// enum QTableProfile : uint8_t {
//     PROFILE_GENERAL = 0,
//     PROFILE_FRONTAL = 1,
//     PROFILE_LATERAL = 2
// };

// const unsigned long PERFIL_MUESTREO_MS = 1200;
// const int PERFIL_MARGEN_DECISION = 6;

// struct PerfilControl {
//     float kp;
//     float ki;
//     float kd;
//     uint8_t epsilonInicial;
//     unsigned long duracionesBusqueda[6];
// };

// const PerfilControl PERFIL_CONTROL[QTABLE_PROFILE_COUNT] = {
//     // GENERAL
//     {50.0f, 2.0f, 4.0f, 10, {780, 500, 780, 500, 520, 300}},
//     // FRONTAL
//     {56.0f, 2.5f, 4.5f, 8,  {700, 420, 700, 420, 470, 260}},
//     // LATERAL
//     {44.0f, 1.6f, 3.6f, 14, {860, 560, 860, 560, 590, 340}}
// };

// struct QTableHeader {
//     uint16_t magic;
//     uint8_t version;
//     uint8_t profileCount;
// };

// int qtableEepromHeaderAddress() {
//     return 0;
// }

// int qtableEepromDataAddress() {
//     return (int)sizeof(QTableHeader);
// }

// int qtableEepromProfileAddress(uint8_t profile) {
//     return qtableEepromDataAddress() + (int)profile * (int)sizeof(int8_t[QTABLE_STATES][QTABLE_ACTIONS]);
// }

// bool qtableHeaderValido(const QTableHeader &h) {
//     return h.magic == QTABLE_EEPROM_MAGIC &&
//            h.version == QTABLE_EEPROM_VERSION &&
//            h.profileCount == QTABLE_PROFILE_COUNT;
// }

// bool cargarQTableDesdeEEPROM(uint8_t profile, int8_t (&qtable)[QTABLE_STATES][QTABLE_ACTIONS]) {
//     if (profile >= QTABLE_PROFILE_COUNT) {
//         return false;
//     }

//     QTableHeader h;
//     EEPROM.get(qtableEepromHeaderAddress(), h);
//     if (!qtableHeaderValido(h)) {
//         return false;
//     }

//     EEPROM.get(qtableEepromProfileAddress(profile), qtable);
//     return true;
// }

// void guardarQTableEnEEPROM(uint8_t profile, const int8_t (&qtable)[QTABLE_STATES][QTABLE_ACTIONS]) {
//     if (profile >= QTABLE_PROFILE_COUNT) {
//         return;
//     }

//     const QTableHeader h = {QTABLE_EEPROM_MAGIC, QTABLE_EEPROM_VERSION, QTABLE_PROFILE_COUNT};
//     EEPROM.put(qtableEepromHeaderAddress(), h);
//     EEPROM.put(qtableEepromProfileAddress(profile), qtable);
// }

// void inicializarPerfilesQTableEnEEPROM(const int8_t (&qtableDefaults)[QTABLE_STATES][QTABLE_ACTIONS]) {
//     const QTableHeader h = {QTABLE_EEPROM_MAGIC, QTABLE_EEPROM_VERSION, QTABLE_PROFILE_COUNT};
//     EEPROM.put(qtableEepromHeaderAddress(), h);
//     for (uint8_t p = 0; p < QTABLE_PROFILE_COUNT; p++) {
//         EEPROM.put(qtableEepromProfileAddress(p), qtableDefaults);
//     }
// }

// }

// // ------------------ Control remoto (nivel) ------------------
// bool robot_encendido = false;
// unsigned long ultimo_cambio_boton = 0;
// const unsigned long debounce_delay = 300;

// int estado_control_anterior = LOW;
// // Si tu receptor es activo-alto (START=1, STOP=0), true.
// // Si lo vieras invertido, pon false.
// const bool REMOTE_ACTIVE_HIGH = true;

// // --- Variables globales faltantes ---
// bool bordeDetectado = false;
// const int Velocidad_borde = 120;


// Robot::Robot() :
//     motores(),
//     sensorPisoIzq(SENSOR_DE_PISO_IZQUIERDO, BLANCO),
//     sensorPisoDer(SENSOR_DE_PISO_DERECHO, BLANCO),
//     sensorFrontal(SENSOR_FRONTAL_CENTRA),
//     sensorFrontalIzq(SENSOR_FRONTAL_IZQUIERDO),
//     sensorFrontalDer(SENSOR_FRONTAL_DERECHO),
//     sensorLateralIzq(SENSOR_LATERAL_IZQUIERDO),
//     sensorLateralDer(SENSOR_LATERAL_DERECHO)
// {}

// bool Robot::leerPiso(bool &pisoIzq, bool &pisoDer) {
//     pisoIzq = sensorPisoIzq.detectar();
//     pisoDer = sensorPisoDer.detectar();
//         // Refuerzo: doble verificación para evitar falsas negativas
//         bool izq1 = sensorPisoIzq.detectar();
//         bool der1 = sensorPisoDer.detectar();
//         delay(2); // breve pausa
//         bool izq2 = sensorPisoIzq.detectar();
//         bool der2 = sensorPisoDer.detectar();
//         pisoIzq = izq1 || izq2;
//         pisoDer = der1 || der2;
//         return pisoIzq || pisoDer;
// }

// bool Robot::esperarConPrioridadPiso(unsigned long duracionMs) {
//     unsigned long inicio = millis();
//     while (millis() - inicio < duracionMs) {
//         bool pisoIzq = false;
//         bool pisoDer = false;
//         if (leerPiso(pisoIzq, pisoDer)) {
            
//             return true;
//         }
//         delay(5);
//     }
//     return false;
// }

// bool Robot::enemigoVistoRapido() {
//     return sensorFrontalDer.detectar() ||
//            sensorFrontalIzq.detectar() ||
//            sensorFrontal.detectar() ||
//            sensorLateralDer.detectar() ||
//            sensorLateralIzq.detectar();
// }

// bool Robot::esperarConPrioridadPisoYEnemigo(unsigned long duracionMs) {
//     unsigned long inicio = millis();
//     while (millis() - inicio < duracionMs) {
//         bool pisoIzq = false;
//         bool pisoDer = false;
//         if (leerPiso(pisoIzq, pisoDer)) {
//             return true;
//         }

//         // Si vuelve a ver enemigo, corta la espera para reevaluar de inmediato.
//         if (enemigoVistoRapido()) {
//             return false;
//         }
//         delay(3);
//     }
//     return false;
// }

// void Robot::retrocesoSeguro(unsigned long duracionMs) {
//     unsigned long inicio = millis();
//     while (millis() - inicio < duracionMs) {
//         retroceder();
//         delay(5);
//     }
// }

// void Robot::giroEscapeSeguro(bool haciaDerecha, unsigned long duracionMs) {
//     const unsigned long giroMinimoMs = 90;
//     const unsigned long giroCorreccionMs = 40;
//     unsigned long inicio = millis();
//     bool bordeLiberado = false;

//     while (millis() - inicio < duracionMs) {
//         bool pisoIzq = false;
//         bool pisoDer = false;
//         bool enBorde = leerPiso(pisoIzq, pisoDer);

//         if (haciaDerecha) {
//             moverDerecha();
//         } else {
//             moverIzquierda();
//         }

//         unsigned long tiempoGirando = millis() - inicio;

//         // Obliga un giro corto para despegarse del borde, pero no deja que siga rotando de mas.
//         if (tiempoGirando < giroMinimoMs) {
//             delay(5);
//             continue;
//         }

//         if (!enBorde) {
//             if (bordeLiberado || tiempoGirando >= (giroMinimoMs + giroCorreccionMs)) {
//                 return;
//             }
//             bordeLiberado = true;
//         } else {
//             bordeLiberado = false;
//         }

//         if (tiempoGirando >= duracionMs) {
//             return;
//         }

//         delay(5);
//     }
// }

// void Robot::avanceEscapeSeguro(unsigned long duracionMs) {
//     unsigned long inicio = millis();
//     while (millis() - inicio < duracionMs) {
//         bool pisoIzq = false;
//         bool pisoDer = false;
//         if (leerPiso(pisoIzq, pisoDer)) {
//             return;
//         }

//         moverAdelante();
//         delay(5);
//     }
// }

// void Robot::ejecutarBusquedaCompacta(bool haciaDerecha, unsigned long tiempoEnCiclo, unsigned long avanceMs) {
//     if (tiempoEnCiclo < avanceMs) {
//         moverAdelante();
//         return;
//     }

//     if (haciaDerecha) {
//         motores.curvaDerecha(Velocidad_maxima);
//     } else {
//         motores.curvaIzquierda(Velocidad_maxima);
//     }
// }

// void Robot::setup() {
//     pinMode(Pin_Control_Remoto, INPUT);
//     pinMode(SENSOR_DE_PISO_IZQUIERDO, INPUT);
//     pinMode(SENSOR_DE_PISO_DERECHO, INPUT);
//     pinMode(SENSOR_FRONTAL_DERECHO, INPUT);
//     pinMode(SENSOR_FRONTAL_CENTRA, INPUT);
//     pinMode(SENSOR_FRONTAL_IZQUIERDO, INPUT);
//     pinMode(SENSOR_LATERAL_IZQUIERDO, INPUT);
//     pinMode(SENSOR_LATERAL_DERECHO, INPUT);
//     pinMode(MA2A, OUTPUT);
//     pinMode(MA1A, OUTPUT);
//     pinMode(PWMA, OUTPUT);
//     pinMode(MA2B, OUTPUT);
//     pinMode(MA1B, OUTPUT);
//     pinMode(PWMB, OUTPUT);

//     // Ya no se usa control remoto para encendido.
//     estado_control_anterior = LOW;
//     robot_encendido = true;
// }

// void Robot::detenerse() {
//     motores.detener();
// }

// void Robot::ataqueEnemigo() {
//     static unsigned long ultimoEnemigoDetectadoMs = 0;
//     static bool persistirAtaque = false;
//     unsigned long ahora = millis();
//     if (enemigoVistoRapido()) {
//         ultimoEnemigoDetectadoMs = ahora;
//         persistirAtaque = true;
//     } else if (persistirAtaque && (ahora - ultimoEnemigoDetectadoMs > 1200)) {
//         persistirAtaque = false;
//     }
//     if (persistirAtaque) {
//         motores.adelante(Velocidad_maxima);
//     } else {
//         motores.detener();
//     }
// }

// void Robot::moverAdelante() {
//     motores.adelante(Velocidad_movimiento_seguir);
// }

// void Robot::retroceder() {
//     motores.retroceder(Velocidad_estandar);
// }

// void Robot::moverDerecha() {
//     motores.derecha(Velocidad_maxima);
// }

// void Robot::moverIzquierda() {
//     motores.izquierda(Velocidad_maxima);
// }

// void Robot::sensoresPiso(bool pisoIzq, bool pisoDer) {
//     static bool giroAlternadoDerecha = true;

//     if (pisoIzq && pisoDer) {
//         retrocesoSeguro(RETROCESO_BORDE_DOBLE_MS);
//         giroEscapeSeguro(giroAlternadoDerecha, GIRO_ESCAPE_BORDE_DOBLE_MS);
//         avanceEscapeSeguro(AVANCE_INTERIOR_DOBLE_MS);
//         giroAlternadoDerecha = !giroAlternadoDerecha;
//     } else if (pisoDer) {
//         retrocesoSeguro(RETROCESO_BORDE_SIMPLE_MS);
//         giroEscapeSeguro(false, GIRO_ESCAPE_BORDE_SIMPLE_MS);
//         avanceEscapeSeguro(AVANCE_INTERIOR_SIMPLE_MS);
//     } else if (pisoIzq) {
//         retrocesoSeguro(RETROCESO_BORDE_SIMPLE_MS);
//         giroEscapeSeguro(true, GIRO_ESCAPE_BORDE_SIMPLE_MS);
//         avanceEscapeSeguro(AVANCE_INTERIOR_SIMPLE_MS);
//     }
// }

// void Robot::sensoresFrontales(bool central, bool derecho, bool izquierdo) {
//     if (central || (derecho && izquierdo)) {
//         ataqueEnemigo();
//     } else if (derecho) {
//         moverDerecha();
//         if (esperarConPrioridadPisoYEnemigo(55)) {
//             return;
//         }
//         motores.adelante(Velocidad_maxima);
//         esperarConPrioridadPisoYEnemigo(55);
//     } else if (izquierdo) {
//         moverIzquierda();
//         if (esperarConPrioridadPisoYEnemigo(55)) {
//             return;
//         }
//         motores.adelante(Velocidad_maxima);
//         esperarConPrioridadPisoYEnemigo(55);
//     }
// }

// void Robot::sensoresLaterales(bool sensorIzquierdo, bool sensorDerecho) {
//     if (sensorIzquierdo && sensorDerecho) return;

//     bool pisoIzq = false, pisoDer = false;
//     leerPiso(pisoIzq, pisoDer);
//     if (pisoIzq || pisoDer) return;

//     // Gira hacia el enemigo lateral hasta que el sensor frontal lo detecte o hasta un tiempo máximo
//     const unsigned long tiempoMaxGiro = 600; // ms
//     unsigned long inicio = millis();
//     if (sensorIzquierdo) {
//         while (!sensorFrontal.detectar() && (millis() - inicio < tiempoMaxGiro)) {
//             motores.curvaIzquierda(Velocidad_maxima);
//             if (leerPiso(pisoIzq, pisoDer) && (pisoIzq || pisoDer)) break;
//             delay(5);
//         }
//     } else if (sensorDerecho) {
//         while (!sensorFrontal.detectar() && (millis() - inicio < tiempoMaxGiro)) {
//             motores.curvaDerecha(Velocidad_maxima);
//             if (leerPiso(pisoIzq, pisoDer) && (pisoIzq || pisoDer)) break;
//             delay(5);
//         }
//     }
//     motores.detener();
//     // Si ya ve al enemigo de frente, ataca persistentemente
//     if (sensorFrontal.detectar()) {
//         ataqueEnemigo();
//     }
// }

// // Valores iniciales de la Q-tabla. Se usan para restaurar al arrancar.
// static const int8_t QTABLE_DEFAULTS[32][6] = {
//     /* s00 */ { 0,  0,  0,  0,  0, 20},
//     /* s01 */ { 5, 10,  0, 20,  0,  0},
//     /* s02 */ { 5,  0, 10,  0, 20,  0},
//     /* s03 */ {20,  0,  0,  5,  5,  0},
//     /* s04 */ {20,  0,  0,  0,  0,  0},
//     /* s05 */ {20,  0,  0, 10,  0,  0},
//     /* s06 */ {20,  0,  0,  0, 10,  0},
//     /* s07 */ {20,  0,  0,  0,  0,  0},
//     /* s08 */ { 0, 20,  0, 10,  0,  0},
//     /* s09 */ {10,  5,  0, 20,  0,  0},
//     /* s10 */ {20,  0,  0,  5,  0,  0},
//     /* s11 */ {20,  0,  0, 10,  0,  0},
//     /* s12 */ {20,  0,  0,  5,  0,  0},
//     /* s13 */ {20,  0,  0, 10,  0,  0},
//     /* s14 */ {10,  5,  0, 20,  0,  0},
//     /* s15 */ {20,  0,  0,  5,  0,  0},
//     /* s16 */ { 0,  0, 20,  0, 10,  0},
//     /* s17 */ {20,  0,  0,  0,  0,  0},
//     /* s18 */ {10,  0,  5,  0, 20,  0},
//     /* s19 */ {20,  0,  0,  0,  5,  0},
//     /* s20 */ {20,  0,  0,  0,  5,  0},
//     /* s21 */ {10,  0,  5,  0, 20,  0},
//     /* s22 */ {20,  0,  0,  0, 10,  0},
//     /* s23 */ {20,  0,  0,  0,  5,  0},
//     /* s24 */ {20,  0,  0,  0,  0,  0},
//     /* s25 */ {20,  0,  0, 10,  0,  0},
//     /* s26 */ {20,  0,  0,  0, 10,  0},
//     /* s27 */ {20,  0,  0,  0,  0,  0},
//     /* s28 */ {20,  0,  0,  0,  0,  0},
//     /* s29 */ {20,  0,  0,  5,  0,  0},
//     /* s30 */ {20,  0,  0,  0,  5,  0},
//     /* s31 */ {20,  0,  0,  0,  0,  0},
// };

// void Robot::loop() {
//     // LECTURA DE SENSORES
//     bool PisoIzq = false, PisoDer = false;
//     int valorPisoIzq = analogRead(SENSOR_DE_PISO_IZQUIERDO);
//     int valorPisoDer = analogRead(SENSOR_DE_PISO_DERECHO);
//     PisoIzq = (valorPisoIzq <= BLANCO);
//     PisoDer = (valorPisoDer <= BLANCO);
//     bool FrontalDer = sensorFrontalDer.detectar();
//     bool FrontalIzq = sensorFrontalIzq.detectar();
//     bool FrontalCentral = sensorFrontal.detectar();
//     bool LateralDer = sensorLateralDer.detectar();
//     bool LateralIzq = sensorLateralIzq.detectar();
//     const bool enemigoDetectado = FrontalDer || FrontalIzq || FrontalCentral || LateralDer || LateralIzq;

//     // --- NUEVA LÓGICA: avanzar hasta detectar borde antes de rutina normal ---
//     if (!bordeDetectado) {
//         motores.adelante(Velocidad_borde);
//         if (PisoIzq || PisoDer) {
//             bordeDetectado = true;
//             retrocesoSeguro(350);
//             sensoresPiso(PisoIzq, PisoDer);
//         }
//         return;
//     }

//     // SISTEMA DE PRIORIDADES
//     if (PisoIzq || PisoDer) {
//         sensoresPiso(PisoIzq, PisoDer);
//         return;
//     }
//     if (FrontalCentral || (FrontalDer && FrontalIzq)) {
//         ataqueEnemigo();
//         return;
//     }
//     if (FrontalDer || FrontalIzq) {
//         sensoresFrontales(false, FrontalDer, FrontalIzq);
//         return;
//     }
//     if (LateralDer || LateralIzq) {
//         sensoresLaterales(LateralIzq, LateralDer);
//         return;
//     }

//     static unsigned long ultimoContacto = 0;
//     if (enemigoDetectado) {
//         ultimoContacto = millis();
//     }

//     // BÚSQUEDA AUTOMÁTICA SIN CONTROL REMOTO
//     static bool busquedaDerecha = true;
//     static uint8_t faseBusqueda = 0;
//     static unsigned long inicioFase = 0;
//     static bool estadoAnterior = false;
//     const bool recienEncendido = !estadoAnterior;
//     estadoAnterior = true;

//     if (recienEncendido) {
//         faseBusqueda = 0;
//         inicioFase = 0;
//     }

//     const unsigned long ahora = millis();
//     const unsigned long tiempoSinContacto = ahora - ultimoContacto;
//     const int margenSeguridadPiso = 35;
//     const bool cercaBordeIzq = valorPisoIzq <= (BLANCO + margenSeguridadPiso);
//     const bool cercaBordeDer = valorPisoDer <= (BLANCO + margenSeguridadPiso);

//     if (tiempoSinContacto < 130) {
//         if (cercaBordeIzq || cercaBordeDer) {
//             if (cercaBordeIzq && !cercaBordeDer) {
//                 moverDerecha();
//             } else if (cercaBordeDer && !cercaBordeIzq) {
//                 moverIzquierda();
//             } else if (busquedaDerecha) {
//                 moverDerecha();
//             } else {
//                 moverIzquierda();
//             }
//         } else {
//             if (busquedaDerecha) moverDerecha(); else moverIzquierda();
//         }
//         faseBusqueda = 0;
//         inicioFase = ahora;
//         return;
//     }

//     if (inicioFase == 0) {
//         inicioFase = ahora;
//     }

//     const unsigned long duraciones[4] = {350, 80, 350, 80};
//     if ((ahora - inicioFase) >= duraciones[faseBusqueda]) {
//         inicioFase = ahora;
//         faseBusqueda = (faseBusqueda + 1) % 4;
//         if (faseBusqueda == 0) {
//             busquedaDerecha = !busquedaDerecha;
//         }
//     }

//     if (cercaBordeIzq || cercaBordeDer) {
//         if (cercaBordeIzq && !cercaBordeDer) {
//             moverDerecha();
//         } else if (cercaBordeDer && !cercaBordeIzq) {
//             moverIzquierda();
//         } else if (busquedaDerecha) {
//             moverDerecha();
//         } else {
//             moverIzquierda();
//         }
//         faseBusqueda = 0;
//         inicioFase = ahora;
//         return;
//     }

//     if (faseBusqueda == 0) {
//         if (busquedaDerecha) moverDerecha(); else moverIzquierda();
//     } else if (faseBusqueda == 1) {
//         motores.adelante(Velocidad_maxima);
//     } else if (faseBusqueda == 2) {
//         if (busquedaDerecha) moverIzquierda(); else moverDerecha();
//     } else {
//         motores.adelante(Velocidad_maxima);
//     }
// }
