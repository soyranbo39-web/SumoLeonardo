
#include "AtaqueAdaptativo.h"
#include "Motor.h"
#include "Definiciones.h"
#include "Telemetria.h"

extern Motores motores;

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
