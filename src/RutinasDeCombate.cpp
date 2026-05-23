#include "RutinasDeCombate.h"
#include "Motor.h"
#include "Definiciones.h"
#include "SensorPiso.h"

extern Motores motores;
extern SensorPiso sensorPisoIzq;
extern SensorPiso sensorPisoDer;

void rutinaAvanceRectoYVuelta() {
    // Avanza recto hasta detectar el borde
    while (!sensorPisoIzq.detectar() && !sensorPisoDer.detectar()) {
        motores.adelante(Velocidad_maxima);
        delay(10); // Pequeño delay para evitar bloqueo
    }
    motores.detener();
    delay(100);
    // Da una vuelta (giro de 360° aprox)
    for (int i = 0; i < 36; ++i) {
        motores.derecha(Velocidad_normal);
        delay(70);
        motores.detener();
        delay(20);
    }
    motores.detener();
    delay(100);
    // Aquí el robot puede continuar con su lógica normal
}

void rutinaGiroInicialDeteccion() {
    // Da una vuelta completa (360°) para buscar enemigos detrás
    for (int i = 0; i < 36; ++i) {
        motores.derecha(Velocidad_normal);
        delay(70);
        motores.detener();
        delay(20);
    }
    motores.detener();
    delay(100);
    // Aquí el robot puede continuar con su lógica normal
}
