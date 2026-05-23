// estrategias de búsqueda para el robot sumo
#include "EstrategiasBusqueda.h"
#include "Definiciones.h"
#include "Motor.h"
#include "SensorPiso.h"
#include "SensorEnemigo.h"



// Usar las instancias globales reales
extern Motores motores;
extern SensorEnemigo sensorFrontal;
extern SensorEnemigo sensorFrontalIzq;
extern SensorEnemigo sensorFrontalDer;
extern SensorPiso sensorPisoIzq;
extern SensorPiso sensorPisoDer;

void EstrategiasBusqueda::busquedaEnemigo() {

    // Giro a la derecha en pequeños pasos, verificando piso y enemigo
    for (int i = 0; i < 18; ++i) {
        if (sensorPisoIzq.detectar() || sensorPisoDer.detectar()) {
            motores.detener();
            return; // Cerca del borde, abortar búsqueda
        }
        motores.derecha(Velocidad_normal);
        delay(70); // paso corto
        motores.detener();
        delay(20);
        if (sensorFrontal.detectar() || sensorFrontalIzq.detectar() || sensorFrontalDer.detectar()) {
            motores.detener();
            return; // Enemigo detectado
        }
    }
    motores.detener();

    // Giro a la izquierda en pequeños pasos, verificando piso y enemigo
    for (int i = 0; i < 36; ++i) {
        if (sensorPisoIzq.detectar() || sensorPisoDer.detectar()) {
            motores.detener();
            return;
        }
        motores.izquierda(Velocidad_normal);
        delay(70);
        motores.detener();
        delay(20);
        if (sensorFrontal.detectar() || sensorFrontalIzq.detectar() || sensorFrontalDer.detectar()) {
            motores.detener();
            return;
        }
    }
    motores.detener();

    // Avance muy corto si no se detecta enemigo ni borde
    if (!(sensorPisoIzq.detectar() || sensorPisoDer.detectar())) {
        motores.adelante(Velocidad_normal);
        delay(120);
        motores.detener();
    }
}

void EstrategiasBusqueda::busquedaPiso() {
    // Usar instancias globales
    extern Motores motores;
    extern SensorPiso sensorPisoIzq;
    extern SensorPiso sensorPisoDer;

    // Si detecta borde en ambos sensores, retroceder, girar aleatoriamente y avanzar para salir del borde
    if (sensorPisoIzq.detectar() && sensorPisoDer.detectar()) {
        motores.retroceder(Velocidad_normal);
        delay(250);
        motores.detener();
        // Elegir giro aleatorio
        if (random(0, 2) == 0) {
            motores.derecha(Velocidad_normal);
        } else {
            motores.izquierda(Velocidad_normal);
        }
        delay(220);
        motores.detener();
        // Avanzar un poco para alejarse del borde
        motores.adelante(Velocidad_normal);
        delay(120);
        motores.detener();
        return;
    }
    // Si detecta borde solo en un lado, girar hacia el lado seguro
    if (sensorPisoIzq.detectar()) {
        motores.derecha(Velocidad_normal);
        delay(150);
        motores.detener();
        return;
    }
    if (sensorPisoDer.detectar()) {
        motores.izquierda(Velocidad_normal);
        delay(150);
        motores.detener();
        return;
    }
    // Si no detecta borde, avanzar lentamente
    motores.adelante(Velocidad_normal);
    delay(100);
    motores.detener();
}
