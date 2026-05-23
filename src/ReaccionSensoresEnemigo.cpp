
#include "Definiciones.h"
#include "Motor.h"
#include "SensorEnemigo.h"
#include "SensorPiso.h"
#include "Definiciones.h"

// Instancias globales de sensores y motores (ajustar según tu arquitectura real)
extern Motores motores;
extern SensorEnemigo sensorFrontal;
extern SensorEnemigo sensorFrontalIzq;
extern SensorEnemigo sensorFrontalDer;
extern SensorEnemigo sensorLateralIzq;
extern SensorEnemigo sensorLateralDer;
extern SensorPiso sensorPisoIzq;
extern SensorPiso sensorPisoDer;

void reaccionarSensoresEnemigo() {
	// Validar piso antes de cualquier acción
	if (sensorPisoIzq.detectar() || sensorPisoDer.detectar()) {
		motores.detener();
		return;
	}

	bool enemigoFrontal = sensorFrontal.detectar();
	bool enemigoIzq = sensorFrontalIzq.detectar();
	bool enemigoDer = sensorFrontalDer.detectar();
	bool enemigoLatIzq = sensorLateralIzq.detectar();
	bool enemigoLatDer = sensorLateralDer.detectar();

	// Prioridad: frontal > 45° > laterales
	if (enemigoFrontal) {
		motores.adelante(Velocidad_maxima);
		delay(120);
		motores.detener();
		return;
	}
	if (enemigoIzq) {
		motores.curvaIzquierda(Velocidad_maxima);
		delay(100);
		motores.detener();
		return;
	}
	if (enemigoDer) {
		motores.curvaDerecha(Velocidad_maxima);
		delay(100);
		motores.detener();
		return;
	}
	if (enemigoLatIzq) {
		motores.izquierda(Velocidad_maxima);
		delay(120);
		motores.detener();
		return;
	}
	if (enemigoLatDer) {
		motores.derecha(Velocidad_maxima);
		delay(120);
		motores.detener();
		return;
	}
    motores.detener();
}