

#include "Robot.h"
#include <Arduino.h>
#include "Definiciones.h"
#include "Motor.h"
#include "SensorPiso.h"
#include "SensorEnemigo.h"
#include "EstrategiasBusqueda.h"
#include "BusquedaAvanzada.h"
#include "DetectorAtascos.h"
#include "AtaqueAdaptativo.h"
#include "Telemetria.h"

BusquedaAvanzada busquedaAvanzada;
DetectorAtascos detectorAtascos;
AtaqueAdaptativo ataqueAdaptativo;

Robot::Robot()
		: sensorPisoIzq(S_PISO_IZQ, BLANCO),
			sensorPisoDer(S_PISO_DER, BLANCO),
			sensorFrontal(S_FRONT_CEN),
			sensorFrontalIzq(S_FRONT_IZQ),
			sensorFrontalDer(S_FRONT_DER),
			sensorLateralIzq(S_LAT_IZQ),
			sensorLateralDer(S_LAT_DER)
{}

void Robot::setup() {
	if (Usar_Arrancador) {
		pinMode(Pin_Control_Remoto, INPUT);
		while (digitalRead(Pin_Control_Remoto) == LOW) {
			motores.detener();
			delay(10);
		}
	}

	// Sin arrancador: pequeno retardo para estabilizar sensores/alimentacion.
	delay(Retardo_Autoinicio_ms);
}

void Robot::detenerse() {
	motores.detener();
}

void Robot::ataqueEnemigo() {
	// Validar piso antes de cualquier acción de ataque
	if (sensorPisoIzq.detectar() || sensorPisoDer.detectar()) {
		motores.detener();
		return;
	}

	// Detectar enemigo con sensores (ya filtrados por Kalman en detectar())
	bool enemigoFrontal = sensorFrontal.detectar();
	bool enemigoIzq = sensorFrontalIzq.detectar();
	bool enemigoDer = sensorFrontalDer.detectar();
	bool enemigoLatIzq = sensorLateralIzq.detectar();
	bool enemigoLatDer = sensorLateralDer.detectar();

	// Prioridad: frontal > 45° > laterales
	if (enemigoFrontal) {
		motores.adelante(Velocidad_maxima);
	} else if (enemigoIzq) {
		motores.curvaIzquierda(Velocidad_maxima);
	} else if (enemigoDer) {
		motores.curvaDerecha(Velocidad_maxima);
	} else if (enemigoLatIzq) {
		motores.izquierda(Velocidad_maxima);
	} else if (enemigoLatDer) {
		motores.derecha(Velocidad_maxima);
	} else {
		motores.detener();
		return;
	}
	delay(120);
	motores.detener();
}

void Robot::moverAdelante() { motores.adelante(Velocidad_normal); }
void Robot::retroceder() { motores.retroceder(Velocidad_normal); }
void Robot::moverDerecha() { motores.derecha(Velocidad_normal); }
void Robot::moverIzquierda() { motores.izquierda(Velocidad_normal); }

void Robot::sensoresPiso(bool pisoIzq, bool pisoDer) {}
void Robot::sensoresFrontales(bool central, bool derecho, bool izquierdo) {}
void Robot::sensoresLaterales(bool sensorIzquierdo, bool sensorDerecho) {}

void Robot::loop() {
	bool pisoIzq = sensorPisoIzq.detectar();
	bool pisoDer = sensorPisoDer.detectar();
	bool enemigoFrontal = sensorFrontal.detectar();
	bool enemigoIzq = sensorFrontalIzq.detectar();
	bool enemigoDer = sensorFrontalDer.detectar();
	bool enemigoLatIzq = sensorLateralIzq.detectar();
	bool enemigoLatDer = sensorLateralDer.detectar();

	Telemetria::logEstadoSensores(pisoIzq, pisoDer, enemigoFrontal, enemigoIzq, enemigoDer, enemigoLatIzq, enemigoLatDer);

	// Detección de atascos
	bool movimiento = !(pisoIzq || pisoDer);
	detectorAtascos.actualizar(movimiento, enemigoFrontal || enemigoIzq || enemigoDer || enemigoLatIzq || enemigoLatDer, pisoIzq || pisoDer);
	if (detectorAtascos.estaAtascado()) {
		Telemetria::logAccion("Atasco detectado");
		detectorAtascos.resolverAtasco();
		return;
	}

	EstrategiasBusqueda estrategia;

	// 1. Prioridad máxima: evitar el borde
	if (pisoIzq || pisoDer) {
		estrategia.busquedaPiso();
		return;
	}

	// 2. Si detecta enemigo en cualquier sensor, ataque adaptativo
	if (enemigoFrontal || enemigoIzq || enemigoDer || enemigoLatIzq || enemigoLatDer) {
		ataqueAdaptativo.atacar(enemigoFrontal, enemigoIzq, enemigoDer, enemigoLatIzq, enemigoLatDer);
		return;
	}

	// 3. Si no detecta enemigo ni borde, búsqueda avanzada
	busquedaAvanzada.ejecutar();
}