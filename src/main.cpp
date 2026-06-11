
#include <Arduino.h>
#include "Robot.h"
#include "Motor.h"
#include "SensorPiso.h"
#include "SensorEnemigo.h"
#include "Definiciones.h"

Motores motores;
SensorPiso sensorPisoIzq(S_PISO_IZQ, BLANCO);
SensorPiso sensorPisoDer(S_PISO_DER, BLANCO);
SensorEnemigo sensorFrontal(S_FRONT_CEN);
SensorEnemigo sensorFrontalIzq(S_FRONT_IZQ);
SensorEnemigo sensorFrontalDer(S_FRONT_DER);
SensorEnemigo sensorLateralIzq(S_LAT_IZQ);
SensorEnemigo sensorLateralDer(S_LAT_DER);
Robot robot;

void setup() {
  robot.setup();
}

void loop() {
  robot.loop();
}