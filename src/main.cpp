
#include <Arduino.h>
#include "Robot.h"
#include "Motor.h"

Motores motores;
Robot robot;

void setup() {
  robot.setup();
}

void loop() {
  robot.loop();
}