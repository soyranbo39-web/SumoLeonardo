#include <Arduino.h>
#include "Robot.h"

Robot robot;

void setup() {
  robot.setup();
}

void loop() {
  robot.loop();
}