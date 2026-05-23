#ifndef AUTODIAGNOSTICO_H
#define AUTODIAGNOSTICO_H

#include <Arduino.h>

class Autodiagnostico {
public:
    Autodiagnostico();
    void checarSensores();
    void checarMotores();
    bool hayFallo() const;
private:
    bool falloSensor;
    bool falloMotor;
};

#endif // AUTODIAGNOSTICO_H
