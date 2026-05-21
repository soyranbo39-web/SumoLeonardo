#ifndef DEFINICIONES_H
#define DEFINICIONES_H

// Pines de sensores
#define S_PISO_IZQ   A1
#define S_PISO_DER   A2
#define S_FRONT_IZQ  2
#define S_FRONT_CEN  4
#define S_FRONT_DER  A5
#define S_LAT_IZQ    1
#define S_LAT_DER    A4

// Compatibilidad con nombres anteriores
#define SENSOR_DE_PISO_IZQUIERDO  S_PISO_IZQ
#define SENSOR_DE_PISO_DERECHO    S_PISO_DER
#define SENSOR_FRONTAL_CENTRA     S_FRONT_CEN
#define SENSOR_FRONTAL_DERECHO    S_FRONT_DER
#define SENSOR_FRONTAL_IZQUIERDO  S_FRONT_IZQ
#define SENSOR_LATERAL_DERECHO    S_LAT_DER
#define SENSOR_LATERAL_IZQUIERDO  S_LAT_IZQ

// Pines de motores
#define PWMA 10
#define MA1A 9
#define MA2A 13
#define PWMB 11
#define MA1B 8
#define MA2B 12

// Pin de control remoto
const int Pin_Control_Remoto = A0; // Start=1, Stop=0

// Parámetros
#define BLANCO 120
#define Velocidad_movimiento_seguir 200// 160 
#define Velocidad_estandar 200 //180  
#define Velocidad_normal 200  // 120 
#define Velocidad_maxima 200

#endif // DEFINICIONES_H
