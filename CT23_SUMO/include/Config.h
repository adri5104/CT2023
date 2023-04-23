#pragma once
#include <Arduino.h>

#define SERIAL_PRINT_

// Movidas del mando PS4
const char MAC_ADDRESS[] = "08:3a:f2:af:02:34";
const int DS4_MAX_ANALOG_VALUE = 255;
const int DS4_TRIGGER_HP_VALUE = 100;

// -------- Movidas de la lectura de tension ----
const uint8_t PIN_VSENSOR = 38;
const int V_MAX_ADC = 2457;
const int V_MIN_ADC = 0;


// --------------- Parametros motores ----------

const uint8_t PWM_MAX_LP = 100;
const uint8_t PWM_MAX_HP = 200;

// --------------- PINES ----------------

const uint8_t PIN_MOTOR_DIR_R = 18;
const uint8_t PIN_MOTOR_PWM_R = 19;
const uint8_t PIN_MOTOR_DIR_L = 17;
const uint8_t PIN_MOTOR_PWM_L = 5;




#define R 0
#define G 1 
#define B 2
#define X 0
#define Y 1
#define R 0
#define L 1