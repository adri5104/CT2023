#pragma once
#include <Arduino.h>

#define SERIAL_PRINT_

enum state
{
    MODE_NORMAL,
    MODE_TURBO
};

// Movidas del mando PS4
const char MAC_ADDRESS[] = "70:b8:f6:5d:6a:b8";
const int DS4_MAX_ANALOG_VALUE = 125;
const int DS4_TRIGGER_HP_VALUE = 100;

// -------- Movidas de la lectura de tension ----
const uint8_t PIN_VSENSOR = 38;
const int V_MAX_ADC = 2457;
const int V_MIN_ADC = 0;


// --------------- Parametros motores ----------

const uint8_t PWM_MAX_LP = 120;
const uint8_t PWM_MAX_HP = 240;

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