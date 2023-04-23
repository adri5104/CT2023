#include <Arduino.h>
#include <PS4Controller.h>
#include <Wire.h>
#include "Config.h"
#include "Motor.h"


// Motores
Motor Motor_R(PIN_MOTOR_DIR_R, PIN_MOTOR_PWM_R);
Motor Motor_L(PIN_MOTOR_DIR_L, PIN_MOTOR_PWM_L);

// Velocidades
int vel_R = 0;
int vel_L = 0;

// Datos del mando
int left_stick[2] = {0, 0};
int right_stick[2] = {0, 0};
int gatillos[2] = {0, 0};

void setup()
{
  Serial.begin(115200);
  PS4.begin(MAC_ADDRESS);

  Serial.println("Holi");
  
  // Pin para medicion de tension
  pinMode(PIN_VSENSOR, INPUT);

  // Inicializacion motores
  Motor_R.init();
  Motor_L.init();
}

void loop()
{
  // Lectura de tension de la bateria
  int v_bat = analogRead(PIN_VSENSOR);
  v_bat = map(v_bat, V_MIN_ADC, V_MAX_ADC, 0, 100);

  /*
  Serial.print("L: (");
  Serial.print(left_stick[X]);
  Serial.print(",");
  Serial.print(left_stick[Y]);
  Serial.print(") R: (");
  Serial.print(right_stick[X]);
  Serial.print(",");
  Serial.print(right_stick[Y]);
  Serial.print(") Gatillos: (");
  Serial.print(gatillos[X]);
  Serial.print(",");
  Serial.print(gatillos[Y]);
  Serial.print(") VBat: ");
  Serial.println(v_bat);
  */

  
  // Leemos movidas del mando
  if (PS4.isConnected())
  {
    left_stick[X] = PS4.LStickX();
    left_stick[Y] = PS4.LStickY();

    right_stick[X] = PS4.RStickX();
    right_stick[Y] = PS4.RStickY();

    gatillos[L] = PS4.L2Value();
    gatillos[R] = PS4.R2Value();
  }

  // --------------------- Control de los motores -------------------

  // Direccion 
  if(right_stick[Y] > 0)
  {
    Motor_R.setFwd();
  }
  else
  {
    Motor_R.setBack();  
  }

  if(left_stick[Y] > 0)
  {
    Motor_L.setFwd();
  }
  else
  {
    Motor_L.setBack();  
  }

  //PWM
  if(abs(gatillos[R]) > DS4_TRIGGER_HP_VALUE &&(gatillos[L]) > DS4_TRIGGER_HP_VALUE)
  {
    vel_R = map(abs(right_stick[Y]),0,DS4_MAX_ANALOG_VALUE, 0,PWM_MAX_HP);
    vel_L = map(abs(left_stick[Y]),0,DS4_MAX_ANALOG_VALUE, 0,PWM_MAX_HP);
    Motor_R.setPWM(vel_R > PWM_MAX_HP? PWM_MAX_HP : vel_R);
    Motor_L.setPWM(vel_L > PWM_MAX_HP? PWM_MAX_HP : vel_L);
  }
  // Modo BP
  else
  {
    vel_R = map(abs(right_stick[Y]),0,DS4_MAX_ANALOG_VALUE, 0,PWM_MAX_LP);
    vel_L = map(abs(left_stick[Y]),0,DS4_MAX_ANALOG_VALUE, 0,PWM_MAX_LP);
    Motor_R.setPWM(vel_R > PWM_MAX_HP? PWM_MAX_LP : vel_R);
    Motor_L.setPWM(vel_L > PWM_MAX_HP? PWM_MAX_LP : vel_L);
  }

  String str = "Vel_R: " + String(vel_R) + ", Vel_L: " + String(vel_L);
  Serial.println(str);


  

  PS4.setLed(gatillos[X], 0, gatillos[Y]);
  PS4.setFlashRate(50, 50);
  PS4.setRumble(PS4.L2Value(), PS4.R2Value());
  PS4.sendToController();

  
}
