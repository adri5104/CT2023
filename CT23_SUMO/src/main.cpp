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
bool button = false;
bool button_pre = false;

state Estado = MODE_NORMAL;

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

  // Leemos datos incoming del mando
  if (PS4.isConnected())
  {
    left_stick[X] = PS4.LStickX();
    left_stick[Y] = PS4.LStickY();

    right_stick[X] = PS4.RStickX();
    right_stick[Y] = PS4.RStickY();

    gatillos[L] = PS4.L2Value();
    gatillos[R] = PS4.R2Value();

    button = PS4.Square();
  }

  // Cambio de modo
  if(button && button_pre == false)
  {
    if(Estado == MODE_NORMAL)
    {
      Estado = MODE_TURBO;
    }
    else
    {
      Estado = MODE_NORMAL;
    }
    button_pre = true;
  }
  else if(!button)
  {
    button_pre = false;
  }
  
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

  
  

  // --------------------- Control de los motores -------------------


  // Modo normal
  if(Estado == MODE_NORMAL)
  {
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
    PS4.setLed(0,255,0);
  }
  

  // Modo turbo
  if(Estado == MODE_TURBO)
  {
    int vel_base = left_stick[Y];
    int vel_giro = left_stick[X];

    vel_R = vel_base - vel_giro;
    vel_L = vel_base + vel_giro;

    vel_R = map(vel_R, -(2*(DS4_MAX_ANALOG_VALUE)), (2*DS4_MAX_ANALOG_VALUE), gatillos[L] > DS4_TRIGGER_HP_VALUE? -PWM_MAX_HP : -PWM_MAX_LP,gatillos[L] > DS4_TRIGGER_HP_VALUE? PWM_MAX_HP : PWM_MAX_LP);
    vel_L = map(vel_L, -(2*DS4_MAX_ANALOG_VALUE), (2*DS4_MAX_ANALOG_VALUE), gatillos[L] > DS4_TRIGGER_HP_VALUE? -PWM_MAX_HP : -PWM_MAX_LP,gatillos[L] > DS4_TRIGGER_HP_VALUE? PWM_MAX_HP : PWM_MAX_LP);

    if(vel_R > 0)
    {
      Motor_R.setFwd();
      Motor_R.setPWM(abs(vel_R));
    }
    else
    {
      Motor_R.setBack();
      Motor_R.setPWM(abs(vel_R));
    }

    if(vel_L > 0)
    {
      Motor_L.setFwd();
      Motor_L.setPWM(abs(vel_L));
    }
    else
    {
      Motor_L.setBack();
      Motor_L.setPWM(abs(vel_L));
    }
    
    PS4.setLed(0,0,255);
  }



  // Salida de datos por serial
  String str = "Vel_R: " + String(vel_R) + ", Vel_L: " + String(vel_L);
  Serial.println(str);


  

  // Enviamos datos outcoming al mando
  PS4.setFlashRate(50, 50);
  //PS4.setRumble(PS4.L2Value(), PS4.R2Value());
  PS4.sendToController();

  
}
