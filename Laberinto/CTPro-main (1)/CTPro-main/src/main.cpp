#include <Arduino.h>
#include "CTPro.h"
#include "Motor.h"
#include "TofSensors.h"
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>
#include "Encoder.h"
#include "PID_v1.h"
#include "RDrive.h"
#include "mojojojo.h"


extern const unsigned char mojojojo [];


//#define _PRINT_DISTANCE_
// ---------------------- Objetos globales -------------



Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Encoder_p encoderL = Encoder_p(ENC_LA, ENC_LB, 4.0);
Encoder_p encoderR = Encoder_p(ENC_RA, ENC_RB, 4.0);
Motor motorL = Motor(DIR_LA, DIR_LB, PWM_L);
Motor motorR = Motor(DIR_RA, DIR_RB, PWM_R);
TofSensors mySensors(XSH_1, XSH_2, XSH_3);

state Estado = S_INIT;
bool s_boton = false;

RDRIVE::RDrive myRobot(&motorL, &motorR, &encoderL, &encoderR, &mySensors);

int mydistance[3];

// Funciones callback para las interrupciones de los encoders
void header_encoderL()
{
  encoderL.actualizar_posicion();
}

void header_encoderR()
{
  encoderR.actualizar_posicion();
}



void setup(){

  Wire.setClock(400000);
  Serial.begin(9600);

  pinMode(BTN_2, INPUT);

  myRobot.getPID().SetTunings(kP, kI, kD);
  myRobot.setVelBase(velocidad_base);
  myRobot.setMode(RDRIVE::MODE::MANTENERSE_CENTRADO);
  
  
  // ---------------------- Movidas display -------------

  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); 
  }
  display.display();
  delay(2000); 

  // Clear the buffer
  display.clearDisplay();
  display.display();
  delay(2000); 
  display.clearDisplay();

  
  // ---------------------- Inicializaciones tof -------
  
  mySensors.init();
  mySensors.setID(
    TOF_SENSOR_FRONT,
    TOF_SENSOR_LEFT,
    TOF_SENSOR_RIGHT
  );
  
  display.clearDisplay();

  // ---------------------- Inicializaciones encoders -------
  
  encoderL.init();
  encoderR.init();
  attachInterrupt(digitalPinToInterrupt(ENC_LA),header_encoderL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_RA),header_encoderR, CHANGE);

  // ---------------------- Inicializaciones motores -------
  
  motorL.init();
  motorR.init();

  
  Estado = S_AVANZANDO;


  


  display.clearDisplay();
  display.drawBitmap(
  (display.width()  - LOGO_WIDTH ) / 2,
  (display.height() - LOGO_HEIGHT) / 2,
  mojojojo, LOGO_WIDTH, LOGO_HEIGHT, 1);
  display.display();

  delay(2000);
  display.clearDisplay();
  display.display();
} 


//#define __TEST__

void loop() {
  #ifdef __TEST__
    myRobot.compute(mydistance);
    Serial.println(mydistance[FRONT]);
  #else
  myRobot.compute(mydistance);
  

  int checkWalls = myRobot.checkWalls();

  /*display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextColor(SSD1306_WHITE);
  display.println(checkWalls);
  display.display();*/

  switch(checkWalls)
  {
    case 0: // Cruce en X.
      myRobot.avanzaPoco();
      //myRobot.girar90(RIGHT);
      myRobot.resetEncoders(); 


      myRobot.girar(90);
      myRobot.avanzaPoco();
      myRobot.avanzaPoco();
      myRobot.resetEncoders(); 
      break;
    case 1:
      myRobot.avanzar();
      myRobot.resetEncoders(); 
     break;
    case 2: 
      myRobot.avanzar();
      myRobot.resetEncoders(); 
      break;
    case 3:
      myRobot.avanzarLaberinto();

     break;
    case 4: // Cruce en T. 
      //myRobot.avanzaPoco();
      delay(500);
      myRobot.resetEncoders();  
      //myRobot.girar90(RIGHT);
      myRobot.girar(90);
      myRobot.avanzaPoco();
      myRobot.avanzaPoco();
      myRobot.resetEncoders(); 
      break;

    case 5: // Giro a derechas.
      //myRobot.avanzaPoco();
      //myRobot.girar90(RIGHT);
      delay(500);
      myRobot.resetEncoders(); 
      myRobot.girar(90);
      myRobot.avanzaPoco();
      myRobot.avanzaPoco();
      myRobot.resetEncoders(); 
      break;
    case 6: // Giro a izquierdas.
      //myRobot.avanzaPoco();
      //myRobot.girar90(LEFT);
      delay(500);
      myRobot.resetEncoders(); 
      myRobot.girar(-90);
      myRobot.avanzaPoco();
      myRobot.avanzaPoco();
      myRobot.resetEncoders(); 
      break;
    case 7: // Hoja
      //myRobot.avanzaPoco();
      delay(500);
      myRobot.resetEncoders(); 
      myRobot.girar(90);
      myRobot.resetEncoders(); 
      myRobot.girar(90);
      myRobot.resetEncoders(); 
      //myRobot.girar90(RIGHT);
      //myRobot.girar90(RIGHT);
      //myRobot.avanzaPoco();
     // myRobot.avanzaPoco();


      break;
  }
  #endif

  

  
}

  




  
  


    
