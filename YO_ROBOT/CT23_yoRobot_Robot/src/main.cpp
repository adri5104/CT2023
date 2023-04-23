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

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Encoder_p encoderL = Encoder_p(ENC_LA, ENC_LB, 4.0);
Encoder_p encoderR = Encoder_p(ENC_RA, ENC_RB, 4.0);
Motor motorL = Motor(DIR_LA, DIR_LB, PWM_L);
Motor motorR = Motor(DIR_RA, DIR_RB, PWM_R);
TofSensors mySensors(XSH_1, XSH_2, XSH_3);

const int DEVICE = 8;
const int VEC_MAX = 2;
float vec[VEC_MAX] ;



// Funciones callback para las interrupciones de los encoders
void header_encoderL()
{
  encoderL.actualizar_posicion();
}

void header_encoderR()
{
  encoderR.actualizar_posicion();
}



void setup() {


 
  Serial.begin(9600);
  Serial.println("HOLA");

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
  

  // ---------------------- Inicializaciones encoders -------
  
  encoderL.init();
  encoderR.init();
  attachInterrupt(digitalPinToInterrupt(ENC_LA),header_encoderL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_RA),header_encoderR, CHANGE);

  // ---------------------- Inicializaciones motores -------
  
  motorL.init();
  motorR.init();

  

  

  display.clearDisplay();
  display.drawBitmap(
  (display.width()  - LOGO_WIDTH ) / 2,
  (display.height() - LOGO_HEIGHT) / 2,
  mojojojo, LOGO_WIDTH, LOGO_HEIGHT, 1);
  display.display();

  delay(2000);
  display.clearDisplay();
  display.display();

  // Serial 2

  Serial1.begin(115200);
} 


float accel[2] = {-1,-1};
uint8_t velD = 0;
uint8_t velI = 0;
bool marcha_atras = false;

void loop() {
  motorL.setFwd();
  motorR.setFwd();
  motorL.setPWM(255);
  motorR.setPWM(255);

  
  int aux;
  if(Serial1.available())
  { 
    
    aux = Serial1.parseInt();
    if(aux == 101)
    {
    accel[0] = Serial1.parseFloat();
    accel[1] = Serial1.parseFloat();
    Serial.print(accel[0]);
    Serial.print("'");
    Serial.println(accel[1]);
    }
  }

  

  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextColor(SSD1306_WHITE);
  String disp =  String(accel[0]) + " | " + String(accel[1]);
  display.println(disp);
  display.display();

  marcha_atras = accel[0] < 0? true : false;
  uint8_t aux1 =  map(abs(accel[0]), 0, 100, 0, VEL_MAX);
  uint8_t vel_base = aux1 > VEL_MAX ? VEL_MAX : aux1   ;

  
  
  delay(10);
  
}