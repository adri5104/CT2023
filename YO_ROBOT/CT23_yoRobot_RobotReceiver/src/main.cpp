#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <espnow.h>
#include <SoftwareSerial.h>

// Wire Slave Sender
// by Nicholas Zambetti <http://www.zambetti.com>

// Demonstrates use of the Wire library
// Sends data as an I2C/TWI slave device
// Refer to the "Wire Master Reader" example for use with this

// Created 29 March 2006

// This example code is in the public domain.


#include <Wire.h>
SoftwareSerial mySUART(4, 5);  //D2, D1 = SRX, STX

float voltage = 0.0F;
char voltBuff[7];


int RXp2  =13;
int TXp2 =15;
char op = 'a';

// direccion mac del server
uint8_t  broadcasAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
String success;
const int DEVICE = 8;
const int VEC_MAX = 2;
int vec[VEC_MAX] = {0, 0};

// tipo del mensaje
typedef struct data_message
{
  float accelX;
  float accelY;
} struct_message;

// mensaje
struct_message message;

// callback function executed when data is received
void OnRecv( uint8_t * mac,  uint8_t *incomingData, uint8_t len) {
  memcpy(&message, incomingData, sizeof(message));
   /* Print out the values */
  //Serial.print("AccelX:");
  //Serial.print(message.accelX);
  //Serial.print(",");
  //Serial.print("AccelY:");
  //Serial.print(message.accelY);
  //Serial.print(",");  
  //Serial.println("");

  
}






void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);
  mySUART.begin(115200);
  Serial.println("HOLA");
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  // Init ESP-NOW
  esp_now_init(); 
  
  // Once the ESP-Now protocol is initialized, we will register the callback function
  // to be able to react when a package arrives in near to real time without pooling every loop.
  esp_now_set_self_role(ESP_NOW_ROLE_SLAVE);
  esp_now_register_recv_cb(OnRecv);


 
}

void loop() 
{
  
  String dataString =  "101," + String(message.accelX > 100? 100 : message.accelX) + "," + String(message.accelY > 100 ? 100 : message.accelY);
  Serial.println(dataString);
  mySUART.println(dataString);
  delay(50);
}