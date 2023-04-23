#include "TofSensors.h"

TofSensors::TofSensors(int xshut_front, int xshut_right, int xshut_left)
{
    myXSHUTpins[FRONT] = xshut_front;
    myXSHUTpins[RIGHT] = xshut_right;
    myXSHUTpins[LEFT] = xshut_left;
    myTofSensors[FRONT] = new Adafruit_VL53L0X();
    myTofSensors[LEFT] =new   Adafruit_VL53L0X();
    myTofSensors[RIGHT] =new  Adafruit_VL53L0X();
}

void TofSensors::setID(uint8_t adress_front, uint8_t adress_right, uint8_t adress_left)
{
    //todos reset
    digitalWrite(myXSHUTpins[FRONT], LOW);
    digitalWrite(myXSHUTpins[RIGHT], LOW);                  
    digitalWrite(myXSHUTpins[LEFT], LOW);
    delay(10);

    //todos unreset
    digitalWrite(myXSHUTpins[FRONT], HIGH);
    digitalWrite(myXSHUTpins[RIGHT], HIGH);
    digitalWrite(myXSHUTpins[LEFT], HIGH);
    delay(10);
                    
    //activamos front y reset right y left
    digitalWrite(myXSHUTpins[FRONT], HIGH);                  
    digitalWrite(myXSHUTpins[RIGHT], LOW);
    digitalWrite(myXSHUTpins[LEFT], LOW);                  


    Serial.println("xd");
    
    //Inicializamos front
    if(!myTofSensors[FRONT]->begin(adress_front)) {
    Serial.println(F("Failed to boot front VL53L0X"));
    while(1);}
    Serial.println("Sensor front init");

    //Activamos right
    digitalWrite(myXSHUTpins[RIGHT], HIGH);
    delay(10);

    //Inicializamos right
    if(!myTofSensors[RIGHT]->begin(adress_right)) {
    Serial.println(F("Failed to boot right VL53L0X"));
    while(1);}
    Serial.println("Sensor right init");

    //Activamos left 
    digitalWrite(myXSHUTpins[LEFT], HIGH);
    delay(10);

    //Inicializamos left
    if(!myTofSensors[LEFT]->begin(adress_left)) {
    Serial.println(F("Failed to boot left VL53L0X"));
    while(1);}
    Serial.println("Sensor left init");
    
}

void TofSensors::init()
{
    Serial.print("A");
    pinMode(myXSHUTpins[FRONT], OUTPUT);
    pinMode(myXSHUTpins[RIGHT], OUTPUT);
    pinMode(myXSHUTpins[LEFT], OUTPUT);
    digitalWrite(myXSHUTpins[FRONT], LOW);
    digitalWrite(myXSHUTpins[RIGHT], LOW);
    digitalWrite(myXSHUTpins[LEFT], LOW);
    delay(10);   
}

void TofSensors::printMeasurements()
{
    int i;
    for(i = 0; i< 3; i++)
        myTofSensors[i]->rangingTest(&myMeasurements[i], false);
        
    for(i = 0; i< 3; i++)
    {
        Serial.print(i + ": ");
        if(myMeasurements[i].RangeStatus != 4) {     
            
            Serial.print(myMeasurements[i].RangeMilliMeter);
            Serial.print("mm");    
        } else {
            Serial.print("Out of range");
        }
        Serial.print(" ");
    }
    Serial.println("");
}

int* TofSensors::measureDistance(int* par)
{
    int i;
    for(i = 0; i< 3; i++)
        myTofSensors[i]->rangingTest(&myMeasurements[i], false);
        
    for(i = 0; i< 3; i++)
    {
        if(myMeasurements[i].RangeStatus != 4) {     
            
            distance[i] = myMeasurements[i].RangeMilliMeter;
            par[i] = distance[i];
        } else {
            distance[i] = -1;
        }   
    }  


    return distance; 
}

void TofSensors::getDistance(int* pos)
{
    this->measureDistance(pos);
    
}
