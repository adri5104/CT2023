#ifndef TOFSENSORS_
#define TOFSENSORS_



#include <Arduino.h>

#include<Adafruit_VL53L0X.h>

#define FRONT 2
#define LEFT 0
#define RIGHT 1

class TofSensors
{
    private:
        Adafruit_VL53L0X* myTofSensors[3];
        int myXSHUTpins [3];
        VL53L0X_RangingMeasurementData_t myMeasurements[3];
        int distance[3];
        int* measureDistance(int*);
       
        
    public:
        //TofSensors();
        TofSensors(int, int, int);
       
        //Inicializa los pines para los xshut 
        void init();
        //uint16_t getDistanceMM(int);
        
        //Imprime por serial las distancias medidas
        void printMeasurements();
        
        //Establece las dirreciones de los 3 sensores
        //e inicializa la comunicacion i2c. 
        //Como parametros se reciben las 3 direcciones en formato
        //hexadecimal (FRONT, RIGHT, LEFT)
        void setID(uint8_t, uint8_t, uint8_t);
        
        //Devuelve la distancia del sensor dado como parametro (FRONT, LEFT, RIGHT)
        void getDistance(int*);

};
#endif