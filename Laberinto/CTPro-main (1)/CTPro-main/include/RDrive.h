#ifndef RDRIVE_H
#define RDRIVE_H

#include <Arduino.h>
#include"TofSensors.h"
#include"PID_v1.h"
#include "Motor.h"
#include "TofSensors.h"
#include "Encoder.h"

namespace RDRIVE
{

    const int distancia_pared = 100;    
    #define VEL_MAX_ENC 25

    enum DIRECCION
    {
        DERECHA,
        IZQUIERDA
    };

    enum MODE
    {
        MANTENERSE_CENTRADO,
        SEGUIR_PARED_DERECHA
    };
    

    class RDrive
    {
        private:
            int vel_base;
            int vel_d;
            int vel_i;

            double kp;
            double ki;
            double kd;

            double input;
            double output;
            double setpoint;

            PID* myPID;

            Motor* misMotores[2];
            Encoder_p* misEncoder[2];
            TofSensors* mySensors;

            int myDistance[3];

            MODE myModo;


        
        public:

            RDrive();

            // Constructor con parametros (Motor de la izquierda, Motor de la derecha, Encoder de la izquierda, Encoder de la derecha, Sensores de distancia)
            RDrive(Motor*, Motor*, Encoder_p*, Encoder_p*, TofSensors*);
            
            // Inicializa el pid y algunas cosas mas
            void init();

            //Avanza sin mas
            void avanzar();

            //Avanzar distancia
            void avanzarDistancia(float distance)
            {
                float angle = 360.0*distance/(ROBOT_RADIUS*PI);
                this->setPosition(angle, angle);
            }

            void resetEncoders()
            {
                misEncoder[LEFT]->resetPosicion();
                misEncoder[RIGHT]->resetPosicion();
            }

            //Sigue la pared derecha
            void seguirPared();

            //Avanza manteniendo la misma distancia con las dos paredes 
            void avanzarLaberinto();

            //Devuelve el PID
            PID& getPID()
            {
                return *myPID;
            }

            Encoder_p*& getEncoders(){return *misEncoder;};

            //Para el robot
            void parar();

            //Logica
            void compute(int*);

            //Setters
            void setVelBase(int a){vel_base = a;};


            // Hace que el robot gire indefinidamente en la direccion indicada
            void rotar(const DIRECCION);
            
            // Gira X grados sobre sí mismo
            float girar(float beta);

            void girar90(int dir);

            void avanzaPoco();

            void setPosition(float setpoint_l, float setpoint_r);

            // Setea el modo de operacion (seguir una pared o mantenerse centrado)
            void setMode(const MODE);

            //
            float getOutput(){return output;};

            int getVelI(){return vel_i;};

            int getVelD(){return vel_d;};

            /**
             * @brief Checks the surrounding walls, separated a LAB_WIDTH distance, and returns:
             * 0. There are no Walls.
             * 1. Detects only LEFT wall.
             * 2. Detects only RIGHT wall.
             * 3. Detects both LEFT and RIGHT wall.
             * 4. Detects only FRONT wall.
             * 5. Detects LEFT and FRONT wall.
             * 6. Detects RIGHT and FRONT wall.
             * 7. Detects LEFT, RIGHT and FRONT wall.
             * 
             * @return int 
             */
            int checkWalls();
    };
}


#endif