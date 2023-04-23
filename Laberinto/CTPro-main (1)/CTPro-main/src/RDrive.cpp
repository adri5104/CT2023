#include  "CTPro.h"
#include  "RDrive.h"

using namespace RDRIVE; 

RDrive::RDrive()
{
    
    
}

RDrive::RDrive(Motor* motor1, Motor* motor2, Encoder_p* encoder1, Encoder_p* encoder2, TofSensors* sensors)
{
    misMotores[0] = motor1;
    misMotores[1] = motor2;
    misEncoder[0] = encoder1;
    misEncoder[1] = encoder2;
    mySensors = sensors;

    input = 0;
    setpoint = 0;
    //output = 0;
    kp = 0.5;
    ki = 0.0;
    kd = 0.0;
    myPID = new PID(&input, &output, &setpoint, kp, ki, kd, DIRECT);
    myPID->SetMode(AUTOMATIC);
    myPID->SetTunings(kp, ki, kd);
    myPID->SetSampleTime(0.001);
    myPID->SetOutputLimits(-255, 255);
}

void RDrive::avanzar()
{
    misMotores[LEFT]->setPWM(vel_base);
    misMotores[RIGHT]->setPWM(vel_base);
    misMotores[LEFT]->setFwd();
    misMotores[RIGHT]->setFwd();
}

void RDrive::parar()
{
    misMotores[LEFT]->setStop();
    misMotores[RIGHT]->setStop();
}

void RDrive::seguirPared()
{
    //TODO
}

void RDrive::rotar(const DIRECCION dir)
{
    if(dir == DERECHA)
    {
        misMotores[LEFT]->setPWM(vel_base);
        misMotores[RIGHT]->setPWM(vel_base);
        misMotores[LEFT]->setFwd();
        misMotores[RIGHT]->setBack();
    }
    else
    {
        misMotores[LEFT]->setPWM(vel_base);
        misMotores[RIGHT]->setPWM(vel_base);
        misMotores[LEFT]->setBack();
        misMotores[RIGHT]->setFwd();
    }
}

void RDrive::setPosition(float setpoint_l, float setpoint_r)
{
  float gradosl, gradosr;
  float error_l, error_r;
  float error_l_prev, error_r_prev;
  float error_l_integral = 0.0, error_r_integral = 0.0;
  float error_l_deriv = 0.0, error_r_deriv = 0.0;
  float kp = 0.5, ki = 0.0005, kd = 0.35;    //ki=0.0005

  float vel_l, vel_r;
  
  gradosl = misEncoder[LEFT]->getPosicionGrados();
  gradosr = misEncoder[RIGHT]->getPosicionGrados();

  error_l = setpoint_l - gradosl;
  error_r = setpoint_r - gradosr;

  error_l_prev = error_l;
  error_r_prev = error_r;
    
  while((error_l > 1.0 || error_l< -1.0) || (error_r > 1.0 || error_r < -1.0))
  {
    // Lectura
    gradosl = misEncoder[LEFT]->getPosicionGrados();
    gradosr = misEncoder[RIGHT]->getPosicionGrados();

    // Cálculos
    error_l = setpoint_l - gradosl;
    error_l_integral += error_l;
    error_l_deriv = error_l - error_l_prev;

    error_r = setpoint_r - gradosr;
    error_r_integral += error_r;
    error_r_deriv = error_r - error_r_prev;

    vel_l = kp * error_l + ki * error_l_integral + kd * error_l_deriv;
    vel_r = kp * error_r + ki * error_r_integral + kd * error_r_deriv;

    // Motor Izquierdo
    if (vel_l > 0.0)
    {
      misMotores[LEFT]->setFwd();
      misMotores[LEFT]->setPWM(vel_l > VEL_MAX_ENC? VEL_MAX_ENC : vel_l);
    }
    else{
      misMotores[LEFT]->setBack();
      misMotores[LEFT]->setPWM(-vel_l > VEL_MAX_ENC? VEL_MAX_ENC : -vel_l);
    }

    // Motor Derecho
    if (vel_r>0.0)
    {
      misMotores[RIGHT]->setFwd();
      misMotores[RIGHT]->setPWM(vel_r > VEL_MAX_ENC? VEL_MAX_ENC : vel_r);
    }
    else
    {
      misMotores[RIGHT]->setBack();
      misMotores[RIGHT]->setPWM(-vel_r > VEL_MAX_ENC? VEL_MAX_ENC : -vel_r);
    }
  }

  misMotores[LEFT]->setFree();
  misMotores[RIGHT]->setFree();
}

float RDrive::girar(float beta)
{
    float phi = ROBOT_WIDTH*beta/ROBOT_RADIUS;
    this->setPosition(phi, -phi);
    return phi;
}

void RDrive::girar90(int dir)
{
    switch(dir)
    {
        case 0 :// LEFT
            misMotores[LEFT]->setBack();
            misMotores[RIGHT]->setFwd();
            misMotores[LEFT]->setPWM(50);
            misMotores[RIGHT]->setPWM(50);
            delay(180);
            break;
            
        case 1 :// RIGHT
            misMotores[LEFT]->setFwd();
            misMotores[RIGHT]->setBack();
            misMotores[LEFT]->setPWM(50);
            misMotores[RIGHT]->setPWM(50);
            delay(120);
            break;
    }

    
    misMotores[LEFT]->setFree();
    misMotores[RIGHT]->setFree();
    delay(1000);
}

void RDrive::avanzaPoco()
{
    misMotores[LEFT]->setFwd();
    misMotores[RIGHT]->setFwd();

    misMotores[LEFT]->setPWM(VEL_MAX_ENC);
    misMotores[RIGHT]->setPWM(VEL_MAX_ENC);
    delay(200);
    misMotores[LEFT]->setFree();
    misMotores[RIGHT]->setFree();
}

void RDrive::avanzarLaberinto()
{
    vel_i = vel_base + output;
    vel_d = vel_base - output;

    misMotores[LEFT]->setPWM(abs(vel_i) > 120 ? 120 : abs(vel_i));
    misMotores[RIGHT]->setPWM(abs(vel_d) > 120 ? 120 : abs(vel_d));



    if (vel_i > 0)
    {
        misMotores[LEFT]->setFwd();
        
    }
    else
    {
        misMotores[LEFT]->setBack();
    }
    
    if (vel_d > 0)
    {
        misMotores[RIGHT]->setFwd();
    }
    else
    {
        misMotores[RIGHT]->setBack();
    }
}

void RDrive::compute(int* distances_)
{
    int i;
    
    mySensors->getDistance(myDistance);

    for (i = 0; i< 3; i++)

    {
        distances_[i] = myDistance[i];
    }

    if(myModo == MANTENERSE_CENTRADO)
        input = myDistance[RIGHT] - myDistance[LEFT];
    
    else
        input = myDistance[RIGHT];    
    
    myPID->Compute();
}

void RDrive::setMode(const MODE modo)
{
    if(modo == MANTENERSE_CENTRADO)
    {
        setpoint = 0;

    }
    else
    {
        setpoint = distancia_pared;
    }
    myModo = modo;
}

int RDrive::checkWalls()
{
    int dL = myDistance[LEFT], dR = myDistance[RIGHT], dF = myDistance[FRONT];

    if(dL <= LAB_WIDTH)
    {
        if(dR <= LAB_WIDTH)
        {
            if(dF <= LAB_DISTANCE_FRONT)
                return 7;
            else
                return 3;
        }
        else
        {
            if(dF <= LAB_DISTANCE_FRONT)
                return 6;
            else
                return 2;
        }
    }
    
    else if(dR <= LAB_WIDTH)
    {
            if(dF <= LAB_DISTANCE_FRONT)
                return 5;
            else
                return 1;
    }    
    
    else if(dF <= LAB_DISTANCE_FRONT)
    {
        return 4;
    }

    else
    {
        return 0;
    }
}