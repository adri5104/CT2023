// LUCIA, ADRI Y JAIME ---> Zomo lo mejore der mundo.

#ifndef __CTPRO__
    #define __CTPRO__

    #include <Arduino.h>

    // Dimensions
    #define ROBOT_WIDTH 83.0
    #define ROBOT_RADIUS 30.0

    // I2C
    #define SDA 21
    #define SCL 20

    // INTERRUPCIONES
    #define ENC_LA 19
    #define ENC_LB 18
    #define ENC_RA 2
    #define ENC_RB 3

    // PWM
    #define PWM_L 12
    #define PWM_R 4

    // DIGITALES
    #define BTN_1 40
    #define BTN_2 38
    #define BTN_3 36
    #define XSH_1 15
    #define XSH_2 16
    #define XSH_3 17
    #define DIR_LB 14
    #define DIR_LA 10
    #define DIR_RA 6
    #define DIR_RB 8


    // OLED	
    #define SCREEN_WIDTH 128 // OLED display width, in pixels
    #define SCREEN_HEIGHT 64 // OLED display height, in pixels
    #define SCREEN_ADDRESS 0x3C // OLED I2C address
    #define OLED_RESET -1 // Reset pin # (or -1 if sharing Arduino reset pin)


    // DISTANCIAS
    #define LAB_DISTANCE_FRONT 60
    #define LAB_DISTANCE_LEFT 120
    #define LAB_DISTANCE_RIGHT 120
    

    #define LAB_WIDTH 168.0
    #define LAB_ADVANCE 20.0


    // ---------------------- 

    enum state
    {
        S_INIT,
        S_AVANZANDO,
        S_VOLVIENDO,
        S_HUECO_IZQ,
        S_HUECO_DER,
        S_ALGO_DELANTE
    };

    // ---------------------- PID
    /*const double kP = 0.5;
    const double kI = 0.0;
    const double kD = 0.0;*/

    const double kP = 0.1; //0.04
    const double kI = 0.00;
    const double kD = 0.02; //0.02

    const int velocidad_base = 30; //90
   
    //Direcciones memoria sensores tof
    const uint8_t TOF_SENSOR_FRONT  = 0x30;
    const uint8_t TOF_SENSOR_LEFT = 0x31;
    const uint8_t TOF_SENSOR_RIGHT = 0x32;
    

#endif