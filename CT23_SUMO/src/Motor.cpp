
// Provides low-level control of a the speed and direction of a single DC motor
// by means of a motor driver such as the TB6612FNG

#include <Arduino.h>
#include "Motor.h"

/**
 * @brief Construct a new Motor:: Motor object
 * 
 * @param dir1 Direction control 1.
 * @param pwm  Speed control.
 */
Motor::Motor(int dir1, int pwm)
{
	_dir = dir1;
	_pwm = pwm;
}

/**
 * @brief Initializes the motor pins as I/O.
 * 
 */
void Motor::init()
{
	pinMode(_dir, OUTPUT);
	pinMode(_pwm, OUTPUT);
}

// sets the motor's direction to forward
void Motor::setFwd()
{
	digitalWrite(_dir, HIGH);
}

// sets the motor's direction to backward
void Motor::setBack()
{
	digitalWrite(_dir, LOW);
}

// sets the motor to freewheel
void Motor::setFree()
{
	digitalWrite(_dir, LOW);
	analogWrite(_dir, 0);
}

// sets the motor to brake
void Motor::setStop()
{
	digitalWrite(_dir, HIGH);
	analogWrite(_dir, 5);
}

// accepts an int, the PWM level, as a parameter
// sets the PWM output to the motor to the given int
// level must be between 0 and 255 inclusive
// behavior is undefined if level is outside this range
void Motor::setPWM(int level)
{
	analogWrite(_pwm, level);
}
