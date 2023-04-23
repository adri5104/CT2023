

#ifndef Motor_h
#define Motor_h

#include "Arduino.h"

class Motor
{
public:
	Motor(int dir, int pwm);
	void init();
	void setFwd();
	void setBack();
	void setFree();
	void setStop();
	void setPWM(int level);
private:
	int _dir, _pwm;
};

#endif