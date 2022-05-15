#include <Arduino.h>

#define FORWARD -1
#define BACKWARD 1
#define STOP 0

void setMotor(int dir, int pwmVal, int pwm, int in1, int in2)
{
	analogWrite(pwm, pwmVal);

	if (dir == FORWARD)
	{
		digitalWrite(in1, HIGH);
		digitalWrite(in2, LOW);
	}
	else if (dir == BACKWARD)
	{
		digitalWrite(in1, LOW);
		digitalWrite(in2, HIGH);
	}
	else if(dir == STOP)
	{
		digitalWrite(in1, LOW);
		digitalWrite(in2, LOW);
	}
}

