#include "motor.h"
#include <Arduino.h>

#define ENCA 2
#define ENCB 3

#define IN1 6
#define IN2 7

#define PWM 5

#define BTN_IN 10
#define BTN_OUT 11

#define POW_IN 12
#define POW_OUT 13

long pos = 0;
long previousTime = 0;
float errorPrevious = 0;
float integral = 0;

bool isOpen = false;
bool isRunning = false;

void readEncoder()
{
	int b = digitalRead(ENCB);

	if (b > 0)
		pos++;
	else
		pos--;
}

void PID()
{
	int target = 10000;

	// PID constants
	float kp = 0.5;
	float ki = 0;
	float kd = 0.08;

	// time difference
	long currentTime = micros();
	float deltaT = ((float)(currentTime - previousTime)) / 1.0e6;
	previousTime = currentTime;

	// error
	float error = -target + pos;

	// integral
	integral += error * deltaT;

	// derivative
	float derivative = (error - errorPrevious) / deltaT;

	// output
	float output = kp * error + ki * integral + kd * derivative;

	// motor power
	float pwr = fabs(output);

	if (pwr > 255)
		pwr = 255;

	// motor direction
	int dir = output >= 0 ? 1 : -1;

	if (abs(target - pos) < (0.01 * target))
	{
		pwr = 0;
		dir = 0;
	}

	// signal the motor
	setMotor(dir, pwr, PWM, IN1, IN2);

	// store prev error
	errorPrevious = error;

	Serial.print(target);
	Serial.print(" ");
	Serial.print(pos);
	Serial.println();
}

void setup()
{
	Serial.begin(9600);

	pinMode(IN1, OUTPUT);
	pinMode(IN2, OUTPUT);

	pinMode(PWM, OUTPUT);

	pinMode(ENCA, INPUT);
	pinMode(ENCB, INPUT);

	pinMode(BTN_IN, INPUT_PULLUP);
	pinMode(BTN_OUT, OUTPUT);

	pinMode(POW_IN, INPUT_PULLUP);
	pinMode(POW_OUT, OUTPUT);

	attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);
}

void loop()
{
	// PID();

	// digitalWrite(BTN_OUT, LOW);

	// if the power button is turned off the motor will not be available
	if (digitalRead(POW_IN) == HIGH)
	{
		setMotor(STOP, 0, PWM, IN1, IN2);

		return;
	}
	
	if (digitalRead(BTN_IN) == LOW)
	{
		isRunning = !isRunning;
	}

	if (isRunning)
	{
		if (!isOpen)
		{
			while (pos < 10000)
			{
				setMotor(BACKWARD, 255, PWM, IN1, IN2);

				Serial.print("backward ");
				Serial.println(pos);
			}
			isOpen = true;
		}
		else
		{
			while (pos > 0)
			{
				setMotor(FORWARD, 255, PWM, IN1, IN2);

				Serial.print("forward ");
				Serial.println(pos);
			}
			isOpen = false;
		}

		isRunning = false;
	}
	else
	{
		setMotor(STOP, 0, PWM, IN1, IN2);
	}
}
