#include <stdint.h>
#include <stdbool.h>

#define MAXSPEED 1250
#define MINSPEED 625

typedef struct
{
	//These two motors should correspond to the motors on the white arms
	//These should have a value ranging as a percentage 
	int motorRight;
	int motorLeft;
  int motorFront;
	int motorBack;
	
	int currentSpeed;
	
}
tMotor;

void motorInit(tMotor *motorInst, int motorLeft, int motorRight);
