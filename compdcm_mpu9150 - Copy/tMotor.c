#include <stdint.h>
#include <stdbool.h>
#include "tMotor.h"

void initMotor(tMotor *motorInst, int motorLeft, int motorRight)
{
	//Initial motor right speed
	motorInst->motorLeft = motorLeft;
	
	//Initial motor left speed
	motorInst->motorRight = motorRight;
}
