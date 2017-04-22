#include <stdint.h>
#include <stdbool.h>
#include "tMotor.h"

typedef struct
{
	//Set Point value input to the PID
	float spEuler;
	float spGyro;
	
	//The Process variable i.e. the feedback value attained from the sensor
	float pvEuler;
	float pvGyro;
	
	
	//Manipulated Variable. The value that will be use to set the new PWM duty cycle to drive the motors for stabilization
	float mvEuler;
	float mvGyro;
	
	float errorEuler;
	float integralErrorEuler;
	float p_errorEuler;
	float errorGyro;
	
	float Integral;
	
}
tPID;

typedef struct
{
	float KpEuler;
	float KpGyro;
	
	float KiEuler;
	float KiGyro;
	
	float KdEuler;
	float KdGyro;
	
}
pidConstants;

//Function prototype for setting the initial roll set point (Roll Calibration)
void PID_InitRoll_SetPoint(tPID *pidInstance, float rollSetPoint);
void PID_InitPitch_SetPoint(tPID *pidInstance, float pitchSetPoint);

void updateCopterRoll(tPID *pidInstance, tMotor *motorInstance, pidConstants *rollConsts);
void updateCopterPitch(tPID *pidInstance, tMotor *motorInstance, pidConstants *pitchConsts);

void initPidConstants(pidConstants *consts);

void updateProcessVar(tPID *pidInst, float pvEulerValue, float pvGyroValue);




