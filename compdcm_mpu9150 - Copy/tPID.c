#include <stdint.h>
#include <stdbool.h>
#include "tPID.h"

//Degree ranges for the roll angles of the sensor
//Angles go from -180 to 180 degrees
float ROLL_FULL	= 360;
float ROLL_HALF = 180;
float ROLL_STABLE_ANGLE = 0;
float PITCH_FULL = 180;
float PITCH_HALF = 90;
float PITCH_STABLE_ANGLE = 0;
float PPM_RESOLUTION = 625; // 1250-625 = 625 Range from largest to smallest pwm signal to esc's

//This function will set the roll set point for the PID 
void PID_InitRoll_SetPoint(tPID *pidInstance, float rollSetPoint)
{

	pidInstance->spEuler = (ROLL_STABLE_ANGLE+ROLL_HALF)/ROLL_FULL; //Keep setpoint between 0 and 1. .5 ~ stable setpoint angle
	pidInstance->spGyro = 0;
	
	pidInstance->integralErrorEuler = 0;
 
}

void PID_InitPitch_SetPoint(tPID *pidInstance, float pitchSetPoint)
{
	pidInstance->spEuler = (PITCH_STABLE_ANGLE+PITCH_HALF)/PITCH_FULL;
	
	pidInstance->integralErrorEuler = 0;
}

void updateCopterPitch(tPID *pidInstance, tMotor *motorInstance, pidConstants *rollConsts)
{
	// u(t) = Kp*e(t) + Ki*[sum(e(t)dt)] + Kd*[e(t)/dt]
	
	//float error,p_error;
	
	
	
	pidInstance->errorEuler = pidInstance->pvEuler-pidInstance->spEuler;  // error = pv - sp
	pidInstance->integralErrorEuler += pidInstance->errorEuler;
	//pidInstance->p_errorEuler = pidInstance->errorEuler;
	pidInstance->errorGyro = pidInstance->pvGyro - pidInstance->spGyro; //Error in angular speed (deg/s)
	
	pidInstance->Integral = rollConsts->KiEuler*pidInstance->integralErrorEuler; //Ki*sum(e(t))
	

	//Checking the Integral wind up to make sure it does not get out of control
	if(pidInstance->Integral > 1)
	{
		pidInstance->Integral = 1;
	}
	else if(pidInstance->Integral < 0)
	{
		pidInstance->Integral = 0;
	}
			
	
	pidInstance->mvEuler = rollConsts->KpEuler*pidInstance->errorEuler + pidInstance->Integral; // Derivative: rollConsts->Kd*(pidInstance->error - pidInstance->p_error)(add dt);
	pidInstance->mvGyro = rollConsts->KpGyro*pidInstance->errorGyro;
	
	// This checks that mv will not set a value that will make the motor go past its max speed
	if(pidInstance->mvEuler > ((MAXSPEED - motorInstance->currentSpeed)/PPM_RESOLUTION))
	{
		pidInstance->mvEuler = (MAXSPEED-motorInstance->currentSpeed)/PPM_RESOLUTION;
	}
	
	// For Derivative: pidInstance->p_error = pidInstance->error;
	//Perspective of quadcopter is looking at the copter from the front side
	//TM4C JTAG usb should be considered the left side
	//Meaning whenever it moves right the Euler angles are positive
	//When it moves left the Euler angles are negative
	if(pidInstance->errorEuler < 0)
	{
		motorInstance->motorFront = motorInstance->currentSpeed - ((pidInstance->mvEuler*PPM_RESOLUTION));//
		motorInstance->motorBack = motorInstance->currentSpeed;
	}
	else
	{
		motorInstance->motorBack = motorInstance->currentSpeed + (pidInstance->mvEuler*PPM_RESOLUTION); //
		motorInstance->motorFront = motorInstance->currentSpeed;
	}
		
	
	//motorInstance->motorLeft = motorInstance->currentSpeed - pidInstance->mvGyro;
	
	
	//motorInstance->motorRight = motorInstance->currentSpeed + pidInstance->mvGyro;
}

//Function updates a value used for pwm duty cycle
//Duty Cycle values: 625 = 5% duty cycle for a 50Hz pwm signal but 0% speed for the motor and is the minimum for the motor(No Movement)
//                   1250 = 10% duty cycle for a 50Hz signal and 100% speed for the motor(Top Speed)
void updateCopterRoll(tPID *pidInstance, tMotor *motorInstance, pidConstants *rollConsts)
{
	// u(t) = Kp*e(t) + Ki*[sum(e(t)dt)] + Kd*[e(t)/dt]
	
	//float error,p_error;
	
	
	
	pidInstance->errorEuler = pidInstance->pvEuler-pidInstance->spEuler;  // error = pv - sp
	pidInstance->integralErrorEuler += pidInstance->errorEuler;
	//pidInstance->p_errorEuler = pidInstance->errorEuler;
	pidInstance->errorGyro = pidInstance->pvGyro - pidInstance->spGyro; //Error in angular speed (deg/s)
	
	pidInstance->Integral = rollConsts->KiEuler*pidInstance->integralErrorEuler; //Ki*sum(e(t))
	

	//Checking the Integral wind up to make sure it does not get out of control
	if(pidInstance->Integral > 1)
	{
		pidInstance->Integral = 1;
	}
	else if(pidInstance->Integral < 0)
	{
		pidInstance->Integral = 0;
	}
			
	
	pidInstance->mvEuler = rollConsts->KpEuler*pidInstance->errorEuler + pidInstance->Integral; // Derivative: rollConsts->Kd*(pidInstance->error - pidInstance->p_error)(add dt);
	pidInstance->mvGyro = rollConsts->KpGyro*pidInstance->errorGyro;
	
	// This checks that mv will not set a value that will make the motor go past its max speed
	if(pidInstance->mvEuler > ((MAXSPEED - motorInstance->currentSpeed)/PPM_RESOLUTION))
	{
		pidInstance->mvEuler = (MAXSPEED-motorInstance->currentSpeed)/PPM_RESOLUTION;
	}
	
	// For Derivative: pidInstance->p_error = pidInstance->error;
	//Perspective of quadcopter is looking at the copter from the front side
	//TM4C JTAG usb should be considered the left side
	//Meaning whenever it moves right the Euler angles are positive
	//When it moves left the Euler angles are negative
	if(pidInstance->errorEuler < 0)
	{
		motorInstance->motorLeft = motorInstance->currentSpeed - ((pidInstance->mvEuler*PPM_RESOLUTION));//
		motorInstance->motorRight = motorInstance->currentSpeed;
	}
	else
	{
		motorInstance->motorRight = motorInstance->currentSpeed + (pidInstance->mvEuler*PPM_RESOLUTION); //
		motorInstance->motorLeft = motorInstance->currentSpeed;
	}
		
	
	//motorInstance->motorLeft = motorInstance->currentSpeed - pidInstance->mvGyro;
	
	
	//motorInstance->motorRight = motorInstance->currentSpeed + pidInstance->mvGyro;
	
			
			if(motorInstance->motorLeft > MAXSPEED)
			{
				motorInstance->motorLeft = motorInstance->currentSpeed; //If we increase past the maximum turn off motors
							
			}
			else if(motorInstance->motorRight > MAXSPEED)
			{
				motorInstance->motorRight = motorInstance->currentSpeed; //Turn off motor if past max speed value
			}
			else if(motorInstance->motorLeft < MINSPEED)
			{
				motorInstance->motorLeft = MINSPEED;
			}
			else if(motorInstance->motorRight < MINSPEED)
			{
				motorInstance->motorRight = MINSPEED;
			}
	
	
}

void initPidConstants(pidConstants *consts)
{
	consts->KpEuler = .57;
	consts->KiEuler = .0017;
	consts->KpGyro = 0;
	
	consts->KdEuler = 0;
	
	
}
//Using only for roll as of now
void updateProcessVar(tPID *pidInst, float pvEulerValue, float pvGyroValue)
{
	pidInst->pvEuler = (pvEulerValue+ROLL_HALF)/ROLL_FULL; //To keep PV betwee 0 and 1
	pidInst->pvGyro = pvGyroValue;
}

