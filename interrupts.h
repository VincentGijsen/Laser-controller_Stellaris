/*
 * interrupts.h
 *
 *  Created on: Sep 11, 2013
 *      Author: vincent
 */

#ifndef INTERRUPTS_H_
#define INTERRUPTS_H_

#define PWM_FREQUENCY 1000

#define MAX_DRIVE_COIL1 50
#define MAX_DRIVE_COIL2 50

//Prototypes

//Data types
typedef struct
{
	double dState; // Last position input
	double iState; // Integrator state
	double iMax, iMin;
	// Maximum and minimum allowable integrator state
	double iGain, // integral gain
	pGain, // proportional gain
	dGain; // derivative gain
} SPid;


/**
 * Prototypes
 */

void printDouble(double);
double UpdatePID(SPid *, double , double);
long map(long , long , long , long , long );

void driveCoil1(long);
void driveCoil2(long);

long limitCoil1Drive(long);
long limitCoil2Drive(long);


#endif /* INTERRUPTS_H_ */
