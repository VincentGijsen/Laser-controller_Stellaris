/*
 * interrupts.h
 *
 *  Created on: Sep 11, 2013
 *      Author: vincent
 */

#ifndef INTERRUPTS_H_
#define INTERRUPTS_H_

#define PWM_FREQUENCY 400

#define MAX_DRIVE_COIL1 10
#define MAX_DRIVE_COIL2 10

#define INITIALDRIVE 10

#define STARTUP_DELAY_CALIB 2

#define SCALE_MAX 500



 #define RED_LED   GPIO_PIN_1
 #define BLUE_LED  GPIO_PIN_2
 #define GREEN_LED GPIO_PIN_3

#define RED_ON  GPIOPinWrite(GPIO_PORTF_BASE, RED_LED, RED_LED)
#define RED_OFF GPIOPinWrite(GPIO_PORTF_BASE, RED_LED, 0)
#define GREEN_ON  GPIOPinWrite(GPIO_PORTF_BASE, GREEN_LED, GREEN_LED)
#define GREEN_OFF GPIOPinWrite(GPIO_PORTF_BASE, GREEN_LED, 0)
#define BLUE_ON  GPIOPinWrite(GPIO_PORTF_BASE, BLUE_LED, BLUE_LED)
#define BLUE_OFF GPIOPinWrite(GPIO_PORTF_BASE, BLUE_LED, 0)


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
void setDriveY(long);

long limitDriveX(long);
long limitDriveY(long);

void positionSetter(void);


#endif /* INTERRUPTS_H_ */
