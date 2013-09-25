/*
 * Interrupt based laser/harddrive scanning thinggy,
 * based on ti's intterupt example
 *
 */
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_nvic.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/timer.h"
#include "utils/uartstdio.h"

#include "interrupts.h"
#include "helper.h"

#ifdef DEBUG
void
__error__(char *pcFilename, unsigned long ulLine)
{
}
#endif

//ANIMATION FIRST POS, IS THE INDEX OF A FRAME (EG 24/S), 2ND ARE ALL THE POINT IN ONE FRAME)
//THE POINTS ARE IN TRIPLETS, X,Y,B SO XPOS, YPOS AND BLANKING. THE MIDDLE IS 0 so +/- 127

//tmp for now a firgure consists of 8 points
const int ani0[][8*3] = {
		{
				-10, 0, 1,
				-8,  0, 1,
				-6, 0, 1,
				-4, 0, 1,
				-2, 0 ,1,
				0, 0 ,1,
				2, 0, 1,
				4, 0, 1,
		},
		{
				-8,  0, 1,
				-6, 0, 1,
				-4, 0, 1,
				-2, 0 ,1,
				0, 0 ,1,
				2, 0, 1,
				4, 0, 1,
				6, 0, 1,

		},
		{
				-6, 0, 1,
				-4, 0, 1,
				-2, 0 ,1,
				0, 0 ,1,
				2, 0, 1,
				4, 0, 1,
				6, 0, 1,
				8, 0, 1,
		},
		{
				-4, 0, 1,
				-2, 0 ,1,
				0, 0 ,1,
				2, 0, 1,
				4, 0, 1,
				6, 0, 1,
				8, 0, 1,
				10,0,1

		},
		{
				-2, 0 ,1,
				0, 0 ,1,
				2, 0, 1,
				4, 0, 1,
				6, 0, 1,
				8, 0, 1,
				10,0,1,
				12,0,1

		},
		{
				0, 0 ,1,
				2, 0, 1,
				4, 0, 1,
				6, 0, 1,
				8, 0, 1,
				10,0,1,
				12,0,1,
				14,0,1,
		}
};

// 'M'
const int ani1[][8*3] = {
		{ //x y, blank
						-20, -20 ,1,
						-10, 20, 1,
						-5, 5, 1,
						-2, 0, 1,
						 2, 0, 1,
						 5,5,5,
						10, 20,1,
						20,-20,1,
		}

};

//3d cubus


//THE interupt handler routine for our light-reciever

volatile unsigned long lastPinA = 0;
volatile unsigned long lastPinB = 0;
volatile unsigned long lastDurationA = 0;
volatile unsigned long lastDurationY = 0;
volatile int xPosition = 0;

volatile long calibMaxY=1;
volatile long calibMinY=2;

volatile float currentfeedbackY = 0.0f;

//fired when pin triggers high
void GPIO_PortA_IntHandler(void){
	unsigned long now = ROM_SysTickValueGet();

	long lIntPinStatus = GPIOPinIntStatus(GPIO_PORTA_BASE,GPIO_PIN_6 | GPIO_PIN_7);

	switch(lIntPinStatus) {
	case  GPIO_PIN_6:
	{
		GPIOPinIntClear(GPIO_PORTA_BASE, GPIO_PIN_6);        // Clear the INT now
		if(lastPinA > now){
			//fix for counter-roll_over
			lastDurationA = (SysTickPeriodGet() - lastPinA) + now;
		}
		else{
			lastDurationA = now - lastPinA;
		}

		lastPinA = now;
	}
	break;

	case  GPIO_PIN_7:
	{
		GPIOPinIntClear(GPIO_PORTA_BASE, GPIO_PIN_7);        // Clear the INT now
		if(lastPinB > now){
			//fix for counter-roll_over
			lastDurationY = (SysTickPeriodGet() - lastPinB) + now;
		}
		else{
			lastDurationY = now - lastPinB;
		}

		lastPinB = now;
	}
	break;

	default:
	{
		GPIOPinIntClear(GPIO_PORTA_BASE, lIntPinStatus);
		break;

	}
	}
}
void setupTimers(){

	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF); // Enable port F
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB); // Enable port B
	GPIOPinConfigure(GPIO_PB4_T1CCP0); // Configure pin PF2 as output of Timer 1_A
	GPIOPinConfigure(GPIO_PB5_T1CCP1); // Configure pin PF3 as output of Timer 1_B
	GPIOPinConfigure(GPIO_PB6_T0CCP0); // Configure pin PB6 as output of Timer 0_A
	GPIOPinConfigure(GPIO_PB7_T0CCP1); // Configure pin PB7 as output of Timer 0_B
	GPIOPinTypeTimer(GPIO_PORTB_BASE, GPIO_PIN_4 ); // Enable pin PF2 as output of timer addressed to it
	GPIOPinTypeTimer(GPIO_PORTB_BASE, GPIO_PIN_5 ); // Enable pin PF3 as output of timer addressed to it
	GPIOPinTypeTimer(GPIO_PORTB_BASE, GPIO_PIN_6 ); // Enable pin PB6 as output of timer addressed to it
	GPIOPinTypeTimer(GPIO_PORTB_BASE, GPIO_PIN_7 ); // Enable pin PB7 as output of timer addressed to it

	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1); // Enable Timer 1
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0); // Enable Timer 0
	TimerConfigure(TIMER1_BASE, (TIMER_CFG_SPLIT_PAIR|TIMER_CFG_A_PWM|TIMER_CFG_B_PWM)); // Configure Timer 1 as two 16 but timers with both functioning as PWM
	TimerConfigure(TIMER0_BASE, (TIMER_CFG_SPLIT_PAIR|TIMER_CFG_A_PWM|TIMER_CFG_B_PWM)); // Configure Timer 0 as two 16 but timers with both functioning as PWM
	TimerControlLevel(TIMER1_BASE, TIMER_BOTH, 1); //  Timer 1 is trigger low
	TimerControlLevel(TIMER0_BASE, TIMER_BOTH, 1); // Timer 0 is trigger low
	TimerLoadSet(TIMER1_BASE, TIMER_A, PWM_FREQUENCY -1); // Timer 1 Load set
	TimerLoadSet(TIMER1_BASE, TIMER_B, PWM_FREQUENCY -1);
	TimerLoadSet(TIMER0_BASE, TIMER_A, PWM_FREQUENCY -1); // Timer 0 Load set
	TimerLoadSet(TIMER0_BASE, TIMER_B, PWM_FREQUENCY -1);
	TimerMatchSet(TIMER1_BASE, TIMER_A, 100); // Timer 1 Match set
	TimerMatchSet(TIMER1_BASE, TIMER_B, 100);
	TimerMatchSet(TIMER0_BASE, TIMER_A, 100); // Timer 0 Match set
	TimerMatchSet(TIMER0_BASE, TIMER_B, 100);
	TimerEnable(TIMER1_BASE, TIMER_BOTH);
	TimerEnable(TIMER0_BASE, TIMER_BOTH);

}



void setupCapture(){

	//the TSL235R runs between 0 - 100khz (dark/full saturation) @5v
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

	//Make a pin an input:
	GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_7);//GPIO_PIN_6 | GPIO_PIN_7);

	//we select falling edge, our light sensor has always 50% duty-cycle, so the freq, determines light-intensity.

	//TODO fix for 2
	GPIOIntTypeSet(GPIO_PORTA_BASE,  GPIO_PIN_7, GPIO_FALLING_EDGE); //(GPIO_PIN_6 | GPIO_PIN_7), GPIO_FALLING_EDGE);

	//clear the bit from this interrupt as it has occurred, to enable the next interrupt
	GPIOPinIntClear(GPIO_PORTA_BASE,  GPIO_PIN_7); // (GPIO_PIN_6 | GPIO_PIN_7));

	//now enable this interrupt actually
	GPIOPinIntEnable(GPIO_PORTA_BASE, GPIO_PIN_7);//(GPIO_PIN_6 | GPIO_PIN_7));


	// Enable interrupts to the processor.
	IntEnable(INT_GPIOA);

	// Set up and enable the SysTick timer.  It will be used as a reference
	// for delay loops in the interrupt handlers.  The SysTick timer period
	// will be set up for one second.
	//
}

void setupUart(){
	//
	// Initialize the UART.
	//
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	GPIOPinConfigure(GPIO_PA0_U0RX);
	GPIOPinConfigure(GPIO_PA1_U0TX);
	ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
	UARTStdioInit(0);

}
//*****************************************************************************
//
// This is the main example program.  It checks to see that the interrupts are
// processed in the correct order when they have identical priorities,
// increasing priorities, and decreasing priorities.  This exercises interrupt
// preemption and tail chaining.
//
//*****************************************************************************
int
main(void)
{
	volatile unsigned long dutyCycle = 0;

	//Config PID stuff;
	SPid pidY;
	pidY.pGain = 0.5;
	pidY.iGain = 0.001;
	pidY.dGain = 0.001;

	//
	// Enable lazy stacking for interrupt handlers.  This allows floating-point
	// instructions to be used within interrupt handlers, but at the expense of
	// extra stack usage.
	//
	ROM_FPULazyStackingEnable();

	// Set the clocking to run directly from the crystal.
	//
	//   ROM_SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN |
	//                    SYSCTL_XTAL_16MHZ);

	SysCtlClockSet(SYSCTL_SYSDIV_2_5| SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);  //80MHz
	//
	// Enable the peripherals used by this example.
	//
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

	setupUart();

	//TIMER
	setupTimers();



	//VINCENT
	//ENABLE portD for interrupts connection


	ROM_SysTickPeriodSet(ROM_SysCtlClockGet());
	ROM_SysTickEnable();

	//capture
	setupCapture();


	ROM_IntMasterEnable();
	UARTprintf("System Clock=%d  %d Mhz\n",SysCtlClockGet(),SysCtlClockGet()/1000000L);

	UARTprintf("put MAX value on sensor\n");

	setDriveY(INITIALDRIVE);

	//wait for the position
	Delay(20);
	calibMaxY = lastDurationY;

	//TEST PID CODE

	//disable coild
	setDriveY(0);
	Delay(-INITIALDRIVE);
	UARTprintf("put MIN value on sensor\n");

	setDriveY(-20);
	//wait for the position
	Delay(20);
	calibMinY = lastDurationY;
	setDriveY(0);

	unsigned long range = calibMaxY - calibMinY;

	UARTprintf("done calibrating\n");
	UARTprintf("MAX: ");
	printDouble(calibMaxY);
	UARTprintf("\n");

	UARTprintf("Min: ");
	printDouble(calibMinY);
	UARTprintf("\n");

	UARTprintf("range: ");
	printDouble(range);
	UARTprintf("\n");



	pidY.iMax = 0;
	pidY.iMin = 100;

	//setting setpoint to 0
	UARTprintf("setting set-point to 500\n");
	double setpointAxisY = 500; //

	UARTprintf("setpointAxisX set to: ");
	printDouble(setpointAxisY);
	UARTprintf("\n");

	//
	UARTprintf("Started Program laserControlls\n");


	while(1)
	{

		double currentPositionY = map(lastDurationY, calibMinY, calibMaxY, 0, 1000);
		long errorY = (setpointAxisY - currentPositionY);
		double driveY = UpdatePID(&pidY, errorY, currentPositionY);
		//do output to pwm or something
		/*

     	UARTprintf("sensdata:: ");
    		printDouble(lastDurationB);

  		UARTprintf("current: ");
		printDouble(currentPosition);

		UARTprintf("\nerror: ");
		printDouble(error);

		UARTprintf("\npid cmd:  ");
		printDouble(drive);
		UARTprintf(" \n---\n");
		  		Delay(10);

		 */
		setDriveY(driveY);
	}
}


void setDriveX(long driveValue){
	if (driveValue > 0){
		TimerMatchSet(TIMER0_BASE, TIMER_A, limitDriveX(driveValue));
		TimerMatchSet(TIMER0_BASE, TIMER_B, 0);
	}
	else{
		TimerMatchSet(TIMER0_BASE, TIMER_B, limitDriveX(abs(driveValue)));
		TimerMatchSet(TIMER0_BASE, TIMER_A, 0);
	}
}
void setDriveY(long driveValue){
	if (driveValue > 0){
		TimerMatchSet(TIMER1_BASE, TIMER_A, limitDriveY(driveValue));
		TimerMatchSet(TIMER1_BASE, TIMER_B, 0);
	}
	else{
		TimerMatchSet(TIMER1_BASE, TIMER_B, limitDriveY(abs(driveValue)));
		TimerMatchSet(TIMER1_BASE, TIMER_A, 0);
	}

}


long limitDriveX(long setpoint){
	return setpoint > MAX_DRIVE_COIL1? MAX_DRIVE_COIL1 : setpoint;
}

long limitDriveY(long setpoint){
	return setpoint > MAX_DRIVE_COIL2? MAX_DRIVE_COIL2 : setpoint;
}

long map(long x, long in_min, long in_max, long out_min, long out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


void printDouble(double input){
	char buff[10];
	ftoa(input, buff);
	UARTprintf("%s", buff);
}

double UpdatePID(SPid * pid, double error, double position)
{
	double pTerm,
	dTerm, iTerm;
	pTerm = pid->pGain * error;
	// calculate the proportional term
	// calculate the integral state with appropriate limiting
	pid->iState += error;
	if (pid->iState > pid->iMax)
		pid->iState = pid->iMax;
	else if (pid->iState < pid->iMin)
		pid->iState = pid->iMin;

	iTerm = pid->iGain * pid->iState; // calculate the integral term
	dTerm = pid->dGain * (position - pid->dState);
	pid->dState = position;
	return pTerm + iTerm - dTerm;
}
