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


//THE interupt handler routine for our light-reciever

volatile unsigned long lastPinA = 0;
volatile unsigned long lastPinB = 0;
volatile unsigned long lastDurationA = 0;
volatile unsigned long lastDurationB = 0;
volatile int xPosition = 0;

volatile long calibMaxX=1;
volatile long calibMinX=2;

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
						lastDurationB = (SysTickPeriodGet() - lastPinB) + now;
					}
					else{
						lastDurationB = now - lastPinB;
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
void timerSetup(){

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
	unsigned long ulPeriod = PWM_FREQUENCY ;
	volatile unsigned long dutyCycle = 0;

	//Config PID stuff;
	SPid xAxis;
	xAxis.pGain = 0.5;
	xAxis.iGain = 0.001;
	xAxis.dGain = 0.001;

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

    //
    // Initialize the UART.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    UARTStdioInit(0);


    //TIMER
    timerSetup();



    //VINCENT
    //ENABLE portD for interrupts connection


    ROM_SysTickPeriodSet(ROM_SysCtlClockGet());
    ROM_SysTickEnable();

    //capture
    setupCapture();


    ROM_IntMasterEnable();
    UARTprintf("System Clock=%d  %d Mhz\n",SysCtlClockGet(),SysCtlClockGet()/1000000L);

    UARTprintf("put MAX value on sensor\n");

    driveCoil2(20);

    //wait for the position
    Delay(20);
    calibMaxX = lastDurationB;

    //TEST PID CODE

    //disable coild
    driveCoil2(0);
    Delay(20);
    UARTprintf("put MIN value on sensor\n");

    driveCoil2(-20);
    //wait for the position
    Delay(20);
    calibMinX = lastDurationB;
    driveCoil2(0);

    unsigned long range = calibMaxX - calibMinX;

    UARTprintf("done calibrating\n");
    UARTprintf("MAX: ");
    printDouble(calibMaxX);
    UARTprintf("\n");

    UARTprintf("Min: ");
    printDouble(calibMinX);
    UARTprintf("\n");

    UARTprintf("range: ");
    printDouble(range);
    UARTprintf("\n");


    xAxis.iMax = 0;
    xAxis.iMin = 100;

    //setting setpoint to 0
    UARTprintf("setting set-point to 500\n");
    double newPosition = 500;

    UARTprintf("setpoint set to: ");
    printDouble(newPosition);
    UARTprintf("\n");

    //
    UARTprintf("Started Program laserControlls\n");


    while(1)
    {

		double currentPosition = map(lastDurationB, calibMinX, calibMaxX, 0, 1000);
		long error = (newPosition - currentPosition);
     	double drive = UpdatePID(&xAxis, error, currentPosition);
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
		driveCoil2(drive);
    	}
    }


void driveCoil1(long driveValue){
	if (driveValue > 0){
		 TimerMatchSet(TIMER0_BASE, TIMER_A, limitCoil1Drive(driveValue));
		 TimerMatchSet(TIMER0_BASE, TIMER_B, 0);
	}
	else{
		 TimerMatchSet(TIMER0_BASE, TIMER_B, limitCoil1Drive(abs(driveValue)));
		 TimerMatchSet(TIMER0_BASE, TIMER_A, 0);
	}
}
void driveCoil2(long driveValue){
	if (driveValue > 0){
		 TimerMatchSet(TIMER1_BASE, TIMER_A, limitCoil1Drive(driveValue));
		 TimerMatchSet(TIMER1_BASE, TIMER_B, 0);
	}
	else{
		 TimerMatchSet(TIMER1_BASE, TIMER_B, limitCoil1Drive(abs(driveValue)));
		 TimerMatchSet(TIMER1_BASE, TIMER_A, 0);
	}

}


long limitCoil1Drive(long setpoint){
	return setpoint > MAX_DRIVE_COIL1? MAX_DRIVE_COIL1 : setpoint;
}

long limitCoil2Drive(long setpoint){
	return setpoint > MAX_DRIVE_COIL2? MAX_DRIVE_COIL2 : setpoint;
}

long map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

double calcProjection(long in){
	long t =  in - calibMinX;
	long min = t<0? calibMinX : t;



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
