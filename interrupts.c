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
#ifdef DEBUG
void
__error__(char *pcFilename, unsigned long ulLine)
{
}
#endif


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

//Prototypes

void printDouble(double);
double UpdatePID(SPid *, double , double);
//*****************************************************************************
//
// Delay for the specified number of seconds.  Depending upon the current
// SysTick value, the delay will be between N-1 and N seconds (i.e. N-1 full
// seconds are guaranteed, along with the remainder of the current second).
//
//*****************************************************************************
void
Delay(unsigned long ulSeconds)
{
    //
    // Loop while there are more seconds to wait.
    //
    while(ulSeconds--)
    {
        //
        // Wait until the SysTick value is less than 1000.
        //
        while(ROM_SysTickValueGet() > 1000)
        {
        }

        //
        // Wait until the SysTick value is greater than 1000.
        //
        while(ROM_SysTickValueGet() < 1000)
        {
        }
    }
}
void
DisplayIntStatus(void)
{
   // unsigned long ulTemp;

    //
    // Display the currently active interrupts.
    //
  //  ulTemp = HWREG(NVIC_ACTIVE0);
   // UARTprintf("\rActive: %c%c%c ", (ulTemp & 1) ? '1' : ' ',
    //           (ulTemp & 2) ? '2' : ' ', (ulTemp & 4) ? '3' : ' ');

    //
    // Display the currently pending interrupts.
    //
   // ulTemp = HWREG(NVIC_PEND0);
   // UARTprintf("Pending: %c%c%c", (ulTemp & 1) ? '1' : ' ',
      //         (ulTemp & 2) ? '2' : ' ', (ulTemp & 4) ? '3' : ' ');
}

//THE interupt handler routine for our light-reciever

volatile unsigned long lastPin0Time = 0;
volatile unsigned long lastDurationPin0 = 0;


volatile unsigned int calibMaxX=0;
volatile unsigned int calibMinX=0;

volatile float currentfeedbackY = 0.0f;


void GPIO_PortD_IntHandler(void){
	unsigned long ulCurrentPin0 = ROM_SysTickValueGet();

	if(lastPin0Time > ulCurrentPin0){
		//fix for counter-rolover
		lastDurationPin0 = (ulCurrentPin0 + SysTickPeriodGet()) - lastPin0Time;
	}
	else{
		lastDurationPin0 = ulCurrentPin0 - lastPin0Time;
	}
	//make last time current time for next roll;


	lastPin0Time = ulCurrentPin0;
    //some variables
    GPIOPinIntClear(GPIO_PORTD_BASE, GPIO_PIN_0);
    //some code
}

void ftoa(float f,char *buf)
{
    int pos=0,ix,dp,num;
    if (f<0)
    {
        buf[pos++]='-';
        f = -f;
    }
    dp=0;
    while (f>=10.0)
    {
        f=f/10.0;
        dp++;
    }
    for (ix=1;ix<8;ix++)
    {
            num = (int)f;
            f=f-num;
            if (num>9)
                buf[pos++]='#';
            else
                buf[pos++]='0'+num;
            if (dp==0) buf[pos++]='.';
            f=f*10.0;
            dp--;
    }
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
	unsigned long ulPeriod =1000 ;
	volatile unsigned long dutyCycle = 800;

	//Config PID stuff;
	SPid xAxis;
	xAxis.pGain = 10;
	xAxis.iGain = 0.1;
	xAxis.dGain = 50;


    //
    // Enable lazy stacking for interrupt handlers.  This allows floating-point
    // instructions to be used within interrupt handlers, but at the expense of
    // extra stack usage.
    //
    ROM_FPULazyStackingEnable();

    //
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


    //VINCENT
    //ENABLE portD for interrupts conection

    //the TSL235R runs between 0 - 100khz (dark/full saturation) @5v
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);

    //Make a pin an input:
    GPIOPinTypeGPIOInput(GPIO_PORTD_BASE, GPIO_PIN_0);

    //we select falling edge, our light sensor has always 50% duty-cycle, so the freq, determines light-intensity.
    GPIOIntTypeSet(GPIO_PORTD_BASE, GPIO_PIN_0, GPIO_FALLING_EDGE);

    //clear the bit from this interrupt as it has occurred, to enable the next interrupt
    GPIOPinIntClear(GPIO_PORTD_BASE, GPIO_PIN_0);

    //now enable this interrupt actually
    GPIOPinIntEnable(GPIO_PORTD_BASE, GPIO_PIN_0);

    //add an reference in the interrupt-service
    IntEnable(INT_GPIOD);

    //enable PWM STUFF

    // Turn off LEDs
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 0);

    // Configure LEDs PortF as Timer outputs -> see pg 659 of datasheet
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    //GPIOPinConfigure(GPIO_PF1_T0CCP1|GPIO_PF2_T1CCP0|GPIO_PF3_T1CCP1);


    ROM_GPIOPinConfigure(GPIO_PF3_T1CCP1);
    ROM_GPIOPinConfigure(GPIO_PF2_T1CCP0);
    ROM_GPIOPinConfigure(GPIO_PF1_T0CCP1);

    GPIOPinTypeTimer(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);


    // Configure timer 0 – this timer outputs to pf1 (led)
    //https://sites.google.com/site/narasimhaweb/projects/pwm-on-stellaris-launchpad

    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    TimerConfigure(TIMER0_BASE, TIMER_CFG_SPLIT_PAIR|TIMER_CFG_B_PWM);
    TimerLoadSet(TIMER0_BASE, TIMER_B, ulPeriod -1);
    TimerMatchSet(TIMER0_BASE, TIMER_B, dutyCycle); // PWM
    TimerEnable(TIMER0_BASE, TIMER_B);

    // Set up and enable the SysTick timer.  It will be used as a reference
    // for delay loops in the interrupt handlers.  The SysTick timer period
    // will be set up for one second.
    //
    ROM_SysTickPeriodSet(ROM_SysCtlClockGet());
    ROM_SysTickEnable();

    //
    // Enable interrupts to the processor.
    //
    ROM_IntMasterEnable();

    //
    // Enable the interrupts.
    //

    //TEST PID CODE
    UARTprintf("put MAX value on sensor\n");
    Delay(20);
    calibMaxX = lastDurationPin0;

    UARTprintf("put MIN value on sensor\n");
    Delay(20);
    calibMinX = lastDurationPin0;


    unsigned int range = calibMaxX - calibMinX;

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


    xAxis.iMax = 100;
    xAxis.iMin = -100;

    //setting setpoint to 0
    UARTprintf("setting set-point to 0\n");
    double newPosition = (calibMinX + (range/2));

    UARTprintf("setpoint set to: ");
       printDouble(newPosition);
       UARTprintf("\n");

    //
    UARTprintf("Started Program laserControlls\n");


    while(1)
    {
       	TimerMatchSet(TIMER0_BASE, TIMER_B, dutyCycle++);

		if(dutyCycle >= ulPeriod - 1)
			dutyCycle = 0;

		SysCtlDelay(10000000);
//		SysCtlDelay(200000);

		double currentPosition = calibMinX +lastDurationPin0;
		double error = (newPosition - currentPosition);
		double drive = UpdatePID(&xAxis, error, currentPosition);
		//do output to pwm or something
		UARTprintf("current: ");
		printDouble(currentPosition);

		UARTprintf("\nerror: ");
		printDouble(error);

		UARTprintf("\npid cmd:  ");
		printDouble(drive);
		UARTprintf(" \n---\n");
		//double error =
    }
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
