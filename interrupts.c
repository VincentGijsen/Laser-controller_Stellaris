//*****************************************************************************
//
// interrupts.c - Interrupt preemption and tail-chaining example.
//
// Copyright (c) 2012 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
// 
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
// 
// This is part of revision 9453 of the EK-LM4F120XL Firmware Package.
//
//*****************************************************************************

#define PART_LM4F120H5QR

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

//*****************************************************************************
//
//! \addtogroup example_list
//! <h1>Interrupts (interrupts)</h1>
//!
//! This example application demonstrates the interrupt preemption and
//! tail-chaining capabilities of Cortex-M4 microprocessor and NVIC.  Nested
//! interrupts are synthesized when the interrupts have the same priority,
//! increasing priorities, and decreasing priorities.  With increasing
//! priorities, preemption will occur; in the other two cases tail-chaining
//! will occur.  The currently pending interrupts and the currently executing
//! interrupt will be displayed on the display; GPIO pins E1, E2 and E3 will
//! be asserted upon interrupt handler entry and de-asserted before interrupt
//! handler exit so that the off-to-on time can be observed with a scope or
//! logic analyzer to see the speed of tail-chaining (for the two cases where
//! tail-chaining is occurring).
//
//*****************************************************************************


//*****************************************************************************
//
// The count of interrupts received.  This is incremented as each interrupt
// handler runs, and its value saved into interrupt handler specific values to
// determine the order in which the interrupt handlers were executed.
//
//*****************************************************************************
volatile unsigned long g_ulIndex;

//*****************************************************************************
//
// The value of g_ulIndex when the INT_GPIOA interrupt was processed.
//
//*****************************************************************************
volatile unsigned long g_ulGPIOa;

//*****************************************************************************
//
// The value of g_ulIndex when the INT_GPIOB interrupt was processed.
//
//*****************************************************************************
volatile unsigned long g_ulGPIOb;

//*****************************************************************************
//
// The value of g_ulIndex when the INT_GPIOC interrupt was processed.
//
//*****************************************************************************
volatile unsigned long g_ulGPIOc;

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, unsigned long ulLine)
{
}
#endif

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

//*****************************************************************************
//
// Display the interrupt state on the UART.  The currently active and pending
// interrupts are displayed.
//
//*****************************************************************************
void
DisplayIntStatus(void)
{
    unsigned long ulTemp;

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
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);

    //Make a pin an input:
    GPIOPinTypeGPIOInput(GPIO_PORTD_BASE, GPIO_PIN_0);

    //we select faling edge, our light sensor has always 50% dutycylcle, so the freq, determines light-intensity.
    GPIOIntTypeSet(GPIO_PORTD_BASE, GPIO_PIN_0, GPIO_FALLING_EDGE);

    //clear the bit from this interupt as it has occurd, to enable the next interrupt
    GPIOPinIntClear(GPIO_PORTD_BASE, GPIO_PIN_0);

    //now enable this interupt actulally
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

    //
    UARTprintf("Started Program laserControlls\n");

    while(1)
    {


    	char buff[10];
    	unsigned long lastVal = lastDurationPin0;
    	//sprintf(buff, "%d", lastDurationPin0, lastVal);
    	//ftoa(lastVal, buff);
    	//UARTprintf("Current delay: %s \n", buff );

    	TimerMatchSet(TIMER0_BASE, TIMER_A, dutyCycle);

		if(dutyCycle >= ulPeriod - 1){
			dutyCycle = 0;
		}

		dutyCycle++;

		SysCtlDelay(200000);
    }
}
