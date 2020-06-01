/*
 * ADC1.c
 *
 *  Created on: Apr 9, 2020
 *      Author: Ardoli
 */
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"
#include "driverlib/adc.h"
#include "sysctl_pll.h"
#include "buttons.h"
#include "inc/tm4c1294ncpdt.h"

#define ADC_BUFFER_SIZE 2048 // size must be a power of 2
#define ADC_BUFFER_WRAP(i) ((i) & (ADC_BUFFER_SIZE - 1)) // index wrapping macro
#define ADC_OFFSET 2048
volatile int32_t gADCBufferIndex = ADC_BUFFER_SIZE - 1; // latest sample index
volatile uint16_t gADCBuffer[ADC_BUFFER_SIZE]; // circular buffer
volatile uint32_t gADCErrors = 0; // number of missed ADC deadlines


void ADCInit(){
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_0); // GPIO setup for analog input AIN3
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0); // initialize ADC peripherals
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC1);
    // ADC clock
    uint32_t pll_frequency = SysCtlFrequencyGet(CRYSTAL_FREQUENCY);
    uint32_t pll_divisor = (pll_frequency - 1) / (16 * ADC_SAMPLING_RATE) + 1; //round up
    ADCClockConfigSet(ADC0_BASE, ADC_CLOCK_SRC_PLL | ADC_CLOCK_RATE_FULL, pll_divisor);
    ADCClockConfigSet(ADC1_BASE, ADC_CLOCK_SRC_PLL | ADC_CLOCK_RATE_FULL, pll_divisor);
    ADCSequenceDisable(ADC1_BASE, 0); // choose ADC1 sequence 0; disable before configuring
    ADCSequenceConfigure(ADC1_BASE, 0, ADC_TRIGGER_ALWAYS, 32); // specify the "Always" trigger
    ADCSequenceStepConfigure(ADC1_BASE, 0, 0, ADC_CTL_CH3|ADC_CTL_IE|ADC_CTL_END);// in the 0th step, sample channel 3 (AIN3)
     // enable interrupt, and make it the end of sequence
    ADCSequenceEnable(ADC1_BASE, 0); // enable the sequence. it is now sampling
    ADCIntEnable(ADC1_BASE, 0); // enable sequence 0 interrupt in the ADC1 peripheral
}





void ADC_ISR(void)
{
 ADC1_ISC_R |= 1; // clear ADC1 sequence0 interrupt flag in the ADCISC register
 if (ADC1_OSTAT_R & ADC_OSTAT_OV0) { // check for ADC FIFO overflow
 gADCErrors++; // count errors
 ADC1_OSTAT_R = ADC_OSTAT_OV0; // clear overflow condition
 }
 gADCBuffer[
 gADCBufferIndex = ADC_BUFFER_WRAP(gADCBufferIndex + 1)
 ] = (ADC1_SSFIFO0_R & 0xfff); // read sample from the ADC1 sequence 0 FIFO
}



int FallingTrigger(void) // search for falling edge trigger
{
 // Step 1
 int x = gADCBufferIndex - 64/* half screen width; don’t use a magic number */;
 // Step 2
 int x_stop = x - ADC_BUFFER_SIZE/2;
 for (; x > x_stop; x--) {
     if ( gADCBuffer[ADC_BUFFER_WRAP(x)] <= ADC_OFFSET &&
                 gADCBuffer[ADC_BUFFER_WRAP(x-1)] > ADC_OFFSET)
         break;
 }
 // Step 3
 if (x == x_stop) // for loop ran to the end
      x = gADCBufferIndex - 64; // reset x back to how it was initialized
 return x;
}

int RisingTrigger(void) // search for rising edge trigger
{
 // Step 1
 int x = gADCBufferIndex - 64/* half screen width; don’t use a magic number */;
 // Step 2
 int x_stop = x - ADC_BUFFER_SIZE/2;
 for (; x > x_stop; x--) {
     if ( gADCBuffer[ADC_BUFFER_WRAP(x)] >= ADC_OFFSET &&
                 gADCBuffer[ADC_BUFFER_WRAP(x-1)] < ADC_OFFSET)
         break;
 }
 // Step 3
 if (x == x_stop) // for loop ran to the end
      x = gADCBufferIndex - 64; // reset x back to how it was initialized
 return x;
}

