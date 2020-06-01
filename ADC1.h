/*
 * ADC1.h
 *
 *  Created on: Apr 9, 2020
 *      Author: Ardoli
 */

#ifndef ADC1_H_
#define ADC1_H_


#include <stdint.h>

extern uint32_t gADCErrors;

void ADCInit(void);
int RisingTrigger(void);
int FallingTrigger(void);
extern volatile int32_t gADCBufferIndex; //= ADC_BUFFER_SIZE - 1; // latest sample index
extern volatile uint16_t gADCBuffer[2048]; // circular buffer



#endif /* ADC1_H_ */
