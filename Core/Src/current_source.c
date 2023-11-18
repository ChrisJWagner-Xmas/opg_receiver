/*
 * current_source.cpp
 *
 *  Created on: Feb 6, 2022
 *      Author: chris
 *      Includes all functionality for the vcsel current source.
 */

#include "stm32f103xb.h"
#include "current_source.h"
#include "math.h"

float get_emitter_current_mA()
{
	ADC1->CR2 |= ADC_CR2_ADON;
	// Wait for the ADC to finish conversion.
	while(!(ADC1->SR & ADC_SR_EOC))
	{
	  asm("NOP");
	}
	uint32_t adc_value = ADC1->DR;
	float current_mA = (float)adc_value*(float)LSB_TO_mA_CONVERSION_FACTOR;

	return current_mA;
}

uint16_t get_emitter_current_lsb()
{
	// Function to return the adc value of the current directly.

	ADC1->CR2 |= ADC_CR2_ADON;
	// Wait for the ADC to finish conversion.
	while(!(ADC1->SR & ADC_SR_EOC))
	{
	  asm("NOP");
	}
	uint32_t adc_value = ADC1->DR;

	return (uint16_t)adc_value;
}

/*
 * Wait for the current source to settle to a steady state value.
 * Current source settling time is approximately 300-400 ns.
 * Note: The Photodiode rush-in current overshoot and the gate
 * drive voltage are currently much slower, in which case this
 * function does not need to be called.
 * @param: None.
 * @return: uint16_t index: number of ADC read cycles until cs settled.
 */

uint16_t wait_for_cs_to_settle()
{
	uint16_t prev_current_lsb = 0;
	uint16_t current_lsb = 0;
	uint16_t index = 0;

	while(1)
	{
	  current_lsb = get_emitter_current_lsb();

	  // Check, if the current source has settled to a steady state after being turned on.
	  if(abs(current_lsb - prev_current_lsb) <= CURRENT_SOURCE_SETTLE_MARGIN_LSB ||
			  index >= CURRENT_SOURCE_MAX_NUM_CYCLES_TO_SETTLE)
	  {
		  break; // leave while loop
	  }
	  prev_current_lsb = current_lsb;
	  index++;
	}
	return index;
}
