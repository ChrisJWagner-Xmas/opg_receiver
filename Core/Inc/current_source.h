/*
 * current_source.h
 *
 *  Created on: Feb 6, 2022
 *      Author: chris
 */

#ifndef INC_CURRENT_SOURCE_H_
#define INC_CURRENT_SOURCE_H_

#include "constants.h"

// Constants.

#define MAX_CURRENT_mA 8.0 // TODO: change if necessary.
#define THRESHOLD_CURRENT_mA (double)2.5 // TODO: find out.
#define CURRENT_SENSE_GAIN (float)100.0
#define CURRENT_SENSE_RESISTOR_OHM (float)1.0
#define CURRENT_SOURCE_A_ON GPIO_BSRR_BS1  // On GPIOA
#define CURRENT_SOURCE_A_OFF GPIO_BSRR_BR1 // On GPIOA
#define LSB_TO_mA_CONVERSION_FACTOR (float)((ADC1_REF_VOLTAGE_V*1000.0)/(ADC1_MAX_VALUE_LSB*CURRENT_SENSE_RESISTOR_OHM*CURRENT_SENSE_GAIN))

#define CURRENT_SOURCE_SETTLE_MARGIN_LSB 9 // approximately 0.2 mA. TODO: adjust if necessary
#define CURRENT_SOURCE_MAX_NUM_CYCLES_TO_SETTLE 20
// Functions.

float get_emitter_current_mA();
uint16_t get_emitter_current_lsb();
uint16_t wait_for_cs_to_settle();

#endif /* INC_CURRENT_SOURCE_H_ */
