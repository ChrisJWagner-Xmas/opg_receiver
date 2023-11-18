/*
 * sensor.h
 *
 *  Created on: 25.02.2022
 *      Author: chris
 */

#ifndef INC_SENSOR_H_
#define INC_SENSOR_H_

#include <stdint.h> // for uint8_t etc.
#include "stm32f103xb.h" // For GPIOA etc.

#define NUM_SENSORS 15

#define NUM_MEASUREMENTS_PER_SENSOR_DC_OFFSET 32
#define NUM_MEASUREMENTS_PER_SENSOR 32
#define NUM_TOTAL_MEASUREMENTS_PER_SENSOR 64

#define MEASUREMENT_OFFSET_INDEX 20 // Defines the number of measurements to wait for the PD to settle.

#define NUM_TX_CURRENT_MEASUREMENTS 1 // How many current measurements are transmitted to the PC.

// [31:16]: GPIO resets (BR), [15:0] GPIO sets (BS).
// [15:0]: GPIOx_ODR are used to set these bits directly.

// GPIOA/B pin bit fields (BS) for sensor selection.
// All 4 MUXES for Emitter and Detector selection are
// tied to GPIOA/B pins. Sxx_GPIOx_BS defines the bit field
// in the GPIOx_ODR register to select both the emitter
// and detector of that particular sensor.
// MUX GPIOs [A1, A0]: LED_OUT: A10,A9   (cols)
//            		   LED_IN: A8,B15    (rows)
//                     PD_Gates: B13,B14 (rows)
//                     PD: B11,B12       (cols)

// LED current bit fields to cycle through all 15 LEDs
// row-wise.
// row 1 (A9=0,A10=0)
#define S11_LED_GPIOA_BS 0
#define S12_LED_GPIOA_BS (1<<8)
#define S13_LED_GPIOA_BS
#define S14_LED_GPIOA_BS (1<<8)
//						  cols
#define S11_LED_GPIOB_BS 0
#define S12_LED_GPIOB_BS 0
#define S13_LED_GPIOB_BS (1<<15)
#define S14_LED_GPIOB_BS (1<<15)

// row 2 (A9=0,A10=1)	  row    | cols
#define S21_LED_GPIOA_BS (1<<10)
#define S22_LED_GPIOA_BS (1<<10) + (1<<8)
#define S23_LED_GPIOA_BS (1<<10)
#define S24_LED_GPIOA_BS (1<<10) + (1<<8)
//						  cols
#define S21_LED_GPIOB_BS 0
#define S22_LED_GPIOB_BS 0
#define S23_LED_GPIOB_BS (1<<15)
#define S24_LED_GPIOB_BS (1<<15)

// row 3 (A9=1,A10=0)     row   |  cols
#define S31_LED_GPIOA_BS (1<<9)
#define S32_LED_GPIOA_BS (1<<9) + (1<<8)
#define S33_LED_GPIOA_BS (1<<9)
#define S34_LED_GPIOA_BS (1<<9) + (1<<8)
//						  cols
#define S31_LED_GPIOB_BS 0
#define S32_LED_GPIOB_BS 0
#define S33_LED_GPIOB_BS (1<<15)
#define S34_LED_GPIOB_BS (1<<15)

// row 4 (A9=1, A10=1)    row             | cols
#define S41_LED_GPIOA_BS (1<<9) + (1<<10)
#define S42_LED_GPIOA_BS (1<<9) + (1<<10) + (1<<8)
#define S43_LED_GPIOA_BS (1<<9) + (1<<10)
//						  cols
#define S41_LED_GPIOB_BS 0
#define S42_LED_GPIOB_BS 0
#define S43_LED_GPIOB_BS (1<<15)

// Photodiode bit fields to cycle through them one by one
// row-wise.
// row 1
#define Sxx_PD_GPIOA_BS 0 // Same for all PDs since it only uses GPIO B pins.
// (B13=0,B14=0)         cols
#define S11_PD_GPIOB_BS 0
#define S12_PD_GPIOB_BS (1<<12)
#define S13_PD_GPIOB_BS (1<<11)
#define S14_PD_GPIOB_BS (1<<11) + (1<<12)

// row 2 (B13=0,B14=1)    row   |   cols
#define S21_PD_GPIOB_BS (1<<14)
#define S22_PD_GPIOB_BS (1<<14) + (1<<12)
#define S23_PD_GPIOB_BS (1<<14) + (1<<11)
#define S24_PD_GPIOB_BS (1<<14) + (1<<11) + (1<<12)

// row 3 (B13=1,B14=0)    row   |   cols
#define S31_PD_GPIOB_BS (1<<13)
#define S32_PD_GPIOB_BS (1<<13) + (1<<12)
#define S33_PD_GPIOB_BS (1<<13) + (1<<11)
#define S34_PD_GPIOB_BS (1<<13) + (1<<11) + (1<<12)

// row 4 (B14=1, B13=1)   row             |   cols
#define S41_PD_GPIOB_BS (1<<14) + (1<<13)
#define S42_PD_GPIOB_BS (1<<14) + (1<<13) + (1<<12)
#define S43_PD_GPIOB_BS (1<<14) + (1<<13) + (1<<11)

// Combine the LED and PD bit fields to the sensor bit fields.
#define S11_GPIOA_BS (S11_LED_GPIOA_BS + Sxx_PD_GPIOA_BS)
#define S11_GPIOB_BS (S11_LED_GPIOB_BS + S11_PD_GPIOB_BS)

#define S12_GPIOA_BS (S12_LED_GPIOA_BS + Sxx_PD_GPIOA_BS)
#define S12_GPIOB_BS (S12_LED_GPIOB_BS + S12_PD_GPIOB_BS)

#define S13_GPIOA_BS (S13_LED_GPIOA_BS + Sxx_PD_GPIOA_BS)
#define S13_GPIOB_BS (S13_LED_GPIOB_BS + S13_PD_GPIOB_BS)

#define S14_GPIOA_BS (S14_LED_GPIOA_BS + Sxx_PD_GPIOA_BS)
#define S14_GPIOB_BS (S14_LED_GPIOB_BS + S14_PD_GPIOB_BS)

#define S21_GPIOA_BS (S21_LED_GPIOA_BS + Sxx_PD_GPIOA_BS)
#define S21_GPIOB_BS (S21_LED_GPIOB_BS + S21_PD_GPIOB_BS)

#define S22_GPIOA_BS (S22_LED_GPIOA_BS + Sxx_PD_GPIOA_BS)
#define S22_GPIOB_BS (S22_LED_GPIOB_BS + S22_PD_GPIOB_BS)

#define S23_GPIOA_BS (S23_LED_GPIOA_BS + Sxx_PD_GPIOA_BS)
#define S23_GPIOB_BS (S23_LED_GPIOB_BS + S23_PD_GPIOB_BS)

#define S24_GPIOA_BS (S24_LED_GPIOA_BS + Sxx_PD_GPIOA_BS)
#define S24_GPIOB_BS (S24_LED_GPIOB_BS + S24_PD_GPIOB_BS)

#define S31_GPIOA_BS (S31_LED_GPIOA_BS + Sxx_PD_GPIOA_BS)
#define S31_GPIOB_BS (S31_LED_GPIOB_BS + S31_PD_GPIOB_BS)

#define S32_GPIOA_BS (S32_LED_GPIOA_BS + Sxx_PD_GPIOA_BS)
#define S32_GPIOB_BS (S32_LED_GPIOB_BS + S32_PD_GPIOB_BS)

#define S33_GPIOA_BS (S33_LED_GPIOA_BS + Sxx_PD_GPIOA_BS)
#define S33_GPIOB_BS (S33_LED_GPIOB_BS + S33_PD_GPIOB_BS)

#define S34_GPIOA_BS (S34_LED_GPIOA_BS + Sxx_PD_GPIOA_BS)
#define S34_GPIOB_BS (S34_LED_GPIOB_BS + S34_PD_GPIOB_BS)

#define S41_GPIOA_BS (S41_LED_GPIOA_BS + Sxx_PD_GPIOA_BS)
#define S41_GPIOB_BS (S41_LED_GPIOB_BS + S41_PD_GPIOB_BS)

#define S42_GPIOA_BS (S42_LED_GPIOA_BS + Sxx_PD_GPIOA_BS)
#define S42_GPIOB_BS (S42_LED_GPIOB_BS + S42_PD_GPIOB_BS)

#define S43_GPIOA_BS (S43_LED_GPIOA_BS + Sxx_PD_GPIOA_BS)
#define S43_GPIOB_BS (S43_LED_GPIOB_BS + S43_PD_GPIOB_BS)

// GPIOA pin bit fields for sensor selection.
// Set the new sensor gpio output and reset the previous one.
#define S11_GPIOA_BSRR (S43_GPIOA_BS << 16) + S11_GPIOA_BS
#define S12_GPIOA_BSRR (S11_GPIOA_BS << 16) + S12_GPIOA_BS
#define S13_GPIOA_BSRR (S12_GPIOA_BS << 16) + S13_GPIOA_BS
#define S14_GPIOA_BSRR (S13_GPIOA_BS << 16) + S14_GPIOA_BS
#define S21_GPIOA_BSRR (S14_GPIOA_BS << 16) + S21_GPIOA_BS
#define S22_GPIOA_BSRR (S21_GPIOA_BS << 16) + S22_GPIOA_BS
#define S23_GPIOA_BSRR (S22_GPIOA_BS << 16) + S23_GPIOA_BS
#define S24_GPIOA_BSRR (S23_GPIOA_BS << 16) + S24_GPIOA_BS
#define S31_GPIOA_BSRR (S24_GPIOA_BS << 16) + S31_GPIOA_BS
#define S32_GPIOA_BSRR (S31_GPIOA_BS << 16) + S32_GPIOA_BS
#define S33_GPIOA_BSRR (S32_GPIOA_BS << 16) + S33_GPIOA_BS
#define S34_GPIOA_BSRR (S33_GPIOA_BS << 16) + S34_GPIOA_BS
#define S41_GPIOA_BSRR (S34_GPIOA_BS << 16) + S41_GPIOA_BS
#define S42_GPIOA_BSRR (S41_GPIOA_BS << 16) + S42_GPIOA_BS
#define S43_GPIOA_BSRR (S42_GPIOA_BS << 16) + S43_GPIOA_BS

#define S11_GPIOB_BSRR (S43_GPIOB_BS << 16) + S11_GPIOB_BS
#define S12_GPIOB_BSRR (S11_GPIOB_BS << 16) + S12_GPIOB_BS
#define S13_GPIOB_BSRR (S12_GPIOB_BS << 16) + S13_GPIOB_BS
#define S14_GPIOB_BSRR (S13_GPIOB_BS << 16) + S14_GPIOB_BS
#define S21_GPIOB_BSRR (S14_GPIOB_BS << 16) + S21_GPIOB_BS
#define S22_GPIOB_BSRR (S21_GPIOB_BS << 16) + S22_GPIOB_BS
#define S23_GPIOB_BSRR (S22_GPIOB_BS << 16) + S23_GPIOB_BS
#define S24_GPIOB_BSRR (S23_GPIOB_BS << 16) + S24_GPIOB_BS
#define S31_GPIOB_BSRR (S24_GPIOB_BS << 16) + S31_GPIOB_BS
#define S32_GPIOB_BSRR (S31_GPIOB_BS << 16) + S32_GPIOB_BS
#define S33_GPIOB_BSRR (S32_GPIOB_BS << 16) + S33_GPIOB_BS
#define S34_GPIOB_BSRR (S33_GPIOB_BS << 16) + S34_GPIOB_BS
#define S41_GPIOB_BSRR (S34_GPIOB_BS << 16) + S41_GPIOB_BS
#define S42_GPIOB_BSRR (S41_GPIOB_BS << 16) + S42_GPIOB_BS
#define S43_GPIOB_BSRR (S42_GPIOB_BS << 16) + S43_GPIOB_BS

uint16_t measure_sensors(int16_t* sensor_values_lsb, const uint8_t num_measurements_per_sensor);
void measure_sensors_with_dc_offset(int16_t* sensor_values_lsb, const uint8_t num_measurements_per_sensor);
int check_sensor_currents();
void calculate_crosstalk(int16_t* crosstalk_values_lsb);

#endif /* INC_SENSOR_H_ */
