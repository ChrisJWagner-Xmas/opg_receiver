/*
 * sensor.c
 *
 *  Created on: 25.02.2022
 *      Author: chris
 */


#include "sensor.h"
#include "ads8671.h"
#include "current_source.h"
#include "stm32f1xx_hal.h"

const uint32_t S_row_col_GPIOA_BSRR[NUM_SENSORS] = {
	  S11_GPIOA_BSRR, S12_GPIOA_BSRR, S13_GPIOA_BSRR, S14_GPIOA_BSRR,
	  S21_GPIOA_BSRR, S22_GPIOA_BSRR, S23_GPIOA_BSRR, S24_GPIOA_BSRR,
	  S31_GPIOA_BSRR, S32_GPIOA_BSRR, S33_GPIOA_BSRR, S34_GPIOA_BSRR,
	  S41_GPIOA_BSRR, S42_GPIOA_BSRR, S43_GPIOA_BSRR};

const uint32_t S_row_col_GPIOB_BSRR[NUM_SENSORS] = {
	  S11_GPIOB_BSRR, S12_GPIOB_BSRR, S13_GPIOB_BSRR, S14_GPIOB_BSRR,
	  S21_GPIOB_BSRR, S22_GPIOB_BSRR, S23_GPIOB_BSRR, S24_GPIOB_BSRR,
	  S31_GPIOB_BSRR, S32_GPIOB_BSRR, S33_GPIOB_BSRR, S34_GPIOB_BSRR,
	  S41_GPIOB_BSRR, S42_GPIOB_BSRR, S43_GPIOB_BSRR};

/**
 * Measures the photodiode values for every of the 15 sensors and averages them
 * across the specified number of measurements.
 * The transient timing between opening each PD MUX and onset of the measurement
 * is accounted for in this function.
 * The current source needs to be toggled manually.
 * TODO: TWO FUNCTIONS FOR DC OFFSET AND SENSORS? OR ONE FOR BOTH? DEPENDS ON DYNAMICS.
 * @param int16_t* sensor_values: pointer to the array to store the averaged sensor
 * 								  values in.
 * @param uint8_t num_measurements_per_sensor: number of values each sensor is averaged across.
 * @return uint16_t emitter_current_lsb: The emitter current as a diagnostic.
 */
uint16_t measure_sensors(int16_t* sensor_values_lsb, const uint8_t num_measurements_per_sensor)
{
	uint32_t measurement_buffer[num_measurements_per_sensor]; //  = {0};
	uint8_t spi_rx_buffer[4] = {0};
	uint16_t emitter_current_lsb = 0;

	for(int sensor_index = 0; sensor_index < NUM_SENSORS; sensor_index++)
	{
	  // Select sensor (vcsel + PD).
	  GPIOA->BSRR = S_row_col_GPIOA_BSRR[sensor_index];
	  GPIOB->BSRR = S_row_col_GPIOB_BSRR[sensor_index];

	  // Collect measurements.
	  for(int meas_index = 0; meas_index < num_measurements_per_sensor; meas_index++)
	  {
		  measurement_buffer[meas_index] = get_ads8671_value_ext_lsb(spi_rx_buffer);
	  }

	  // Average across measurements with first index offset'.
	  uint32_t mean_sensor_value_lsb = 0;
	  for(int meas_index = MEASUREMENT_OFFSET_INDEX; meas_index < num_measurements_per_sensor; meas_index++)
	  {
		  mean_sensor_value_lsb += measurement_buffer[meas_index];
	  }

	  sensor_values_lsb[sensor_index] = (int16_t)(mean_sensor_value_lsb/(num_measurements_per_sensor-MEASUREMENT_OFFSET_INDEX));

	  if(sensor_index == 0) // Done because the single optode board has only sensor S11 populated.
	  {
		  emitter_current_lsb = get_emitter_current_lsb();
	  }
	}

	return emitter_current_lsb;
}



void measure_sensors_with_dc_offset(int16_t* sensor_values_lsb, const uint8_t num_measurements_per_sensor)
{
	uint32_t measurement_buffer[num_measurements_per_sensor];
	uint8_t spi_rx_buffer[4] = {0};

	for(int sensor_index = 0; sensor_index < NUM_SENSORS; sensor_index++)
	{
	  // Select sensor (vcsel + PD).
	  GPIOA->BSRR = S_row_col_GPIOA_BSRR[sensor_index];
	  GPIOB->BSRR = S_row_col_GPIOB_BSRR[sensor_index];

	  // Collect DC offset.
	  for(int meas_index = 0; meas_index < num_measurements_per_sensor/2; meas_index++)
	  {
		  measurement_buffer[meas_index] = get_ads8671_value_ext_lsb(spi_rx_buffer);
	  }

	  // Average across offset measurements with first index offset'.
	  uint32_t mean_dc_offset_value_lsb = 0;
	  for(int meas_index = MEASUREMENT_OFFSET_INDEX; meas_index < num_measurements_per_sensor/2; meas_index++)
	  {
		  mean_dc_offset_value_lsb += measurement_buffer[meas_index];
	  }

	  mean_dc_offset_value_lsb = mean_dc_offset_value_lsb/(num_measurements_per_sensor/2-MEASUREMENT_OFFSET_INDEX);

	  // Collect actual measurements.
	  GPIOA->BSRR = CURRENT_SOURCE_A_ON;
	  for(int meas_index = num_measurements_per_sensor/2; meas_index < num_measurements_per_sensor; meas_index++)
	  {
		  measurement_buffer[meas_index] = get_ads8671_value_ext_lsb(spi_rx_buffer);
	  }
	  GPIOA->BSRR = CURRENT_SOURCE_A_OFF;

	  // Average across measurements.
	  uint32_t mean_sensor_value_lsb = 0;
	  for(int meas_index = num_measurements_per_sensor/2 + MEASUREMENT_OFFSET_INDEX; meas_index < num_measurements_per_sensor; meas_index++)
	  {
		  mean_sensor_value_lsb += measurement_buffer[meas_index];
	  }

	  mean_sensor_value_lsb = mean_sensor_value_lsb/(num_measurements_per_sensor/2-MEASUREMENT_OFFSET_INDEX);
	  sensor_values_lsb[sensor_index] = (int16_t)(mean_sensor_value_lsb - mean_dc_offset_value_lsb);
	}
}


/**
 * Function to check each sensor's emitter current and return the number of the sensor
 * which exceeds it.
 * @param: None.
 * @return: failing sensor index or -1 of all sensors are within current limits.
 */
int check_sensor_currents()
{
	int failing_sensor_index = -1;

	GPIOA->BSRR = CURRENT_SOURCE_A_ON;
	HAL_Delay(10); // Let current source settle.

	for(int sensor_index = 0; sensor_index < NUM_SENSORS; sensor_index++)
	{
	  // Read the current sense adc1.
	  float current_mA = get_emitter_current_mA();
	  if(current_mA > MAX_CURRENT_mA)
	  {
		  failing_sensor_index = sensor_index;
		  break;
	  }
	  else
	  {
		  if(current_mA < THRESHOLD_CURRENT_mA)
		  {
			  // Throw warning.
		  }
		  // Set all 8 GPIOs to address the current sensor.
		  // TODO: Check: writing to BSRR register with the current sensor
		  // gpio config in [15:0] and the previous sensor in [31:16]
		  // resets only the old sensor bits and sets the new ones (does
		  // not alter the current source or any other bits that are also
		  // part of the GPIOA/Bs.
		  GPIOA->BSRR = S_row_col_GPIOA_BSRR[sensor_index]; // TODO: Put in function
		  GPIOB->BSRR = S_row_col_GPIOB_BSRR[sensor_index];
	  }
	  HAL_Delay(100); // Delay looping for visual inspection in the test inits.
	}

	GPIOA->BSRR = CURRENT_SOURCE_A_OFF;

	return failing_sensor_index;
}


/**
 * Calculate the amount of cross-talk (in LSB) for each sensor by subtracting
 * the dc offset from the sensor value measured when no object is in front
 * (i.e., at infinite distance).
 * @param crosstalk_values_lsb: array to store the cross-talk values in LSBs.
 * @return None.
 */
void calculate_crosstalk(int16_t* crosstalk_values_lsb)
{
	int16_t offset_values_lsb[NUM_SENSORS] = {0};
	int16_t sensor_values_lsb[NUM_SENSORS] = {0};

	// Measure PD values when the current source is off.
	measure_sensors(offset_values_lsb, 128);

	// Measure PD values when the current source is on with no object in front.
	GPIOA->BSRR = CURRENT_SOURCE_A_ON;
	measure_sensors(sensor_values_lsb, 128);
	GPIOA->BSRR = CURRENT_SOURCE_A_OFF;

	// Subtract the dc offset from the sensor values to get the cross-talk.
	for(int sensor_index = 0; sensor_index < NUM_SENSORS; sensor_index++)
	{
		crosstalk_values_lsb[sensor_index] = sensor_values_lsb[sensor_index]- offset_values_lsb[sensor_index];
	}
}
