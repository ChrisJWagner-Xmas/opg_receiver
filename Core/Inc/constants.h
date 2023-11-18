/*
 * Header file for various constant values.
 */

// ***********************************************************
// ********************** ADC1 *******************************
// ***********************************************************

#define ADC1_MAX_VALUE_LSB (float)4095.0
#define ADC1_REF_VOLTAGE_V (float)3.3

// ***********************************************************
// *********************** TIMER 3 ***************************
// ***********************************************************

#define TIM3_PERIPHERAL_CLOCK_FREQ_HZ 48000000 // On APB2. TODO: GET FROM SYSTEM WITH HAL_RCC_GetSysClockFreq()
#define MEASURING_FREQUENCY_HZ 100
#define ARR_VALUE 10000 // 1: 1 MHz, 10000: 100 Hz.

// ***********************************************************
// *********************** TIMER 2 ***************************
// ***********************************************************
#define BUTTON_CAPTURE_MIN_FREQ_HZ 10

// ***********************************************************
// ****************** CURRENT SOURCE *************************
// ***********************************************************

// Now in its own header file.

// ***********************************************************
// ********************** Frame encoding *********************
// ***********************************************************

#define NUM_START_BYTES 2
#define CHECKSUM_BYTES 1
#define START_BYTE_0 0xAA
#define START_BYTE_1 0x55
#define FRAME_BUFFER_LENGTH_BYTES (NUM_START_BYTES + FRAME_NUMBER_BYTES + NUM_TX_CURRENT_MEASUREMENTS*CURRENT_BYTES + NUM_SENSORS*BYTES_PER_SENSOR_VALUE + CHECKSUM_BYTES)

// ***********************************************************
// ************************* Misc. ***************************
// ***********************************************************
#define STATUS_LED_B_HIGH GPIO_BSRR_BS0 // On GPIOB
#define STATUS_LED_B_LOW GPIO_BSRR_BR0  // On GPIOB

#define STATUS_LED_RED_B_HIGH GPIO_BSRR_BS1 // On GPIOB
#define STATUS_LED_RED_B_LOW GPIO_BSRR_BR1  // On GPIOB

