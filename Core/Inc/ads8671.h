/*
 * ads8671.h
 *
 *  Created on: Dec 5, 2021
 *      Author: chris
 */

#ifndef INC_ADS8671_H_
#define INC_ADS8671_H_

#include <stdint.h> // for uint8_t etc.

// ADS8671 instructions. Registers are 32 bit wide.
// Syntax is: [<OP-code (8 bits)>, <8-bit address>, <data MSB>, <data LSB>]

#define ADS8671_OPC_NOP 0
// Op-codes for writing (including the _xx_ field and the trailing 0).
#define ADS8671_OPC_CLEAR_HWORD 0b11000000
#define ADS8671_OPC_READ_HWORD 0b11001000
#define ADS8671_OPC_READ 0b01001000
#define ADS8671_OPC_WRITE 0b11010000
#define ADS8671_OPC_WRITE_MSB_ONLY 0b11010010
#define ADS8671_OPC_WRITE_LSB_ONLY 0b11010100
#define ADS8671_OPC_SET_HWORD 0b11011000

// 8-bit register addresses.
#define DEVICE_ID_REG_BASE 0x00
#define DEVICE_ID_REG_DEVICE_ADDR 0x02 // bit field [19:16]
#define RST_PWRCTL_REG_BASE 0x04
#define SDI_CTL_REG_BASE 0x08
#define SDO_CTL_REG_BASE  0x0C
#define DATAOUT_CTL_REG_BASE 0x10
#define RANGE_SEL_REG_BASE 0x14
#define ALARM_REG_BASE 0x20
#define ALARM_H_TH_REG_BASE 0x24
#define ALARM_L_TH_REG_BASE 0x28

// Spi command to write the ID=7 (arbitrary) to the device and read it back (for comm. testing).
#define SPI_CS_HIGH GPIO_BSRR_BS4   // Chip select GPIO pin register value.
#define SPI_CS_LOW GPIO_BSRR_BR4 // Chip deselect GPIO register value.
#define DEVICE_ID 5 // Can be any integer from [0,15].
#define RANGE_SELECTION 0x08
// TODO: RANGE_SELECTION 0x08 did cause the ADC to read [val,val,0,0,val,val,0,0] -> but resolved itself (?).

#define WRITE_ID_TO_DEVICE ((uint32_t)ADS8671_OPC_WRITE << 24) + ((uint32_t)DEVICE_ID_REG_DEVICE_ADDR << 16) + DEVICE_ID
#define READ_ID_FROM_DEVICE ((uint32_t)ADS8671_OPC_READ << 24) + ((uint32_t)DEVICE_ID_REG_DEVICE_ADDR << 16) // data 0's omitted.
#define WRITE_RANGE_SELECTION ((uint32_t)ADS8671_OPC_WRITE << 24) + ((uint32_t)RANGE_SEL_REG_BASE << 16) + RANGE_SELECTION
#define READ_RANGE_SEL_REG ((uint32_t)ADS8671_OPC_READ << 24) + ((uint32_t)RANGE_SEL_REG_BASE << 16) // data 0's omitted.

// Equations:
// adc_value/ADC_RESOLUTION_LSB*3*4.096 for 0...3 x Vref
// adc_value/ADC_RESOLUTION_LSB*6*4.096 - 3*4.096 // for +-3 x Vref
#define ADC_MODULE_MAX_VALUE_LSB (double)16383.0
#define ADC_MODULE_RANGE_POS (double)3*4.096

// ********* FUNCTIONS **************************
void serialize_cmd_msb_first(uint8_t spi_tx_buffer[], uint32_t cmd_word, uint32_t num_bytes);
uint32_t spi_rx_word();
void spi_tx_word(uint32_t tx_cmd_word);
uint32_t spi_rx_tx_word(uint32_t tx_cmd_word); // TODO: TEST
uint16_t get_ads8671_value_lsb();
uint16_t get_ads8671_value_ext_lsb(uint8_t *spi_rx_buffer);

#endif /* INC_ADS8671_H_ */
