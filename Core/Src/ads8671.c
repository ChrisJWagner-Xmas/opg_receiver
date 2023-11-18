#include "ads8671.h"
#include "main.h"

void serialize_cmd_msb_first(uint8_t spi_tx_buffer[], uint32_t cmd_word, uint32_t num_bytes)
{
	// Serialize MSB first.
	int shift = 8*(num_bytes-1);
	for(int byte_index=0; byte_index<num_bytes; byte_index++)
	{
		spi_tx_buffer[byte_index] = (cmd_word >> shift);
		shift -= 8;
	}
}

uint32_t spi_rx_word()
{
	uint8_t spi_rx_buffer[4] = {0};

	// CS -> LOW
	GPIOA->BSRR = SPI_CS_LOW;
	// HAL_GPIO_WritePin(GPIOA, SPI1_MANUAL_CS_Pin, GPIO_PIN_RESET);

	// Receive a 32 bit word byte wise.
	for(int byte_index = 0; byte_index < 4; byte_index++)
	{
		// Wait for the SPI to finish tx/rx communication.
		while(SPI1->SR & SPI_SR_BSY){}
		SPI1->DR = 0xAA; // Write something to the DR to trigger the spi clock.
		// wait for reception to finish
		while((SPI1->SR & SPI_SR_RXNE) == 0){}

		spi_rx_buffer[byte_index] = SPI1->DR;
	}
	// CS -> HIGH.
	GPIOA->BSRR = SPI_CS_HIGH;

	// Reconstruct data word (32 bit) MSB first.
	uint32_t data_word = (spi_rx_buffer[0] << 24)
					   + (spi_rx_buffer[1] << 16)
					   + (spi_rx_buffer[2] << 8)
					   + spi_rx_buffer[3];

	return data_word;
}


void spi_tx_word(uint32_t tx_cmd_word)
{
	// TODO: is buffer alloc and sending slower than
	// serializing the word right before sending?

	uint8_t spi_tx_buffer[4] = {0};

	// Fast serialization with MSB fist.
	spi_tx_buffer[0] = tx_cmd_word >> 24;
	spi_tx_buffer[1] = tx_cmd_word >> 16;
	spi_tx_buffer[2] = tx_cmd_word >> 8;
	spi_tx_buffer[3] = tx_cmd_word;

	// CS -> LOW
	GPIOA->BSRR = SPI_CS_LOW;

	// Transmit a 32 bit word byte wise.
	for(int byte_index = 0; byte_index < 4; byte_index++)
	{
		// Wait for the TX bit to set, indicating tx buffer is empty.
		while((SPI1->SR & SPI_SR_TXE) == 0){}
		// Fill the tx buffer and trigger the clock for simul. tx.
		// Ignore any rx bytes.
		SPI1->DR = spi_tx_buffer[byte_index];
	}

	// Before exiting, wait for the last TX to finish,
	// for the BSY flag to lift and read the DR/SR again
	// (https://controllerstech.com/spi-using-registers-in-stm32/)
	while((SPI1->SR & SPI_SR_TXE) == 0){}
	while(SPI1->SR & SPI_SR_BSY){}

	uint8_t garbage_val = SPI1->DR;
	garbage_val = SPI1->SR;
	// CS -> HIGH.
	GPIOA->BSRR = SPI_CS_HIGH;
}


uint32_t spi_rx_tx_word(uint32_t tx_cmd_word)
{
	uint8_t spi_rx_buffer[4] = {0};
	uint8_t spi_tx_buffer[4] = {0};

	// Fast serialization with MSB fist.
	spi_tx_buffer[0] = tx_cmd_word >> 24;
	spi_tx_buffer[1] = tx_cmd_word >> 16;
	spi_tx_buffer[2] = tx_cmd_word >> 8;
	spi_tx_buffer[3] = tx_cmd_word;

	// CS -> LOW
	GPIOA->BSRR = SPI_CS_LOW;

	// Receive a 32 bit word byte wise.
	for(int byte_index = 0; byte_index < 4; byte_index++)
	{
		// Fill the tx buffer and trigger the clock for simul. tx and rx.
		SPI1->DR = spi_tx_buffer[byte_index];

		while((SPI1->SR & SPI_SR_RXNE) == 0)
		{
		}
		spi_rx_buffer[byte_index] = SPI1->DR;
	}

	// CS -> HIGH.
	GPIOA->BSRR = SPI_CS_HIGH;

	// Reconstruct received data word (32 bit) MSB first.
	uint32_t data_word = (spi_rx_buffer[0] << 24)
					   + (spi_rx_buffer[1] << 16)
					   + (spi_rx_buffer[2] << 8)
					   + spi_rx_buffer[3];

	return data_word;
}


uint16_t get_ads8671_value_lsb()
{
	uint8_t spi_rx_buffer[4]; // TODO: GGF. MEMORY ALLOC. OUTSIDE OF FUNC.

	// CS -> LOW
	GPIOA->BSRR = SPI_CS_LOW;

	// Receive a 32 bit word byte wise.
	for(int byte_index = 0; byte_index < 4; byte_index++)
	{
		SPI1->DR = 0; // Write something to the DR to trigger the spi clock.
					  // As long as it is not a valid data word for the ads8671,
		  	  	  	  // it will return the current adc value.
		while(!(SPI1->SR & SPI_SR_RXNE))
		{
			asm("NOP");
		}
		spi_rx_buffer[byte_index] = SPI1->DR;
	}

	// CS -> HIGH.
	GPIOA->BSRR = SPI_CS_HIGH;

	// Reconstruct the adc value from the first two MSBs.
	uint16_t adc_value_lsb = ((spi_rx_buffer[0] << 6) + (spi_rx_buffer[1] >> 2));

	return adc_value_lsb;
}


/* "Overloaded" adc function with exernal memory management.
 * The number of received bytes is fixed to 4, still.
 */
uint16_t get_ads8671_value_ext_lsb(uint8_t *spi_rx_buffer)
{
	// CS -> LOW
	GPIOA->BSRR = SPI_CS_LOW;

	// Receive a 32 bit word byte wise.
	for(int byte_index = 0; byte_index < 4; byte_index++)
	{
		SPI1->DR = 0; // Write something to the DR to trigger the spi clock.
					  // As long as it is not a valid data word for the ads8671,
		  	  	  	  // it will return the current adc value.
		while(!(SPI1->SR & SPI_SR_RXNE))
		{
			asm("NOP");
		}
		spi_rx_buffer[byte_index] = SPI1->DR;
	}

	// CS -> HIGH.
	GPIOA->BSRR = SPI_CS_HIGH;

	// Reconstruct the adc value from the first two MSBs.
	uint16_t adc_value_lsb = ((spi_rx_buffer[0] << 6) + (spi_rx_buffer[1] >> 2));

	return adc_value_lsb;
}






