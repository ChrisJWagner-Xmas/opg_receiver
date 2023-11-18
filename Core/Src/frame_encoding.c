/*
 * frame_encoding.c
 *
 *  Created on: Dec 11, 2021
 *      Author: chris
 */

#include <stdint.h> // for uint8_t etc.

/**
 * @brief Calculates the 8-bit rolling-sum checksum for a byte array.
 * @param buffer: pointer to the buffer array.
 * @param startIndex: start index from which the checksum is calculated.
 * @param stopIndex: stop index (not including it) until the checksum is calculated.
 * @return None.
 */
uint8_t calculate_checksum(uint8_t* buffer, uint32_t startIndex, uint32_t stopIndex)
{
	uint8_t checksum = 0;

	for(int index = startIndex; index < stopIndex; index++)
	{
		checksum += buffer[index];
	}

	return checksum;
}
