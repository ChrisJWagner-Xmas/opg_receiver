/*
 * frame_encoding.h
 *
 *  Created on: Dec 11, 2021
 *      Author: chris
 */

#ifndef INC_FRAME_ENCODING_H_
#define INC_FRAME_ENCODING_H_

#define FRAME_NUMBER_BYTES 4
#define BYTES_PER_SENSOR_VALUE 2
#define CURRENT_BYTES 2

uint8_t calculate_checksum(uint8_t* buffer, uint32_t startIndex, uint32_t stopIndex);

#endif /* INC_FRAME_ENCODING_H_ */
