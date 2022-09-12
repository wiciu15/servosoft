/*
 * tamagawa_encoder.h
 *
 *  Created on: 12 wrz 2022
 *      Author: Wiktor
 */

#ifndef INC_TAMAGAWA_ENCODER_H_
#define INC_TAMAGAWA_ENCODER_H_
#include "main.h"
#include "mitsubishi_encoder.h"

typedef struct _tamagawa_encoder_data_t {
	encoder_state_t encoder_state;
	uint32_t encoder_position;
	uint32_t last_encoder_position;
	uint32_t last_encoder_position_speed_loop;
	uint8_t checksum_error_count;
	uint8_t excessive_acceleration_error_count;
	uint8_t motor_data_response_packet[10];//full response to 0xA2 command
	uint8_t encoder_command;
}tamagawa_encoder_data_t;
extern tamagawa_encoder_data_t tamagawa_encoder_data;

void motor_encoder_read_position(void);

#endif /* INC_TAMAGAWA_ENCODER_H_ */
