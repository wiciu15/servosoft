/*
 * mitsubishi_encoder.h
 *
 *  Created on: Feb 15, 2022
 *      Author: Wiktor
 */

#ifndef INC_MITSUBISHI_ENCODER_H_
#define INC_MITSUBISHI_ENCODER_H_

#include "main.h"


typedef enum  {encoder_ok, encoder_error_cheksum, encoder_error_acceleration, encoder_error_no_communication,encoder_error_uart_busy}encoder_state_t;
typedef enum  {unknown_resolution,p8192ppr,p131072ppr}encoder_resolution_t;
typedef enum  {unknown_family,j2,j2super,hf}motor_family_t;
typedef enum  {unknown_formfactor,mf,kf,kn}motor_formfactor_t;
typedef struct _ssi_encoder_data_t {
	encoder_state_t encoder_state;
	uint32_t encoder_position;
	uint32_t last_encoder_position;
	uint32_t last_encoder_position_speed_loop;
	uint8_t checksum_error_count;
	uint8_t excessive_acceleration_error_count;
	uint8_t motor_data_response_packet[9];//full response to 0xA2 command
	uint8_t encoder_command;
	encoder_resolution_t encoder_resolution;
	motor_family_t motor_family;
	motor_formfactor_t motor_formfactor;
	uint16_t motor_power;
	uint16_t motor_speed;
}ssi_encoder_data_t;
extern ssi_encoder_data_t ssi_encoder_data;

void motor_identification(void);
void mitsubishi_encoder_read_position(void);


#endif /* INC_MITSUBISHI_ENCODER_H_ */
