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
typedef enum  {unknown_family,j2_13bit,j2_14bit,j2super,j3j4,je}motor_family_t;
typedef struct _ssi_encoder_data_t {
	encoder_state_t encoder_state;
	uint32_t encoder_position;
	uint32_t last_encoder_position;
	uint32_t last_encoder_position_speed_loop;
	uint8_t checksum_error_count;
	uint8_t excessive_acceleration_error_count;
	uint8_t communication_error_count;
	uint8_t motor_data_response_packet[9];//full response to 0x7A command containing motor data
	uint8_t encoder_command;
	uint32_t encoder_resolution;
	motor_family_t motor_family;
	uint8_t motor_series_id;
	uint16_t motor_power;
	uint16_t motor_speed;
}mitsubishi_encoder_data_t;
extern mitsubishi_encoder_data_t mitsubishi_encoder_data;

HAL_StatusTypeDef USART2_fast_transmit_RS485(uint8_t byte_to_send);
void mitsubishi_motor_identification(void);
void mitsubishi_encoder_process_data(void);
void mitsubishi_encoder_send_command(void);


#endif /* INC_MITSUBISHI_ENCODER_H_ */
