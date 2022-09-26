/*
 * tamagawa_encoder.c
 *
 *  Created on: 12 wrz 2022
 *      Author: Wiktor
 */
#include "tamagawa_encoder.h"

extern UART_HandleTypeDef huart2;
//uint8_t UART2_RX_raw[10];
tamagawa_encoder_data_t tamagawa_encoder_data;

void tamagawa_encoder_read_position(void){
	/*uint8_t xor_cheksum=0;
	for(uint8_t i=0;i<9;i++){
		xor_cheksum^=tamagawa_encoder_data.motor_data_response_packet[i];
	}
	if(xor_cheksum!=UART2_RX_raw[9]){
		ssi_encoder_data.checksum_error_count++;
		if(ssi_encoder_data.checksum_error_count>3 && ssi_encoder_data.encoder_state!=encoder_error_no_communication){ssi_encoder_data.encoder_state=encoder_error_cheksum;}
		if(ssi_encoder_data.checksum_error_count>100){ssi_encoder_data.encoder_state=encoder_error_no_communication;}
	}
	else{*/ //calculate position and speed from received earlier data

	tamagawa_encoder_data.last_encoder_position=ssi_encoder_data.encoder_position;
	tamagawa_encoder_data.encoder_position=tamagawa_encoder_data.motor_data_response_packet[2] | tamagawa_encoder_data.motor_data_response_packet[3]<<8 | tamagawa_encoder_data.motor_data_response_packet[4]<<16;
	//if(ssi_encoder_data.encoder_position>262143){ssi_encoder_data.encoder_position=ssi_encoder_data.last_encoder_position;}//@TODO:encoder error handling
	//tamagawa_encoder_data.encoder_position=131072-tamagawa_encoder_data.encoder_position; //switch direction of encoder rotation
	int32_t speed = tamagawa_encoder_data.last_encoder_position-tamagawa_encoder_data.encoder_position;
	if(((speed>2000) && (speed<129000))|| //may not work, not used not tested
			((speed<(-2000)) && (speed>(-129000)))){
		tamagawa_encoder_data.excessive_acceleration_error_count++;
		if(tamagawa_encoder_data.excessive_acceleration_error_count>2){tamagawa_encoder_data.encoder_state=encoder_error_acceleration;}
}
	tamagawa_encoder_data.encoder_command=0xA2;
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13, 1);//toggle driver enable pin in half-duplex rs485
	HAL_StatusTypeDef err_code = HAL_UART_Transmit(&huart2, &tamagawa_encoder_data.encoder_command, 1,1); //using blocking mode TX because DE pin has to be toggled reaaly fast to not break first byte of received data
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13, 0);
	if(err_code!=HAL_OK){
		ssi_encoder_data.encoder_state=encoder_error_uart_busy;
	}
	HAL_UART_Receive_DMA(&huart2, &tamagawa_encoder_data.motor_data_response_packet, 10);//start listening for response, it will be automatically copied by DMA after reception
}

