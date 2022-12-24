#include "mitsubishi_encoder.h"
#include "cmsis_os.h"
#include "inverter.h"
#include <string.h>

extern UART_HandleTypeDef huart2;
uint8_t UART2_RX_raw[9];
mitsubishi_encoder_data_t mitsubishi_encoder_data;
HAL_StatusTypeDef status;

//very fast register level UART transmit of 1 byte with RE/DE gpio line support.
HAL_StatusTypeDef USART2_fast_transmit_RS485(uint8_t byte_to_send){
	uint32_t timeout_clock=0;
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13, 1);
	huart2.Instance->DR=byte_to_send;
	while(!__HAL_UART_GET_FLAG(&huart2,UART_FLAG_TC)){timeout_clock++;if(timeout_clock>30){break;}	} //break out of loop if transfer is not finished within <1us to prevent blocking the CPU in case of error
	GPIOC->BSRR = (uint32_t)GPIO_PIN_13 << 16U;
	if(timeout_clock<30){return HAL_OK;}
	return HAL_ERROR;
}

void mitsubishi_motor_identification(void){
	//first send 4 packets with 0x92 command, then 8 with 0x7A command (motor data read)
	uint8_t command = 0x92;
	for(uint8_t i=0;i<=12;i++){
		if(i>=4){command=0x7A;}
		if(HAL_UART_Receive_DMA(&huart2, UART2_RX_raw, 9)){inverter_error_trip(internal_software);}
		//HAL_UART_Transmit_DMA(&huart2, &command, 1); THIS IS TOO SLOW for j2s encoders in half-duplex
		if(USART2_fast_transmit_RS485(command)){inverter_error_trip(internal_software);}
		osDelay(1);
	}
	//check if encoder sent data back ok
	memcpy(&mitsubishi_encoder_data.motor_data_response_packet,&UART2_RX_raw,9);
	if(UART2_RX_raw[1]!=0x21){mitsubishi_encoder_data.encoder_state=encoder_error_no_communication;}
	else{
		mitsubishi_encoder_data.encoder_state=encoder_ok;
		//determine motor family and encoder resolution

		if(UART2_RX_raw[2]==0x3D){mitsubishi_encoder_data.encoder_resolution=8192;mitsubishi_encoder_data.motor_family=j2_13bit;} //j2 encoder
		else if(UART2_RX_raw[2]==0x3C){mitsubishi_encoder_data.encoder_resolution=16384;mitsubishi_encoder_data.motor_family=j2_14bit;} //j2 encoder
		else if(UART2_RX_raw[2]==0x41){mitsubishi_encoder_data.encoder_resolution=131072;mitsubishi_encoder_data.motor_family=j2super;} //j2super 17 bit encoders
		else if(UART2_RX_raw[2]==0x4B){mitsubishi_encoder_data.encoder_resolution=131072;mitsubishi_encoder_data.motor_family=je;} //mr-je/mr-e encoder 17bit
		else if(UART2_RX_raw[2]==0x44){mitsubishi_encoder_data.encoder_resolution=262144;mitsubishi_encoder_data.motor_family=j3j4;} //j3 and j4 encoder have the same encoder id but j4 claims 22bit resolution, maybe different command is needed
		else{mitsubishi_encoder_data.encoder_resolution=131072;mitsubishi_encoder_data.motor_family=unknown_family;}
		mitsubishi_encoder_data.motor_series_id=UART2_RX_raw[3];
		//determine speed and power
		mitsubishi_encoder_data.motor_power=(UART2_RX_raw[4] >> 4)*100;
		mitsubishi_encoder_data.motor_speed=(UART2_RX_raw[4] & 0x0F)*1000;
		//set command to send to encoder depending on its type
		if(mitsubishi_encoder_data.encoder_resolution==8192){mitsubishi_encoder_data.encoder_command=0x1A;}//j2s series also gives position after this command but resolution is limited to 16-bit
		else {mitsubishi_encoder_data.encoder_command=0xA2;}
	}
}

void mitsubishi_encoder_process_data(void){
	uint8_t xor_cheksum=0;
	for(uint8_t i=0;i<8;i++){
		xor_cheksum^=UART2_RX_raw[i];
	}
	if(xor_cheksum!=UART2_RX_raw[8]){
		mitsubishi_encoder_data.checksum_error_count++;
		if(mitsubishi_encoder_data.checksum_error_count>3 && mitsubishi_encoder_data.encoder_state!=encoder_error_no_communication){mitsubishi_encoder_data.encoder_state=encoder_error_cheksum;}
		if(mitsubishi_encoder_data.checksum_error_count>100){mitsubishi_encoder_data.encoder_state=encoder_error_no_communication;}
	}
	else{ //calculate position and speed from received earlier data
		if(mitsubishi_encoder_data.encoder_resolution==131072){
			mitsubishi_encoder_data.last_encoder_position=mitsubishi_encoder_data.encoder_position;
			if(UART2_RX_raw[0]==0xA2){mitsubishi_encoder_data.encoder_position=UART2_RX_raw[2]>>3 | UART2_RX_raw[3]<<5 | UART2_RX_raw[4]<<13;}
			if(mitsubishi_encoder_data.encoder_position>131073){mitsubishi_encoder_data.encoder_position=mitsubishi_encoder_data.last_encoder_position;}//error handling
			int32_t speed = mitsubishi_encoder_data.last_encoder_position-mitsubishi_encoder_data.encoder_position;
			if(((speed>2000) && (speed<129000))||
					((speed<(-2000)) && (speed>(-129000)))){
				mitsubishi_encoder_data.excessive_acceleration_error_count++;
				if(mitsubishi_encoder_data.excessive_acceleration_error_count>2){mitsubishi_encoder_data.encoder_state=encoder_error_acceleration;}
			}
		}
		if(mitsubishi_encoder_data.encoder_resolution==8192){
			mitsubishi_encoder_data.last_encoder_position=mitsubishi_encoder_data.encoder_position;
			if(UART2_RX_raw[0]==0x1A){mitsubishi_encoder_data.encoder_position=UART2_RX_raw[2] | UART2_RX_raw[3]<<8;}
			if(mitsubishi_encoder_data.encoder_position>8193){mitsubishi_encoder_data.encoder_position=mitsubishi_encoder_data.last_encoder_position;}//error handling
			int32_t speed = mitsubishi_encoder_data.last_encoder_position-mitsubishi_encoder_data.encoder_position;
			if(((speed>50) && (speed<8100))||
					((speed<(-50)) && (speed>(-8100)))){
				mitsubishi_encoder_data.excessive_acceleration_error_count++;
				if(mitsubishi_encoder_data.excessive_acceleration_error_count>2){mitsubishi_encoder_data.encoder_state=encoder_error_acceleration;}
			}
		}
	}


}

void mitsubishi_encoder_send_command(void){
	if(HAL_UART_Receive_DMA(&huart2, UART2_RX_raw, 9)==HAL_BUSY){mitsubishi_encoder_data.communication_error_count++;};
	//HAL_UART_Transmit_DMA(&huart2, &command, 1); THIS IS TOO SLOW for j2s encoders in half-duplex
	if(USART2_fast_transmit_RS485(mitsubishi_encoder_data.encoder_command)){inverter_error_trip(internal_software);}
}
