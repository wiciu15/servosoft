#include "mitsubishi_encoder.h"
#include "cmsis_os.h"


extern UART_HandleTypeDef huart2;
uint8_t UART2_RX_raw[9];
ssi_encoder_data_t ssi_encoder_data;

void motor_identification(void){
	//first send 2 packets with 0x92 command, then 8 with 0x7A command (motor data read)
	uint8_t command = 0x92;
	for(uint8_t i=0;i<10;i++){
		if(i>2){command=0x7A;}
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13, 1);
		HAL_UART_Transmit(&huart2, &command, 1, 100);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13, 0);
		osDelay(1);
		HAL_UART_Receive_DMA(&huart2, UART2_RX_raw, 9);
		osDelay(1);
	}
	//check if encoder sent data back ok
	if(UART2_RX_raw[1]!=0x21){ssi_encoder_data.encoder_state=encoder_error_no_communication;}
	else{
		ssi_encoder_data.encoder_state=encoder_ok;
		//determine motor family and encoder resolution
		if(UART2_RX_raw[2]==0x41){ssi_encoder_data.encoder_resolution=p131072ppr;ssi_encoder_data.motor_family=j2super;}
		else if(UART2_RX_raw[2]==0x3D){ssi_encoder_data.encoder_resolution=p8192ppr;ssi_encoder_data.motor_family=j2;}
		else if(UART2_RX_raw[2]==75){ssi_encoder_data.encoder_resolution=p131072ppr;ssi_encoder_data.motor_family=hf;}
		else{ssi_encoder_data.encoder_resolution=p131072ppr;ssi_encoder_data.motor_family=unknown_family;}
		//determine form factor
		switch(UART2_RX_raw[3]){
		case 0x12:
			ssi_encoder_data.motor_formfactor=kf;break;
		case 0x02:
			ssi_encoder_data.motor_formfactor=mf;break;
		case 0x0F:
			ssi_encoder_data.motor_formfactor=kn;break;
		default:
			ssi_encoder_data.motor_formfactor=unknown_formfactor;break;
		}
		//determine speed and power
		ssi_encoder_data.motor_power=(UART2_RX_raw[4] >> 4)*100;
		ssi_encoder_data.motor_speed=(UART2_RX_raw[4] & 0x0F)*1000;
	}
}

void mitsubishi_encoder_read_position(void){
	uint8_t xor_cheksum=0;
	for(uint8_t i=0;i<8;i++){
		xor_cheksum^=UART2_RX_raw[i];
	}
	if(xor_cheksum!=UART2_RX_raw[8]){
		ssi_encoder_data.checksum_error_count++;
		if(ssi_encoder_data.checksum_error_count>3 && ssi_encoder_data.encoder_state!=encoder_error_no_communication){ssi_encoder_data.encoder_state=encoder_error_cheksum;}
		if(ssi_encoder_data.checksum_error_count>100){ssi_encoder_data.encoder_state=encoder_error_no_communication;}
	}
	else{ //calculate position and speed from received earlier data
		if(ssi_encoder_data.encoder_resolution==p131072ppr){
			ssi_encoder_data.last_encoder_position=ssi_encoder_data.encoder_position;
			if(UART2_RX_raw[0]==0xA2){ssi_encoder_data.encoder_position=UART2_RX_raw[2]>>3 | UART2_RX_raw[3]<<5 | UART2_RX_raw[4]<<13;}
			if(ssi_encoder_data.encoder_position>131073){ssi_encoder_data.encoder_position=ssi_encoder_data.last_encoder_position;}//error handling
			int32_t speed = ssi_encoder_data.last_encoder_position-ssi_encoder_data.encoder_position;
			if(((speed>2000) && (speed<129000))||
					((speed<(-2000)) && (speed>(-129000)))){
				ssi_encoder_data.excessive_acceleration_error_count++;
				if(ssi_encoder_data.excessive_acceleration_error_count>2){ssi_encoder_data.encoder_state=encoder_error_acceleration;}
			}
		}
		if(ssi_encoder_data.encoder_resolution==p8192ppr){
			ssi_encoder_data.last_encoder_position=ssi_encoder_data.encoder_position;
			if(UART2_RX_raw[0]==0x1A){ssi_encoder_data.encoder_position=UART2_RX_raw[2] | UART2_RX_raw[3]<<8;}
			if(ssi_encoder_data.encoder_position>8193){ssi_encoder_data.encoder_position=ssi_encoder_data.last_encoder_position;}//error handling
			int32_t speed = ssi_encoder_data.last_encoder_position-ssi_encoder_data.encoder_position;
			if(((speed>50) && (speed<8100))||
					((speed<(-50)) && (speed>(-8100)))){
				ssi_encoder_data.excessive_acceleration_error_count++;
				if(ssi_encoder_data.excessive_acceleration_error_count>2){ssi_encoder_data.encoder_state=encoder_error_acceleration;}
			}
		}
	}

	//set command to send to encoder depending on its type
	if(ssi_encoder_data.encoder_resolution==p8192ppr){ssi_encoder_data.encoder_command=0x1A;}//j2s series also gives position after this command but resolution is limited to 16-bit
	else {ssi_encoder_data.encoder_command=0xA2;}
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13, 1);
	HAL_StatusTypeDef err_code = HAL_UART_Transmit(&huart2, &ssi_encoder_data.encoder_command, 1,1);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13, 0);
	if(err_code!=HAL_OK){
		ssi_encoder_data.encoder_state=encoder_error_uart_busy;
	}
	HAL_UART_Receive_DMA(&huart2, UART2_RX_raw, 9);//start listening for response, it will be automatically copied by DMA after reception
}
