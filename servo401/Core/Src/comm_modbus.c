/*
 * comm_modbus.c
 *
 *  Created on: Feb 27, 2022
 *      Author: Wiktor
 */

#include "main.h"
#include "modbus.h"
#include "inverter.h"
#include <string.h>

mbus_t modbus;
Modbus_Conf_t mb_config;

uint8_t UART_RX_buf[200];
uint8_t UART_RX_FIFO [500];
uint16_t fifo_oldpos;
uint16_t fifo_newpos;
uint16_t fifo_read_pos;

uint8_t modbusSendBufer[100];
uint8_t modbusReceiveBufer[100];

uint16_t modbus_registers_buffer[30] = {0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}; //modbus holding registers

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;

uint16_t modbus_protocol_read(uint32_t la){

	if(la>=40001 && la<=40030)
	{return modbus_registers_buffer[la-40001];}else{
		return 0xFFFF;
	}
}

uint16_t modbus_protocol_write(uint32_t la, uint16_t value)
{
	uint8_t local_address=la-40001;
	switch (local_address){
	case 2://error register
		if(value==0){inverter_error = no_error;modbus_registers_buffer[2]=value;}break; //acknowledge error
	case 3: //control register
		switch(value){
		case 0:
			modbus_registers_buffer[3] = value;inverter_disable();break;
		case 1:
			modbus_registers_buffer[3] = value;inv_control_mode = modbus_registers_buffer[5];inverter_enable();break;
		default:
			inverter_disable();break;
		}
		break;
	case 5: //operation mode register
		if(value<=2){inv_control_mode=value;
		modbus_registers_buffer[5]=inv_control_mode;}
		break;
	case 6: //speed setpoint in rpm
		{int16_t received_speed=value;
		if((received_speed)<=1000 && (received_speed)>=(-1000) ){speed_setpoint_deg_s = (float)received_speed*6.0f;}
		modbus_registers_buffer[6]=(uint16_t)speed_setpoint_deg_s;
		break;}
	case 7: //set output voltage in manual/torque in foc
		{if(inv_control_mode==manual){
			uint8_t received_duty_cycle_percent=value;
			if(value<=100 && value>=0){duty_cycle = ((float)received_duty_cycle_percent/100.0f)*(float)duty_cycle_limit;}
			modbus_registers_buffer[7]=duty_cycle;
		}
		if(inv_control_mode==foc){
			int8_t received_torque_setpoint = (int16_t)value;
			if(received_torque_setpoint>=-100 && received_torque_setpoint<=100){
			torque_setpoint=received_torque_setpoint;
			modbus_registers_buffer[7]=(int16_t)torque_setpoint;
			}
		}

		break;}
	default:
		//if not handled inside switch, then read-only parameter
		break;
	}
	return value;
}

int mbus_send(const mbus_t context,const uint8_t* data, const uint16_t size){
	UNUSED(context);
	HAL_GPIO_WritePin(MODBUS_DE_GPIO_Port,MODBUS_DE_Pin, 1);
	if(HAL_UART_Transmit_DMA( &huart1, (uint8_t*) data,size)==HAL_OK){
		return MBUS_OK;
	}else{return MBUS_ERROR;}
}

void Modbus_init(){
	/* Device slave address */
	mb_config.devaddr = 0x01;

	/* Just ptr on any external object, you can get it by context */
	mb_config.device = (void*) 0;

	uint8_t * pmodbusSendBuffer;
	pmodbusSendBuffer=&modbusSendBufer;
	mb_config.sendbuf = pmodbusSendBuffer;
	mb_config.sendbuf_sz = sizeof(modbusSendBufer);

	uint8_t * pmodbusRecvBuffer;
	pmodbusRecvBuffer=&modbusReceiveBufer;
	mb_config.recvbuf = pmodbusRecvBuffer;
	mb_config.recvbuf_sz = sizeof(modbusReceiveBufer);

	/* This that function for sending some data (use sendbuf for buf) */
	mb_config.send = &mbus_send;

	Modbus_Conf_t * pconf;
	pconf=&mb_config;
	//User Read callback function ( read by logical address)
	pconf->read = modbus_protocol_read;

	//Write callback function
	pconf->write = modbus_protocol_write;

	//Open modbus contex
	modbus = mbus_open(pconf);

	HAL_UARTEx_ReceiveToIdle_DMA(&huart1, UART_RX_buf, sizeof(UART_RX_buf));
}

void modbus_process_new_data_to_fifo(UART_HandleTypeDef *huart, uint16_t Size){
	if (huart->Instance == USART1)
	{
		/* start the DMA again */
		HAL_UARTEx_ReceiveToIdle_DMA(&huart1, (uint8_t *) UART_RX_buf, sizeof(UART_RX_buf));
		__HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);

		fifo_oldpos = fifo_newpos;  // Update the last position before copying new data

		/* If the data in large and it is about to exceed the buffer size, we have to route it to the start of the buffer
		 * This is to maintain the circular buffer
		 * The old data in the main buffer will be overlapped
		 */
		if (fifo_oldpos+Size > sizeof(UART_RX_FIFO)-1)  // If the current position + new data size is greater than the main buffer
		{
			uint16_t datatocopy = sizeof(UART_RX_FIFO)-fifo_oldpos;  // find out how much space is left in the main buffer
			memcpy ((uint8_t *)UART_RX_FIFO+fifo_oldpos, UART_RX_buf, datatocopy);  // copy data in that remaining space

			fifo_oldpos = 0;  // point to the start of the buffer
			memcpy ((uint8_t *)UART_RX_FIFO, (uint8_t *)UART_RX_buf+datatocopy, (Size-datatocopy));  // copy the remaining data
			fifo_newpos = (Size-datatocopy);  // update the position
		}

		/* if the current position + new data size is less than the main buffer
		 * we will simply copy the data into the buffer and update the position
		 */
		else
		{
			memcpy ((uint8_t *)UART_RX_FIFO+fifo_oldpos, UART_RX_buf, Size);
			fifo_newpos = Size+fifo_oldpos;
		}



	}
}

void process_modbus_command(){
	while(fifo_read_pos!=fifo_newpos){
		mbus_poll(modbus, UART_RX_FIFO[fifo_read_pos] );
		fifo_read_pos++;
		if(fifo_read_pos>=sizeof(UART_RX_FIFO)){
			fifo_read_pos=0;
		}
	}

}
