/*
 * comm_modbus.c
 *
 *  Created on: Feb 27, 2022
 *      Author: Wiktor
 */

#include "main.h"
#include "modbus.h"
#include <string.h>
#include "inverter.h"
#include "mitsubishi_encoder.h"
#include "tamagawa_encoder.h"

mbus_t modbus;
Modbus_Conf_t mb_config;

uint8_t UART_RX_buf[250];
uint8_t UART_RX_FIFO [500];
uint16_t fifo_oldpos;
uint16_t fifo_newpos;
uint16_t fifo_read_pos;

uint8_t modbusSendBufer[250];
uint8_t modbusReceiveBufer[250];



extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;

uint16_t modbus_protocol_read(uint32_t la){

	uint8_t local_address=la-40000;
	uint16_t response=0xFFFF;
	switch (local_address){
	case 2: response = inverter_error;break;
	case 3: response = inverter_state;break;
	case 5: response = control_mode;break;
	case 6:{if(control_mode==manual || control_mode==open_loop_current){response = (int16_t)(speed_setpoint_deg_s/6.0f);}if(control_mode==foc){response = speed_setpoint_rpm;} break;}
	case 7:{if(control_mode==manual){response = (duty_cycle/(float)duty_cycle_limit)*100.0f;}if(control_mode==open_loop_current || control_mode == foc){response = (int16_t)((torque_setpoint/parameter_set.motor_nominal_current)*1000.0f);}break;}
	case 8:{if(control_mode==foc){response = (int16_t)((id_setpoint/parameter_set.motor_nominal_current)*1000.0f);}break;}
	case 10: response = (int16_t)(I_out *100.0f);break;
	case 11: response = (uint16_t)(actual_electric_angle);break;
	case 12: response = (int16_t)(actual_torque_angle);break;
	case 13: response = (int16_t)(filtered_speed);break;
	case 14: response = (uint16_t)(U_DClink_filtered*10.0f);break;
	case 15: response = (int16_t)(I_d_filtered*100.0f);break;
	case 16: response = (int16_t)(I_q_filtered*100.0f);break;
	case 17: response = encoder_actual_position;break;
	case 18: response = calculated_voltage_vector*10.0f;break;

	case 20: response = parameter_set.motor_feedback_type;break;
	case 21: response = parameter_set.encoder_electric_angle_correction;break;
	case 22: response = parameter_set.encoder_resolution;break;
	case 23: if(parameter_set.motor_feedback_type==mitsubishi_encoder){response = mitsubishi_encoder_data.excessive_acceleration_error_count;}if(parameter_set.motor_feedback_type==tamagawa_encoder){response = tamagawa_encoder_data.excessive_acceleration_error_count;}break;
	case 31: response = parameter_set.motor_pole_pairs;break;
	case 32: response = parameter_set.motor_nominal_current*100.0f;break;
	case 33: response = (parameter_set.motor_max_current/parameter_set.motor_nominal_current)*100.0f;break;
	case 34: response = parameter_set.motor_max_current*100.0f;break;
	case 35: response = parameter_set.motor_max_voltage;break;
	case 36: response = parameter_set.motor_nominal_torque*100.0f;break;
	case 37: response = parameter_set.motor_nominal_speed;break;
	case 38: response = parameter_set.motor_max_speed;break;
	case 39: response = parameter_set.motor_rs*1000.0f;break;
	case 40: response = parameter_set.motor_ls*1000000.0f;break;
	case 41: response = parameter_set.motor_K*100000.0f;break;
	case 50: response = parameter_set.torque_current_ctrl_proportional_gain*10.0f;break;
	case 51: response = parameter_set.torque_current_ctrl_integral_gain;break;
	case 52: response = parameter_set.field_current_ctrl_proportional_gain*10.0f;break;
	case 53: response = parameter_set.field_current_ctrl_integral_gain;break;
	case 54: response = parameter_set.current_filter_ts*10000.0f;break;
	case 55: response = parameter_set.speed_controller_proportional_gain*1000.0f;break;
	case 56: response = parameter_set.speed_controller_integral_gain*1000.0f;break;
	case 57: response = parameter_set.speed_filter_ts*10000.0f;break;
	}
	return (uint16_t)response;
}

uint16_t modbus_protocol_write(uint32_t la, uint16_t value)
{
	uint8_t local_address=la-40000;
	switch (local_address){

	case 2://error register
		{if(value==0){inverter_error = no_error;inverter_state=stop;}break; //acknowledge error
		break;}
	case 3: //control register
		{switch(value){
			case 0:
				inverter_disable();inverter_state=stop;break;
			case 1:
				inverter_state=run;inverter_enable();break;
			case 3:
				inverter_error_trip(external_comm);
			default:
				inverter_disable();break;
			}
	break;}

	case 5: //operation mode register
	{if(value<=2){control_mode=value;}
	break;
	}

	case 6: //speed setpoint in rpm
	{int16_t received_speed=value;
	if(control_mode==manual || control_mode==open_loop_current){
		if((received_speed)<=5000 && (received_speed)>=(-5000) ){speed_setpoint_deg_s = (float)received_speed*6.0f;}
	}
	if(control_mode==foc){
		if((received_speed)<=5000 && (received_speed)>=(-5000) ){speed_setpoint_rpm = received_speed;}
	}
	break;}

	case 7: //set output voltage in manual/torque in foc
	{if(control_mode==manual){
		uint16_t received_duty_cycle_percent=value;
		if(value<=1000 && value>=0){duty_cycle = ((float)received_duty_cycle_percent/1000.0f)*(float)duty_cycle_limit;}
		}
	if(control_mode==open_loop_current){
		int16_t received_torque_setpoint = (int16_t)value;
		if(received_torque_setpoint>=0 && received_torque_setpoint<=1000){
			if(speed_setpoint_rpm==0){
				torque_setpoint=(received_torque_setpoint/1000.0f)*parameter_set.motor_nominal_current;
			}
		}
	}
	if(control_mode==foc){
		int16_t received_torque_setpoint = (int16_t)value;
		if(received_torque_setpoint>=-4000 && received_torque_setpoint<=4000){
			if(speed_setpoint_rpm==0){
				torque_setpoint=(received_torque_setpoint/1000.0f)*parameter_set.motor_nominal_current;
			}
		}
	}
	break;
	}
	case 8:
	{
		if(control_mode==foc){
			int16_t received_field_setpoint = value;
			if(received_field_setpoint>=-1000 && received_field_setpoint<=1000){
				id_setpoint=(received_field_setpoint/1000.0f)*parameter_set.motor_nominal_current;
			}
		}
		break;
	}
	//feedback type
	case 20:
	{
		uint16_t received_feedback_type = value;
		if(received_feedback_type<=3 && inverter_state!=run){
			parameter_set.motor_feedback_type=value;}
		break;}
	//Encoder angle correction
	case 21:
	{
		int16_t received_value = value;
		if(received_value>=-180 && received_value<=180){
			parameter_set.encoder_electric_angle_correction=received_value;
		}
		break;}
	//Encoder resolution
	case 22:
	{
		uint16_t received_value = value;
		if(received_value>=250 && received_value<=65536){
			parameter_set.encoder_resolution=received_value;
		}
		break;}
	//Motor pole pairs
	case 31:
	{
		uint16_t received_value = value;
		if(received_value>=1 && received_value<=12){
			parameter_set.motor_pole_pairs=received_value;
		}
		break;}
	//Motor nominal current
	case 32:
	{
		uint16_t received_value = value;
		if(received_value>=1 && received_value<=1600){
			parameter_set.motor_nominal_current=received_value/100.0f;
		}
		break;}
	//Motor overload factor
	case 33:
	{
		uint16_t received_value = value;
		if(received_value>=1 && received_value<=400){
			parameter_set.motor_max_current=parameter_set.motor_nominal_current*(received_value/100.0f);
		}
		break;}
	//Motor max voltage
	case 35:
	{
		uint16_t received_value = value;
		if(received_value>=1 && received_value<=400){
			parameter_set.motor_max_voltage=(float)received_value;
		}
		break;}
	//Motor nominal torque
	case 36:
	{
		uint16_t received_value = value;
		if(received_value>=1 && received_value<=4000){
			parameter_set.motor_nominal_torque=(float)received_value/100.0f;
		}
		break;}
	//Motor nominal speed
	case 37:
	{
		uint16_t received_value = value;
		if(received_value>=1 && received_value<=6000){
			parameter_set.motor_nominal_speed=received_value;
		}
		break;}
	//Motor max speed @TODO: implement speed limiter
	case 38:
	{
		uint16_t received_value = value;
		if(received_value>=1 && received_value<=8000){
			parameter_set.motor_max_speed=received_value;
		}
		break;}
	//Motor Rs
	case 39:
	{
		uint16_t received_value = value;
		if(received_value>=1 && received_value<=60000){
			parameter_set.motor_rs=(float)received_value/1000.0f;
		}
		break;}
	//Motor Ls
	case 40:
	{
		uint16_t received_value = value;
		if(received_value>=1 && received_value<=60000){
			parameter_set.motor_ls=(float)received_value/1000000.0f;
		}
		break;}
	//Motor Kv
	case 41:
	{
		uint16_t received_value = value;
		if(received_value>=1 && received_value<=60000){
			parameter_set.motor_K=(float)received_value/100000.0f;
		}
		break;}
	case 50:
	{
		uint16_t received_value = value;
		if(received_value>=1 && received_value<=900){
			parameter_set.torque_current_ctrl_proportional_gain=(float)received_value/10.0f;
		}
		break;}
	case 51:
	{
		uint16_t received_value = value;
		if(received_value>=1 && received_value<=20000){
			parameter_set.torque_current_ctrl_integral_gain=(float)received_value;
		}
		break;}
	case 52:
	{
		uint16_t received_value = value;
		if(received_value>=1 && received_value<=900){
			parameter_set.field_current_ctrl_proportional_gain=(float)received_value/10.0f;
		}
		break;}

	case 53:
	{
		uint16_t received_value = value;
		if(received_value>=1 && received_value<=20000){
			parameter_set.field_current_ctrl_integral_gain=(float)received_value;
		}
		break;}
	//current filter
	case 54:
	{
		uint16_t received_value = value;
		if(received_value>=1 && received_value<=10000){
			parameter_set.current_filter_ts=(float)received_value/10000.0f;
		}
		break;}
	case 55:
		{
			uint16_t received_value = value;
			if(received_value>=1 && received_value<=20000){
				parameter_set.speed_controller_proportional_gain=(float)received_value/1000.0f;
			}
			break;}
	case 56:
		{
			uint16_t received_value = value;
			if(received_value>=1 && received_value<=20000){
				parameter_set.speed_controller_integral_gain=(float)received_value/1000.0f;
			}
			break;}
	//speed filter
	case 57:
	{
		uint16_t received_value = value;
		if(received_value>=1 && received_value<=10000){
			parameter_set.speed_filter_ts=(float)received_value/10000.0f;
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
		mbus_flush(modbus);
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
