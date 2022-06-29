/*
 * inverter.c
 *
 *  Created on: Feb 11, 2022
 *      Author: Wiktor
 */

#include "main.h"
#include "inverter.h"
#include "comm_modbus.h"
#include <stdio.h>
#include <math.h>

extern TIM_HandleTypeDef htim1;
extern UART_HandleTypeDef huart1;
volatile inverter_error_t inverter_error=no_error;

void inverter_enable(){
	if(inverter_error==0){
		speed_setpoint_deg_s=0.0f;
		duty_cycle=0.0f;
		TIM1->CCR1=0;
		TIM1->CCR2=0;
		TIM1->CCR3=0;
		HAL_GPIO_WritePin(INV_ENABLE_GPIO_Port, INV_ENABLE_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(INV_ENABLE_GPIO_Port, INV_ENABLE_Pin, GPIO_PIN_SET);
		HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
		HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
		HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
	}
}

void inverter_disable(){
	speed_setpoint_deg_s=0.0f;
	duty_cycle=0.0f;
	//TIM1->CCR1=0;TIM1->CCR2=0;TIM1->CCR3=0;
	HAL_GPIO_WritePin(INV_DISABLE_GPIO_Port, INV_DISABLE_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(INV_DISABLE_GPIO_Port, INV_DISABLE_Pin, GPIO_PIN_SET);
	HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_2);
	HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_3);
	HAL_TIM_Base_Stop_IT(&htim1);
	HAL_TIM_Base_Start_IT(&htim1); //restart base timer to keep 16khz tim1_update ISR active for reading current and encoder data when inverter not working
}

void inverter_error_trip(inverter_error_t error){
	inverter_disable();
	inverter_error=error;
	modbus_registers_buffer[2]=error;
}

void park_transform(float I_U,float I_V,float motor_angle,float * I_d,float * I_q){
	float Ialfa=I_U;
	float Ibeta=(0.5773502f * I_U) + (1.1547005f * I_V);
	float motor_angle_rad = (motor_angle/180.0f)*3.141592f;
	*I_d = (Ialfa * cosf(motor_angle_rad)) + (Ibeta * sinf(motor_angle_rad));
	*I_q = (Ialfa * sinf(motor_angle_rad)*(-1)) + (Ibeta * cosf(motor_angle_rad));
}


void inv_park_transform(float U_d,float U_q, float motor_angle, float * U_alpha, float * U_beta){
	float motor_angle_rad = (motor_angle/180.0f)*3.141592f;
	*U_alpha= (U_d * cosf(motor_angle_rad)) - (U_q * sinf(motor_angle_rad));
	*U_beta = (U_d * sinf(motor_angle_rad)) + (U_q * cosf(motor_angle_rad));
}

//Tf - filter time constant in seconds
float LowPassFilter(float Tf,float actual_measurement, float * last_filtered_value){
	float alpha = Tf/(Tf + 0.0000625f); //0.0000625 = 1/16kHz - pwm interrupt frequency and sampling
	float filtered_value = (alpha*(*last_filtered_value)) + ((1.0f - alpha)*actual_measurement);
	*last_filtered_value = filtered_value;
	return filtered_value;
}