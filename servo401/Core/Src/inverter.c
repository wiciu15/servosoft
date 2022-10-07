/*
 * inverter.c
 *
 *  Created on: Feb 11, 2022
 *      Author: Wiktor
 */

#include "main.h"
#include "inverter.h"
#include "comm_modbus.h"
#include "lookuptables.h"
#include <stdio.h>
#include <math.h>

extern TIM_HandleTypeDef htim1;
extern UART_HandleTypeDef huart1;
volatile inverter_error_t inverter_error=no_error;


void inverter_enable(){
	if(inverter_error==0){
		inverter_state=run;
		speed_setpoint_deg_s=0.0f;
		duty_cycle=0.0f;
		TIM1->CCR1=0;
		TIM1->CCR2=0;
		TIM1->CCR3=0;
		HAL_GPIO_WritePin(INV_ENABLE_GPIO_Port, INV_ENABLE_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(INV_ENABLE_GPIO_Port, INV_ENABLE_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(SOFTSTART_GPIO_Port, SOFTSTART_Pin, 1);
		HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
		HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
		HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
	}
}

void inverter_disable(){
	speed_setpoint_deg_s=0.0f;
	duty_cycle=0.0f;
	inverter_state=stop;
	//TIM1->CCR1=0;TIM1->CCR2=0;TIM1->CCR3=0;
	HAL_GPIO_WritePin(INV_DISABLE_GPIO_Port, INV_DISABLE_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(INV_DISABLE_GPIO_Port, INV_DISABLE_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(SOFTSTART_GPIO_Port, SOFTSTART_Pin, 0);
	HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_2);
	HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_3);
	HAL_TIM_Base_Stop_IT(&htim1);
	HAL_TIM_Base_Start_IT(&htim1); //restart base timer to keep 16khz tim1_update ISR active for reading current and encoder data when inverter not working
}

void inverter_error_trip(inverter_error_t error){
	inverter_disable();
	inverter_state=trip;
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
	float alpha = Tf/(Tf + 0.0002f); //0.0002 = 1/5kHz - pwm interrupt frequency and sampling
	float filtered_value = (alpha*(*last_filtered_value)) + ((1.0f - alpha)*actual_measurement);
	*last_filtered_value = filtered_value;
	return filtered_value;
}



void output_sine_pwm(float angle,uint16_t duty_cycle){
	float angle_rad = (angle/180.0f)*3.141592f;
	uint16_t commanded_duty_cycle=duty_cycle;
	float sin_u = 0;
	float sin_v = 0;
	float sin_w = 0;
	if(angle>=360){ //fault
		TIM1->CCR1=0;
		TIM1->CCR2=0;
		TIM1->CCR3=0;
	}else{
		if(commanded_duty_cycle>DUTY_CYCLE_LIMIT){commanded_duty_cycle=DUTY_CYCLE_LIMIT;}//prevent timer from writing duty cycle over 100%
		sin_u=cosf(angle_rad);
		sin_v=cosf(angle_rad-2.094395f);//120 deg offset
		sin_w=cosf(angle_rad+2.094395f);
		TIM1->CCR1=(DUTY_CYCLE_LIMIT/2.0f)+sin_u*(commanded_duty_cycle/2.0f);
		TIM1->CCR2=(DUTY_CYCLE_LIMIT/2.0f)+sin_v*(commanded_duty_cycle/2.0f);
		TIM1->CCR3=(DUTY_CYCLE_LIMIT/2.0f)+sin_w*(commanded_duty_cycle/2.0f);
	}
}

void output_svpwm(uint16_t angle,uint16_t duty_cycle){
	uint16_t commanded_duty_cycle=duty_cycle;
	if(commanded_duty_cycle>DUTY_CYCLE_LIMIT){commanded_duty_cycle=DUTY_CYCLE_LIMIT;}//prevent timer from writing duty cycle over 100%
	uint8_t sector=(angle/60)+1;
	float t1=0.0f;
	float t2=0.0f;
	float t0=0.0f;
	if(sector%2==1){
		t1=t1calculated[angle%60]*(float)commanded_duty_cycle;
		t2=t2calculated[angle%60]*(float)commanded_duty_cycle;
		t0=((float)DUTY_CYCLE_LIMIT-t1-t2)/2.0f;
	}else{
		t1=t1calculated[60-(angle%60)]*(float)commanded_duty_cycle;
		t2=t2calculated[60-(angle%60)]*(float)commanded_duty_cycle;
		t0=((float)DUTY_CYCLE_LIMIT-t1-t2)/2.0f;
	}
	switch(sector){
	case 1:
		TIM1->CCR1=(uint32_t)t0+(uint32_t)t1+(uint32_t)t2;
		TIM1->CCR2=(uint32_t)t0+(uint32_t)t2;
		TIM1->CCR3=(uint32_t)t0;
		break;
	case 2:
		TIM1->CCR1=(uint32_t)t0+(uint32_t)t2;
		TIM1->CCR2=(uint32_t)t0+(uint32_t)t1+(uint32_t)t2;
		TIM1->CCR3=(uint32_t)t0;
		break;
	case 3:
		TIM1->CCR1=(uint32_t)t0;
		TIM1->CCR2=(uint32_t)t0+(uint32_t)t1+(uint32_t)t2;
		TIM1->CCR3=(uint32_t)t0+(uint32_t)t2;
		break;
	case 4:
		TIM1->CCR1=(uint32_t)t0;
		TIM1->CCR2=(uint32_t)t0+(uint32_t)t2;
		TIM1->CCR3=(uint32_t)t0+(uint32_t)t1+(uint32_t)t2;
		break;
	case 5:
		TIM1->CCR1=(uint32_t)t0+(uint32_t)t2;
		TIM1->CCR2=(uint32_t)t0;
		TIM1->CCR3=(uint32_t)t0+(uint32_t)t1+(uint32_t)t2;
		break;
	case 6:
		TIM1->CCR1=(uint32_t)t0+(uint32_t)t1+(uint32_t)t2;
		TIM1->CCR2=(uint32_t)t0;
		TIM1->CCR3=(uint32_t)t0+(uint32_t)t2;
		break;
	}
}

void output_inverse_park_transform(float U_alpha,float U_beta){  //direct voltage output from inverse park transform, produce jagged current sine waves but useful for debugging
	float U_U=2500.0f+(U_alpha/2.0f);
	float U_V=2500.0f+(((-U_alpha)+(1.732f*U_beta))/4.0f);
	float U_W=2500.0f+(((-U_alpha)-(1.732f*U_beta))/4.0f);
	if(U_U>DUTY_CYCLE_LIMIT){U_U=DUTY_CYCLE_LIMIT;} //prevent timer from writing duty cycle over 100%
	if(U_V>DUTY_CYCLE_LIMIT){U_V=DUTY_CYCLE_LIMIT;}
	if(U_W>DUTY_CYCLE_LIMIT){U_W=DUTY_CYCLE_LIMIT;}
	TIM1->CCR1=(uint32_t)U_U;
	TIM1->CCR2=(uint32_t)U_V;
	TIM1->CCR3=(uint32_t)U_W;
}
