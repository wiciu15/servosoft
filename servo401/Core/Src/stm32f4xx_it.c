/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"
#include "FreeRTOS.h"
#include "task.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "comm_modbus.h"
#include <math.h>
#include "modbus.h"
#include "pid.h"
#include "inverter.h"
#include "lookuptables.h"
#include "mitsubishi_encoder.h"
#include "tamagawa_encoder.h"
#include "parameter_set.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

//DEFAULT PARAMETER SET FROM DRIVE ROM, values would reset between restarts, @TODO: read and write parameter set from flash on boot
parameter_set_t parameter_set={
		.motor_max_current=14.3f, //14.3 according to datasheet
		.motor_nominal_current=5.3f,
		.motor_pole_pairs=5, //4 for abb motor
		.motor_max_voltage=150,
		.motor_max_torque=7.17,
		.motor_nominal_torque=2.39,
		.motor_max_speed=3000,
		.motor_feedback_type=abz_encoder,
		.encoder_electric_angle_correction=-15,//-122 for abb motor
		.encoder_resolution=5000,

		.current_filter_ts=0.007,
		.torque_current_ctrl_proportional_gain=2000.0f, //float torque_current_ctrl_integral_gain; --NOT USED-- if integral other than zero then unloaded motor will saturate the current controller integral gain and motor will generate torque even if commanded to 0 torque
		.field_current_ctrl_proportional_gain=2000.0f,
		.field_current_ctrl_integral_gain=1000.0f,

		.speed_filter_ts=0.005,
		.speed_controller_proportional_gain=0.06f,
		.speed_controller_integral_gain=0.8f,
		.speed_controller_output_torque_limit=50.0f, //limit torque, Id/10 is the output so the calcualtion is needed to convert N/m to A
		.speed_controller_integral_limit=50.0f
};

volatile uint16_t ADC_rawdata[4];

 PID_t id_current_controller_data = {
		0.0f,
		0.0f,
		DUTY_CYCLE_LIMIT,DUTY_CYCLE_LIMIT,
		2E-04f,
		0.0f,0.0f,0.0f,0.0f
};
 PID_t iq_current_controller_data = {
		 0.0f,
		0.0f,
		DUTY_CYCLE_LIMIT,
		DUTY_CYCLE_LIMIT,
		2E-04f,
		0.0f,0.0f,0.0f,0.0f
};

PID_t speed_controller_data = {
		0.0f,
		0.0f,
		0.0f,
		0.0f,
		0.0016f,
		0.0f,0.0f,0.0f,0.0f
};

volatile control_mode_t control_mode = manual;
volatile inverter_state_t inverter_state= stop;
volatile float speed_setpoint_deg_s=0.0f; //speed in degrees/s
volatile int16_t speed_setpoint_rpm=0;
float motor_angle=0.0f;
float electric_angle=0.0f;
volatile float duty_cycle=0.0f;
const uint16_t duty_cycle_limit=DUTY_CYCLE_LIMIT;

float U_DClink=0;
float U_DClink_last=0;
float U_DClink_filtered=0;

uint8_t measurement_error_counter=0;
uint8_t OC_measurement_error_counter=0;
uint8_t UV_measurement_error_counter=0;
uint8_t OV_measurement_error_counter=0;

float torque_setpoint=0.0f;
float id_setpoint=0.0f;

uint8_t zerocurrent_reading_loop_i=0;
uint16_t I_U_zerocurrentreading=0;
uint16_t I_V_zerocurrentreading=0;
int16_t I_U_raw=0;
int16_t I_V_raw=0;
volatile float I_U=0.0f;
volatile float I_V=0.0f;
volatile float I_W=0.0f;
volatile uint16_t rms_count=0;
volatile float I_U_square_sum=0;
volatile float I_V_square_sum=0;
volatile float I_W_square_sum=0;
volatile float I_U_RMS=0;
volatile float I_V_RMS=0;
volatile float I_W_RMS=0;
volatile float I_out=0;

float I_d=0.0f;
float I_d_last=0.0f;
float I_d_filtered=0.0f;
float I_q=0.0f;
float I_q_last=0.0f;
float I_q_filtered=0.0f;

float U_d = 0.0f;
float U_q = 0.0f;

float U_alpha = 0.0f;
float U_beta = 0.0f;

uint8_t encoder_positioned=0;
uint16_t encoder_correction_abz=40;
int16_t encoder_actual_position=0;
int16_t encoder_correction_angle=-122; //offset in degrees between encoder 0 position and stator zero electric angle
float actual_electric_angle=0.0f;
float last_actual_electric_angle=0.0f;
float actual_torque_angle=0.0f;
motor_feedback_type_t motor_feedback_type=tamagawa_encoder;

float speed_measurement_loop_i=0;
float speed=0.0f;
float filtered_speed=0.0f;
float speed_filter_ts=0.05;
float last_filtered_actual_speed=0.0f;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
void output_svpwm(uint16_t angle,uint16_t max_duty_cycle);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_adc1;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
/* USER CODE BEGIN EV */
extern ADC_HandleTypeDef hadc1;
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
#if (INCLUDE_xTaskGetSchedulerState == 1 )
  if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
  {
#endif /* INCLUDE_xTaskGetSchedulerState */
  xPortSysTickHandler();
#if (INCLUDE_xTaskGetSchedulerState == 1 )
  }
#endif /* INCLUDE_xTaskGetSchedulerState */
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 stream5 global interrupt.
  */
void DMA1_Stream5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream5_IRQn 0 */

  /* USER CODE END DMA1_Stream5_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart2_rx);
  /* USER CODE BEGIN DMA1_Stream5_IRQn 1 */

  /* USER CODE END DMA1_Stream5_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream6 global interrupt.
  */
void DMA1_Stream6_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream6_IRQn 0 */

  /* USER CODE END DMA1_Stream6_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart2_tx);
  /* USER CODE BEGIN DMA1_Stream6_IRQn 1 */

  /* USER CODE END DMA1_Stream6_IRQn 1 */
}

/**
  * @brief This function handles TIM1 update interrupt and TIM10 global interrupt.
  */
void TIM1_UP_TIM10_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 0 */

  /* USER CODE END TIM1_UP_TIM10_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 1 */

  /* USER CODE END TIM1_UP_TIM10_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */
	HAL_ADC_Start_DMA(&hadc1, ADC_rawdata, 4);
	if(parameter_set.motor_feedback_type==tamagawa_encoder){tamagawa_encoder_read_position();}
	HAL_GPIO_WritePin(DISP_LATCH_GPIO_Port, DISP_LATCH_Pin, 1);

  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */

  /* USER CODE END TIM3_IRQn 1 */
}

/**
  * @brief This function handles TIM4 global interrupt.
  */
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */
if(speed_setpoint_rpm!=0){
	torque_setpoint = PI_control(&speed_controller_data, speed_setpoint_rpm-filtered_speed);
}
  /* USER CODE END TIM4_IRQn 0 */
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM4_IRQn 1 */

  /* USER CODE END TIM4_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles USART2 global interrupt.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */

  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */

  /* USER CODE END USART2_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[15:10] interrupts.
  */
void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */
	if(HAL_GPIO_ReadPin(ENC_Z_GPIO_Port, ENC_Z_Pin)==1){
			TIM2->CNT=parameter_set.encoder_resolution;
			encoder_positioned=1;
		}
  /* USER CODE END EXTI15_10_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(STEP_Pin);
  HAL_GPIO_EXTI_IRQHandler(DIR_Pin);
  HAL_GPIO_EXTI_IRQHandler(ENC_Z_Pin);
  /* USER CODE BEGIN EXTI15_10_IRQn 1 */

  /* USER CODE END EXTI15_10_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream0 global interrupt.
  */
void DMA2_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream0_IRQn 0 */
	//update pi controllers parameters from parameter set
	id_current_controller_data.proportional_gain=parameter_set.field_current_ctrl_proportional_gain;
	id_current_controller_data.integral_gain=parameter_set.field_current_ctrl_integral_gain;
	iq_current_controller_data.proportional_gain=parameter_set.torque_current_ctrl_proportional_gain;
	speed_controller_data.proportional_gain=parameter_set.speed_controller_proportional_gain;
	speed_controller_data.integral_gain=parameter_set.speed_controller_integral_gain;
	speed_controller_data.antiwindup_limit=parameter_set.speed_controller_integral_limit;
	speed_controller_data.output_limit=parameter_set.speed_controller_output_torque_limit;

	if(zerocurrent_reading_loop_i<15){ //after starting servo read ADC samples when output current is zero to minimize current transducers, opamps and ADC offset
				I_U_zerocurrentreading+=ADC_rawdata[0];
				I_V_zerocurrentreading+=ADC_rawdata[1];
				if(zerocurrent_reading_loop_i==14){I_U_zerocurrentreading/=15;I_V_zerocurrentreading/=15;}
				zerocurrent_reading_loop_i++;
			}else{

				if(ADC_rawdata[0]<40 || ADC_rawdata[1]<40 ||ADC_rawdata[0]>4000 || ADC_rawdata[1]>4000){
					if(measurement_error_counter==1)inverter_error_trip(shortcircuit);
					measurement_error_counter++;
				}else{measurement_error_counter=0;}

				//DC link voltage
				U_DClink = (float)ADC_rawdata[2]*0.0250945f;
				U_DClink_filtered = LowPassFilter(0.01f, U_DClink, &U_DClink_last);
				if(U_DClink_filtered>INVERTER_OVERVOLTAGE_LEVEL && OV_measurement_error_counter<2){if(OV_measurement_error_counter==1){inverter_error_trip(overvoltage);}OV_measurement_error_counter++;}else{OV_measurement_error_counter=0;}
				if(U_DClink_filtered<INVERTER_UNDERVOLTAGE_LEVEL && UV_measurement_error_counter<2){if(UV_measurement_error_counter==1){inverter_error_trip(undervoltage);}UV_measurement_error_counter++;}else{UV_measurement_error_counter=0;} //2 measurements under a treshold must happen in a row
				modbus_registers_buffer[14] = (uint16_t)(U_DClink_filtered*10.0f);
				//current calculation
				I_U_raw=ADC_rawdata[0]-I_U_zerocurrentreading;
				I_V_raw=ADC_rawdata[1]-I_V_zerocurrentreading;
				I_U=(float)I_U_raw*CURRENT_SENSE_RATIO;
				I_V=(float)I_V_raw*CURRENT_SENSE_RATIO;
				I_W=-I_U-I_V;
				//RMS current calculation loop

				rms_count++;
				I_U_square_sum+=(I_U*I_U);
				I_V_square_sum+=(I_V*I_V);
				I_W_square_sum+=(I_W*I_W);

				if(rms_count>CURRENT_RMS_SAMPLING_COUNT){
					I_U_RMS=sqrtf(I_U_square_sum/(float)rms_count);
					I_V_RMS=sqrtf(I_V_square_sum/(float)rms_count);
					I_W_RMS=sqrtf(I_W_square_sum/(float)rms_count);
					I_out=(I_U_RMS+I_V_RMS+I_W_RMS)/3.0f;
					modbus_registers_buffer[10]=(uint16_t)(I_out*100.0f);
					rms_count=0;I_U_square_sum=0.0f;I_V_square_sum=0.0f;I_W_square_sum=0.0f;}

				if((I_U>INVERTER_OVERCURRENT_TRIP_LEVEL || I_U < (-INVERTER_OVERCURRENT_TRIP_LEVEL) || I_V>INVERTER_OVERCURRENT_TRIP_LEVEL || I_V < (-INVERTER_OVERCURRENT_TRIP_LEVEL) || I_W > INVERTER_OVERCURRENT_TRIP_LEVEL || I_W <(-INVERTER_OVERCURRENT_TRIP_LEVEL)) && OC_measurement_error_counter<3){
					OC_measurement_error_counter++;
					if(OC_measurement_error_counter==2){inverter_error_trip(inverter_overcurrent);}
				}else{OC_measurement_error_counter=0;}

				if((I_U>parameter_set.motor_max_current || I_U < (-parameter_set.motor_max_current) || I_V>parameter_set.motor_max_current || I_V < (-parameter_set.motor_max_current) || I_W > parameter_set.motor_max_current || I_W <(-parameter_set.motor_max_current)) && OC_measurement_error_counter<3){
					OC_measurement_error_counter++;
					if(OC_measurement_error_counter==2){inverter_error_trip(motor_overcurrent);}
				}else{OC_measurement_error_counter=0;}

				if(parameter_set.motor_feedback_type==abz_encoder){
					if(encoder_positioned){
						if(TIM2->CNT <parameter_set.encoder_resolution){encoder_actual_position=parameter_set.encoder_resolution-TIM2->CNT;}
						else{encoder_actual_position=parameter_set.encoder_resolution-TIM2->CNT;}
						modbus_registers_buffer[11]=encoder_actual_position;
						int16_t corrected_encoder_position=encoder_actual_position % (parameter_set.encoder_resolution/parameter_set.motor_pole_pairs);
						if(corrected_encoder_position<0){corrected_encoder_position+=(parameter_set.encoder_resolution/parameter_set.motor_pole_pairs);}
						actual_electric_angle=(float)(corrected_encoder_position)*0.36f-parameter_set.encoder_electric_angle_correction;
						if(actual_electric_angle-electric_angle>180.0f){actual_torque_angle=(actual_electric_angle-electric_angle) - 360.0f;}
						else if(actual_electric_angle-electric_angle<(-180.0f)){actual_torque_angle=actual_electric_angle-electric_angle + 360.0f;}
						else{actual_torque_angle=actual_electric_angle-electric_angle;}
						modbus_registers_buffer[12]=(int16_t)actual_torque_angle;//write calculated value to modbus array
						speed_measurement_loop_i++;
						if(speed_measurement_loop_i>=10){
							speed=((actual_electric_angle-last_actual_electric_angle)/parameter_set.motor_pole_pairs)/0.012f; //speed(rpm) = ((x(deg)/polepairs)/360deg)/(0,002(s)/60s)
							float theoretical_encoder_speed=(360.0f/parameter_set.motor_pole_pairs)/0.012f;
							if(speed>theoretical_encoder_speed/2.0f){speed-=theoretical_encoder_speed;}if(speed<(-theoretical_encoder_speed/2.0f)){speed+=theoretical_encoder_speed;}
							modbus_registers_buffer[13]=(int16_t)(filtered_speed);
							last_actual_electric_angle = actual_electric_angle;
							speed_measurement_loop_i=0;
						}
						filtered_speed=LowPassFilter(speed_filter_ts,speed, &last_filtered_actual_speed);
					}
				}
				if(parameter_set.motor_feedback_type==ssi_encoder){
					modbus_registers_buffer[11]=ssi_encoder_data.encoder_position;
					if(ssi_encoder_data.encoder_resolution==p8192ppr){
						actual_electric_angle=(((fmodf(ssi_encoder_data.encoder_position, 8192.0f/(float)parameter_set.motor_pole_pairs))/(8192.0f/(float)parameter_set.motor_pole_pairs))*360.0f)+parameter_set.encoder_electric_angle_correction;
						if(actual_electric_angle>=360.0f){actual_electric_angle-=360.0f;}
						if(actual_electric_angle<0){actual_electric_angle+=360.0f;}
					}
					if(ssi_encoder_data.encoder_resolution==p131072ppr){
						//not verified if working with 17bit, no compatible motor to test :(
						actual_electric_angle=(((fmodf(ssi_encoder_data.encoder_position,8192.0f/(float)parameter_set.motor_pole_pairs))/(131072.0f/(float)parameter_set.motor_pole_pairs))*360.0f)+parameter_set.encoder_electric_angle_correction;
						if(actual_electric_angle>=360.0f){actual_electric_angle-=360.0f;}
						if(actual_electric_angle<0){actual_electric_angle+=360.0f;}
					}
					if(ssi_encoder_data.encoder_resolution==unknown_resolution){actual_electric_angle=0;}//@TODO: encoder error trip
					if(actual_electric_angle-electric_angle>180.0f){actual_torque_angle=(actual_electric_angle-electric_angle) - 360.0f;}
					else if(actual_electric_angle-electric_angle<(-180.0f)){actual_torque_angle=actual_electric_angle-electric_angle + 360.0f;}
					else{actual_torque_angle=actual_electric_angle-electric_angle;}
					modbus_registers_buffer[12]=(int16_t)actual_torque_angle;//write calculated value to modbus array
					speed_measurement_loop_i++;
					if(speed_measurement_loop_i>=30){
						if(ssi_encoder_data.encoder_resolution==p8192ppr){
							//speed(rpm)=(position pulse delta/enc resolution)*(60s/sample time(s))
							//speed=(delta/8192)*(60/0,006)
							speed=((float)ssi_encoder_data.encoder_position-(float)ssi_encoder_data.last_encoder_position_speed_loop)*0.66f;
							if(speed>5000.0f){speed-=10000.0f;}if(speed<-5000.0f){speed+=10000.0f;}
						}
						//@TODO: add speed measurement for 17bit encoders
						ssi_encoder_data.last_encoder_position_speed_loop=ssi_encoder_data.encoder_position;
						speed_measurement_loop_i=0;
					}
					filtered_speed=LowPassFilter(speed_filter_ts,speed, &last_filtered_actual_speed);
					modbus_registers_buffer[13]=(int16_t)(speed);

				}
				if(parameter_set.motor_feedback_type==tamagawa_encoder){
					modbus_registers_buffer[11]=tamagawa_encoder_data.encoder_position/2; //encoder output is 17-bit but modbus register can hold only 16-bit
					actual_electric_angle=(((fmodf(tamagawa_encoder_data.encoder_position, 131072.0f/(float)parameter_set.motor_pole_pairs))/(131072.0f/(float)parameter_set.motor_pole_pairs))*360.0f)+parameter_set.encoder_electric_angle_correction;
					if(actual_electric_angle>=360.0f){actual_electric_angle-=360.0f;}
					if(actual_electric_angle<0){actual_electric_angle+=360.0f;}

					if(actual_electric_angle-electric_angle>180.0f){actual_torque_angle=(actual_electric_angle-electric_angle) - 360.0f;}
					else if(actual_electric_angle-electric_angle<(-180.0f)){actual_torque_angle=actual_electric_angle-electric_angle + 360.0f;}
					else{actual_torque_angle=actual_electric_angle-electric_angle;}
					modbus_registers_buffer[12]=(int16_t)actual_torque_angle;//write calculated value to modbus array
					speed_measurement_loop_i++;
					if(speed_measurement_loop_i>=10){

						//speed(rpm)=(position pulse delta/enc resolution)*(60s/sample time(s))
						//speed=(delta/131072)*(60/0,006)
						speed=((float)tamagawa_encoder_data.encoder_position-(float)tamagawa_encoder_data.last_encoder_position_speed_loop)*0.228879f;
						if(speed>15000.0f){speed-=30000.0f;}if(speed<-15000.0f){speed+=30000.0f;}

						tamagawa_encoder_data.last_encoder_position_speed_loop=tamagawa_encoder_data.encoder_position;
						speed_measurement_loop_i=0;
					}
					filtered_speed=LowPassFilter(parameter_set.speed_filter_ts,speed, &last_filtered_actual_speed);
					modbus_registers_buffer[13]=(int16_t)(filtered_speed);
				}

				park_transform(I_U, I_V, 360.0f-actual_electric_angle, &I_d, &I_q); //360-electric angle is needed because for some reasons all encoders are going reverse direction than my electric angle generator
				I_d_filtered = LowPassFilter(parameter_set.current_filter_ts, I_d, &I_d_last);
				I_q_filtered = LowPassFilter(parameter_set.current_filter_ts, I_q, &I_q_last);
				modbus_registers_buffer[15]=(int16_t)(I_d_filtered*100);
				modbus_registers_buffer[16]=(int16_t)(I_q_filtered*100);

				if(control_mode==manual){
					electric_angle+=(speed_setpoint_deg_s*(float)parameter_set.motor_pole_pairs)/5000.0f;  //5000hz control/sampling loop
					if(electric_angle>=360.0f){	electric_angle=0.0f;}
					if(electric_angle<0.0f){electric_angle=359.0f;}
				}
				if(control_mode==foc && modbus_registers_buffer[3] ==1){
					U_d = PI_control(&id_current_controller_data, id_setpoint-I_d_filtered);
					U_q = PI_control(&iq_current_controller_data,(torque_setpoint)-I_q_filtered);
					inv_park_transform(U_d, U_q, actual_electric_angle, &U_alpha, &U_beta);
					duty_cycle=sqrtf(U_alpha*U_alpha+U_beta*U_beta);

					float electric_angle_rad=0.0f;
					if(U_alpha>=0 && U_beta >=0){electric_angle_rad=atan2f(fabs(U_beta),fabs(U_alpha));}
					if(U_alpha<0 && U_beta >=0){electric_angle_rad=atan2f(fabs(U_alpha),fabs(U_beta)) + (3.141592f/2.0f);}
					if(U_alpha<0 && U_beta <0){electric_angle_rad=atan2f(fabs(U_beta),fabs(U_alpha)) + 3.141592f;}
					if(U_alpha>=0 && U_beta <0){electric_angle_rad=atan2f(fabs(U_alpha),fabs(U_beta)) + (3.141592f*1.5f);}

					electric_angle=(electric_angle_rad/3.141592f)*180.0f;
				}

				if(inverter_state==run){
					//output_svpwm((uint16_t)electric_angle, (uint16_t)duty_cycle);
					output_sine_pwm(electric_angle, (uint16_t)duty_cycle);
				}
				else{TIM1->CCR1=0;TIM1->CCR2=0;TIM1->CCR3=0;}//if inverter in stop mode stop producing PWM signal while timer1 is still active to keep this interrupt alive for measurements on switched off inverter

			}
  /* USER CODE END DMA2_Stream0_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc1);
  /* USER CODE BEGIN DMA2_Stream0_IRQn 1 */

  /* USER CODE END DMA2_Stream0_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream2 global interrupt.
  */
void DMA2_Stream2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream2_IRQn 0 */

  /* USER CODE END DMA2_Stream2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_rx);
  /* USER CODE BEGIN DMA2_Stream2_IRQn 1 */

  /* USER CODE END DMA2_Stream2_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream7 global interrupt.
  */
void DMA2_Stream7_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream7_IRQn 0 */

  /* USER CODE END DMA2_Stream7_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_tx);
  /* USER CODE BEGIN DMA2_Stream7_IRQn 1 */

  /* USER CODE END DMA2_Stream7_IRQn 1 */
}

/* USER CODE BEGIN 1 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){

}
/* USER CODE END 1 */
