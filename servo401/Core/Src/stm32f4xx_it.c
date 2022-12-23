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
#include "estimator.h"
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
		.motor_max_current=8.0f, //14.3 according to datasheet
		.motor_nominal_current=2.7f,
		.motor_pole_pairs=5, //4 for abb motor 5 for bch and mitsubishi hf-kn43
		.motor_max_voltage=150.0f,
		.motor_max_torque=7.17f,
		.motor_nominal_torque=2.39f,
		.motor_max_speed=3000,
		.motor_rs=0.42f,
		.motor_ls=0.00353f, //winding inducatnce in H
		.motor_K=0.03288f,  //electical constant in V/(rad/s*pole_pairs) 1000RPM=104.719rad/s UPDATE WHEN CHANGING POLE PAIRS
		.motor_feedback_type=mitsubishi_encoder,
		.encoder_electric_angle_correction=60, //-90 for abb BSM, 0 for bch, 0 for abb esm18, 60 for hf-kn43
		.encoder_resolution=5000,

		.current_filter_ts=0.003f,
		.torque_current_ctrl_proportional_gain=4.0f, //gain in V/A
		.torque_current_ctrl_integral_gain=2000.0f, //
		.field_current_ctrl_proportional_gain=6.0f,
		.field_current_ctrl_integral_gain=1600.0f,

		.speed_filter_ts=0.02f,
		.speed_controller_proportional_gain=0.005f,
		.speed_controller_integral_gain=0.45f,
		.speed_controller_output_torque_limit=1.0f, //limit torque, Iq is the output so the calcualtion is needed to convert N/m to A
		.speed_controller_integral_limit=1.0f //1.0 is for example, valid iq current gets copied from motor nominal current
};

volatile uint16_t ADC_rawdata[4];

 PID_t id_current_controller_data = {
		0.0f,
		0.0f,
		0.0f,0.0f,
		2E-04f,
		0.0f,0.0f,0.0f,0.0f
};
 PID_t iq_current_controller_data = {
		 0.0f,
		0.0f,
		0.0f,
		0.0f,
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
float electric_angle=0.0f; //stator voltage vector angle
float electric_angle_setpoint=0.0f; //electric angle to set in manual mode
float dc_link_to_duty_cycle_ratio=0.0f;
volatile float duty_cycle=0.0f;
float calculated_duty_cycle=0.0f;
const uint16_t duty_cycle_limit=DUTY_CYCLE_LIMIT;

float U_DClink=0.0f;
float U_DClink_last=0.0f;
float U_DClink_filtered=0.0f;

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
volatile float I_U_square_sum=0.0f;
volatile float I_V_square_sum=0.0f;
volatile float I_W_square_sum=0.0f;
volatile float I_U_RMS=0.0f;
volatile float I_V_RMS=0.0f;
volatile float I_W_RMS=0.0f;
volatile float I_out=0.0f;

float I_alpha=0.0f;
float I_beta=0.0f;
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
//motor_feedback_type_t motor_feedback_type=tamagawa_encoder;

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
	HAL_ADC_Start_DMA(&hadc1, ADC_rawdata, 4);//start ADC sampling and wait for ADC to finish to continue with control loop, in meantime read position from serial encoders
	if(parameter_set.motor_feedback_type==tamagawa_encoder){tamagawa_encoder_read_position();}
	if(parameter_set.motor_feedback_type==mitsubishi_encoder){mitsubishi_encoder_read_position();}
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
	//handle ABZ encoder indexing with 0 degree angle
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
	//This gets called after ADC samples actual phase currents and actual encoder position is already read
	//whole control loop runs with frequency of TIM3 interrupt, except for speed controller PID loop which is run by TIM4 interrput
	//update pi controllers parameters from parameter set @TODO: only update them when parameter set changed
	id_current_controller_data.proportional_gain=parameter_set.field_current_ctrl_proportional_gain;
	id_current_controller_data.integral_gain=parameter_set.field_current_ctrl_integral_gain;
	id_current_controller_data.antiwindup_limit=U_DClink_filtered;
	id_current_controller_data.output_limit=U_DClink_filtered;
	iq_current_controller_data.proportional_gain=parameter_set.torque_current_ctrl_proportional_gain;
	iq_current_controller_data.integral_gain=parameter_set.torque_current_ctrl_integral_gain;
	iq_current_controller_data.antiwindup_limit=U_DClink_filtered;
	iq_current_controller_data.output_limit=U_DClink_filtered;
	speed_controller_data.proportional_gain=parameter_set.speed_controller_proportional_gain;
	speed_controller_data.integral_gain=parameter_set.speed_controller_integral_gain;
	speed_controller_data.antiwindup_limit=parameter_set.motor_nominal_current;
	speed_controller_data.output_limit=parameter_set.motor_nominal_current;

	if(zerocurrent_reading_loop_i<15){ //after starting system read 15 ADC samples when output current is zero to minimize current transducers, opamps and ADC offset and some of noise
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
				dc_link_to_duty_cycle_ratio=DUTY_CYCLE_LIMIT/U_DClink_filtered;

				if(U_DClink_filtered>INVERTER_OVERVOLTAGE_LEVEL ){OV_measurement_error_counter++;}else{OV_measurement_error_counter=0;}
				if(U_DClink_filtered<INVERTER_UNDERVOLTAGE_LEVEL){UV_measurement_error_counter++;}else{UV_measurement_error_counter=0;}
				if(OV_measurement_error_counter>3){inverter_error_trip(overvoltage);}
				if(UV_measurement_error_counter>3){inverter_error_trip(undervoltage);}
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
						encoder_actual_position=parameter_set.encoder_resolution-TIM2->CNT; //flip direction of encoder
						modbus_registers_buffer[11]=encoder_actual_position;
						actual_electric_angle=(float)(encoder_actual_position % (parameter_set.encoder_resolution/parameter_set.motor_pole_pairs))*0.36f-parameter_set.encoder_electric_angle_correction;  //calculate rotor electric angle
						if(electric_angle-actual_electric_angle>180.0f){actual_torque_angle=(electric_angle-actual_electric_angle) - 360.0f;}
						else if(electric_angle-actual_electric_angle<(-180.0f)){actual_torque_angle=electric_angle-actual_electric_angle + 360.0f;}
						else{actual_torque_angle=electric_angle-actual_electric_angle;}modbus_registers_buffer[12]=(int16_t)actual_torque_angle;//write calculated value to modbus array
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
				if(parameter_set.motor_feedback_type==mitsubishi_encoder){
					if(mitsubishi_encoder_data.encoder_resolution==8192){
						modbus_registers_buffer[11]=mitsubishi_encoder_data.encoder_position;
						actual_electric_angle=(((fmodf(mitsubishi_encoder_data.encoder_position, 8192.0f/(float)parameter_set.motor_pole_pairs))/(8192.0f/(float)parameter_set.motor_pole_pairs))*360.0f)+parameter_set.encoder_electric_angle_correction;
						if(actual_electric_angle>=360.0f){actual_electric_angle-=360.0f;}
						if(actual_electric_angle<0){actual_electric_angle+=360.0f;}
					}
					if(mitsubishi_encoder_data.encoder_resolution==131072){
						modbus_registers_buffer[11]=mitsubishi_encoder_data.encoder_position/2;
						actual_electric_angle=(((fmodf(mitsubishi_encoder_data.encoder_position,131072.0f/(float)parameter_set.motor_pole_pairs))/(131072.0f/(float)parameter_set.motor_pole_pairs))*360.0f)+parameter_set.encoder_electric_angle_correction;
						if(actual_electric_angle>=360.0f){actual_electric_angle-=360.0f;}
						if(actual_electric_angle<0){actual_electric_angle+=360.0f;}
					}
					if(electric_angle-actual_electric_angle>180.0f){actual_torque_angle=(electric_angle-actual_electric_angle) - 360.0f;}
					else if(electric_angle-actual_electric_angle<(-180.0f)){actual_torque_angle=electric_angle-actual_electric_angle + 360.0f;}
					else{actual_torque_angle=electric_angle-actual_electric_angle;}
					modbus_registers_buffer[12]=(int16_t)actual_torque_angle;//write calculated value to modbus array
					speed_measurement_loop_i++;
					if(speed_measurement_loop_i>=10){

						//speed(rpm)=(position pulse delta/enc resolution)*(60s/sample time(s))
						//speed=(delta/131072)*(60/0,006)
						speed=((float)mitsubishi_encoder_data.encoder_position-(float)mitsubishi_encoder_data.last_encoder_position_speed_loop)*0.228879f;
						if(speed>15000.0f){speed-=30000.0f;}if(speed<-15000.0f){speed+=30000.0f;}

						mitsubishi_encoder_data.last_encoder_position_speed_loop=mitsubishi_encoder_data.encoder_position;
						speed_measurement_loop_i=0;
					}
					filtered_speed=LowPassFilter(parameter_set.speed_filter_ts,speed, &last_filtered_actual_speed);
					modbus_registers_buffer[13]=(int16_t)(filtered_speed);

				}
				if(parameter_set.motor_feedback_type==tamagawa_encoder){
					modbus_registers_buffer[11]=tamagawa_encoder_data.encoder_position/2; //encoder output is 17-bit but modbus register can hold only 16-bit
					actual_electric_angle=(((fmodf(tamagawa_encoder_data.encoder_position, 131072.0f/(float)parameter_set.motor_pole_pairs))/(131072.0f/(float)parameter_set.motor_pole_pairs))*360.0f)+parameter_set.encoder_electric_angle_correction;
					if(actual_electric_angle>=360.0f){actual_electric_angle-=360.0f;}
					if(actual_electric_angle<0){actual_electric_angle+=360.0f;} //make sure to get 0-360 deg after correction
					if(electric_angle-actual_electric_angle>180.0f){actual_torque_angle=(electric_angle-actual_electric_angle) - 360.0f;}
					else if(electric_angle-actual_electric_angle<(-180.0f)){actual_torque_angle=electric_angle-actual_electric_angle + 360.0f;}
					else{actual_torque_angle=electric_angle-actual_electric_angle;}
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

				//calculate id/iq vetors based on rotor angle calculation/estimation
				clarke_transform(I_U, I_V, &I_alpha, &I_beta);
				if(control_mode==manual||control_mode==foc){park_transform(I_alpha, I_beta, actual_electric_angle, &I_d, &I_q);} //in manual mode calculates id/iq for read only if encoder available, in FOC uses id/iq for closed loop motor control
				if(control_mode==open_loop_current){park_transform(I_alpha, I_beta, electric_angle_setpoint, &I_d, &I_q);} //angle for park transform is given from stator voltage angle setpoint, which switches id/iq current
				I_d_filtered = LowPassFilter(parameter_set.current_filter_ts, I_d, &I_d_last);
				I_q_filtered = LowPassFilter(parameter_set.current_filter_ts, I_q, &I_q_last);
				modbus_registers_buffer[15]=(int16_t)(I_d_filtered*100);
				modbus_registers_buffer[16]=(int16_t)(I_q_filtered*100);

				//calculate voltage vectors
				if(control_mode==manual){
					electric_angle_setpoint+=(speed_setpoint_deg_s*(float)parameter_set.motor_pole_pairs)/5000.0f;  //5000hz control/sampling loop
					if(electric_angle_setpoint>=360.0f){electric_angle_setpoint=0.0f;}
					if(electric_angle_setpoint<0.0f){electric_angle_setpoint=359.0f;}
					float electric_angle_setpoint_rad = ((electric_angle_setpoint)/180.0f)*3.141592f;
					U_alpha=cosf(electric_angle_setpoint_rad)*(duty_cycle/dc_link_to_duty_cycle_ratio);
					U_beta=sinf(electric_angle_setpoint_rad)*(duty_cycle/dc_link_to_duty_cycle_ratio);
					//park_transform(U_alpha, U_beta, actual_electric_angle, &U_d, &U_q);
				}
				//motor angle for park transform is given from stator angle setpoint in open loop current mode, then id/iq will switch places and torque setpoint will magnetize the rotor, field setpoint must be 0 to NOT generate torque
				if(control_mode==open_loop_current && inverter_state==run){
					electric_angle_setpoint+=(speed_setpoint_deg_s*(float)parameter_set.motor_pole_pairs)/5000.0f;  //5000hz control/sampling loop
					if(electric_angle_setpoint>=360.0f){electric_angle_setpoint=0.0f;}
					if(electric_angle_setpoint<0.0f){electric_angle_setpoint=359.0f;}
					U_q = PI_control(&iq_current_controller_data,torque_setpoint-I_q_filtered);
					U_d = PI_control(&id_current_controller_data,0-I_d_filtered);
					inv_park_transform(U_d, U_q, electric_angle_setpoint, &U_alpha, &U_beta);
				}
				if(control_mode==foc && inverter_state==run){
					U_d = PI_control(&id_current_controller_data, id_setpoint-I_d_filtered);
					U_q = PI_control(&iq_current_controller_data,(torque_setpoint)-I_q_filtered);
					inv_park_transform(U_d, U_q, actual_electric_angle, &U_alpha, &U_beta);
				}
				calculateBEMF(&estimator, I_alpha, I_beta, U_alpha, U_beta);

				//calculate output vector angle in degrees
				float temp_electric_angle_rad=0.0f;
				temp_electric_angle_rad=atan2f(U_beta,U_alpha);
				if(temp_electric_angle_rad<0.0f){
					temp_electric_angle_rad=3.1415f+(3.1415f+temp_electric_angle_rad); //convert value from +-180 deg to 0-360deg
				}
				electric_angle=(temp_electric_angle_rad/3.141592f)*180.0f;
				calculated_duty_cycle=hypotf(U_alpha,U_beta)*dc_link_to_duty_cycle_ratio;

				//reset pid controllers data for next startup if stopped at speed/torque !=0. This avoids high Ud/Uq voltage output at startup
				if(inverter_state!=run){
					iq_current_controller_data.last_integral=0.0f;
					iq_current_controller_data.last_error=0.0f;
					iq_current_controller_data.last_output=0.0f;
					id_current_controller_data.last_integral=0.0f;
					id_current_controller_data.last_error=0.0f;
					id_current_controller_data.last_output=0.0f;
					speed_controller_data.last_error=0.0f;
					speed_controller_data.last_integral=0.0f;
					speed_controller_data.last_output=0.0f;
				}
				//update voltages in TIM1 compare registers to output them to motor
				if(inverter_state==run){
					//output_svpwm((uint16_t)electric_angle, (uint16_t)calculated_duty_cycle);
					output_sine_pwm(electric_angle, (uint16_t) calculated_duty_cycle);
				}else{
					//output_svpwm(0, 0); //svpwm produces vibrations at full speed and current ripple at low speeds
					output_sine_pwm(0.0f, 0);
				}

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
