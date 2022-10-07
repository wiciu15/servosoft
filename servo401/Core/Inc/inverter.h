/*
 * inverter.h
 *
 *  Created on: Feb 11, 2022
 *      Author: Wiktor
 */

#ifndef INC_INVERTER_H_
#define INC_INVERTER_H_

#include "parameter_set.h"

#define INVERTER_OVERCURRENT_TRIP_LEVEL 5.0f  //overcurrrent trip setting level in Amperes
#define INVERTER_OVERVOLTAGE_LEVEL 90.0f
#define INVERTER_UNDERVOLTAGE_LEVEL 20.0f
#define DUTY_CYCLE_LIMIT 4999
#define CURRENT_SENSE_RATIO 0.0166023f //multiply this number with number of adc samples measured from current sensor output 0.0119827@4.17A 0.012126 0.0132173@1.52a
#define CURRENT_RMS_SAMPLING_COUNT 500 //(2*pwm frequency)/this define=rms current sampling frequency, for 500 = 32 calculations per second

typedef enum {no_error,undervoltage,overvoltage,shortcircuit,inverter_overcurrent,motor_overcurrent,encoder_error,internal_hardfault}inverter_error_t;
typedef enum {stop,run,inhibit,trip}inverter_state_t;
typedef enum {manual,scalar,foc}control_mode_t;


extern parameter_set_t parameter_set;

extern volatile inverter_error_t inverter_error;
extern volatile control_mode_t control_mode;
extern volatile inverter_state_t inverter_state;
extern volatile float speed_setpoint_deg_s; //speed in degrees/s
extern volatile int16_t speed_setpoint_rpm;
extern float motor_angle;
extern float electric_angle;
extern float electric_angle_setpoint;
extern volatile float duty_cycle;
extern const uint16_t duty_cycle_limit;

extern float torque_setpoint;
extern float id_setpoint;
extern float U_DClink;
extern float U_DClink_last;
extern float U_DClink_filtered;

extern uint16_t I_U_zerocurrentreading;
extern uint16_t I_V_zerocurrentreading;
extern int16_t I_U_raw;
extern int16_t I_V_raw;
extern volatile float I_U;
extern volatile float I_V;
extern volatile float I_W;
extern volatile uint16_t rms_count;
extern volatile float I_U_square_sum;
extern volatile float I_V_square_sum;
extern volatile float I_W_square_sum;
extern volatile float I_U_RMS;
extern volatile float I_V_RMS;
extern volatile float I_W_RMS;
extern volatile float I_out; //inverter RMS output current average from 3 phases

extern float I_d;
extern float I_d_last;
extern float I_d_filtered;
extern float I_q;
extern float I_q_last;
extern float I_q_filtered;

extern uint8_t encoder_positioned;
extern uint16_t encoder_correction_abz;
extern int16_t encoder_actual_position;
extern float actual_electric_angle;
extern float last_actual_electric_angle;
extern float actual_torque_angle;

extern motor_feedback_type_t motor_feedback_type;

extern float speed_measurement_loop_i;
extern float speed;
extern float filtered_speed;
extern float last_filtered_actual_speed;

void inverter_enable(void);
void inverter_disable(void);
void inverter_error_trip(uint8_t error);
void park_transform(float I_U,float I_V,float motor_angle,float * I_d,float * I_q);
void inv_park_transform(float U_d,float U_q, float motor_angle, float * U_alpha, float * U_beta);
float LowPassFilter(float Tf, float actual_measurement, float * last_filtered_value);
void output_sine_pwm(float angle,uint16_t duty_cycle);
void output_svpwm(uint16_t angle,uint16_t duty_cycle);
void output_inverse_park_transform(float U_alpha,float U_beta);

#endif /* INC_INVERTER_H_ */
