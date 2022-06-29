/*
 * inverter.h
 *
 *  Created on: Feb 11, 2022
 *      Author: Wiktor
 */

#ifndef INC_INVERTER_H_
#define INC_INVERTER_H_

#define INVERTER_OVERCURRENT_TRIP_LEVEL 8.0f  //overcurrrent trip setting level in Amperes
#define INVERTER_OVERVOLTAGE_LEVEL 19.0f
#define INVERTER_UNDERVOLTAGE_LEVEL 13.0f
#define DUTY_CYCLE_LIMIT 5249
#define CURRENT_SENSE_RATIO 0.0132129f //multiply this number with number of adc samples measured from current sensor output 0.0119827@4.17A 0.012126 0.0132173@1.52a
#define POLE_PAIRS 5.0f
#define CURRENT_RMS_SAMPLING_COUNT 500 //(2*pwm frequency)/this define=rms current sampling frequency, for 500 = 32 calculations per second

typedef enum {no_error,undervoltage,overvoltage,shortcircuit,overcurrent,encoder_error,internal_hardfault}inverter_error_t;
enum inverter_control_mode_t {stop,manual,foc};

extern volatile inverter_error_t inverter_error;
extern volatile enum inverter_control_mode_t inv_control_mode;
extern volatile float speed_setpoint_deg_s; //speed in degrees/s
extern float motor_angle;
extern float electric_angle;
extern volatile float duty_cycle;
extern const uint16_t duty_cycle_limit;

extern float torque_setpoint;
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
extern uint16_t encoder_correction;
extern int16_t encoder_actual_position;
extern float actual_electric_angle;
extern float last_actual_electric_angle;
extern float actual_torque_angle;

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

#endif /* INC_INVERTER_H_ */
