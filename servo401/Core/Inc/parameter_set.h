/*
 * parameter_set.h
 *
 *  Created on: 19 wrz 2022
 *      Author: Wiktor
 */

#ifndef INC_PARAMETER_SET_H_
#define INC_PARAMETER_SET_H_

#include "inverter.h"

typedef enum {no_feedback,abz_encoder,mitsubishi_encoder,tamagawa_encoder,delta_encoder}motor_feedback_type_t;

typedef struct _parameter_set_t{
	float motor_max_current; //current for motor overcurrent trip
	float motor_nominal_current; //current for torque calculation
	uint8_t motor_pole_pairs;  //needed for actual electric angle calculation in FOC, and accurate speed setpoint in manual mode
	float motor_max_voltage;
	float motor_max_torque;
	float motor_nominal_torque;
	uint16_t motor_nominal_speed;
	uint16_t motor_max_speed;
	float motor_rs;
	float motor_ls;
	float motor_K;

	motor_feedback_type_t motor_feedback_type;  //type of feedback
	int16_t encoder_electric_angle_correction;  //correction of difference in electric angle calculated from encoder and electric angle of stator current. needed to achieve 90/-90 torque angle
	uint16_t encoder_resolution;

	float current_filter_ts; //measured Id/Iq-low pass filter time constant for torque regulator
	float torque_current_ctrl_proportional_gain; //float torque_current_ctrl_integral_gain; --NOT USED-- if integral other than zero then unloaded motor will saturate the current controller integral gain and motor will generate torque even if commanded to 0 torque
	float torque_current_ctrl_integral_gain;
	float field_current_ctrl_proportional_gain;
	float field_current_ctrl_integral_gain;

	float speed_filter_ts; //measured speed-low pass filter time constant for speed regulator
	float speed_controller_proportional_gain;
	float speed_controller_integral_gain;
	float speed_controller_output_torque_limit; //limit torque, Id is the output so the calcualtion is needed to convert N/m to A
	float speed_controller_integral_limit; //same conversion like output torque limit
}parameter_set_t;

#endif /* INC_PARAMETER_SET_H_ */
