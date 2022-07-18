/*
 * pid.h
 *
 *  Created on: Apr 9, 2022
 *      Author: Wiktor
 */

#ifndef INC_PID_H_
#define INC_PID_H_

typedef struct PID_t {
	float proportional_gain;
	float integral_gain;
	float antiwindup_limit;
	float output_limit;
	float sampling_time;
	float output_ramp;
	float last_integral;
	float last_error;
	float last_output;

} PID_t;


float PI_control(PID_t *pid_data, float error);
#endif /* INC_PID_H_ */
