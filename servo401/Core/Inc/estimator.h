/*
 * estimator.h
 *
 *  Created on: 23 wrz 2022
 *      Author: Wiktor
 */

#ifndef INC_ESTIMATOR_H_
#define INC_ESTIMATOR_H_
typedef struct _estimator_t {
	float sampling_time;
	float prev_Ialpha;
	float prev_Ibeta;
	float dIalpha;
	float dIbeta;
	float bemf_alpha;
	float bemf_beta;
	float U_d; //estimated d voltage calculated based on estimated angle
	float U_q;
	float estim_speed;
	float estim_angle_sum; //integrated value
	float estim_angle; //estimated angle in rad
	float estim_angle_abs;//absolute value of angle, no matter what direction of rotation
}estimator_t;

void calculateBEMF(estimator_t * estimator, float Ialpha, float Ibeta, float Ualpha, float Ubeta);
#endif /* INC_ESTIMATOR_H_ */
