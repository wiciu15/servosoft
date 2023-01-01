/*
 * estimator.c
 *
 *  Created on: 23 wrz 2022
 *      Author: Wiktor
 */

#include "estimator.h"
#include "main.h"
#include "inverter.h"
#include "math.h"

estimator_t estimator={
		.sampling_time=0.0002,
		.prev_Ialpha=0.0f,
		.prev_Ibeta=0.0f,
		.dIalpha=0.0f,
		.dIbeta=0.0f,
		.prev_U_alpha=0.0f,
		.bemf_alpha=0.0f,
		.bemf_beta=0.0f,
		.U_d=0.0f,
		.U_q=0.0f,
		.estim_speed=0.0f,
		.estim_speed_rpm=0.0f,
		.estim_angle=0.0f,
		.estim_angle_deg=0.0f,
		.estim_angle_abs=0.0f
};

void calculateBEMF(estimator_t * estimator, float Ialpha, float Ibeta, float Ualpha, float Ubeta){

	//calculate derivative of current vectors alpha and beta
	estimator->dIalpha=Ialpha-estimator->prev_Ialpha;
	estimator->dIbeta=Ibeta-estimator->prev_Ibeta;
	estimator->prev_Ialpha=Ialpha;
	estimator->prev_Ibeta=Ibeta;

	//calculate BEMF for alpha and beta BEMF = Ualpha - Rs Ialpha - Ls dIalpha/dt
	estimator->bemf_alpha=Ualpha-(parameter_set.motor_rs*Ialpha)-(parameter_set.motor_ls*(estimator->dIalpha/estimator->sampling_time));
	estimator->bemf_beta=Ubeta-(parameter_set.motor_rs*Ibeta)-(parameter_set.motor_ls*(estimator->dIbeta/estimator->sampling_time));

	estimator->U_d=(estimator->bemf_alpha*cosf(estimator->estim_angle))+(estimator->bemf_beta*sinf(estimator->estim_angle));
	estimator->U_q=(estimator->bemf_beta*cosf(estimator->estim_angle))-(estimator->bemf_alpha*sinf(estimator->estim_angle));

	if(estimator->U_q>=0){
		estimator->estim_speed=(estimator->U_q+estimator->U_d)/parameter_set.motor_K;
	}else{
		estimator->estim_speed=(estimator->U_q-estimator->U_d)/parameter_set.motor_K;
	}
	estimator->estim_speed_rpm=(estimator->estim_speed*9.549297)/parameter_set.motor_pole_pairs;

	estimator->estim_angle_sum+=estimator->estim_speed*estimator->sampling_time;
	estimator->estim_angle=fmodf(estimator->estim_angle_sum,6.28f);
	estimator->estim_angle_deg=(estimator->estim_angle*180.0f)/3.14159f;
	//estimator->estim_angle_abs=fabs(estimator->estim_angle);
}
