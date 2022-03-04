/*
 * calibration.c
 *
 *  Created on: Aug 11, 2020
 *      Author: ben
 */


#include "calibration.h"
#include "hw_config.h"
#include "user_config.h"
#include <stdio.h>
#include <stdlib.h>
#include "usart.h"
#include "math_ops.h"

void order_phases(EncoderStruct *encoder, ControllerStruct *controller, CalStruct * cal, int loop_count){
	/*检查相序，以确保产生正Q电流位置传感器正方向上的扭矩*/
	PHASE_ORDER = 0;

	if(!cal->started){
		printf("Checking phase sign, pole pairs\r\n");
		cal->started = 1;
		cal->start_count = loop_count;
	}
	cal->time = (float)(loop_count - cal->start_count)*DT;

    if(cal->time < T1){
        // 将电压角设置为零，等待转子位置稳定下来
        cal->theta_ref = 0;//W_CAL*cal->time;
        cal->cal_position.elec_angle = cal->theta_ref;
        cal->cal_position.elec_velocity = 0;
        controller->i_d_des = I_CAL;
        controller->i_q_des = 0.0f;
        commutate(controller, &cal->cal_position);
    	cal->theta_start = encoder->angle_multiturn[0];
    	return;
    }

    else if(cal->time < T1+2.0f*PI_F/W_CAL){
    	// 将电压矢量旋转一个电气循环
    	cal->theta_ref = W_CAL*(cal->time-T1);
    	cal->cal_position.elec_angle = cal->theta_ref;
		commutate(controller, &cal->cal_position);
    	return;
    }

	reset_foc(controller);

	float theta_end = encoder->angle_multiturn[0];
	cal->ppairs = round(2.0f*PI_F/fabsf(theta_end-cal->theta_start));

	if(cal->theta_start < theta_end){
		cal->phase_order = 0;
		printf("Phase order correct\r\n");
	}
	else{
		cal->phase_order = 1;
		printf("Swapping phase sign\r\n");
	}
    printf("Pole Pairs: %d\r\n", cal->ppairs);
    printf("Start: %.3f   End: %.3f\r\n", cal->theta_start, theta_end);
    PHASE_ORDER = cal->phase_order;
    PPAIRS = (float)cal->ppairs;
    cal->started = 0;
    cal->done_ordering = 1;	// 完成了相序检查
}

void calibrate_encoder(EncoderStruct *encoder, ControllerStruct *controller, CalStruct * cal, int loop_count){
	/* 校准零点和编码器非线性*/
	if(!cal->started){
			printf("Starting offset cal and linearization\r\n");		//启动偏移校准和线性化
			cal->started = 1;
			cal->start_count = loop_count;
			cal->next_sample_time = T1;
			cal->sample_count = 0;
		}

	cal->time = (float)(loop_count - cal->start_count)*DT;				//计算采样时间

    if(cal->time < T1){
        // 将电压角设置为零，等待转子位置稳定下来	
        cal->theta_ref = 0;//W_CAL*cal->time;
        cal->cal_position.elec_angle = cal->theta_ref;
        controller->i_d_des = I_CAL;
        controller->i_q_des = 0.0f;
        commutate(controller, &cal->cal_position);

    	cal->theta_start = encoder->angle_multiturn[0];
    	cal->next_sample_time = cal->time;
    	return;
    }else if (cal->time < T1+2.0f*PI_F*PPAIRS/W_CAL){
    	// 通过正向机械旋转旋转电压矢量
		cal->theta_ref += W_CAL*DT;//(cal->time-T1);
		cal->cal_position.elec_angle = cal->theta_ref;
		commutate(controller, &cal->cal_position);

		// sample SAMPLES_PER_PPAIR times per pole-pair
		if(cal->time > cal->next_sample_time){
			int count_ref = cal->theta_ref * (float)ENC_CPR/(2.0f*PI_F*PPAIRS);
			int error = encoder->raw - count_ref;//- encoder->raw;
			cal->error_arr[cal->sample_count] = error + ENC_CPR*(error<0);
			printf("%d %d %d %.3f\r\n", cal->sample_count, count_ref, cal->error_arr[cal->sample_count], cal->theta_ref);
			cal->next_sample_time += 2.0f*PI_F/(W_CAL*SAMPLES_PER_PPAIR);
			if(cal->sample_count == PPAIRS*SAMPLES_PER_PPAIR-1){
				return;
			}
			cal->sample_count++;

		}
		return;
    }else if (cal->time < T1+4.0f*PI_F*PPAIRS/W_CAL){
		// rotate voltage vector through one mechanical rotation in the negative direction
		cal->theta_ref -= W_CAL*DT;//(cal->time-T1);
		controller->i_d_des = I_CAL;
		controller->i_q_des = 0.0f;
		cal->cal_position.elec_angle = cal->theta_ref;
		commutate(controller, &cal->cal_position);

		// sample SAMPLES_PER_PPAIR times per pole-pair
		if((cal->time > cal->next_sample_time)&&(cal->sample_count>0)){
			int count_ref = cal->theta_ref * (float)ENC_CPR/(2.0f*PI_F*PPAIRS);
			int error = encoder->raw - count_ref;// - encoder->raw;
			error = error + ENC_CPR*(error<0);

			cal->error_arr[cal->sample_count] = (cal->error_arr[cal->sample_count] + error)/2;
			printf("%d %d %d %.3f\r\n", cal->sample_count, count_ref, cal->error_arr[cal->sample_count], cal->theta_ref);
			cal->sample_count--;
			cal->next_sample_time += 2.0f*PI_F/(W_CAL*SAMPLES_PER_PPAIR);
		}
		return;
    }

    reset_foc(controller);

    // Calculate average offset
    int ezero_mean = 0;
	for(int i = 0; i<((int)PPAIRS*SAMPLES_PER_PPAIR); i++){
		ezero_mean += cal->error_arr[i];
	}
	cal->ezero = ezero_mean/(SAMPLES_PER_PPAIR*PPAIRS);

	// Moving average to filter out cogging ripple

	int window = SAMPLES_PER_PPAIR;
	int lut_offset = (ENC_CPR-cal->error_arr[0])*N_LUT/ENC_CPR;
	for(int i = 0; i<N_LUT; i++){
			int moving_avg = 0;
			for(int j = (-window)/2; j<(window)/2; j++){
				int index = i*PPAIRS*SAMPLES_PER_PPAIR/N_LUT + j;
				if(index<0){index += (SAMPLES_PER_PPAIR*PPAIRS);}
				else if(index>(SAMPLES_PER_PPAIR*PPAIRS-1)){index -= (SAMPLES_PER_PPAIR*PPAIRS);}
				moving_avg += cal->error_arr[index];
			}
			moving_avg = moving_avg/window;
			int lut_index = lut_offset + i;
			if(lut_index>(N_LUT-1)){lut_index -= N_LUT;}
			cal->lut_arr[lut_index] = moving_avg - cal->ezero;
			printf("%d  %d\r\n", lut_index, moving_avg - cal->ezero);

		}

	cal->started = 0;
	cal->done_cal = 1;
}

void measure_lr(EncoderStruct *encoder, ControllerStruct *controller, CalStruct * cal, int loop_count){

}
