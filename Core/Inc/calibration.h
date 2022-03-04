/*
 * calibration.h
 *
 *  Created on: Aug 11, 2020
 *      Author: ben
 */

#ifndef INC_CALIBRATION_H_
#define INC_CALIBRATION_H_

#include "position_sensor.h"
#include "foc.h"

#define V_CAL 	1.0f							// 标定电压			Calibration voltage
//#define I_CAL	5.0f
#define W_CAL 	10.0f							// 标定速度			Calibration speed in rad/s
#define T1		1.0f							// 标定设置周期		Cal settling period
#define PPAIRS_MAX 64
#define SAMPLES_PER_PPAIR N_LUT
#define N_CAL SAMPLES_PER_PPAIR*PPAIRS_MAX		// Calibration lookup table maximum length, so I don't have to deal with dynamic allocation based on number of pole pairs

typedef struct{
	uint8_t ppairs;									// 测量的极对数					number of pole pairs measured
	int offset;										// 计数中的电气零位				electrical zero position in counts
	float theta_ref;								// 用于校准的参考角				reference angle used for calibration
	int start_count;								// cal开始时的循环计数			loop count at cal start
	uint8_t started;								// 标定状态	    				has cal started or not?
	float time;										// 标定时间						cal time
	float theta_start;								// 校准起始角度					cal start angle
	int ezero;
	uint8_t phase_order;							// 相序正确（0）或交换（1）		phase order correct (0) or swapped (1)
	uint8_t done_ordering, done_cal, done_rl;		// 不同CAL的标志已完成			flags for different cals finished
	uint16_t sample_count;							// 记录采集的样本数量			keep track of how many samples taken
	float next_sample_time;							// time to take next sample
	int error_arr[N_CAL];
	int lut_arr[N_LUT];
	EncoderStruct cal_position;						// 用于校准的位置参考			Position reference used for calibration

} CalStruct;

void order_phases(EncoderStruct *encoder, ControllerStruct *controller, CalStruct *cal, int loop_count);
void calibrate_encoder(EncoderStruct *encoder, ControllerStruct *controller, CalStruct *cal,
		int loop_count);
void measure_lr(EncoderStruct *encoder, ControllerStruct *controller, CalStruct *cal,  int loop_count);

//extern int *error_array;
//extern int *lut_array;

#endif /* INC_CALIBRATION_H_ */
