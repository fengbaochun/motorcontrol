/// Values stored in flash, which are modified by user actions ///
/// 存储在flash中的值，由用户操作修改                            ///

#ifndef USER_CONFIG_H
#define USER_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif


#define I_BW                    __float_reg[2]          // 电流环带宽                   Current loop bandwidth
#define I_MAX                   __float_reg[3]          // 电流限制                     Current limit        
#define THETA_MIN               __float_reg[4]          // 最小位置设定点               Minimum position setpoint
#define THETA_MAX               __float_reg[5]          // 最大位置设定点               Maximum position setpoint
#define I_FW_MAX                __float_reg[6]          // 最大弱磁电流                 Maximum field weakening current
#define R_NOMINAL               __float_reg[7]          // 标定期间设置的标称电机电阻   Nominal motor resistance, set during calibration
#define TEMP_MAX                __float_reg[8]          // 安全温度限制                 Temperature safety lmit
#define I_MAX_CONT              __float_reg[9]          // 连续最大电流                 Continuous max current
#define PPAIRS					__float_reg[10]			// 电机极对数                   Number of motor pole-pairs
//#define L_D						__float_reg[11]			// D-axis inductance
//#define L_Q						__float_reg[12]			// Q-axis inductance
#define R_PHASE					__float_reg[13]			// 单相电阻                     Single phase resistance
#define KT						__float_reg[14]			// 扭矩常数（N-m/A）            Torque Constant (N-m/A)
#define R_TH					__float_reg[15]			// 热阻（C/W）                  Thermal resistance (C/W)
#define C_TH					__float_reg[16]			// 热质量（C/J）                Thermal mass (C/J)
#define GR						__float_reg[17]			// 传动比                       Gear ratio
#define I_CAL					__float_reg[18]			// 校准电流                     Calibration Current
#define P_MIN					__float_reg[19]			// 位置设置点的下限             Position setpoint lower limit (rad)
#define P_MAX					__float_reg[20]			// 位置设置点的上限             Position setupoint upper bound (rad)
#define V_MIN					__float_reg[21]			// 速度下限                     Velocity setpoint lower bound (rad/s)
#define V_MAX					__float_reg[22]			// 速度上限                     Velocity setpoint upper bound (rad/s)
#define KP_MAX					__float_reg[23]			// 最大位置增益(N-m/rad)
#define KD_MAX					__float_reg[24]			// 最大速度增益(N-m/rad/s)


#define PHASE_ORDER             __int_reg[0]            // 校准期间的相位交换           Phase swapping during calibration
#define CAN_ID                  __int_reg[1]            // CAN bus ID
#define CAN_MASTER              __int_reg[2]            // CAN bus "master" ID
#define CAN_TIMEOUT             __int_reg[3]            // CAN bus timeout period
#define M_ZERO					__int_reg[4]
#define E_ZERO					__int_reg[5]
#define ENCODER_LUT             __int_reg[6]            // 编码器偏移LUT-128个元素长    Encoder offset LUT - 128 elements long




extern float __float_reg[];
extern int __int_reg[];

#ifdef __cplusplus
}
#endif

#endif
