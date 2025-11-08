#ifndef __LIBFOC_H__
#define __LIBFOC_H__
#include "stdint.h"
#include "libfoc_interface.h"

typedef enum
{
    FOC_MODE_CURRENT = 0,      // 力矩(电流)模式
    FOC_MODE_SPEED = 1,        // 速度模式
    FOC_MODE_POSITION = 2,     // 位置模式（三环）
    FOC_MODE_POSITION_TWO = 3, // 位置模式（二环）
    FOC_MODE_MAX
} foc_mode_t;

// foc初始化
int foc_init(uint8_t pdrv, uint8_t pole_pairs, uint16_t motor_pwm_max, float i_max);

// 设置电流环PID
void foc_current_set_pid_param(uint8_t pdrv, float scale, float iq_kp, float iq_ki, float id_kp, float id_ki);
// 电流环更新，参考调用频率4khz
void foc_current_update(uint8_t pdrv, float phase_a_current, float phase_b_current, float phase_c_current, float Filter_coefficient);

// 设置速度环PID
void foc_speed_set_pid_param(uint8_t pdrv, float scale, float alpha, float kp, float ki, float i_max);
// 速度环更新，参考调用频率1khz
void foc_speed_update(uint8_t pdrv, float interval);

// 设置位置环PID
void foc_position_set_pid_param(uint8_t pdrv, float scale, float alpha, float kp, float ki, float kd, float imax);
// 位置环更新，参考调用频率1khz
void foc_position_update(uint8_t pdrv);     // 三环版本
void foc_position_update_two(uint8_t pdrv); // 双环版本

// 设置控制模式
void foc_set_mode(uint8_t pdrv, foc_mode_t mode);
// 设置电机控制目标值
void foc_set_target(uint8_t pdrv, float target);

// 获取电机力矩（IQ，和力矩成正比，需要手动乘以常数）
float foc_get_torque(uint8_t pdrv);
// 获取电机速度
float foc_get_speed(uint8_t pdrv);
// 获取电机位置
float foc_get_position(uint8_t pdrv);

/*
    ====================================调试函数=====================================

/*
    获得电机极对数。以pwm_max输出（注意控制此值，否则电机发烫或烧坏电机），固定为0电角度，此时手动转动电机，转一圈需要用的间隔数即电机的极对数。不依赖极对数、角度传感器和电流传感器。
    注意：使用demo函数不要调用电流环更新函数！！！
*/
void foc_demo_0(uint8_t pdrv);

/*
    电机转轴朝上放置，每隔1s以pwm_max顺时针转动90电角度。如果为逆时针需要换线序。不依赖极对数、角度传感器和电流传感器。
*/
void foc_demo_1(uint8_t pdrv);

/*
    以90°固定电压矢量连续顺时针转动。顺时针角度增加。极对数不对则无法连续转动或者明显周期晃动。角度通过LOG打印查看。只依赖角度传感器、极对数。
*/
void foc_demo_2(uint8_t pdrv);

/*
    使用方法：请开启中断读取三相电流值，将电流值指针传入此函数，后面几个函数也做相同操作。
    用于调试电流方向。每隔1s顺时针通电ABC三相,并LOG打印显示三相电流。正常情况：A通电A相电流正，B通电B相电流正，C通电C相电流正。错误请修改驱动对应好。只依赖电流传感器。
*/
void foc_demo_31(uint8_t pdrv, float *phase_a_current, float *phase_b_current, float *phase_c_current);

/*
    以90°固定电压矢量控制电机顺时针转动,LOG打印三相电流。用手捏住电机缓慢转动可以看到交流变化波形。依赖角度传感器、电流传感器、极对数。
*/
void foc_demo_32(uint8_t pdrv, uint8_t motor_en, float *phase_a_current, float *phase_b_current, float *phase_c_current, float Filter_coefficient);

/*
    以90°固定电压矢量控制电机，显示IQ和ID电流。正常情况堵转ID接近0。依赖角度传感器、电流传感器、极对数。
    这个如果ok就可以直接调用更新电流环函数开始调PID了
*/
void foc_demo_4(uint8_t pdrv, float *phase_a_current, float *phase_b_current, float *phase_c_current);

#endif
