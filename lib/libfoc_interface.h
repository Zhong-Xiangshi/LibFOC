#ifndef __LIBFOC_INTERFACE_H__
#define __LIBFOC_INTERFACE_H__
#include "stdint.h"

// --------------驱动函数，需要自行实现------------------


typedef struct{

    /// @brief 初始化PWM、编码器、电流传感
    void (*foc_driver_init)(void);

    /// @brief 设置电机三个相PWM波形的占空比，建议PWM频率：20kHZ-30kHZ
    /// @param phase_a
    /// @param phase_b
    /// @param phase_c
    void (*foc_motor_set_phase)(uint16_t phase_a, uint16_t phase_b, uint16_t phase_c);

    /// @brief 电机使能函数
    /// @param enable 1使能，0禁用
    void (*foc_motor_enable)(uint8_t enable);

    /// @brief 获得机械角度。
    /// @param angle 角度，0°-360°，顺时针增加，单位1°
    void (*foc_get_mech_angle)(float *angle);

} foc_interface_t;

void foc_debug_printf(const char *const fmt, ...);
void foc_delay_ms(uint32_t ms);

#endif
