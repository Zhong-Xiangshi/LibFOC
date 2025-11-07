#include "libfoc_interface.h"

// --------------------------------用户实现 start---------------------------------------------

/// @brief 初始化PWM、编码器、电流传感
/// @param
void foc_driver_init(void)
{

}
/// @brief 电机使能函数
/// @param enable 1使能，0禁用
void foc_motor_enable(uint8_t enable)
{
    if (enable)
    {

    }
    else
    {

    }
}

/// @brief 设置电机三个相PWM波形的占空比，输入范围0~motor->motor_pwm_max
/// @param phase_a
/// @param phase_b
/// @param phase_c
void foc_motor_set_phase(uint16_t phase_a, uint16_t phase_b, uint16_t phase_c)
{

}


/// @brief 获得机械角度。
/// @param angle 角度，顺时针增加，单位1°
void foc_get_mech_angle(float *angle)
{

}

foc_interface_t foc_driver = {
    .foc_driver_init = foc_driver_init,
    .foc_motor_set_phase = foc_motor_set_phase,
    .foc_motor_enable = foc_motor_enable,
    .foc_get_mech_angle = foc_get_mech_angle,
};

/// @brief 调试打印
/// @param fmt
/// @param
void foc_debug_printf(const char *const fmt, ...)
{

}

/// @brief 毫秒延时
/// @param ms
void foc_delay_ms(uint32_t ms)
{

}
// --------------------------------用户实现 end---------------------------------------------
