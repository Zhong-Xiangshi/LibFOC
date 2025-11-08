#include "libfoc_interface.h"

// --------------------------------用户实现 start---------------------------------------------


/// @brief 初始化PWM、编码器、电流传感
/// @param pdrv 物理电机标识，用以区分不同电机。小于MOTOR_COUNT
void foc_driver_init(uint8_t pdrv)
{

}
/// @brief 电机使能函数
/// @param enable 1使能，0禁用
void foc_driver_motor_enable(uint8_t pdrv,uint8_t enable)
{
    if (enable)
    {

    }
    else
    {

    }
}

/// @brief 设置电机三个相PWM波形的占空比，建议PWM频率：20kHZ-30kHZ。输入范围0~motor->motor_pwm_max
/// @param phase_a
/// @param phase_b
/// @param phase_c
void foc_driver_set_phase(uint8_t pdrv,uint16_t phase_a, uint16_t phase_b, uint16_t phase_c)
{

}


/// @brief 获得机械角度。
/// @param angle 角度，0°-360°，顺时针增加，单位°
void foc_driver_get_mech_angle(uint8_t pdrv,float *angle)
{

}

/// @brief 调试打印
/// @param fmt
/// @param
void foc_driver_debug_printf(uint8_t pdrv,const char *const fmt, ...)
{

}

/// @brief 毫秒延时
/// @param ms
void foc_driver_delay_ms(uint8_t pdrv,uint32_t ms)
{

}
// --------------------------------用户实现 end---------------------------------------------
