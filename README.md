# LibFOC
(正在开发中)  
An embedded FOC library  
嵌入式 FOC 库

## ⚠️ 安全警告 / Safety Warnings
电机控制可能造成人身伤害或设备损坏  
Motor control may cause personal injury or equipment damage

使用前必须进行安全风险评估  
Safety risk assessment must be performed before use

严禁用于生命支持系统等关键场景  
Strictly prohibited for use in life-support systems or other critical applications

## Driver functions to be implemented (需要实现的驱动函数)  
```C
typedef struct{
    /// @brief Initialize PWM, encoder, current sensing  
    /// @brief 初始化PWM、编码器、电流传感  
    void (*foc_driver_init)(void);

    /// @brief Set duty cycle for three-phase PWM waveforms. Recommended PWM frequency: 20kHZ-30kHZ  
    /// @brief 设置电机三个相PWM波形的占空比，建议PWM频率：20kHZ-30kHZ  
    void (*foc_motor_set_phase)(uint16_t phase_a, uint16_t phase_b, uint16_t phase_c);

    /// @brief Motor enable function  
    /// @brief 电机使能函数  
    void (*foc_motor_enable)(uint8_t enable);

    /// @brief Get mechanical angle  
    /// @brief 获得机械角度  
    void (*foc_get_mech_angle)(float *angle);

    /// @brief Get three-phase current values in Amperes. Positive when outputting (flowing to motor)  
    /// @brief 获得3个相的电流，单位1A，输出(流向电机)为正  
    void (*foc_get_phase_current)(float *phase_a, float *phase_b, float *phase_c);
} foc_interface_t;
```

## How to Use (如何使用)  
### Torque Control Example (力矩控制示例)  
```C
#include "libfoc.h"
motor ms;
// Implement motor driver functions  
// 实现电机驱动函数  
foc_interface_t foc_driver = {
    .foc_driver_init = foc_driver_init,
    .foc_motor_set_phase = foc_motor_set_phase,
    .foc_motor_enable = foc_motor_enable,
    .foc_get_mech_angle = foc_get_mech_angle,
    .foc_get_phase_current = foc_get_phase_current,
};

int main(void){
    ...
    // 1. Initialize motor driver, pole pairs, PWM_MAX  
    // 1.初始化电机驱动、极对数、PWM_MAX  
    foc_init(&ms,foc_driver,7,100);
    
    // 2. Set current loop PID parameters: Iq_P, Iq_I, Id_P, Id_I  
    // 2.设置电流环PID参数：Iq_P、Iq_I、Id_P、Id_I  
    foc_current_set_pid_param(&ms,1,25, 7, 25, 7);
    
    // 3. Set mode to current loop  
    // 3.设置模式为电流环  
    foc_set_mode(&ms,FOC_MODE_CURRENT);
    
    // 4. Set target value to 0.2A  
    // 4.设置目标0.2A  
    foc_set_target(&ms,0.2f);
    ...
    while(1){
        // 5. Update current loop  
        // 5.更新电流环，建议在定时器中断调用  
        foc_current_update(&ms);
    }
}
```
### Debug（调试）
#### Debug Functions (调试函数说明)  
```C
/*
    Obtain motor pole pairs. Output at pwm_max (Note: control this value to prevent motor overheating or damage), 
    fixed at 0 electrical angle. Manually rotate the motor one full circle, the number of intervals required is the motor's pole pairs.
    获得电机极对数。以pwm_max输出（注意控制此值，否则电机发烫或烧坏电机），固定为0电角度，此时手动转动电机，转一圈需要用的间隔数即电机的极对数
*/
void foc_demo_0(motor *motor);  

/*
    Rotate 90 electrical degrees clockwise every 1s at pwm_max. Swap phase sequence if counterclockwise rotation is needed.
    每隔1s以pwm_max顺时针转动90电角度。如果为逆时针需要换线序
*/
void foc_demo_1(motor *motor);  

/*
    Verify pole pair correctness and angle increment direction. Depends on angle sensor/pole pairs.
    Continuous clockwise rotation at fixed 90° voltage vector. Rotation fails if pole pairs incorrect. Angle increases clockwise.
    用于测试极对数是否正确，角度增量方向是否正常。依赖角度传感器、极对数。以90°固定电压矢量连续顺时针转动。极对数不对则无法连续转动。顺时针角度增加。
*/
void foc_demo_2(motor *motor);

/*
    Debug current direction. Energize ABC phases clockwise every 1s and display three-phase currents.
    Normal: A phase current positive when A energized, same for B/C.
    用于调试电流方向。每隔1s顺时针通电ABC三相,并显示三相电流。正常情况：A通电A相电流正，B通电B相电流正，C通电C相电流正
*/
void foc_demo_31(motor *motor); 

/*
    Control motor clockwise at fixed 90° voltage vector. Depends on angle sensor/current sensor/pole pairs. Displays three-phase currents.
    依赖角度传感器、电流传感器、极对数。以90°固定电压矢量控制电机顺时针转动,显示三相电流
*/
void foc_demo_32(motor *motor,uint8_t motor_en);    

/*
    Control motor at fixed 90° voltage vector. Displays IQ and ID currents. 
    Normal: ID near 0 when stalled. Depends on angle/current sensors/pole pairs.
    依赖角度传感器、电流传感器、极对数。以90°固定电压矢量控制电机，显示IQ和ID电流。正常情况堵转ID接近0
*/
void foc_demo_4(motor *motor,uint8_t motor_en); 

/*
    Torque control. Displays IQ, ID, VQ, VD. Depends on angle/current sensors/pole pairs.
    依赖角度传感器、电流传感器、极对数。力矩控制，显示IQ、ID、VQ、VD
*/
void foc_demo_5(motor *motor,uint8_t motor_en,float target_iq); 

/*
    Based on demo4 with IQ/ID low-pass filtering added. Displays IQ and ID currents.
    demo4为原型添加IQ、ID低通滤波，显示IQ和ID电流
*/
void foc_demo_6(motor *motor,uint8_t motor_en); 
```
#### Debug Example (调试示例)  
```C
#include "libfoc.h"
motor ms;
// Implement motor driver functions  
// 实现电机驱动函数  
foc_interface_t foc_driver = {
    .foc_driver_init = foc_driver_init,
    .foc_motor_set_phase = foc_motor_set_phase,
    .foc_motor_enable = foc_motor_enable,
    .foc_get_mech_angle = foc_get_mech_angle,
    .foc_get_phase_current = foc_get_phase_current,
};

int main(void){
    ...
    // 1. Initialize motor driver, pole pairs, PWM_MAX  
    // 1.初始化电机驱动、极对数、PWM_MAX  
    foc_init(&ms,foc_driver,7,30);
    
    // 2.调用debug函数
    // 2. Call debug functions
    foc_demo_0(&ms);
    // foc_demo_1(&ms);
    // foc_demo_2(&ms);
    // foc_demo_31(&ms);
    // foc_demo_4(&ms,1);
}
```
