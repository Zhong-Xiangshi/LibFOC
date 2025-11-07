#ifndef __LIBFOC_H__
#define __LIBFOC_H__
#include "stdint.h"
#include "libfoc_interface.h"

typedef struct{
    float x;
    float y;
}  vector;

typedef struct{
    //缩放倍数
    float scale;
    float input; //输入值
    float alpha; //低通滤波系数
    float kp;
    float ki;
    float kd;
    float i_max;    //积分限幅
    float i_min;    //积分限幅
    float error;
    float i;
    float last_error;

}  pid_param;

typedef enum{
    FOC_MODE_CURRENT = 0, //力矩(电流)模式
    FOC_MODE_SPEED = 1,   //速度模式
    FOC_MODE_POSITION = 2, //位置模式（三环）
    FOC_MODE_POSITION_TWO = 3, //位置模式（二环）
    FOC_MODE_MAX
} foc_mode;

//电机句柄
typedef struct{

    foc_interface_t driver; // 驱动函数接口
    // 是否已经初始化
    uint8_t init_already;
    // 电机极对数，用来计算电角度，电角度=机械角度*极对数。
    uint8_t pole_pairs;
    // 电机最大占空比
    uint16_t motor_pwm_max;
    // 控制模式
    foc_mode mode;

    // 三相PWM值
    int16_t phase_a, phase_b, phase_c;

    
    //机械角度,初始机械角度,上次机械角度
    float mech_angle,mech_angle_zero,mech_angle_last;
    //电角度（单位弧度）
    float elec_angle_rad;

    //电流环
    float target_iq;
    float i_max;    // 电流最大值限制
    float iq,id;
    float vq,vd;
    float phase_a_current, phase_b_current, phase_c_current; //三相电流
    pid_param pid_iq,pid_id;
    vector parker_x,parker_y;
    vector current_by_clarke; //克拉克坐标系电流矢量

    //速度环
    float target_speed;
    float speed;
    pid_param pid_speed;

    //位置环
    float target_position;
    float position_last; // 上次位置
    float position;
    pid_param pid_position;

}  motor;

//foc初始化
int foc_init(motor *motor,foc_interface_t driver,uint8_t pole_pairs,uint16_t motor_pwm_max,float i_max);

//设置电流环PID
void foc_current_set_pid_param(motor *motor,float scale,float iq_kp,float iq_ki,float id_kp,float id_ki);
//电流环更新，参考调用频率4khz
void foc_current_update(motor *motor,float phase_a_current,float phase_b_current,float phase_c_current);

//设置速度环PID
void foc_speed_set_pid_param(motor *motor,float scale,float alpha,float kp,float ki,float i_max);
//速度环更新，参考调用频率1khz
void foc_speed_update(motor *motor,float interval);

//设置位置环PID
void foc_position_set_pid_param(motor *motor,float scale,float alpha,float kp,float ki,float kd,float imax);
//位置环更新，参考调用频率1khz
void foc_position_update(motor *motor); //三环版本
void foc_position_update_two(motor *motor); //双环版本

//设置控制模式
void foc_set_mode(motor *motor,foc_mode mode);
//设置电机控制目标值
void foc_set_target(motor *motor,float target);

//获取电机力矩（IQ，和力矩成正比，需要手动乘以常数）
float foc_get_torque(motor *motor);
//获取电机速度
float foc_get_speed(motor *motor);
//获取电机位置
float foc_get_position(motor *motor);

/*
    调试函数
    debug functions
*/

/*
    获得电机极对数。以pwm_max输出（注意控制此值，否则电机发烫或烧坏电机），固定为0电角度，此时手动转动电机，转一圈需要用的间隔数即电机的极对数。不依赖极对数、角度传感器和电流传感器。
    注意：使用demo函数不要调用电流环更新函数！！！
*/
void foc_demo_0(motor *motor);  

/*
    电机转轴朝上放置，每隔1s以pwm_max顺时针转动90电角度。如果为逆时针需要换线序。不依赖极对数、角度传感器和电流传感器。
*/
void foc_demo_1(motor *motor);  

/*
    以90°固定电压矢量连续顺时针转动。顺时针角度增加。极对数不对则无法连续转动或者明显周期晃动。角度通过LOG打印查看。只依赖角度传感器、极对数。
*/
void foc_demo_2(motor *motor);

/*
    使用方法：请开启中断读取三相电流值，将电流值指针传入此函数，后面几个函数也做相同操作。
    用于调试电流方向。每隔1s顺时针通电ABC三相,并LOG打印显示三相电流。正常情况：A通电A相电流正，B通电B相电流正，C通电C相电流正。错误请修改驱动对应好。只依赖电流传感器。
*/
void foc_demo_31(motor *motor,float *phase_a_current,float *phase_b_current,float *phase_c_current); 

/*
    以90°固定电压矢量控制电机顺时针转动,LOG打印三相电流。用手捏住电机缓慢转动可以看到交流变化波形。依赖角度传感器、电流传感器、极对数。
*/
void foc_demo_32(motor *motor,uint8_t motor_en,float *phase_a_current,float *phase_b_current,float *phase_c_current);    

/*
    以90°固定电压矢量控制电机，显示IQ和ID电流。正常情况堵转ID接近0。依赖角度传感器、电流传感器、极对数。
    这个如果ok就可以直接调用更新电流环函数开始调PID了
*/
void foc_demo_4(motor *motor,float *phase_a_current,float *phase_b_current,float *phase_c_current); 

#endif
