#include "libfoc.h"
#include "math.h"
#include "libfoc_interface.h"
#include "errno.h"
#include "stdio.h"

#define PI 3.1415f
#define SQRT3 1.732f
#define SQRT3_2 0.866f // sqrt(3)/2

#define MOTOR_COUNT 1
#define USE_CURRENT_FILTER
// #define USE_ZERO_SEQUENCE_INJECTION //Zero-Sequence Component Injection

struct vector
{
    float x;
    float y;
};
typedef struct vector vector_t;

struct pid_param
{
    // 缩放倍数
    float scale;
    float input; // 输入值
    float alpha; // 低通滤波系数
    float kp;
    float ki;
    float kd;
    float i_max; // 积分限幅
    float i_min; // 积分限幅
    float error;
    float i;
    float last_error;
};
typedef struct pid_param pid_param_t;

/// @brief 向量相加
/// @param a
/// @param b
/// @return
static inline vector_t vector_add(vector_t a, vector_t b)
{
    vector_t c;
    c.x = a.x + b.x;
    c.y = a.y + b.y;
    return c;
}

/// @brief 向量数乘
/// @param a
/// @param b
/// @return
static inline vector_t vector_multiply(vector_t a, float b)
{
    vector_t c;
    c.x = a.x * b;
    c.y = a.y * b;
    return c;
}

/// @brief 向量点乘
/// @param a
/// @param b
/// @return
static inline float vector_dot(vector_t a, vector_t b)
{
    return a.x * b.x + a.y * b.y;
}

/// @brief 向量长度
/// @param a
/// @return
static inline float vector_length(vector_t a)
{
    return sqrtf(vector_dot(a, a));
}

/// @brief b在a方向上的投影长度
/// @param a
/// @param b
/// @return
static inline float vector_projection(vector_t a, vector_t b)
{
    return vector_dot(a, b) / vector_length(a);
}

/// @brief 向量旋转
/// @param a
/// @param rad
/// @return
static inline vector_t vector_rotate(vector_t a, float rad)
{
    vector_t b;
    b.x = a.x * cosf(rad) - a.y * sinf(rad);
    b.y = a.x * sinf(rad) + a.y * cosf(rad);
    return b;
}

// 电机句柄
struct motor
{
    // 是否已经初始化
    uint8_t init_already;
    // 电机极对数，用来计算电角度，电角度=机械角度*极对数。
    uint8_t pole_pairs;
    // 电机最大占空比
    uint16_t motor_pwm_max;
    // 控制模式
    foc_mode_t mode;

    // 三相PWM值
    int16_t phase_a, phase_b, phase_c;

    // 机械角度,初始机械角度,上次机械角度
    float mech_angle, mech_angle_zero, mech_angle_last;
    // 电角度（单位弧度）
    float elec_angle_rad;

    // 电流环
    float target_iq;
    float i_max; // 电流最大值限制
    float iq, id;
    float vq, vd;
    float current_phase_a_filter;
    float current_phase_b_filter;
    float current_phase_c_filter;

    pid_param_t pid_iq, pid_id;
    vector_t parker_x, parker_y;
    vector_t current_by_clarke; // 克拉克坐标系电流矢量

    // 速度环
    float target_speed;
    float speed;
    pid_param_t pid_speed;

    // 位置环
    float target_position;
    float position_last; // 上次位置
    float position;
    pid_param_t pid_position;
};
typedef struct motor motor_t;

static motor_t motor_array[MOTOR_COUNT];

/// @brief 六步换向法控制电机函数
/// @param step
static void foc_motor_set_step(uint8_t pdrv, uint8_t step)
{
    motor_t *motor = &motor_array[pdrv];
    switch (step)
    {
    case 1:
        foc_driver_set_phase(pdrv,motor->motor_pwm_max, 0, 0);
        break;
    case 2:
        foc_driver_set_phase(pdrv,motor->motor_pwm_max, motor->motor_pwm_max, 0);
        break;
    case 3:
        foc_driver_set_phase(pdrv,0, motor->motor_pwm_max, 0);
        break;
    case 4:
        foc_driver_set_phase(pdrv,0, motor->motor_pwm_max, motor->motor_pwm_max);
        break;
    case 5:
        foc_driver_set_phase(pdrv,0, 0, motor->motor_pwm_max);
        break;
    case 6:
        foc_driver_set_phase(pdrv,motor->motor_pwm_max, 0, motor->motor_pwm_max);
        break;
    default:
        foc_driver_set_phase(pdrv,0, 0, 0);
        break;
    }
}

/// @brief SPWM控制电机函数
/// @param size 单位：pwm值 (峰值，相对于pwm_mid的偏移)
/// @param rad 单位：弧度 (a相的当前电角度)
static void foc_motor_spwm_control_by_rad(uint8_t pdrv, float size, float rad)
{
    motor_t *motor = &motor_array[pdrv];
    int16_t phase_a_pwm, phase_b_pwm, phase_c_pwm;
    uint16_t pwm_max = motor->motor_pwm_max;
    uint16_t pwm_mid = pwm_max / 2;

    // 限制幅值，确保输出在PWM范围内
    if (size > pwm_mid)
    {
        size = pwm_mid;
    }

    // 计算三相瞬时值 (相对于pwm_mid的偏移)
    float val_a = size * cosf(rad);
    float val_b = size * cosf(rad - 2.0f * PI / 3.0f);
    float val_c = size * cosf(rad - 4.0f * PI / 3.0f);

    // 转换为PWM占空比值并进行四舍五入
    phase_a_pwm = (int16_t)(val_a + pwm_mid + 0.5f);
    phase_b_pwm = (int16_t)(val_b + pwm_mid + 0.5f);
    phase_c_pwm = (int16_t)(val_c + pwm_mid + 0.5f);

    // PWM占空比饱和处理
    if (phase_a_pwm < 0)
        phase_a_pwm = 0;
    if (phase_b_pwm < 0)
        phase_b_pwm = 0;
    if (phase_c_pwm < 0)
        phase_c_pwm = 0;
    if (phase_a_pwm > pwm_max)
        phase_a_pwm = pwm_max;
    if (phase_b_pwm > pwm_max)
        phase_b_pwm = pwm_max;
    if (phase_c_pwm > pwm_max)
        phase_c_pwm = pwm_max;

    // foc_driver_debug_printf(pdrv,"PWM OUTPUT angle=%.1f,phase_a=%d, phase_b=%d, phase_c=%d\n", rad*180.0f/PI,phase_a_pwm, phase_b_pwm, phase_c_pwm);
    foc_driver_set_phase(pdrv,(uint16_t)phase_a_pwm, (uint16_t)phase_b_pwm, (uint16_t)phase_c_pwm);
}



/// @brief 计算扇区函数。从x轴开始逆时针每隔60°增加1
/// @param v 克拉克坐标系电压合矢量（U_alpha，U_beta）
/// @return 扇区号 1-6
static uint8_t calc_sector(vector_t v)
{
    float U_alpha = v.x;
    float U_beta = v.y;
    
    float U1 = U_beta;
    float U2 = (SQRT3 * U_alpha - U_beta) / 2.0f;
    float U3 = (-SQRT3 * U_alpha - U_beta) / 2.0f;

    uint8_t sector = 0;
    if (U1 > 0) sector += 1;
    if (U2 > 0) sector += 2;
    if (U3 > 0) sector += 4;
    
    // 扇区映射表
    const uint8_t sector_map[8] = {0, 2, 6, 1, 4, 3, 5, 0}; // N=0,7为非法
    return sector_map[sector];
}

//获得扇区二个基向量
static void get_sector_base_vector(uint8_t sector,vector_t *Va,vector_t *Vb)
{
    switch (sector)
    {
        case 1:
            Va->x = 1.0f;
            Va->y = 0.0f;
            Vb->x = 0.5f;
            Vb->y = SQRT3_2;
            break;
        case 2:
            Va->x = 0.5f;
            Va->y = SQRT3_2;
            Vb->x = -0.5f;
            Vb->y = SQRT3_2;
            break;
        case 3:
            Va->x = -0.5f;
            Va->y = SQRT3_2;
            Vb->x = -1.0f;
            Vb->y = 0.0f;
            break;
        case 4:
            Va->x = -1.0f;
            Va->y = 0.0f;
            Vb->x = -0.5f;
            Vb->y = -SQRT3_2;
            break;
        case 5:
            Va->x = -0.5f;
            Va->y = -SQRT3_2;
            Vb->x = 0.5f;
            Vb->y = -SQRT3_2;
            break;
        case 6:
            Va->x = 0.5f;
            Va->y = -SQRT3_2;
            Vb->x = 1.0f;
            Vb->y = 0.0f;
            break;
        default:
            Va->x = 0.0f;
            Va->y = 0.0f;
            Vb->x = 0.0f;
            Vb->y = 0.0f;
            break;
    }
}

//获得扇区二个基向量对应的三相状态编码(ABC)
static void get_sector_base_vector_code(uint8_t sector,uint8_t *Va_code,uint8_t *Vb_code)
{
    switch (sector)
    {
        case 1:
            *Va_code = 4;
            *Vb_code = 6;
            // *Va_code = 0b100;
            // *Vb_code = 0b110;
            break;
        case 2:
            *Va_code = 6;
            *Vb_code = 2;
            // *Va_code = 0b110;
            // *Vb_code = 0b010;
            break;
        case 3:
            *Va_code = 2;
            *Vb_code = 3;
            // *Va_code = 0b010;
            // *Vb_code = 0b011;
            break;
        case 4:
            *Va_code = 3;
            *Vb_code = 1;
            // *Va_code = 0b011;
            // *Vb_code = 0b001;
            break;
        case 5:
            *Va_code = 1;
            *Vb_code = 5;
            // *Va_code = 0b001;
            // *Vb_code = 0b101;
            break;
        case 6:
            *Va_code = 5;
            *Vb_code = 4;
            // *Va_code = 0b101;
            // *Vb_code = 0b100;
            break;
        default:
            *Va_code = 0;
            *Vb_code = 0;
            // *Va_code = 0b000;
            // *Vb_code = 0b000;
            break;
    }
}

/*
输入一个克拉克坐标系的电压矢量，输出中心对称模式PWM的三个PWM的阈值

1. 先计算合矢量所在的扇区，得到二个扇区基向量
2. 根据伏秒平衡公式解出T0、Ta、Tb
3. 计算P-ABC
*/

/// @brief 输入一个克拉克坐标系的电压矢量，输出中心对称模式PWM的三个PWM的阈值
/// @param [IN]U 克拉克坐标系的电压矢量(模长范围0-1)
/// @param [OUT]pwm_a 范围0-1
/// @param [OUT]pwm_b 范围0-1
/// @param [OUT]pwm_c 范围0-1
static void vector2svpwm(vector_t U, float *pwm_a, float *pwm_b, float *pwm_c)
{

    // 将输入的U，从外接圆归一化(模长0-1) 变换到 以2/3*Udc为基准的物理坐标系
    U.x *= (2.0f / 3.0f);
    U.y *= (2.0f / 3.0f);

    const uint8_t sector= calc_sector(U);
    vector_t Va,Vb; //扇区基向量矢量
    uint8_t Va_code,Vb_code; //扇区基向量对应的三相状态编码
    get_sector_base_vector(sector, &Va, &Vb);
    get_sector_base_vector_code(sector, &Va_code, &Vb_code);
    
    float Ta = (U.x*Vb.y - U.y*Vb.x) / (Va.x*Vb.y - Va.y*Vb.x);
    float Tb = (U.y*Va.x - U.x*Va.y) / (Va.x*Vb.y - Va.y*Vb.x);

    //过调制处理
    if (Ta + Tb > 1.0f)
    {
        float T_sum_inv = 1.0f / (Ta + Tb);
        Ta *= T_sum_inv;
        Tb *= T_sum_inv;
    }

    const float T0 = 1.0f - Ta - Tb;
    *pwm_a = T0 / 2.0f + ((Va_code & 4) ? Ta : 0) + ((Vb_code & 4) ? Tb : 0);
    *pwm_b = T0 / 2.0f + ((Va_code & 2) ? Ta : 0) + ((Vb_code & 2) ? Tb : 0);
    *pwm_c = T0 / 2.0f + ((Va_code & 1) ? Ta : 0) + ((Vb_code & 1) ? Tb : 0);

    if(*pwm_a<0.0f)*pwm_a=0.0f;
    if(*pwm_b<0.0f)*pwm_b=0.0f;
    if(*pwm_c<0.0f)*pwm_c=0.0f;
    if(*pwm_a>1.0f)*pwm_a=1.0f;
    if(*pwm_b>1.0f)*pwm_b=1.0f;
    if(*pwm_c>1.0f)*pwm_c=1.0f;

}

/// @brief 输入一个克拉克坐标系的电压矢量，输出中心对称模式PWM的三个PWM的阈值。使用最小/最大值注入零序分量的调制方法
/// @param [IN]U 克拉克坐标系的电压矢量(模长范围0-1)
/// @param [OUT]pwm_a 范围0-1
/// @param [OUT]pwm_b 范围0-1
/// @param [OUT]pwm_c 范围0-1
static void vector2svpwm_2(vector_t U, float *pwm_a, float *pwm_b, float *pwm_c)
{
    float phase_a_tmp, phase_b_tmp, phase_c_tmp;
    float tmp = SQRT3_2 * U.y;
    phase_a_tmp = U.x;
    phase_b_tmp = -0.5f * U.x + tmp;
    phase_c_tmp = -0.5f * U.x - tmp;
    float max_phase = fmaxf(fmaxf(phase_a_tmp, phase_b_tmp), phase_c_tmp);
    float min_phase = fminf(fminf(phase_a_tmp, phase_b_tmp), phase_c_tmp);
    float mid_offset_phase = (max_phase + min_phase) / 2.0f;
    *pwm_a = phase_a_tmp - mid_offset_phase + 0.5f;
    *pwm_b = phase_b_tmp - mid_offset_phase + 0.5f;
    *pwm_c = phase_c_tmp - mid_offset_phase + 0.5f;

    if(*pwm_a<0.0f)*pwm_a=0.0f;
    if(*pwm_b<0.0f)*pwm_b=0.0f;
    if(*pwm_c<0.0f)*pwm_c=0.0f;
    if(*pwm_a>1.0f)*pwm_a=1.0f;
    if(*pwm_b>1.0f)*pwm_b=1.0f;
    if(*pwm_c>1.0f)*pwm_c=1.0f;
}
static void foc_motor_spwm_control(uint8_t pdrv, vector_t v)
{
    motor_t *motor = &motor_array[pdrv];
    float size, rad;
    size = vector_length(v);
    if (size > motor->motor_pwm_max / 2.0f)
        size = motor->motor_pwm_max / 2.0f;                   // 限制幅值
    rad = atan2f(v.y, v.x);                                   // 计算电角度
    foc_motor_spwm_control_by_rad(pdrv, (uint16_t)size, rad); // 调用SPWM控制函数
}

/// @brief 通过角度矢量控制电机函数
/// @param size
/// @param angle
static void foc_motor_spwm_control_by_angle(uint8_t pdrv, float size, float angle)
{
    motor_t *motor = &motor_array[pdrv];
    float rad = angle * PI / 180.0f;
    foc_motor_spwm_control_by_rad(pdrv, size, rad);
}

// 从三相电流中获得克拉克坐标系电流矢量
static inline vector_t foc_get_current_vector(float phase_a, float phase_b, float phase_c)
{
    vector_t current;
    current.x = phase_a;
    current.y = (phase_b - phase_c) * SQRT3_2;
    return current;
}

/// @brief PID计算
/// @param pid PID参数
/// @param target 目标值
/// @param current 当前值
/// @return 控制量
static float pid_calculate(pid_param_t *pid, float target, float current)
{
    float diff;
    pid->input = current * (1 - pid->alpha) + pid->input * pid->alpha; // 低通滤波
    pid->error = target - pid->input;
    pid->i += pid->ki * pid->error;
    if (pid->i > pid->i_max / pid->scale)
        pid->i = pid->i_max / pid->scale;
    if (pid->i < pid->i_min / pid->scale)
        pid->i = pid->i_min / pid->scale;
    diff = pid->error - pid->last_error;
    pid->last_error = pid->error;
    return pid->scale * (pid->kp * pid->error + pid->i + pid->kd * diff);
}

// 校准初始角度
static void foc_base_angle_calibration(uint8_t pdrv,uint16_t angle_calibration_pwm)
{
    motor_t *motor = &motor_array[pdrv];
    foc_driver_motor_enable(pdrv,0);
    foc_motor_spwm_control_by_angle(pdrv, angle_calibration_pwm, 0);
    foc_driver_delay_ms(pdrv,100);
    foc_driver_motor_enable(pdrv,1); // 使能电机驱动

    foc_driver_delay_ms(pdrv,3000);
    foc_driver_get_mech_angle(pdrv,&motor->mech_angle_zero);
    foc_driver_delay_ms(pdrv,10);
    foc_driver_get_mech_angle(pdrv,&motor->mech_angle_zero);
}

int foc_init(uint8_t pdrv, uint8_t pole_pairs, uint16_t motor_pwm_max, float i_max, uint16_t angle_calibration_pwm)
{
    motor_t *motor = &motor_array[pdrv];
    if (pdrv >= MOTOR_COUNT)
        return -EINVAL; // 参数错误
    if (pole_pairs == 0 || motor_pwm_max == 0)
        return -EINVAL;     // 极对数和最大占空比不能为0
    motor->pole_pairs = pole_pairs;
    motor->motor_pwm_max = motor_pwm_max;
    motor->i_max = i_max;
    motor->init_already = 0;
    foc_driver_init(pdrv);
    foc_driver_set_phase(pdrv,0, 0, 0); // 设置初始占空比为0
    foc_base_angle_calibration(pdrv,angle_calibration_pwm);
    foc_set_mode(pdrv, FOC_MODE_CURRENT); // 设置默认模式为电流模式
    motor->init_already = 1;
    return 0;
}

void foc_set_mode(uint8_t pdrv, foc_mode_t mode)
{
    motor_t *motor = &motor_array[pdrv];
    if (mode >= FOC_MODE_MAX)
        return;
    motor->mode = mode;
}

void foc_current_set_pid_param(uint8_t pdrv, float scale, float iq_kp, float iq_ki, float id_kp, float id_ki)
{
    motor_t *motor = &motor_array[pdrv];
    motor->pid_iq.scale = scale;
    motor->pid_iq.kp = iq_kp;
    motor->pid_iq.ki = iq_ki;
    motor->pid_iq.i_max = 1;
    motor->pid_iq.i_min = -1;
    motor->pid_iq.i = 0;
    motor->pid_iq.alpha = 0;

    motor->pid_id.scale = scale;
    motor->pid_id.kp = id_kp;
    motor->pid_id.ki = id_ki;
    motor->pid_id.i_max = 1;
    motor->pid_id.i_min = -1;
    motor->pid_id.i = 0;
    motor->pid_id.alpha = 0;
}

void foc_current_update(uint8_t pdrv, float phase_a_current, float phase_b_current, float phase_c_current, float Filter_coefficient)
{
    motor_t *motor = &motor_array[pdrv];
    if (motor->init_already == 0)
        return;                                           // 如果没有初始化，直接返回
    foc_driver_get_mech_angle(pdrv,&motor->mech_angle); // 34%
    const float tmp = PI / 180.0f;
    motor->elec_angle_rad = (motor->mech_angle - motor->mech_angle_zero) * motor->pole_pairs * tmp;
    motor->parker_x.x = cosf(motor->elec_angle_rad); // 6%
    motor->parker_x.y = sinf(motor->elec_angle_rad); // 6%
    motor->parker_y.x = -motor->parker_x.y;
    motor->parker_y.y = motor->parker_x.x;
#ifdef USE_CURRENT_FILTER
    motor->current_phase_b_filter = motor->current_phase_b_filter * Filter_coefficient + (1 - Filter_coefficient) * phase_b_current;
    motor->current_phase_c_filter = motor->current_phase_c_filter * Filter_coefficient + (1 - Filter_coefficient) * phase_c_current;
    motor->current_phase_a_filter = -motor->current_phase_b_filter - motor->current_phase_c_filter;
    motor->current_by_clarke = foc_get_current_vector(motor->current_phase_a_filter, motor->current_phase_b_filter, motor->current_phase_c_filter); // 2.6%
#else
    motor->current_by_clarke = foc_get_current_vector(phase_a_current, phase_b_current, phase_c_current); // 2.6%
#endif
    motor->id = vector_projection(motor->parker_x, motor->current_by_clarke); // 4.5%
    motor->iq = vector_projection(motor->parker_y, motor->current_by_clarke); // 4.5%

    if (motor->target_iq > motor->i_max)
        motor->target_iq = motor->i_max;
    if (motor->target_iq < -motor->i_max)
        motor->target_iq = -motor->i_max;

    float vd_raw = pid_calculate(&motor->pid_id, 0, motor->id);                // 4.2%
    float vq_raw = pid_calculate(&motor->pid_iq, motor->target_iq, motor->iq); // 4.2%
    vector_t v_raw={vd_raw,vq_raw};
    float vector_len = vector_length(v_raw);
    //缩放到-1到1之间
    if(vector_len > 1){
        vd_raw /= vector_len;
        vq_raw /= vector_len;
    }
    else if(vector_len<-1){
        vd_raw /= -vector_len;
        vq_raw /= -vector_len;
    }
    motor->vd = vd_raw;
    motor->vq = vq_raw;

    vector_t out_voltage_by_parker=vector_add(vector_multiply(motor->parker_x, motor->vd), vector_multiply(motor->parker_y, motor->vq));

    float pwm_a, pwm_b, pwm_c;
#ifndef USE_ZERO_SEQUENCE_INJECTION 
    vector2svpwm(out_voltage_by_parker, &pwm_a, &pwm_b, &pwm_c);
#else
    vector2svpwm_2(out_voltage_by_parker, &pwm_a, &pwm_b, &pwm_c);
#endif
    motor->phase_a = (int)(pwm_a * motor->motor_pwm_max + 0.5f);
    motor->phase_b = (int)(pwm_b * motor->motor_pwm_max + 0.5f);
    motor->phase_c = (int)(pwm_c * motor->motor_pwm_max + 0.5f);
    foc_driver_set_phase(pdrv,motor->phase_a, motor->phase_b, motor->phase_c);

}
void foc_speed_set_pid_param(uint8_t pdrv, float scale, float alpha, float kp, float ki, float i_max)
{
    motor_t *motor = &motor_array[pdrv];
    motor->pid_speed.scale = scale;
    motor->pid_speed.alpha = alpha; // 低通滤波系数
    motor->pid_speed.kp = kp;
    motor->pid_speed.ki = ki;
    motor->pid_speed.kd = 0;
    motor->pid_speed.i_max = i_max;
    motor->pid_speed.i_min = -i_max;
    motor->pid_speed.i = 0;
}
/// @brief 速度环更新
/// @param motor
/// @param interval 上次调用的间隔 单位ms，频率低了会导致低速控制有停顿，经测试1khz没有问题
void foc_speed_update(uint8_t pdrv, float interval)
{
    motor_t *motor = &motor_array[pdrv];
    if (!(motor->mode == FOC_MODE_SPEED || motor->mode == FOC_MODE_POSITION))
        return;
    if (motor->init_already == 0)
        return; // 如果没有初始化，直接返回
    float delta = motor->mech_angle - motor->mech_angle_last;
    if (delta > 180.0f)
    {
        delta -= 360.0f;
    }
    else if (delta < -180.0f)
    {
        delta += 360.0f;
    }
    motor->speed = delta / interval * 1000.0f; // 速度单位度每秒
    motor->mech_angle_last = motor->mech_angle;
    // foc_driver_debug_printf(pdrv,"%.2f\n",motor->speed);
    motor->target_iq = pid_calculate(&motor->pid_speed, motor->target_speed, motor->speed);
}
/// @brief 设置位置环PID参数，三环(位置->速度->电流)需要设置kp,二环(位置->电流)需要设置kp,ki,kd
/// @param motor
/// @param scale
/// @param alpha
/// @param kp
/// @param ki
/// @param kd
/// @param imax
void foc_position_set_pid_param(uint8_t pdrv, float scale, float alpha, float kp, float ki, float kd, float imax)
{
    motor_t *motor = &motor_array[pdrv];
    motor->pid_position.scale = scale;
    motor->pid_position.alpha = alpha; // 低通滤波系数
    motor->pid_position.kp = kp;
    motor->pid_position.ki = ki;
    motor->pid_position.kd = kd;
    motor->pid_position.i_max = imax;
    motor->pid_position.i_min = -imax;
    motor->pid_position.i = 0;
}

/// @brief 位置环更新，三环(位置->速度->电流)需要调用此函数，二环(位置->电流)不需要调用
/// @param motor
void foc_position_update(uint8_t pdrv)
{
    motor_t *motor = &motor_array[pdrv];
    if (motor->mode != FOC_MODE_POSITION)
        return;
    if (motor->init_already == 0)
        return; // 如果没有初始化，直接返回
    float delta = motor->mech_angle - motor->position_last;
    if (delta > 180.0f)
    {
        delta -= 360.0f;
    }
    else if (delta < -180.0f)
    {
        delta += 360.0f;
    }
    motor->position_last = motor->mech_angle; // 更新上次位置

    motor->position += delta; // 位置单位度
    motor->target_speed = pid_calculate(&motor->pid_position, motor->target_position, motor->position);
}
/// @brief 位置环更新，二环(位置->电流)模式
/// @param motor
void foc_position_update_two(uint8_t pdrv)
{
    motor_t *motor = &motor_array[pdrv];
    if (motor->mode != FOC_MODE_POSITION_TWO)
        return;
    if (motor->init_already == 0)
        return; // 如果没有初始化，直接返回
    float delta = motor->mech_angle - motor->position_last;
    if (delta > 180.0f)
    {
        delta -= 360.0f;
    }
    else if (delta < -180.0f)
    {
        delta += 360.0f;
    }
    motor->position_last = motor->mech_angle; // 更新上次位置

    motor->position += delta; // 位置单位度
    motor->target_iq = pid_calculate(&motor->pid_position, motor->target_position, motor->position);
}

void foc_set_target(uint8_t pdrv, float target)
{
    motor_t *motor = &motor_array[pdrv];
    if (motor->mode == FOC_MODE_CURRENT)
    {
        motor->target_iq = target; // 设置电流目标值
    }
    else if (motor->mode == FOC_MODE_SPEED)
    {
        motor->target_speed = target; // 设置速度目标值
    }
    else if (motor->mode == FOC_MODE_POSITION || motor->mode == FOC_MODE_POSITION_TWO)
    {
        motor->target_position = target; // 设置位置目标值
    }
}

float foc_get_torque(uint8_t pdrv)
{
    motor_t *motor = &motor_array[pdrv];

    return motor->iq; // 扭矩= 电流 * 常数
}

float foc_get_speed(uint8_t pdrv)
{
    // TODO，待实现
    return 0.0f;
}

float foc_get_position(uint8_t pdrv)
{
    motor_t *motor = &motor_array[pdrv];
    return motor->mech_angle; // 返回角度0-360度
}

void foc_demo_0(uint8_t pdrv)
{
    motor_t *motor = &motor_array[pdrv];
    foc_driver_motor_enable(pdrv,1);
    while (1)
    {
        foc_motor_spwm_control_by_angle(pdrv, motor->motor_pwm_max, 0);
        foc_driver_delay_ms(pdrv,1000);
    }
}

void foc_demo_1(uint8_t pdrv)
{
    motor_t *motor = &motor_array[pdrv];
    foc_driver_motor_enable(pdrv,1);
    while (1)
    {
        foc_motor_spwm_control_by_angle(pdrv, motor->motor_pwm_max, 0);
        foc_driver_delay_ms(pdrv,1000);
        foc_motor_spwm_control_by_angle(pdrv, motor->motor_pwm_max, 90);
        foc_driver_delay_ms(pdrv,1000);
        foc_motor_spwm_control_by_angle(pdrv, motor->motor_pwm_max, 180);
        foc_driver_delay_ms(pdrv,1000);
        foc_motor_spwm_control_by_angle(pdrv, motor->motor_pwm_max, 270);
        foc_driver_delay_ms(pdrv,1000);
    }
}


void foc_demo_2(uint8_t pdrv,uint16_t angle_calibration_pwm)
{
    motor_t *motor = &motor_array[pdrv];
    float mech_angle = 0;
    int elec_angle = 0;
    foc_base_angle_calibration(pdrv,angle_calibration_pwm); // 校准初始角度
    foc_driver_debug_printf(pdrv,"mech_angle_zero=%.2f\n", motor->mech_angle_zero);

    while (1)
    {
        foc_driver_get_mech_angle(pdrv,&mech_angle);
        foc_driver_debug_printf(pdrv,"mech_angle=%.2f\n", mech_angle);
        elec_angle = (mech_angle - motor->mech_angle_zero) * motor->pole_pairs;
        foc_motor_spwm_control_by_angle(pdrv, motor->motor_pwm_max / 2, elec_angle + 90.0f);
    }
}

void foc_demo_31(uint8_t pdrv, float *phase_a_current, float *phase_b_current, float *phase_c_current)
{
    motor_t *motor = &motor_array[pdrv];
    foc_driver_motor_enable(pdrv,1);
    while (1)
    {
        foc_motor_spwm_control_by_angle(pdrv, motor->motor_pwm_max, 0);
        foc_driver_delay_ms(pdrv,500);
        foc_driver_debug_printf(pdrv,"A---phase_a_current=%.2f, phase_b_current=%.2f, phase_c_current=%.2f\n", *phase_a_current, *phase_b_current, *phase_c_current);
        foc_driver_delay_ms(pdrv,500);
        foc_motor_spwm_control_by_angle(pdrv, motor->motor_pwm_max, 120);
        foc_driver_delay_ms(pdrv,500);
        foc_driver_debug_printf(pdrv,"B---phase_a_current=%.2f, phase_b_current=%.2f, phase_c_current=%.2f\n", *phase_a_current, *phase_b_current, *phase_c_current);
        foc_driver_delay_ms(pdrv,500);
        foc_motor_spwm_control_by_angle(pdrv, motor->motor_pwm_max, 240);
        foc_driver_delay_ms(pdrv,500);
        foc_driver_debug_printf(pdrv,"C---phase_a_current=%.2f, phase_b_current=%.2f, phase_c_current=%.2f\n", *phase_a_current, *phase_b_current, *phase_c_current);
        foc_driver_delay_ms(pdrv,500);
    }
}

void foc_demo_32(uint8_t pdrv, uint8_t motor_en, uint16_t angle_calibration_pwm , float *phase_a_current, float *phase_b_current, float *phase_c_current, float Filter_coefficient)
{
    motor_t *motor = &motor_array[pdrv];
    float mech_angle = 0;
    int elec_angle = 0;
    foc_base_angle_calibration(pdrv,angle_calibration_pwm); // 校准初始角度
    foc_driver_motor_enable(pdrv,motor_en);
    while (1)
    {
        foc_driver_get_mech_angle(pdrv,&mech_angle);
        elec_angle = (mech_angle - motor->mech_angle_zero) * motor->pole_pairs;
        if (motor_en)
            foc_motor_spwm_control_by_angle(pdrv, motor->motor_pwm_max / 2, elec_angle + 90.0f);
#ifdef USE_CURRENT_FILTER
        motor->current_phase_b_filter = motor->current_phase_b_filter * Filter_coefficient + (1 - Filter_coefficient) * (*phase_b_current);
        motor->current_phase_c_filter = motor->current_phase_c_filter * Filter_coefficient + (1 - Filter_coefficient) * (*phase_c_current);
        motor->current_phase_a_filter = -motor->current_phase_b_filter - motor->current_phase_c_filter;
        foc_driver_debug_printf(pdrv,"%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n", *phase_a_current, *phase_b_current, *phase_c_current,
                         motor->current_phase_a_filter, motor->current_phase_b_filter, motor->current_phase_c_filter);
#else
        foc_driver_debug_printf(pdrv,"%.2f,%.2f,%.2f\n", *phase_a_current, *phase_b_current, *phase_c_current);

#endif
    }
}

void foc_demo_4(uint8_t pdrv,  uint16_t angle_calibration_pwm, float *phase_a_current, float *phase_b_current, float *phase_c_current)
{
    motor_t *motor = &motor_array[pdrv];
    float mech_angle = 0;
    float iq = 0, id = 0;
    int elec_angle = 0;
    vector_t parker_x, parker_y;
    vector_t current_vec;
    vector_t voltage_vec;
    float current_vec_angle = 0;
    foc_base_angle_calibration(pdrv,angle_calibration_pwm); // 校准初始角度

    while (1)
    {
        foc_driver_get_mech_angle(pdrv,&mech_angle);
        elec_angle = (mech_angle - motor->mech_angle_zero) * motor->pole_pairs;
        parker_x.x = cosf(elec_angle * PI / 180.0f);
        parker_x.y = sinf(elec_angle * PI / 180.0f);
        parker_y.x = -parker_x.y;
        parker_y.y = parker_x.x;

        current_vec = foc_get_current_vector(*phase_a_current, *phase_b_current, *phase_c_current);
        current_vec_angle = atan2f(current_vec.y, current_vec.x) * 180.0f / PI;
        id = vector_projection(parker_x, current_vec);
        iq = vector_projection(parker_y, current_vec);

        // 显示电角度，电流矢量角度，IQ，ID。正常情况电角度增加时电流矢量角度也增加
        //  foc_driver_debug_printf(pdrv,"%d,%.1f,%.2f,%.2f\n",elec_angle, current_vec_angle,iq, id);
        // 只显示IQ和ID。正常情况转起来是两条直线，堵转时ID=0
        foc_driver_debug_printf(pdrv,"%.2f,%.2f\n", iq, id);

        // 固定电压矢量
        voltage_vec.x = 0;
        voltage_vec.y = 15;

        foc_motor_spwm_control(pdrv, vector_add(vector_multiply(parker_x, voltage_vec.x), vector_multiply(parker_y, voltage_vec.y)));
    }
}