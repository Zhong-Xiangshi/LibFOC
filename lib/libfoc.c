#include "libfoc.h"
#include "math.h"
#include "libfoc_interface.h"
#include "errno.h"
#include "stdio.h"

#define PI 3.1415f
#define SQRT3 1.732f
#define SQRT3_2 0.866f // sqrt(3)/2
#define INV_SQRT3 0.5773f    // 1/sqrt(3)

#define MOTOR_COUNT 1
// #define USE_CURRENT_FILTER
#define USE_ZERO_SEQUENCE_INJECTION //Zero-Sequence Component Injection

struct vector
{
    float x;
    float y;
};
typedef struct vector vector_t;



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
    float motor_pwm_max;
    // 控制模式
    foc_mode_t mode;

    // 三相PWM占空比
    float phase_a, phase_b, phase_c;

    // 机械角度,初始机械角度,上次机械角度
    float mech_angle, mech_angle_zero, mech_angle_last;
    // 电角度（单位弧度）
    float elec_angle_rad;

    // 电流环
    float target_iq;
    float iq, id;
    float vq, vd;
    pid_t pid_iq, pid_id;
    float current_phase_a_filter;
    float current_phase_b_filter;
    float current_phase_c_filter;
    float phase_a_current;
    float phase_b_current;
    float phase_c_current;
    float i_max; // 电流最大值限制
    vector_t parker_x, parker_y;
    vector_t current_by_clarke; // 克拉克坐标系电流矢量

    // 速度环
    float target_speed;
    float speed;
    float speed_LPF_alpha;
    pid_t pid_speed;

    // 位置环
    float target_position;
    float position_last; // 上次位置
    float position;
    pid_t pid_position;
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

/// @brief 开环SPWM控制电机函数
/// @param size 输出百分比 0-1
/// @param rad 单位：弧度 (a相的当前电角度)
static void foc_motor_spwm_control_by_rad(uint8_t pdrv, float size, float rad)
{
    motor_t *motor = &motor_array[pdrv];
    float phase_a_pwm, phase_b_pwm, phase_c_pwm;
    float pwm_mid = motor->motor_pwm_max / 2;


    // 限制幅值，确保输出在范围内
    if (size > 1)
    {
        size = 1;
    }

    // 计算三相瞬时值 (相对于pwm_mid的偏移)
    float val_a = pwm_mid * size * cosf(rad);
    float val_b = pwm_mid * size * cosf(rad - 2.0f * PI / 3.0f);
    float val_c = pwm_mid * size * cosf(rad - 4.0f * PI / 3.0f);

    // 转换为PWM占空比值并进行四舍五入
    phase_a_pwm = val_a + pwm_mid;
    phase_b_pwm = val_b + pwm_mid;
    phase_c_pwm = val_c + pwm_mid;

    // PWM占空比饱和处理
    if (phase_a_pwm < 0)
        phase_a_pwm = 0;
    if (phase_b_pwm < 0)
        phase_b_pwm = 0;
    if (phase_c_pwm < 0)
        phase_c_pwm = 0;
    if (phase_a_pwm > motor->motor_pwm_max)
        phase_a_pwm = motor->motor_pwm_max;
    if (phase_b_pwm > motor->motor_pwm_max)
        phase_b_pwm = motor->motor_pwm_max;
    if (phase_c_pwm > motor->motor_pwm_max)
        phase_c_pwm = motor->motor_pwm_max;

    // foc_driver_debug_printf(pdrv,"PWM OUTPUT angle=%.1f,phase_a=%d, phase_b=%d, phase_c=%d\n", rad*180.0f/PI,phase_a_pwm, phase_b_pwm, phase_c_pwm);
    foc_driver_set_phase(pdrv,phase_a_pwm, phase_b_pwm, phase_c_pwm);
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
/// @brief spwm开环控制电机
/// @param pdrv 电机编号
/// @param v 模长0-1
static void foc_motor_spwm_control(uint8_t pdrv, vector_t v)
{
    float size, rad;
    size = vector_length(v);
    rad = atan2f(v.y, v.x);                                   // 计算电角度
    foc_motor_spwm_control_by_rad(pdrv, size, rad); // 调用SPWM控制函数
}

/// @brief spwm开环控制电机
/// @param size 0-1
/// @param angle
static void foc_motor_spwm_control_by_angle(uint8_t pdrv, float size, float angle)
{
    float rad = angle * PI / 180.0f;
    foc_motor_spwm_control_by_rad(pdrv, size, rad);
}

// 从三相电流中获得克拉克坐标系电流矢量
static inline vector_t foc_get_current_vector(float phase_a, float phase_b, float phase_c)
{
    vector_t current;
    // 采用幅值不变的Clarke变换
    current.x = phase_a;
    // I_beta = (I_b - I_c) / sqrt(3)
    current.y = (phase_b - phase_c) * INV_SQRT3; 
    return current;
}

/// @brief PID计算
/// @param pid PID参数
/// @param target 目标值
/// @param current 当前值
/// @return 控制量
float pid_calculate(pid_t *pid, float target, float current)
{
    float error, p_term, i_term, d_term, output;

    // 1. 低通滤波 (可选，针对输入有噪声的情况)
    // 注意：如果测量值current非常干净，可以去掉这一步
    pid->_input = current * (1.0f - pid->alpha) + pid->_input * pid->alpha;

    // 2. 计算误差
    error = target - pid->_input;

    // 3. 计算 P 项
    p_term = pid->kp * error;

    // 4. 计算 I 项
    pid->_integral += pid->ki * error;

    // === 积分抗饱和 (Integral Anti-Windup) ===
    // 限制积分项不无限增长。
    // 这里的 max_i_term 通常设置为最大输出的 30% ~ 100%
    if (pid->_integral > pid->max_i_term) {
        pid->_integral = pid->max_i_term;
    } else if (pid->_integral < -pid->max_i_term) {
        pid->_integral = -pid->max_i_term;
    }
    i_term = pid->_integral;

    // 5. 计算 D 项
    d_term = pid->kd * (error - pid->_last_error);
    pid->_last_error = error;

    // 6. 总输出计算 (去掉 scale，直接相加)
    output = p_term + i_term + d_term;

    // === 输出限幅 (Output Saturation) ===
    // 限制最终给执行器的指令，防止越界
    if (output > pid->max_out) {
        output = pid->max_out;
    } else if (output < -pid->max_out) {
        output = -pid->max_out;
    }

    return output;
}

// 校准初始角度
static void foc_base_angle_calibration(uint8_t pdrv,float angle_calibration_pwm)
{
    motor_t *motor = &motor_array[pdrv];
    foc_driver_motor_enable(pdrv,0);
    foc_motor_spwm_control_by_angle(pdrv, angle_calibration_pwm, 0);
    foc_driver_delay_ms(pdrv,100);
    foc_driver_motor_enable(pdrv,1); // 使能电机驱动

    foc_driver_delay_ms(pdrv,1000);
    foc_driver_get_mech_angle(pdrv,&motor->mech_angle_zero);
    foc_driver_delay_ms(pdrv,10);
    foc_driver_get_mech_angle(pdrv,&motor->mech_angle_zero);
}
motor_t *motor0;
int foc_init(uint8_t pdrv, uint8_t pole_pairs, float motor_pwm_max, float i_max, float angle_calibration_pwm)
{
    motor_t *motor = &motor_array[pdrv];
    motor0=motor;
    if (pdrv >= MOTOR_COUNT)
        return -EINVAL; // 参数错误
    if (pole_pairs == 0 || motor_pwm_max <= 0)
        return -EINVAL;     // 极对数和最大占空比错误
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

foc_mode_t foc_get_mode(uint8_t pdrv){
    motor_t *motor = &motor_array[pdrv];
    return motor->mode;
}

void foc_current_set_pid_param(uint8_t pdrv,pid_t current)
{
    motor_t *motor = &motor_array[pdrv];
    motor->pid_iq = current;
    motor->pid_id = current;
}

void foc_current_update(uint8_t pdrv,float Filter_coefficient)
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

    foc_driver_get_phase_current(pdrv,&motor->phase_a_current,&motor->phase_b_current,&motor->phase_c_current);
#ifdef USE_CURRENT_FILTER
    motor->current_phase_b_filter = motor->current_phase_b_filter * Filter_coefficient + (1 - Filter_coefficient) * motor->phase_b_current;
    motor->current_phase_c_filter = motor->current_phase_c_filter * Filter_coefficient + (1 - Filter_coefficient) * motor->phase_c_current;
    motor->current_phase_a_filter = -motor->current_phase_b_filter - motor->current_phase_c_filter;
    motor->current_by_clarke = foc_get_current_vector(motor->current_phase_a_filter, motor->current_phase_b_filter, motor->current_phase_c_filter); // 2.6%
#else
    motor->current_by_clarke = foc_get_current_vector(motor->phase_a_current, motor->phase_b_current, motor->phase_c_current); // 2.6%
#endif
    motor->id = vector_projection(motor->parker_x, motor->current_by_clarke); // 4.5%
    motor->iq = vector_projection(motor->parker_y, motor->current_by_clarke); // 4.5%

    if(motor->mode!=FOC_MODE_VOLTAGE){
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
        motor->vd = vd_raw;
        motor->vq = vq_raw;
    }

    vector_t out_voltage_by_parker=vector_add(vector_multiply(motor->parker_x, motor->vd), vector_multiply(motor->parker_y, motor->vq));

    float pwm_a, pwm_b, pwm_c;
#ifndef USE_ZERO_SEQUENCE_INJECTION 
    vector2svpwm(out_voltage_by_parker, &pwm_a, &pwm_b, &pwm_c);
#else
    vector2svpwm_2(out_voltage_by_parker, &pwm_a, &pwm_b, &pwm_c);
#endif
    motor->phase_a = pwm_a * motor->motor_pwm_max;
    motor->phase_b = pwm_b * motor->motor_pwm_max;
    motor->phase_c = pwm_c * motor->motor_pwm_max;
    foc_driver_set_phase(pdrv,motor->phase_a, motor->phase_b, motor->phase_c);

}
void foc_speed_set_pid_param(uint8_t pdrv, float LPF_alpha, pid_t speed)
{
    motor_t *motor = &motor_array[pdrv];
    motor->pid_speed = speed;
    motor->speed_LPF_alpha=LPF_alpha;
}
/// @brief 速度环更新
/// @param motor
/// @param interval 上次调用的间隔 单位us，频率低了会导致低速控制有停顿，经测试4khz没有问题
void foc_speed_update(uint8_t pdrv, uint32_t interval_us)
{
    motor_t *motor = &motor_array[pdrv];
    
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
    float raw_speed=delta / interval_us*1000000; // 速度单位度每秒
    motor->speed = raw_speed * motor->speed_LPF_alpha + (1 - motor->speed_LPF_alpha) * motor->speed;
    motor->mech_angle_last = motor->mech_angle;
    // foc_driver_debug_printf(pdrv,"%.2f\n",motor->speed);
    if (!((motor->mode == FOC_MODE_SPEED) || (motor->mode == FOC_MODE_POSITION)))
        return;
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
void foc_position_set_pid_param(uint8_t pdrv, pid_t position)
{
    motor_t *motor = &motor_array[pdrv];
    motor->pid_position = position;
}

/// @brief 位置环更新，三环(位置->速度->电流)需要调用此函数，二环(位置->电流)不需要调用
/// @param motor
void foc_position_update(uint8_t pdrv)
{
    motor_t *motor = &motor_array[pdrv];
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
    if (motor->mode != FOC_MODE_POSITION)
        return;
    motor->target_speed = pid_calculate(&motor->pid_position, motor->target_position, motor->position);
}
/// @brief 位置环更新，二环(位置->电流)模式
/// @param motor
void foc_position_update_two(uint8_t pdrv)
{
    motor_t *motor = &motor_array[pdrv];
    
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
    if (motor->mode != FOC_MODE_POSITION_TWO)
        return;
    motor->target_iq = pid_calculate(&motor->pid_position, motor->target_position, motor->position);
}

void foc_set_target_iq(uint8_t pdrv, float target)
{
    motor_t *motor = &motor_array[pdrv];
    motor->target_iq=target;
}
float foc_get_target_iq(uint8_t pdrv)
{
    motor_t *motor = &motor_array[pdrv];
    return motor->target_iq;
}
void foc_set_target_velocity(uint8_t pdrv, float target)
{
    motor_t *motor = &motor_array[pdrv];
    motor->target_speed=target;
}
float foc_get_target_velocity(uint8_t pdrv)
{
    motor_t *motor = &motor_array[pdrv];
    return motor->target_speed;
}
void foc_set_target_position(uint8_t pdrv, float target)
{
    motor_t *motor = &motor_array[pdrv];
    motor->target_position=target;
}
float foc_get_target_position(uint8_t pdrv)
{
    motor_t *motor = &motor_array[pdrv];
    return motor->target_position;
}
float foc_get_mech_angle(uint8_t pdrv){
    motor_t *motor = &motor_array[pdrv];

    return motor->mech_angle;
}


float foc_get_torque(uint8_t pdrv)
{
    motor_t *motor = &motor_array[pdrv];

    return motor->iq; // 扭矩= 电流 * 常数
}

float foc_get_velocity(uint8_t pdrv)
{
    motor_t *motor = &motor_array[pdrv];
    return motor->speed;
}

float foc_get_position(uint8_t pdrv)
{
    motor_t *motor = &motor_array[pdrv];
    return motor->position;
}

void foc_get_iq_id(uint8_t pdrv, float *iq, float *id){
    motor_t *motor = &motor_array[pdrv];
    *iq = motor->iq;
    *id = motor->id;
}
void foc_get_vq_vd(uint8_t pdrv, float *vq, float *vd){
    motor_t *motor = &motor_array[pdrv];
    *vq=motor->vq;
    *vd=motor->vd;
}
void foc_get_phase_current(uint8_t pdrv, float *cur_a, float *cur_b, float *cur_c){
    motor_t *motor = &motor_array[pdrv];

    *cur_a=motor->phase_a_current;
    *cur_b=motor->phase_b_current;
    *cur_c=motor->phase_c_current;
}

void foc_demo_0(uint8_t pdrv)
{
    motor_t *motor = &motor_array[pdrv];
    foc_driver_motor_enable(pdrv,1);
    while (1)
    {
        foc_motor_spwm_control_by_angle(pdrv, 1, 0);
        foc_driver_delay_ms(pdrv,1000);
    }
}

void foc_demo_1(uint8_t pdrv)
{
    foc_driver_motor_enable(pdrv,1);
    while (1)
    {
        foc_motor_spwm_control_by_angle(pdrv, 1, 0);
        foc_driver_delay_ms(pdrv,1000);
        foc_motor_spwm_control_by_angle(pdrv, 1, 90);
        foc_driver_delay_ms(pdrv,1000);
        foc_motor_spwm_control_by_angle(pdrv, 1, 180);
        foc_driver_delay_ms(pdrv,1000);
        foc_motor_spwm_control_by_angle(pdrv, 1, 270);
        foc_driver_delay_ms(pdrv,1000);
    }
}


void foc_demo_2(uint8_t pdrv)
{
    motor_t *motor = &motor_array[pdrv];
    float mech_angle = 0;
    int elec_angle = 0;
    foc_base_angle_calibration(pdrv,1); // 校准初始角度
    foc_driver_debug_printf(pdrv,"mech_angle_zero=%.2f\n", motor->mech_angle_zero);

    while (1)
    {
        foc_driver_get_mech_angle(pdrv,&mech_angle);
        foc_driver_debug_printf(pdrv,"mech_angle=%.2f\n", mech_angle);
        elec_angle = (mech_angle - motor->mech_angle_zero) * motor->pole_pairs;
        foc_motor_spwm_control_by_angle(pdrv, 1, elec_angle + 90.0f);
        foc_driver_delay_ms(pdrv,10);
    }
}

void foc_demo_31(uint8_t pdrv)
{
    motor_t *motor = &motor_array[pdrv];
    foc_driver_motor_enable(pdrv,1);
    while (1)
    {
        foc_motor_spwm_control_by_angle(pdrv, 1, 0);
        foc_driver_delay_ms(pdrv,500);
        foc_driver_get_phase_current(pdrv,&motor->phase_a_current,&motor->phase_b_current,&motor->phase_c_current);
        foc_driver_debug_printf(pdrv,"A---phase_a_current=%.2f, phase_b_current=%.2f, phase_c_current=%.2f\n", motor->phase_a_current, motor->phase_b_current, motor->phase_c_current);
        foc_driver_delay_ms(pdrv,500);

        foc_motor_spwm_control_by_angle(pdrv, 1, 120);
        foc_driver_delay_ms(pdrv,500);
        foc_driver_get_phase_current(pdrv,&motor->phase_a_current,&motor->phase_b_current,&motor->phase_c_current);
        foc_driver_debug_printf(pdrv,"B---phase_a_current=%.2f, phase_b_current=%.2f, phase_c_current=%.2f\n",  motor->phase_a_current, motor->phase_b_current, motor->phase_c_current);
        foc_driver_delay_ms(pdrv,500);

        foc_motor_spwm_control_by_angle(pdrv, 1, 240);
        foc_driver_delay_ms(pdrv,500);
        foc_driver_get_phase_current(pdrv,&motor->phase_a_current,&motor->phase_b_current,&motor->phase_c_current);
        foc_driver_debug_printf(pdrv,"C---phase_a_current=%.2f, phase_b_current=%.2f, phase_c_current=%.2f\n",  motor->phase_a_current, motor->phase_b_current, motor->phase_c_current);
        foc_driver_delay_ms(pdrv,500);
    }
}

void foc_demo_32(uint8_t pdrv, uint8_t motor_en)
{
    motor_t *motor = &motor_array[pdrv];
    float mech_angle = 0;
    int elec_angle = 0;
    foc_base_angle_calibration(pdrv,1); // 校准初始角度
    foc_driver_motor_enable(pdrv,motor_en);
    while (1)
    {
        foc_driver_get_mech_angle(pdrv,&mech_angle);
        elec_angle = (mech_angle - motor->mech_angle_zero) * motor->pole_pairs;
        if (motor_en)
            foc_motor_spwm_control_by_angle(pdrv, 1, elec_angle + 90.0f);
        foc_driver_get_phase_current(pdrv,&motor->phase_a_current,&motor->phase_b_current,&motor->phase_c_current);
        foc_driver_debug_printf(pdrv,"%.2f,%.2f,%.2f\n", motor->phase_a_current, motor->phase_b_current, motor->phase_c_current);

    }
}

void foc_demo_4(uint8_t pdrv)
{
    motor_t *motor = &motor_array[pdrv];
    float mech_angle = 0;
    float iq = 0, id = 0;
    int elec_angle = 0;
    vector_t parker_x, parker_y;
    vector_t current_vec;
    vector_t voltage_vec;
    float current_vec_angle = 0;
    foc_base_angle_calibration(pdrv,1); // 校准初始角度

    while (1)
    {
        foc_driver_get_mech_angle(pdrv,&mech_angle);
        elec_angle = (mech_angle - motor->mech_angle_zero) * motor->pole_pairs;
        parker_x.x = cosf(elec_angle * PI / 180.0f);
        parker_x.y = sinf(elec_angle * PI / 180.0f);
        parker_y.x = -parker_x.y;
        parker_y.y = parker_x.x;

        foc_driver_get_phase_current(pdrv,&motor->phase_a_current,&motor->phase_b_current,&motor->phase_c_current);
        current_vec = foc_get_current_vector(motor->phase_a_current, motor->phase_b_current, motor->phase_c_current);
        current_vec_angle = atan2f(current_vec.y, current_vec.x) * 180.0f / PI;
        id = vector_projection(parker_x, current_vec);
        iq = vector_projection(parker_y, current_vec);

        // 显示电角度，电流矢量角度，IQ，ID。正常情况电角度增加时电流矢量角度也增加
        //  foc_driver_debug_printf(pdrv,"%d,%.1f,%.2f,%.2f\n",elec_angle, current_vec_angle,iq, id);
        // 只显示IQ和ID。正常情况转起来是两条直线，堵转时ID=0
        foc_driver_debug_printf(pdrv,"%.2f,%.2f\n", iq, id);

        // 固定电压矢量
        voltage_vec.x = 0;
        voltage_vec.y = 1;

        foc_motor_spwm_control(pdrv, vector_add(vector_multiply(parker_x, voltage_vec.x), vector_multiply(parker_y, voltage_vec.y)));
    }
}
void foc_demo_5(uint8_t pdrv){
    foc_driver_motor_enable(0,1);
    foc_driver_set_phase(0,0,0.5,0);
    while(1);
}