#include "libfoc.h"
#include "math.h"
#include "libfoc_interface.h"
#include "errno.h"
#include "stdio.h"

#define PI 3.1415f
#define SQRT3 1.732f
#define SQRT3_2 0.866f // sqrt(3)/2

/// @brief 向量相加
/// @param a 
/// @param b 
/// @return 
static inline vector vector_add(vector a, vector b){
    vector c;
    c.x = a.x + b.x;
    c.y = a.y + b.y;
    return c;
}

/// @brief 向量数乘
/// @param a 
/// @param b 
/// @return 
static inline vector vector_multiply(vector a, float b){
    vector c;
    c.x = a.x * b;
    c.y = a.y * b;
    return c;
}

/// @brief 向量点乘
/// @param a 
/// @param b 
/// @return 
static inline float vector_dot(vector a, vector b){
    return a.x * b.x + a.y * b.y;
}

/// @brief 向量长度
/// @param a 
/// @return 
static inline float vector_length(vector a){
    return sqrtf(vector_dot(a, a));
}


/// @brief b在a方向上的投影长度
/// @param a 
/// @param b 
/// @return 
static inline float vector_projection(vector a, vector b){
    return vector_dot(a, b) / vector_length(a);
}

/// @brief 向量旋转
/// @param a 
/// @param rad 
/// @return 
static inline vector vector_rotate(vector a, float rad){
    vector b;
    b.x = a.x * cosf(rad) - a.y * sinf(rad);
    b.y = a.x * sinf(rad) + a.y * cosf(rad);
    return b;
}


/// @brief 六步换向法控制电机函数
/// @param step 
static void foc_motor_set_step(motor *motor,uint8_t step){
    switch (step)
    {
        case 1:
            motor->driver.foc_motor_set_phase(motor->motor_pwm_max, 0, 0);
            break;
        case 2:
            motor->driver.foc_motor_set_phase(motor->motor_pwm_max, motor->motor_pwm_max, 0);
            break;
        case 3:
            motor->driver.foc_motor_set_phase(0, motor->motor_pwm_max, 0);
            break;
        case 4:
            motor->driver.foc_motor_set_phase(0, motor->motor_pwm_max, motor->motor_pwm_max);
            break;
        case 5:
            motor->driver.foc_motor_set_phase(0, 0, motor->motor_pwm_max);
            break;
        case 6:
            motor->driver.foc_motor_set_phase(motor->motor_pwm_max, 0, motor->motor_pwm_max);
            break;
        default:
            motor->driver.foc_motor_set_phase(0, 0, 0);
            break;
    }
}

/// @brief SPWM控制电机函数
/// @param size 单位：pwm值 (峰值，相对于pwm_mid的偏移)
/// @param rad 单位：弧度 (a相的当前电角度)
static void foc_motor_spwm_control_by_rad(motor *motor, float size, float rad)
{
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
    if (phase_a_pwm < 0) phase_a_pwm = 0;
    if (phase_b_pwm < 0) phase_b_pwm = 0;
    if (phase_c_pwm < 0) phase_c_pwm = 0;
    if (phase_a_pwm > pwm_max) phase_a_pwm = pwm_max;
    if (phase_b_pwm > pwm_max) phase_b_pwm = pwm_max;
    if (phase_c_pwm > pwm_max) phase_c_pwm = pwm_max;

    // foc_debug_printf("PWM OUTPUT angle=%.1f,phase_a=%d, phase_b=%d, phase_c=%d\n", rad*180.0f/PI,phase_a_pwm, phase_b_pwm, phase_c_pwm);
    motor->driver.foc_motor_set_phase( (uint16_t)phase_a_pwm, (uint16_t)phase_b_pwm, (uint16_t)phase_c_pwm);
}

/// @brief SVPWM控制电机函数，使用最小/最大值注入零序分量的调制方法
/// @param motor 
/// @param v valpha,vbeta
static void foc_motor_svpwm_control(motor *motor,vector v)
{
    float phase_a_tmp, phase_b_tmp, phase_c_tmp;
    float pwm_mid= motor->motor_pwm_max / 2.0f; // PWM中点
    v.x*=1.15f;
    v.y*=1.15f;
    phase_a_tmp = v.x;
    float tmp=SQRT3_2 * v.y;
    phase_b_tmp = -0.5f * v.x + tmp;
    phase_c_tmp = -0.5f * v.x - tmp;
    float max_phase = fmaxf(fmaxf(phase_a_tmp, phase_b_tmp), phase_c_tmp);
    float min_phase = fminf(fminf(phase_a_tmp, phase_b_tmp), phase_c_tmp);
    float mid_offset_phase =(max_phase+min_phase)/2.0f;
    motor->phase_a = (int)(phase_a_tmp - mid_offset_phase + pwm_mid +0.5f);
    motor->phase_b = (int)(phase_b_tmp - mid_offset_phase + pwm_mid +0.5f);
    motor->phase_c = (int)(phase_c_tmp - mid_offset_phase + pwm_mid +0.5f);

    if(motor->phase_a < 0) motor->phase_a = 0;
    if(motor->phase_b < 0) motor->phase_b = 0;
    if(motor->phase_c < 0) motor->phase_c = 0;
    if(motor->phase_a > motor->motor_pwm_max) motor->phase_a = motor->motor_pwm_max;
    if(motor->phase_b > motor->motor_pwm_max) motor->phase_b = motor->motor_pwm_max;
    if(motor->phase_c > motor->motor_pwm_max) motor->phase_c = motor->motor_pwm_max;
    // foc_debug_printf("PWM OUTPUT rad=%.1f,size=%.1f,mid_offset_phase=%.1f,phase_a=%d, phase_b=%d, phase_c=%d\n", rad*180.0f/3.1415f,size,mid_offset_phase,motor->phase_a, motor->phase_b, motor->phase_c);
    motor->driver.foc_motor_set_phase(motor->phase_a, motor->phase_b, motor->phase_c);
}
static void foc_motor_spwm_control(motor *motor,vector v)
{
    float size,rad;
    size = vector_length(v);
    if(size> motor->motor_pwm_max / 2.0f) size = motor->motor_pwm_max / 2.0f; // 限制幅值
    rad = atan2f(v.y, v.x); // 计算电角度
    foc_motor_spwm_control_by_rad(motor, (uint16_t)size, rad); // 调用SPWM控制函数
}

/// @brief 通过角度矢量控制电机函数
/// @param size 
/// @param angle 
static void foc_motor_spwm_control_by_angle(motor *motor,float size,float angle)
{
    float rad = angle * PI / 180.0f; 
    foc_motor_spwm_control_by_rad(motor,size, rad);
}

//从三相电流中获得克拉克坐标系电流矢量
static inline vector foc_get_current_vector(float phase_a, float phase_b, float phase_c){
    vector current;
    current.x = phase_a;
    current.y = (phase_b-phase_c)*SQRT3_2;
    return current;
}

/// @brief PID计算
/// @param pid PID参数
/// @param target 目标值
/// @param current 当前值
/// @return 控制量
static float pid_calculate(pid_param *pid, float target, float current){
    float diff;
    pid->input =current*(1-pid->alpha)+ pid->input*pid->alpha; // 低通滤波
    pid->error = target-pid->input;
    pid->i += pid->ki * pid->error;
    if (pid->i > pid->i_max/pid->scale) pid->i = pid->i_max/pid->scale;
    if (pid->i < pid->i_min/pid->scale) pid->i = pid->i_min/pid->scale;
    diff = pid->error - pid->last_error;
    pid->last_error = pid->error;
    return pid->scale*(pid->kp * pid->error + pid->i+pid->kd * diff);
}

//校准初始角度
static void foc_base_angle_calibration(motor *motor){
    motor->driver.foc_motor_enable(1); // 使能电机驱动
    foc_motor_spwm_control_by_angle(motor,motor->motor_pwm_max, 0);
    foc_delay_ms(1000);
    motor->driver.foc_get_mech_angle(&motor->mech_angle_zero);
    foc_delay_ms(10);
    motor->driver.foc_get_mech_angle(&motor->mech_angle_zero);
}

int foc_init(motor *motor,foc_interface_t driver,uint8_t pole_pairs,uint16_t motor_pwm_max){
    if(motor==NULL) return -EINVAL; // 参数错误
    if(pole_pairs==0 || motor_pwm_max==0) return -EINVAL; // 极对数和最大占空比不能为0
    motor->driver = driver; // 设置驱动函数接口
    motor->pole_pairs=pole_pairs;
    motor->motor_pwm_max=motor_pwm_max;
    motor->driver.foc_driver_init();
    motor->driver.foc_motor_set_phase(0, 0, 0); // 设置初始占空比为0
    foc_base_angle_calibration(motor);
    foc_set_mode(motor,FOC_MODE_CURRENT); // 设置默认模式为电流模式
    motor->init_already=1;
    return 0;
}

void foc_set_mode(motor *motor,foc_mode mode){
    if(mode >= FOC_MODE_MAX) return;
    motor->mode = mode;
}

void foc_current_set_pid_param(motor *motor,float scale,float iq_kp,float iq_ki,float id_kp,float id_ki){
    motor->pid_iq.scale=scale;
    motor->pid_iq.kp=iq_kp;
    motor->pid_iq.ki=iq_ki;
    motor->pid_iq.i_max=motor->motor_pwm_max;
    motor->pid_iq.i_min=-motor->motor_pwm_max;
    motor->pid_iq.i=0;
    motor->pid_iq.alpha=0;

    motor->pid_id.scale=scale;
    motor->pid_id.kp=id_kp;
    motor->pid_id.ki=id_ki;
    motor->pid_id.i_max=motor->motor_pwm_max;
    motor->pid_id.i_min=-motor->motor_pwm_max;
    motor->pid_id.i=0;
    motor->pid_id.alpha=0;
}

void foc_current_update(motor *motor,float phase_a_current,float phase_b_current,float phase_c_current){

    if(motor->init_already==0) return; // 如果没有初始化，直接返回
    motor->driver.foc_get_mech_angle(&motor->mech_angle);//34%
    const float tmp=PI/180.0f;
    motor->elec_angle_rad = (motor->mech_angle - motor->mech_angle_zero)*motor->pole_pairs*tmp;
    motor->parker_x.x = cosf(motor->elec_angle_rad);//6%
    motor->parker_x.y = sinf(motor->elec_angle_rad);//6%
    motor->parker_y.x = -motor->parker_x.y;
    motor->parker_y.y = motor->parker_x.x;

    motor->current_by_clarke=foc_get_current_vector(phase_a_current, phase_b_current, phase_c_current);//2.6%

    // float alpha=0.5f;
    // motor->id = motor->id*alpha + (1-alpha)*vector_projection(motor->parker_x, motor->current_by_clarke);
    // motor->iq = motor->iq*alpha + (1-alpha)*vector_projection(motor->parker_y, motor->current_by_clarke);
    motor->id = vector_projection(motor->parker_x, motor->current_by_clarke);//4.5%
    motor->iq = vector_projection(motor->parker_y, motor->current_by_clarke);//4.5%

    // static int print_count=0;
    // print_count++;
    // if(print_count>500){  //控制打印频率，频率高很影响速度
    //     print_count=0;
    //     foc_debug_printf("%.2f,%.2f,%.2f,%.2f\n", motor->iq, motor->id,motor->vq,motor->vd);
    // }
    
    motor->vd = pid_calculate(&motor->pid_id, 0, motor->id);//4.2%
    motor->vq = pid_calculate(&motor->pid_iq, motor->target_iq, motor->iq);//4.2%

    foc_motor_svpwm_control(motor,vector_add(vector_multiply(motor->parker_x,motor->vd),vector_multiply(motor->parker_y,motor->vq)));//25%

    
}
void foc_speed_set_pid_param(motor *motor,float scale,float alpha,float kp,float ki,float i_max){
    motor->pid_speed.scale=scale;
    motor->pid_speed.alpha=alpha; // 低通滤波系数
    motor->pid_speed.kp=kp;
    motor->pid_speed.ki=ki;
    motor->pid_speed.kd=0;
    motor->pid_speed.i_max=i_max;
    motor->pid_speed.i_min=-i_max;
    motor->pid_speed.i=0;
}
/// @brief 速度环更新
/// @param motor 
/// @param interval 上次调用的间隔 单位ms，频率低了会导致低速控制有停顿，经测试1khz没有问题
void foc_speed_update(motor *motor,float interval){
    if(!(motor->mode == FOC_MODE_SPEED || motor->mode == FOC_MODE_POSITION)) return;
    if(motor->init_already==0) return; // 如果没有初始化，直接返回
    float delta = motor->mech_angle - motor->mech_angle_last;
    if (delta > 180.0f) {
        delta -= 360.0f;
    } else if (delta < -180.0f) {
        delta += 360.0f;
    }
    motor->speed = delta/interval*1000.0f; // 速度单位度每秒
    motor->mech_angle_last = motor->mech_angle;
    // foc_debug_printf("%.2f\n",motor->speed);          
    motor->target_iq=pid_calculate(&motor->pid_speed, motor->target_speed, motor->speed);
}
/// @brief 设置位置环PID参数，三环(位置->速度->电流)需要设置kp,二环(位置->电流)需要设置kp,ki,kd
/// @param motor 
/// @param scale 
/// @param alpha 
/// @param kp 
/// @param ki 
/// @param kd 
/// @param imax 
void foc_position_set_pid_param(motor *motor,float scale,float alpha,float kp,float ki,float kd,float imax){
    motor->pid_position.scale=scale;
    motor->pid_position.alpha=alpha; // 低通滤波系数
    motor->pid_position.kp=kp;
    motor->pid_position.ki=ki;
    motor->pid_position.kd=kd;
    motor->pid_position.i_max=imax;
    motor->pid_position.i_min=-imax;
    motor->pid_position.i=0;
}

/// @brief 位置环更新，三环(位置->速度->电流)需要调用此函数，二环(位置->电流)不需要调用
/// @param motor 
void foc_position_update(motor *motor){
    if(motor->mode != FOC_MODE_POSITION) return;
    if(motor->init_already==0) return; // 如果没有初始化，直接返回
    float delta = motor->mech_angle - motor->position_last;
    if (delta > 180.0f) {
        delta -= 360.0f;
    } else if (delta < -180.0f) {
        delta += 360.0f;
    }
    motor->position_last = motor->mech_angle; // 更新上次位置

    motor->position += delta; // 位置单位度
    motor->target_speed = pid_calculate(&motor->pid_position, motor->target_position, motor->position);
}
/// @brief 位置环更新，二环(位置->电流)模式
/// @param motor 
void foc_position_update_two(motor *motor){
    if(motor->mode != FOC_MODE_POSITION_TWO) return;
    if(motor->init_already==0) return; // 如果没有初始化，直接返回
    float delta = motor->mech_angle - motor->position_last;
    if (delta > 180.0f) {
        delta -= 360.0f;
    } else if (delta < -180.0f) {
        delta += 360.0f;
    }
    motor->position_last = motor->mech_angle; // 更新上次位置

    motor->position += delta; // 位置单位度
    motor->target_iq = pid_calculate(&motor->pid_position, motor->target_position, motor->position);
}

void foc_set_target(motor *motor,float target){
    if(motor->mode == FOC_MODE_CURRENT){
        motor->target_iq = target; // 设置电流目标值
    }else if(motor->mode == FOC_MODE_SPEED){
        motor->target_speed = target; // 设置速度目标值
    }else if(motor->mode == FOC_MODE_POSITION || motor->mode == FOC_MODE_POSITION_TWO){
        motor->target_position = target; // 设置位置目标值
    }
}

float foc_get_torque(motor *motor)
{

    return motor->iq; // 扭矩= 电流 * 常数
}

float foc_get_speed(motor *motor)
{
    //TODO，待实现
    return 0.0f;
}

float foc_get_position(motor *motor)
{
    return motor->mech_angle; // 返回角度0-360度
}

void foc_demo_0(motor *motor){
    motor->driver.foc_motor_enable(1);
    while (1)
    {
        foc_motor_spwm_control_by_angle(motor,motor->motor_pwm_max, 0);
        foc_delay_ms(1000);
    }
}

void foc_demo_1(motor *motor){
    motor->driver.foc_motor_enable(1);
    while (1)
    {
        foc_motor_spwm_control_by_angle(motor,motor->motor_pwm_max, 0);
        foc_delay_ms(1000);
        foc_motor_spwm_control_by_angle(motor,motor->motor_pwm_max, 90);
        foc_delay_ms(1000);
        foc_motor_spwm_control_by_angle(motor,motor->motor_pwm_max, 180);
        foc_delay_ms(1000);
        foc_motor_spwm_control_by_angle(motor,motor->motor_pwm_max, 270);
        foc_delay_ms(1000);
    }
    
}

float mech_angle=0;
void foc_demo_2(motor *motor){
    int elec_angle=0;
    foc_base_angle_calibration(motor); // 校准初始角度
    foc_debug_printf("mech_angle_zero=%.2f\n",motor->mech_angle_zero);


    while (1)
    {
        motor->driver.foc_get_mech_angle(&mech_angle);
        foc_debug_printf("mech_angle=%.2f\n",mech_angle);
        elec_angle = (mech_angle - motor->mech_angle_zero)*motor->pole_pairs;
        foc_motor_spwm_control_by_angle(motor,motor->motor_pwm_max/2, elec_angle+90.0f);
    }
    
}


void foc_demo_31(motor *motor,float *phase_a_current,float *phase_b_current,float *phase_c_current){

    motor->driver.foc_motor_enable(1);
    while (1)
    {
        foc_motor_spwm_control_by_angle(motor,motor->motor_pwm_max, 0);
        foc_delay_ms(500);
        foc_debug_printf("A---phase_a_current=%.2f, phase_b_current=%.2f, phase_c_current=%.2f\n", *phase_a_current, *phase_b_current, *phase_c_current);
        foc_delay_ms(500);
        foc_motor_spwm_control_by_angle(motor,motor->motor_pwm_max, 120);
        foc_delay_ms(500);
        foc_debug_printf("B---phase_a_current=%.2f, phase_b_current=%.2f, phase_c_current=%.2f\n", *phase_a_current, *phase_b_current, *phase_c_current);
        foc_delay_ms(500);
        foc_motor_spwm_control_by_angle(motor,motor->motor_pwm_max, 240);
        foc_delay_ms(500);
        foc_debug_printf("C---phase_a_current=%.2f, phase_b_current=%.2f, phase_c_current=%.2f\n", *phase_a_current, *phase_b_current, *phase_c_current);
        foc_delay_ms(500);
    }
    
}


void foc_demo_32(motor *motor,uint8_t motor_en,float *phase_a_current,float *phase_b_current,float *phase_c_current){
    float mech_angle=0;
    int elec_angle=0;
    foc_base_angle_calibration(motor); // 校准初始角度
    motor->driver.foc_motor_enable(motor_en);
    while (1)
    {
        motor->driver.foc_get_mech_angle(&mech_angle);
        elec_angle = (mech_angle - motor->mech_angle_zero)*motor->pole_pairs;
        if(motor_en)foc_motor_spwm_control_by_angle(motor,motor->motor_pwm_max/2, elec_angle+90.0f);
        foc_debug_printf("%.2f,%.2f,%.2f\n", *phase_a_current, *phase_b_current, *phase_c_current);
    }
    
}

void foc_demo_4(motor *motor,float *phase_a_current,float *phase_b_current,float *phase_c_current){
    float mech_angle=0;
    float iq=0,id=0;
    int elec_angle=0;
    vector parker_x,parker_y;
    vector current_vec;
    vector voltage_vec;
    float current_vec_angle=0;
    foc_base_angle_calibration(motor); // 校准初始角度

    while (1)
    {
        motor->driver.foc_get_mech_angle(&mech_angle);
        elec_angle = (mech_angle - motor->mech_angle_zero)*motor->pole_pairs;
        parker_x.x = cosf(elec_angle*PI/180.0f);
        parker_x.y = sinf(elec_angle*PI/180.0f);
        parker_y.x = -parker_x.y;
        parker_y.y = parker_x.x;

        current_vec=foc_get_current_vector(*phase_a_current, *phase_b_current, *phase_c_current);
        current_vec_angle = atan2f(current_vec.y, current_vec.x)*180.0f/PI;
        id = vector_projection(parker_x, current_vec);
        iq = vector_projection(parker_y, current_vec);

        //显示电角度，电流矢量角度，IQ，ID。正常情况电角度增加时电流矢量角度也增加
        // foc_debug_printf("%d,%.1f,%.2f,%.2f\n",elec_angle, current_vec_angle,iq, id);
        //只显示IQ和ID。正常情况转起来是两条直线，堵转时ID=0
        foc_debug_printf("%.2f,%.2f\n",iq, id);

        //固定电压矢量
        voltage_vec.x = 0;
        voltage_vec.y = 15;
        
        foc_motor_spwm_control(motor,vector_add(vector_multiply(parker_x,voltage_vec.x),vector_multiply(parker_y,voltage_vec.y)));
    }
    
}