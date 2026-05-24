#include "pid.h"
#include "tim.h"
#include "math.h"
#include "gpio.h"
#include "usart.h"

// PID 参数定义
#define Kpx  0.006f
#define Kix  0.00001f
#define Kdx  0.002f

#define Kpy  0.006f
#define Kiy  0.0001f
#define Kdy  0.002f


// PID 输出对应的速度映射范围
#define SPEED_MAX   1.0f
#define SPEED_MIN   0.05f

// PSC 控制 PWM 占空比（影响电机速度）
#define PSC_MIN     100
#define PSC_MAX     1000

// 像素范围与误差阈值
#define PIXEL_MAX_X 800.0f
#define PIXEL_MAX_Y 600.0f
#define TARGET_THRESHOLD 5.0f

#define INTEGRAL_LIMIT  250.0f  // 积分限幅

#define ABS(x) ((x) > 0 ? (x) : -(x))

// 外部目标像素（由串口接收赋值）
extern int rx_target_pixel_x;
extern int rx_target_pixel_y;

// 当前像素点（建议由图像处理模块更新）
int rx_current_pixel_x = 400;
int rx_current_pixel_y = 300;

// PID 结构体
typedef struct {
    float Bias;
    float Last_bias;
    float Last2_bias;
    float output;
    float integral;
} PID_State;

// 每轴对应 PID 控制器
static PID_State pid_x = {0};
static PID_State pid_y = {0};

// 电机运行方向状态枚举
typedef enum {
    MOTOR_STOP,
    MOTOR_CW,
    MOTOR_CCW
} MotorState;

static MotorState motor_state_x = MOTOR_STOP;
static MotorState motor_state_y = MOTOR_STOP;

static float clamp(float val, float min, float max) {
    if (val > max) return max;
    if (val < min) return min;
    return val;
}

// 通用 PID 控制函数（带积分限幅）
static float PID_Control(float current, float target, float Kp, float Ki, float Kd, PID_State *pid) {
    pid->Bias = target - current;

    if (fabsf(pid->Bias) < 100.0f)  // 只有误差较小时才进行积分，避免过冲
        pid->integral += pid->Bias;
    else
        pid->integral = 0;  // 误差太大时不积分，防止积分饱和

    pid->integral = clamp(pid->integral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);

    pid->output = Kp * pid->Bias
                + Ki * pid->integral
                + Kd * (pid->Bias - 2 * pid->Last_bias + pid->Last2_bias);

    pid->Last2_bias = pid->Last_bias;
    pid->Last_bias = pid->Bias;

    return pid->output;
}

// 各轴 PID 输出
float PID_Control_x(float current, float target) {
    return PID_Control(current, target, Kpx, Kix, Kdx, &pid_x);
}

float PID_Control_y(float current, float target) {
    return PID_Control(current, target, Kpy, Kiy, Kdy, &pid_y);
}

// 将 PID 输出的速度映射为 PSC（步进频率控制）
static int PID_to_PSC(float pid_output) {
    float speed = fabsf(pid_output);
    speed = clamp(speed, SPEED_MIN, SPEED_MAX);

    float ratio = (speed - SPEED_MIN) / (SPEED_MAX - SPEED_MIN);
    int psc = PSC_MAX - (int)(ratio * (PSC_MAX - PSC_MIN));

    return clamp(psc, PSC_MIN, PSC_MAX);
}

// PID 控制主循环（由定时器中断驱动调用）
void PID_ControlLoop_X(void) {
    static uint8_t tim8_started = 0;

    float error_x = rx_target_pixel_x - rx_current_pixel_x;

    // 如果接近目标，则停止电机
    if (fabsf(error_x) <= TARGET_THRESHOLD) {
        HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_1);
        tim8_started = 0;
        return;
    }

    float pid_output_x = PID_Control_x(rx_current_pixel_x, rx_target_pixel_x);
    int direction_x = (pid_output_x > 0) ? 1 : -1;
    int psc_x = PID_to_PSC(pid_output_x);

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, (direction_x > 0) ? GPIO_PIN_RESET : GPIO_PIN_SET);
    __HAL_TIM_SET_PRESCALER(&htim8, psc_x - 1);

    if (!tim8_started) {
        HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
        tim8_started = 1;
    }
}

void PID_ControlLoop_Y(void) {
    static uint8_t tim3_started = 0;

    float error_y = rx_target_pixel_y - rx_current_pixel_y;

    if (fabsf(error_y) <= TARGET_THRESHOLD) {
        HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
        tim3_started = 0;
        return;
    }

    float pid_output_y = PID_Control_y(rx_current_pixel_y, rx_target_pixel_y);
    int direction_y = (pid_output_y > 0) ? 1 : -1;
    int psc_y = PID_to_PSC(pid_output_y);

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, (direction_y > 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    __HAL_TIM_SET_PRESCALER(&htim3, psc_y - 1);

    if (!tim3_started) {
        HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
        tim3_started = 1;
    }
}

