/**
 * 位置式 PID，微分先行 + 抗积分饱和
 *
 * 两个对外接口：
 *   PID_Update_Continuous(pid, actual, dt)  —— 连续式，显式传采样周期
 *   PID_Update_Discrete  (pid, actual)      —— 离散式，dt 视为 1
 *
 * 两者数学上完全等价（dt=1 时 /1.0f 和 *1.0f 都是精确的），只是增益的
 * 归一化方式不同。同一个控制器实例只能用其中一个：
 *
 *                     连续式              离散式（Ts 已折进增益）
 *   Ki_cont = Ki_disc * Ts          Ki_disc = Ki_cont / Ts
 *   Kd_cont = Kd_disc / Ts          Kd_disc = Kd_cont * Ts
 *   integral_max 单位 = 转速·秒      integral_max 单位 = 转速·拍
 *
 * 例：1kHz 速度环，先按连续式调出 Ki_cont、Kd_cont，要用离散式时再换算。
 */

#include "PID.h"


/* .h 里是 extern，定义放这里 */
PID_Controller_t SPEED_LOOP_CONTROLLER;

/* 避免依赖 math.h（C90 下没有 fabsf），顺带省一次库调用 */
static float pid_absf(float x)
{
    return (x < 0.0f) ? -x : x;
}

/* ==================== 积分策略：独立函数 ==================== */

/* 1) 普通积分：累积 + 限幅。下面三个策略最终都复用它 */
static void pid_integrate_plain(PID_Controller_t *pid, float error, float dt)
{
    pid->integral += error * dt;

    if (pid->integral >  pid->integral_max) pid->integral =  pid->integral_max;
    else if (pid->integral < -pid->integral_max) pid->integral = -pid->integral_max;
}

/* 2) 条件积分：输出已饱和，且误差还在往饱和的同方向推 —— 冻结积分。
      误差反向时（out_raw 与 error 反号）允许累积，这样积分能帮输出退出饱和。 */
static void pid_integrate_cond(PID_Controller_t *pid, float error, float dt,
                               float out_raw)
{
    if ((out_raw >  pid->out_max) && (error > 0.0f)) return;
    if ((out_raw < -pid->out_max) && (error < 0.0f)) return;

    pid_integrate_plain(pid, error, dt);
}

/* 3) 积分分离：误差太大时不积分，防止大阶跃期间积分堆起来造成超调。
      阈值 <= 0 视为不启用。 */
static void pid_integrate_sep(PID_Controller_t *pid, float error, float dt)
{
    if ((pid->integ_sep_thr > 0.0f) && (pid_absf(error) > pid->integ_sep_thr)) return;

    pid_integrate_plain(pid, error, dt);
}

/* 4) 分发 */
static void pid_integrate(PID_Controller_t *pid, float error, float dt, float out_raw)
{
    switch (pid->anti_windup) {
    case PID_AW_COND:
        pid_integrate_cond(pid, error, dt, out_raw);
        break;

    case PID_AW_SEP:
        pid_integrate_sep(pid, error, dt);
        break;

    case PID_AW_BOTH:
        if ((pid->integ_sep_thr > 0.0f) &&
            (pid_absf(error) > pid->integ_sep_thr)) break;   /* 先过分离阈值 */
        pid_integrate_cond(pid, error, dt, out_raw);         /* 再过条件积分 */
        break;

    case PID_AW_NONE:
    default:
        pid_integrate_plain(pid, error, dt);
        break;
    }
}

/* ==================== 公共核心 ==================== */

static float pid_core(PID_Controller_t *pid, float actual, float dt)
{
    float error, d_raw, P, D, i_term, out_raw, out;

    error = pid->setpoint - actual;

    /* --- 微分先行：对反馈量 actual 求导（取负号），再做一阶低通（不完全微分）。
           设定值跳变时 dy/dt 是连续的，所以不产生微分冲击。 --- */
    d_raw = -(actual - pid->prev_actual) / dt;
    pid->prev_actual = actual;
    pid->d_filt += pid->d_alpha * (d_raw - pid->d_filt);

    P = pid->Kp * error;
    D = pid->Kd * pid->d_filt;

    /* --- 先用「当前积分」估一个未限幅输出，条件积分要靠它判断饱和。
           这一拍用的是上一拍累积完的积分，存在一拍延迟，属正常做法。 --- */
    i_term  = pid->Ki * pid->integral;
    out_raw = P + i_term + D;

    /* --- 积分策略 --- */
    pid_integrate(pid, error, dt, out_raw);
    i_term = pid->Ki * pid->integral;      /* 策略动过积分，重算 */

    /* --- 输出限幅 --- */
    out = P + i_term + D;
    if (out >  pid->out_max) out =  pid->out_max;
    if (out < -pid->out_max) out = -pid->out_max;

    return out;
}

/* ==================== 对外接口 ==================== */

float PID_Update_Continuous(PID_Controller_t *pid, float actual, float dt)
{
    return pid_core(pid, actual, dt);
}

float PID_Update_Discrete(PID_Controller_t *pid, float actual)
{
    return pid_core(pid, actual, 1.0f);
}

/* ==================== 初始化 ==================== */

void PID_Init(PID_Controller_t *pid,
              float Kp, float Ki, float Kd,
              float integral_max, float out_max, float d_alpha,
              PID_AntiWindup_t anti_windup, float integ_sep_thr)
{
    pid->setpoint      = 0.0f;
    pid->Kp            = Kp;
    pid->Ki            = Ki;
    pid->Kd            = Kd;
    pid->integral      = 0.0f;
    pid->integral_max  = integral_max;
    pid->out_max       = out_max;
    pid->d_filt        = 0.0f;
    pid->d_alpha       = d_alpha;      /* 建议 0.1 ~ 0.5，1.0 = 不滤波 */
    pid->prev_actual   = 0.0f;
    pid->anti_windup   = anti_windup;
    pid->integ_sep_thr = integ_sep_thr;
}