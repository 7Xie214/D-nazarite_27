#ifndef _PID_H
#define _PID_H

#include <stdint.h>

/* 抗积分饱和策略 */
typedef enum {
    PID_AW_NONE = 0,   /* 仅积分限幅 */
    PID_AW_COND,       /* 条件积分：输出饱和且误差同向时冻结积分 */
    PID_AW_SEP,        /* 积分分离：|误差| 超阈值时冻结积分 */
    PID_AW_BOTH        /* 两者叠加 */
} PID_AntiWindup_t;

typedef struct {
    float setpoint;      /* 目标值 */
    float Kp;            /* 比例系数 */
    float Ki;            /* 积分系数（输出端才乘，积分器存原始累积量） */
    float Kd;            /* 微分系数 */

    float integral;      /* 积分累加量：连续式存 Σ(e·dt)，离散式存 Σe */
    float integral_max;  /* 累积量限幅（注意单位随上面那行变化） */
    float out_max;       /* 输出限幅 */

    float d_filt;        /* 微分低通状态 */
    float d_alpha;       /* 微分滤波系数 (0,1]，1 = 不滤波 */
    float prev_actual;   /* 上一次反馈值（微分先行用） */

    PID_AntiWindup_t anti_windup;  /* 抗积分饱和策略 */
    float integ_sep_thr;           /* 积分分离阈值，<= 0 表示不启用 */
} PID_Controller_t;

extern PID_Controller_t SPEED_LOOP_CONTROLLER;

void PID_Init(PID_Controller_t *pid,
              float Kp, float Ki, float Kd,
              float integral_max, float out_max, float d_alpha,
              PID_AntiWindup_t anti_windup, float integ_sep_thr);

/* 连续式写法：显式传入采样周期 dt。Ki/Kd 是物理增益，换采样率不用重调参数。
   例：1kHz 速度环 -> PID_Update_Continuous(&pid, speed, 0.001f) */
float PID_Update_Continuous(PID_Controller_t *pid, float actual, float dt);

/* 离散式写法：采样周期已折进 Ki/Kd，dt 视为 1。固定周期 ISR 里省一次传参。
   两者数学上等价，但增益换算关系不同，详见 PID.c 顶部注释。
   不要拿同一个实例交替调用这两个函数。 */
float PID_Update_Discrete(PID_Controller_t *pid, float actual);

#endif // _PID_H
