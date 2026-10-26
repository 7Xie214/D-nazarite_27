/**
 * 位置环串级（每轴一套）。
 *
 *   参考轨迹 ──► ⊖ ──►[位置环 PI]──┐
 *                ▲                 ⊕──► 速度环设定值 ──► [速度环 PI] ──► 电流
 *             实际位置              │
 *                        速度前馈 ──┘
 *
 * 这一层只干两件事：把位置误差变成速度设定值，再把速度环接上去。
 * PID 怎么算在 PID.c，电机怎么动在 Device 层，规划在 Traj.c。
 *
 * ---------------------------------------------------------------------------
 * 【为什么每轴一套，而不是一对全局变量】
 *
 * 之前 POSITION_LOOP_CONTROLLER / SPEED_LOOP_CONTROLLER 是全局单例。单例
 * 只能控一根轴 —— 而且和电机挂在哪条 CAN 上毫无关系。两根轴共用一个实例，
 * setpoint / integral / prev_actual 三处状态全串味：
 *
 *     A 的 setpoint 被 B 覆盖 → 两根轴互相追对方的目标
 *     两个误差累进同一个 integral → 谁都不是
 *     prev_actual 在 A、B 的反馈之间跳 → 微分通道看到巨大假跳变
 *
 * 所以控制器实例要跟着【轴】走，不是跟着【总线】走。Cascade_t 一个轴一个，
 * 挂在应用层的轴结构体里。
 *
 * ---------------------------------------------------------------------------
 * 位置环固定用 PI + 条件积分：
 *
 *   Kd = 0            位置信号经减速比和编码器量化，噪声比速度环大得多，
 *                     差出来的 D 基本都是噪声。阻尼的活速度环在干。
 *   PID_AW_COND       条件积分 + 累积限幅两步，覆盖位置环的全部抗饱和需求。
 *   不做积分分离      位置环误差量程本来就有限，没有大阶跃要躲。
 *
 * 速度环的参数全部暴露 —— 它才是决定手感的那一环。
 *
 * ---------------------------------------------------------------------------
 * 单位约定（级联里单位不统一是最难查的一类 bug）：
 *
 *   pos_ref / pos_actual    位置单位（rad / 度 / counts，全局统一即可）
 *   spd_actual / vel_ff     速度单位，必须和位置环输出同单位
 *   dt                      秒
 *   位置环输出              速度单位 —— 这就是级联接点，必须和速度环
 *                           setpoint 完全同单位
 *   spd_out_max             速度环输出量程，通常就是电机的电流命令量程
 *
 * 推荐整条链用【输出轴弧度】：位置 rad、速度 rad/s。和 DjiMotor 的
 * angle_rad / speed_rad_s 直接对齐，调参时 v_max = 3.0 rad/s 比
 * 470000 counts/s 好读得多。
 *
 * 调参顺序：先把速度环单独调好（位置环断开，直接往速度环喂设定值），
 * 再合上位置环、从纯 P 开始加。反过来做的话两环的毛病会糊在一起。
 * ---------------------------------------------------------------------------
 */

#ifndef POSITIONLOOP_H
#define POSITIONLOOP_H

#include <stdint.h>
#include <stddef.h>
#include "PID.h"

/* 一根轴的串级控制器。随轴分配，不共享。 */
typedef struct {
    PID_Controller_t pos;       /* 位置环（外环，慢） */
    PID_Controller_t spd;       /* 速度环（内环，快） */
} Cascade_t;

/* 一套参数。应用层按型号各准备一份，见 PID_Config.h。 */
typedef struct {
    /* ---- 位置环 ---- */
    float pos_kp;               /* 速度单位 / 位置单位 */
    float pos_ki;               /* 速度单位 / (位置单位 · 秒) */
    float pos_integral_max;     /* 积分累积限幅（位置单位 · 秒） */
    float pos_speed_limit;      /* 输出限幅 = 速度限幅，同时也是 anti_windup 的饱和判据 */

    /* ---- 速度环 ---- */
    float spd_kp;
    float spd_ki;
    float spd_kd;               /* 微分先行 + 一阶低通，见 PID.c */
    float spd_integral_max;
    float spd_out_max;          /* 输出量程，通常 = 电机电流命令量程 */
    float spd_d_alpha;          /* 微分低通系数 (0,1]，1 = 不滤波 */
    PID_AntiWindup_t spd_anti_windup;
    float spd_integ_sep_thr;    /* <= 0 表示不启用积分分离 */
} Cascade_Cfg_t;

/* 按 cfg 初始化（含清空全部状态）。 */
void Cascade_Init(Cascade_t *c, const Cascade_Cfg_t *cfg);

/* 复位：清积分/微分为 0，但保留增益。
 *   pos_actual / spd_actual 传当前反馈值 —— PID_Init 会把 prev_actual 清成
 *   0，复位后第一拍的微分项就是 (0 - 实际值)/dt 的巨大假跳变，速度环 Kd
 *   不为 0 时那是一记实打实的微分冲击。这里顺手把 prev_actual 设成当前值，
 *   把冲击掐掉。
 *
 * 用在：掉线恢复、模式切换、长时间没跑之后重新起步。 */
void Cascade_Reset(Cascade_t *c, float pos_actual, float spd_actual);

/* 算一拍，返回本拍的电流命令（单位 = spd_out_max 那一套）。
 *
 *   pos_ref   来自 Traj 的参考位置（不是目标位置 —— 目标已经规划过了）
 *   vel_ff    来自 Traj 的参考速度，速度前馈
 *   dt        秒
 *
 * vel_ff 的作用：巡航速度直接喂进来，不用靠积分器去"攒"。位置环的 PI
 * 只剩修残差，误差全程都很小，末端也不容易过冲。传 0 就退化成普通串级。 */
float Cascade_Update(Cascade_t *c,
                     float pos_actual,
                     float spd_actual,
                     float pos_ref,
                     float vel_ff,
                     float dt);

#endif /* POSITIONLOOP_H */
