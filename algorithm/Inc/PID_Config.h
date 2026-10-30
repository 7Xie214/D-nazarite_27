/**
 * 控制参数集中配置（PID + 轨迹规划）。
 *
 * 所有控制环的增益和限幅都写在这一个文件里 —— 调参只翻这里，
 * 不用在各个 .c 里找。实例本身不在这个文件里定义（控制器实例跟着轴走，
 * 见 APP 层的 Axis_t），这里只负责给它们填参数。
 *
 * 每台电机一份（不是每个型号一份）：五根轴装在机构不同位置，负载、惯量、
 * 摩擦都不同，调成一样等于让它们互相将就。将来加 6020 也是再加几份。
 */

#ifndef PID_CONFIG_H
#define PID_CONFIG_H

#include <stddef.h>
#include "main.h"
#include "PID.h"
#include "PositionLoop.h"
#include "Traj.h"

typedef struct {
    Cascade_Cfg_t cascade;      /* 位置环 + 速度环参数 */
    float         traj_v_max;   /* 巡航速度上限（位置单位/秒），必须 > 0 */
    float         traj_a_max;   /* 加速度上限（位置单位/秒²），必须 > 0 */
} Axis_Cfg_t;

typedef struct {
    float traj_v_max;              /* 公共输出轴 rad/s */
    float traj_a_max;              /* 公共输出轴 rad/s² */
    float sync_kp;                 /* 同步位置差到速度，1/s */
    float sync_speed_max;          /* 仅限制同步纠偏速度，不限制末端总速度 */
    float command_a_max;           /* 最终两轮速度命令斜率，rad/s² */
} MotorPair_Cfg_t;

extern const MotorPair_Cfg_t MOTORPAIR_CFG_2006;

/* Drive feedback/setpoint use m/s; steering uses rad and rad/s. */
typedef struct {
    float kp;
    float ki;
    float kd;
    float integral_max;
    float out_max;
    float d_alpha;
    PID_AntiWindup_t anti_windup;
    float integ_sep_thr;
} Chassis_DrivePID_Cfg_t;

typedef struct {
    Cascade_Cfg_t steer;
    Chassis_DrivePID_Cfg_t drive;
} Chassis_PID_Cfg_t;

/* Independent FL, FR, RL, RR parameters; tune each wheel separately. */
extern const Chassis_PID_Cfg_t CHASSIS_PID_CFG[4];
/* Independent FL/FR drive-position cascades; output rad, rad/s, rad/s^2. */
extern const Axis_Cfg_t CHASSIS_FRONT_MOVE_CFG[2];

/* 每根电机一份 —— 装在机构不同位置、负载不同，各调各的。
 * 名字里的编号对应 DjiMotor 实例编号，不代表电调硬件 ID；
 * 具体哪根轴用哪份，在 MotorApp.c 的轴表里绑。
 *
 * ⚠️ spd_out_max 是唯一跟【型号】走、不跟轴走的字段：
 *    3508/C620 = 16384，2006/C610 = 10000。改参数时别动它。 */
extern const Axis_Cfg_t AXIS_CFG_3508_1;    /* M3508 + C620，CAN3 ID5，反馈 0x205 */
extern const Axis_Cfg_t AXIS_CFG_3508_2;    /* M3508 + C620，反馈 0x202 */
extern const Axis_Cfg_t AXIS_CFG_3508_3;    /* M3508 + C620，CAN3 ID1，反馈 0x201 */

extern const Axis_Cfg_t AXIS_CFG_2006_1;    /* M2006 + C610，反馈 0x204 */
extern const Axis_Cfg_t AXIS_CFG_2006_2;    /* M2006 + C610，CAN3 ID3，反馈 0x203 */

/* 把一份参数应用到一根轴的两个控制器和规划器上。
 * 调用时机：只该在开跑之前调一次 —— 它会清空积分和微分状态，
 * 运行中重复调等于把环打回起点。 */
void PID_Config_Apply(Cascade_t *c, Traj_t *t, const Axis_Cfg_t *cfg);

#endif /* PID_CONFIG_H */
