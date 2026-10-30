/**
 * 电机应用层：把「轴」这个概念搭起来，接上 Device 层和算法层。
 *
 *   本层      一根轴 = 一台电机 + 一套串级 PID + 一个规划器 + 一个状态
 *   Device    CAN 收发、反馈解包、命令组帧（DjiMotor）
 *   algorithm 位置环串级（Cascade）、梯形规划器（Traj）、PID
 *
 * 应用层的职责就三件：
 *   1. 把电机和控制器绑成一根轴（配置在这里，不在 Device 层）
 *   2. 每拍把反馈→规划→位置环→速度环→命令 串成一条线
 *   3. 管异常：掉线怎么处理、什么时候可以起步
 *
 * ---------------------------------------------------------------------------
 * 多轴 / 多总线
 *
 * 控制器实例跟着【轴】走，不是跟着总线走。所以：
 *   - 同一时刻多根轴各自转各自的圈数 —— 各用各的 Cascade_t，互不干扰
 *   - 两台电机挂在不同 CAN 上 —— Device 层按 (hfdcan, 反馈 ID) 匹配，
 *     发的时候每条总线各发自己的 0x200 / 0x1FF / 0x2FF，本层按总线去重
 *
 * 换机构（3 个 3508 → 2 个 6020 → 别的）只改这个文件里的 AXIS_COUNT
 * 和电机清单，算法层一个字不用动。
 * ---------------------------------------------------------------------------
 */

#ifndef MOTORAPP_H
#define MOTORAPP_H

#include <stdint.h>
#include <stddef.h>
#include "main.h"
#include "DjiMotor.h"
#include "PositionLoop.h"
#include "Traj.h"

/* 上层 CAN3 的 5 根控制轴；底盘 8 台另行控制。
 * 新增轴时同步配置 MotorApp_Init 清单和 PID_Config。 */
#define AXIS_COUNT      5u

/* 到位容差：转子圈数；3508/独立轴另检查转子rpm。
 * 双2006任一侧到位即结束，分别保持当前位置，不等规划结束或稳定计时。 */
#define MOTORAPP_POSITION_TOLERANCE_ROTOR_TURNS 0.1f //0.04
#define MOTORAPP_SPEED_TOLERANCE_ROTOR_RPM     10.0f
#define MOTORAPP_SETTLE_TIME_S                 0.00f

/* 轴状态 */
typedef enum {
    AXIS_IDLE = 0,      /* 到位/待命，位置控制继续；不代表转速一定为零 */
    AXIS_MOVING,        /* 规划器在走 */
    AXIS_FAULT          /* 电机掉线，命令清零、环已复位 */
} Axis_State_t;

/* 一根轴 */
typedef struct {
    DjiMotor_t  *motor;     /* 绑定的电机（Device 层实例） */
    Cascade_t    cascade;   /* 本轴专属的位置环 + 速度环 */
    Traj_t       traj;      /* 本轴专属的规划器 */
    Axis_State_t state;
    float        settled_s; /* 实际位置/速度连续到位的时间 */
} Axis_t;

/* MotorApp_MoveTurns 的返回码 */
#define MOTORAPP_OK             0
#define MOTORAPP_ERR_PARAM     -1    /* 空指针 / 没注册 */
#define MOTORAPP_ERR_NOFB      -2    /* 还没收到过反馈，起点不可信 */
#define MOTORAPP_ERR_OFFLINE   -3    /* 电机掉线，不接受新指令 */
#define MOTORAPP_ERR_BUSY      -4    /* 三轴同步行程尚未结束 */
#define MOTORAPP_ERR_FAULT     -5    /* 双2006组故障已锁存，重新初始化后重试 */

typedef enum {
    MOTORAPP_PAIR_IDLE = 0,
    MOTORAPP_PAIR_MOVING,
    MOTORAPP_PAIR_DONE,
    MOTORAPP_PAIR_FAULT
} MotorApp_PairState_t;

typedef enum {
    MOTORAPP_PAIR_FAULT_NONE = 0,
    MOTORAPP_PAIR_FAULT_OFFLINE,
    MOTORAPP_PAIR_FAULT_SYNC,     /* 旧Watch编号保留，当前不再触发 */
    MOTORAPP_PAIR_FAULT_TIMEOUT   /* 旧Watch编号保留，当前不再触发 */
} MotorApp_PairFault_t;

/* 纯观察记录，全部时间单位为ms。事件时间相对本次请求接受时刻；
 * 事件尚未发生为UINT32_MAX，任一侧到位结束或故障后冻结到下一次请求。
 * 加速/巡航/减速按参考速度变化划分，follow_ms为参考结束后的实际追位。
 * 数组顺序统一为ID3、ID4。 */
typedef struct {
    uint32_t start_ms;           /* 请求接受时的HAL绝对毫秒 */
    uint32_t total_ms;
    uint32_t accel_ms;
    uint32_t cruise_ms;
    uint32_t decel_ms;
    uint32_t follow_ms;
    uint32_t planner_ms;         /* 参考退出时刻；实际提前到位也会提前退出 */
    uint32_t first_motion_ms[2]; /* 首次偏离起点至少0.01转子圈 */
    uint32_t reach_75_ms;        /* 两侧实际进度首次均达到75% */
    uint32_t first_in_tol_ms[2]; /* 各侧首次进入原始终点容差，未到位侧保留UINT32_MAX */
    uint32_t stopped_ms[2];      /* |反馈速度|<=10rpm且未到位的累计时间，与阶段时间重叠 */
    uint8_t phase;              /* 0未开始/1加速/2巡航/3减速/4追位/5到位/6故障 */
    uint8_t status;             /* 0未开始/1记录中/2首次到位冻结/3故障冻结 */
    uint8_t finish_motor_id;    /* 本段首先到位并触发结束的电机ID，0=尚未结束 */
} MotorApp_2006Timing_t;

typedef struct {
    MotorApp_PairState_t state;
    MotorApp_PairFault_t fault;
    float reference_turns;        /* 统一方向，从本步起点计的转子圈数 */
    float actual_turns[2];        /* 顺序 ID3、ID4，已统一方向 */
    float sync_error_turns;       /* actual[0] - actual[1] */
    float time_scale;             /* 正常运动固定1，掉线故障为0 */
    float correction_rad_s;       /* 同步纠偏速度，输出轴 rad/s */
    float speed_command_rpm[2];   /* ID3、ID4 原始电机方向的目标转子 rpm */
    float settled_ms;
    uint8_t fault_motor_id;       /* 掉线触发源ID3/ID4；其他故障为0 */
    uint32_t fault_tick_ms;       /* 故障发生的HAL时间，可与CAN错误记录对照 */
    uint32_t fault_rx_age_ms[2];  /* 故障入口快照，顺序ID3、ID4 */
    uint32_t fault_rx_count[2];
    int16_t fault_command[2];     /* 清零前的原始电流指令 */
    MotorApp_2006Timing_t timing;
} MotorApp_2006PairStatus_t;

/* 初始化：绑定上层五轴、加载各轴参数。控制任务入口调用一次。
 * main 必须先在 CAN/RTOS 启动前调用 DjiMotor_Init；本函数不重复注册。 */
void MotorApp_Init(void);

/* 算一拍。dt 单位是秒（1kHz 就传 0.001f）。
 * 内部顺序：更新全部反馈 → 逐轴控制 → 按总线组帧发出。 */
void MotorApp_Update(float dt);

/* 取第 index 根轴，越界返回 NULL。给指令层（蓝牙 / 遥控 / 上位机）用。 */
Axis_t *MotorApp_GetAxis(uint8_t index);

/* 相对当前位置转 turns 圈（【减速前转子轴】圈数，正负都行），带梯形速度规划。
 * 内部除一次减速比，反馈和控制器仍使用输出轴 rad / rad/s。
 * 中途重复调用 = 从当前位置重新起步走新目标。
 * 返回 MOTORAPP_OK 或上面的错误码。 */
int MotorApp_MoveTurns(Axis_t *axis, float turns);

/* 上层三台 3508 同步运动：turns 顺序为 CAN3 ID5、ID2、ID1，转子圈数。
 * 共用 0~1 的规划进度，rate / accel 是进度速度/加速度（1/s、1/s²）。
 * 三台均有在线反馈才一次性起步；0 圈保持原位，负数反向。
 * AXIS_IDLE 还要求实际位置、速度稳定到位，不能只看参考轨迹结束。 */
int MotorApp_Move3508TurnsSync(const float turns[3], float rate, float accel);

/* 两轮共同直行：turns 为统一方向的转子圈数，正值使 ID3负转、ID4正转。
 * 两台均就绪才一起起步；组运动和组保持期间不接受单轴 MoveTurns。
 * 一组公共轨迹、同步纠偏；任一侧进容差即结束，两台分别保持当前位置。
 * 结束时清旧积分/速度给定并关闭同步纠偏；DONE锁存到下一次请求。
 * 无跟随减速、额外末端限速、到位计时、同步偏差/运动超时停机。
 * 保留在线反馈检查；掉线故障锁存且两台电流归零。 */
int MotorApp_Move2006Pair(float turns);
const MotorApp_2006PairStatus_t *MotorApp_Get2006PairStatus(void);

#endif /* MOTORAPP_H */
