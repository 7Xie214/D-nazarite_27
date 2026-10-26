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
 *   - 两台电机挂在不同 CAN 上 —— Device 层按 (hfdcan, id) 建索引，
 *     发的时候每条总线各发自己的 0x200 / 0x1FF，本层自动按总线去重
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

/* 本机轴数。改机构就改这个 + MotorApp_Init() 里的清单。 */
#define AXIS_COUNT      5u

/* 轴状态 */
typedef enum {
    AXIS_IDLE = 0,      /* 停着，位置环保持当前位置 */
    AXIS_MOVING,        /* 规划器在走 */
    AXIS_FAULT          /* 电机掉线，命令清零、环已复位 */
} Axis_State_t;

/* 一根轴 */
typedef struct {
    DjiMotor_t  *motor;     /* 绑定的电机（Device 层实例） */
    Cascade_t    cascade;   /* 本轴专属的位置环 + 速度环 */
    Traj_t       traj;      /* 本轴专属的规划器 */
    Axis_State_t state;
} Axis_t;

/* MotorApp_MoveTurns 的返回码 */
#define MOTORAPP_OK             0
#define MOTORAPP_ERR_PARAM     -1    /* 空指针 / 没注册 */
#define MOTORAPP_ERR_NOFB      -2    /* 还没收到过反馈，起点不可信 */
#define MOTORAPP_ERR_OFFLINE   -3    /* 电机掉线，不接受新指令 */

/* 初始化：注册电机、绑轴、按型号套参数。开跑之前调一次。 */
void MotorApp_Init(void);

/* 算一拍。dt 单位是秒（1kHz 就传 0.001f）。
 * 内部顺序：更新全部反馈 → 逐轴控制 → 按总线组帧发出。 */
void MotorApp_Update(float dt);

/* 取第 index 根轴，越界返回 NULL。给指令层（蓝牙 / 遥控 / 上位机）用。 */
Axis_t *MotorApp_GetAxis(uint8_t index);

/* 相对当前位置转 turns 圈（【输出轴】圈数，正负都行），带梯形速度规划。
 * 中途重复调用 = 从当前位置重新起步走新目标。
 * 返回 MOTORAPP_OK 或上面的错误码。 */
int MotorApp_MoveTurns(Axis_t *axis, float turns);

#endif /* MOTORAPP_H */
