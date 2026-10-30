#ifndef MY_CAN_H
#define MY_CAN_H

#include "main.h"

/* 一条标准帧接收滤波器
 * 匹配规则：(收到的ID & mask) == (id & mask)
 *   mask = 0x7FF → 精确匹配单个 ID
 *   mask = 0x000 → 接收全部标准帧
 *   mask = 0x7F8 → 匹配 0x200~0x207（位与，不是范围） */
typedef struct {
    uint16_t id;
    uint16_t mask;
} CAN_Filter_t;

/* CAN1/2/3 的故障记录。有效 LEC 为 1~6，0/7 不覆盖历史错误。 */
typedef struct {
    uint32_t first_error_code;
    uint32_t last_error_code;
    uint32_t bus_off_count;
    uint32_t last_psr;
    uint32_t last_ecr;
    uint32_t error_record_count;  /* 有效错误快照次数，不等同于坏帧数 */
    uint32_t last_error_tick_ms;  /* 最后有效错误快照的HAL时间 */
} BSP_CAN_Diagnostics_t;

extern volatile BSP_CAN_Diagnostics_t g_can_diagnostics[3];

/* 初始化：配置全局滤波器 → 写入滤波器列表 → 开中断 → 启动
 * filters/count 为 0 时表示不装任何滤波器（此时全局滤波器会拒收全部） */
HAL_StatusTypeDef BSP_CAN_Init(FDCAN_HandleTypeDef *hfdcan,
                               const CAN_Filter_t *filters,
                               uint8_t count);

/* 发送一帧标准数据帧，len 取 0~8。发送队列满时返回 HAL_ERROR */
HAL_StatusTypeDef BSP_CAN_Send(FDCAN_HandleTypeDef *hfdcan,
                               uint16_t id,
                               const uint8_t *data,
                               uint8_t len);

/* 接收回调：应用层自己在某个 .c 里重新实现即可覆盖（BSP 里是 __weak 空实现）
 * 在中断上下文里被调用，别在里面做耗时操作 */
void BSP_CAN_OnReceive(FDCAN_HandleTypeDef *hfdcan,
                       uint16_t id,
                       const uint8_t *data,
                       uint8_t len);

#endif
