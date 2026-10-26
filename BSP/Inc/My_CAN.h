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
