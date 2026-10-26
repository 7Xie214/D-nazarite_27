#ifndef MY_USART_H
#define MY_USART_H

#include "main.h"

/* ============================== 可调参数 ============================== */

/* 最多同时管理几个串口实例。
 * 实例表是 .bss 里的静态数组（不消耗只有 15KB 的 FreeRTOS 堆），
 * 每实例约 800 字节，按实际需要调小。 */
#define BSP_USART_MAX_INSTANCES   4u

/* 接收环形缓冲大小。必须是 2 的幂 —— 代码用 &(SIZE-1) 做回绕寻址，
 * 避免在中断里做除法。512 字节 @115200 约等于 44ms 的缓冲深度。 */
#define BSP_USART_RX_SIZE         512u

/* 单次发送的最大长度（同时也是每实例的发送暂存缓冲大小） */
#define BSP_USART_TX_SIZE         256u

#if (BSP_USART_RX_SIZE & (BSP_USART_RX_SIZE - 1u)) != 0u
#error "BSP_USART_RX_SIZE 必须是 2 的幂"
#endif

/* ================================ API ================================= */

/* 注册一个串口实例并启动循环 DMA 接收。对同一个 handle 重复调用是幂等的。
 * 必须在 MX_USARTx_UART_Init() 之后调用。
 * 池满时返回 HAL_ERROR。 */
HAL_StatusTypeDef BSP_USART_Init(UART_HandleTypeDef *huart);

/* 阻塞发送：等这一帧真正发完才返回（或超时）。
 * data 会被拷进内部暂存缓冲，所以可以安全地传栈上的局部数组。 */
HAL_StatusTypeDef BSP_USART_Send(UART_HandleTypeDef *huart,
                                 const uint8_t *data,
                                 uint16_t len,
                                 uint32_t timeout_ms);

/* 非阻塞发送：启动 DMA 后立即返回。
 * 若该实例上一帧还没发完，返回 HAL_BUSY（同一 handle 约定单写者）。 */
HAL_StatusTypeDef BSP_USART_SendDMA(UART_HandleTypeDef *huart,
                                    const uint8_t *data,
                                    uint16_t len);

/* 阻塞接收：语义同 read()。至少读到 1 字节就返回，返回值是实际读到的字节数
 * （可能小于 len）。超时或没数据返回 0。 */
uint16_t BSP_USART_Recv(UART_HandleTypeDef *huart,
                        uint8_t *buf,
                        uint16_t len,
                        uint32_t timeout_ms);

/* 当前缓冲区里还有多少字节可读（非阻塞，随处可调） */
uint16_t BSP_USART_Available(UART_HandleTypeDef *huart);

/* 累计因缓冲区溢出而丢弃的字节数（诊断用） */
uint32_t BSP_USART_RxOverrunCount(UART_HandleTypeDef *huart);

/* 丢弃当前所有未读数据 */
HAL_StatusTypeDef BSP_USART_FlushRx(UART_HandleTypeDef *huart);

#endif
