#ifndef MY_TASK_H
#define MY_TASK_H

#include <stdint.h>
#include "FreeRTOS.h"       /* configTICK_RATE_HZ */
#include "cmsis_os2.h"      /* osKernelGetTickCount / osWaitForever */
#include "MotorApp.h"

/* Keil Watch：上电为 0，改为 1 放行一次上楼流程。
 * 等待期间仍更新反馈和位置保持；启动后改回 0 不会暂停或重新起步。 */
extern volatile uint8_t g_upstairs_start;

typedef struct {
    uint8_t motor_id;       /* CAN3 电调 ID */
    uint8_t state;          /* 0=到位保持，1=运动未完成，2=反馈故障 */
    uint8_t planner_active;
    int16_t command;        /* 原始电流给定 */
    int16_t speed_rpm;      /* 转子 rpm */
    int16_t feedback_current; /* 电调反馈电流原始值 */
    int32_t total_counts;   /* 累计转子编码器计数 */
    float error_turns;      /* 目标-实际，减速前转子圈数 */
    float settled_ms;       /* 实际位置/速度连续满足到位条件的时间 */
    uint32_t rx_count;
} Upstairs_AxisDebug_t;

/* 第二步观察记录：[0/1/2]=ID5/ID2/ID1。分类按规划->位置->转速->稳定计时
 * 的优先级互斥累计；done_ms相对进入第二步，0表示尚未完成。完成后保留。 */
typedef struct {
    uint32_t planner_ms;
    uint32_t position_ms;
    uint32_t speed_ms;
    uint32_t settle_ms;
    uint32_t done_ms;
    float last_error_turns;
    int16_t last_speed_rpm;
    int16_t last_command;
    uint8_t motor_id;
    uint8_t last_reason; /* 最后阻塞原因：0无/1规划/2位置/3转速/4稳定计时/5故障 */
} Upstairs_Step2Debug_t;

typedef struct {
    uint32_t updates;       /* 持续增加表示任务仍在更新观察数据 */
    uint8_t state;          /* 0/1/2=第一/二/三台阶，3=流程故障 */
    uint8_t step;           /* 当前台阶步骤，从 1 开始 */
    uint8_t issued;
    uint8_t started;
    Upstairs_AxisDebug_t axis[5]; /* CAN3 ID5、ID2、ID1、ID4(2006)、ID3(2006) */
    int32_t move_2006_result[2]; /* 保留旧Watch项；两项均记录本次组请求结果 */
    MotorApp_2006PairStatus_t pair_2006;
    uint8_t id5_half_started; /* ID5请求已接受：两侧75%触发，或2006一侧先到位时补发 */
    int32_t move_id5_result;  /* 1=未尝试，0=接受，负数=MotorApp错误码 */
    uint8_t first_stair_done; /* 2006追加行程也按任一侧到位结束后为1；ID5可仍独立运动 */
    struct {
        uint32_t step1_start_ms; /* 放行第一步时的HAL毫秒计数，不含启动前待机 */
        uint32_t step2_start_ms; /* 进入第二步时的HAL毫秒计数 */
        uint32_t step2_end_ms;   /* 第二步结束时的HAL毫秒计数 */
        uint32_t step4_start_ms; /* 进入第四步时的HAL毫秒计数 */
        uint32_t step4_end_ms;   /* 第四步结束时的HAL毫秒计数；完成前无效 */
        uint32_t step1_ms;       /* 第一步耗时，含请求重试/到位等待 */
        uint32_t step2_ms;       /* 第二步耗时，含请求重试/到位等待 */
        uint32_t step3_ms;       /* 第二步结束至进入第四步，含请求重试/等待 */
        uint32_t step4_ms;       /* 进入第四步至结束，含请求重试/等待 */
        uint32_t elapsed_ms;     /* 第二步结束至第四步结束；运行中实时累计 */
        uint32_t total_ms;       /* 第一至第四步总耗时；运行中实时累计 */
        uint32_t step3_after_pair_ms; /* 2006组结束至第四步，可能包含补发ID5的请求重试 */
        uint8_t status;         /* 0=未开始，1=计时中，2=完成锁定，3=故障冻结 */
    } timing;
    MotorApp_2006Timing_t pair_timing[2]; /* [0]第三步，[1]第四步，分别保留 */
    Upstairs_Step2Debug_t step2_axis[3];
    struct {
        uint32_t step1_start_ms; /* 进入第二级第一步的HAL时间 */
        uint32_t step1_ms;       /* 含请求重试和到位等待，完成/故障后冻结 */
        uint32_t step2_start_ms;
        uint32_t step3_start_ms;
        uint32_t step2_ms;
        uint32_t step3_ms;
        uint32_t total_ms;       /* 进入第二级至第二级结束 */
        int32_t move_result;     /* 1未尝试，0接受，负数见MotorApp返回码 */
        int32_t id5_move_result;
        uint8_t id5_started;
        uint8_t status;          /* 0未开始/1等待或运动中/2第二级完成/3故障 */
        MotorApp_2006Timing_t pair_timing[2]; /* 本级第二/三步，第一层记录另存 */
    } second_stair;
} Upstairs_Debug_t;

/* Keil Watch 展开此全局观察副本即可；修改它不改变控制器和任务状态。 */
extern volatile Upstairs_Debug_t g_upstairs_debug;

void upstairs_task(void);

#endif /* MY_TASK_H */
