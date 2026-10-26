/**
 * 整机任务入口。现在只有电机控制一条主线。
 */

#include "My_task.h"
#include "MotorApp.h"

/* 控制周期 1ms（1kHz）。
 *
 * 这个频率不是随便定的 —— DjiMotor_Update 的角度回绕判据（±4096 半圈判决）
 * 要求两次调用之间转子转过不到半圈。3508 满转约 9000rpm（转子侧）≈ 150
 * 转/秒，半圈 4096 counts 只要 3.3ms 就过去了 —— 低于 300Hz 就会把整圈
 * 判成反向半圈，total_counts 直接飞掉。1kHz 留了三倍余量。
 *
 * 速度环是内环，也本来就需要这个带宽。位置环和轨迹规划跟着一起跑，是因为
 * 前馈 vel_ref 必须和控制器严格同拍，降频会让前馈失配，反而不稳。
 *
 * dt 用实测的 tick 差算，不写死 0.001f：万一某拍被抢占拖长了，传真实 dt
 * 数学上仍然是对的（写死会让轨迹和微分项都偏）。 */

 typedef enum{
    frist_stairs=0,
    second_stairs,
    third_stairs
 }up_state;

/* 一根轴是不是已经走完、停在位置上了。
 *
 * ⚠️ 不能用 MotorApp_MoveTurns 的返回值当判据：它返回 MOTORAPP_OK 只说明
 *    "指令收下了"，那一刻电机一动还没动。同一次行程里，"收下"和"走完"
 *    差了整整一段路 —— 拿返回值判完成，第一拍就会全部放行。
 *
 * 只认 AXIS_IDLE：掉线（AXIS_FAULT）不算完成，队伍会停住不再往上走。
 * 这是故意的 —— 带着一台死电机继续爬台阶，比停下来安全。 */
static uint8_t axis_done(uint8_t idx)
{
    Axis_t *ax = MotorApp_GetAxis(idx);

    if (ax == NULL) {
        return 0u;                  /* 轴号越界 —— 永远不会"完成" */
    }
    return (uint8_t)(ax->state == AXIS_IDLE);
}

//51.151971203 中3508的最大转速mm/s.
//34.1013141357 外3508的最大转速mm/s

void upstairs_task(void)
{
    uint32_t last;
    uint32_t now;
    float    dt;
    up_state state  = frist_stairs;
    uint8_t  group  = 0u;       /* frist_stairs 走到第几组：0 = 轴0/1，1 = 轴3/4 */
    uint8_t  issued = 0u;       /* 本组指令发过没有 —— 边沿触发用 */

    MotorApp_Init();

    last = osKernelGetTickCount();

    for (;;) {
        osDelay(1u);

        now = osKernelGetTickCount();
        dt  = (float)(now - last) / (float)configTICK_RATE_HZ;
        last = now;

        /* 理论上 osDelay(1) 至少过一个 tick，dt 不会为 0；
         * 兜一下是因为 PID 里要除以 dt，真拿到 0 就是 NaN 进去。 */
        if (dt < 0.0001f) {
            dt = 0.0001f;
        }

        switch (state) {
            case frist_stairs: {
                uint8_t state_3508   = 1;
                uint8_t state_2006_1 = 1;
                uint8_t state_2006_2 = 1;
                uint8_t state_2006_3 = 1;

                if (issued == 0u) {
                    /* ---- 下发本组指令：组内并行 ---- */
                    if (group == 0u) {
                        state_3508   = MotorApp_MoveTurns(MotorApp_GetAxis(0), 22.9218927f);    //这里算的有问题，这里的圈数是还有除以减速比的
                        state_2006_1 = MotorApp_MoveTurns(MotorApp_GetAxis(1), 34.382839f);
                        if ((state_3508   == MOTORAPP_OK) &&
                            (state_2006_1 == MOTORAPP_OK)) {
                            issued = 1u;            /* 收下了，转去等它走完 */
                        }
                    } else {
                        state_2006_2 = MotorApp_MoveTurns(MotorApp_GetAxis(3), 85.94366926f);
                        state_2006_3 = MotorApp_MoveTurns(MotorApp_GetAxis(4), 85.94366926f);
                        if ((state_2006_2 == MOTORAPP_OK) &&
                            (state_2006_3 == MOTORAPP_OK)) {
                            issued = 1u;
                        }
                    }
                } else {
                    /* ---- 等本组走完，走完才允许进下一组 ---- */
                    if (group == 0u) {
                        if ((axis_done(0u) != 0u) && (axis_done(1u) != 0u)) {
                            group  = 1u;
                            issued = 0u;
                        }
                    } else {
                        if ((axis_done(3u) != 0u) && (axis_done(4u) != 0u)) {
                            state  = second_stairs;
                            issued = 0u;
                        }
                    }
                }

                MotorApp_Update(dt);
                break;
            }

            case second_stairs: {
                uint8_t state_3508   = 1;
                uint8_t state_2006_1 = 1;
                uint8_t state_2006_2 = 1;
                uint8_t state_2006_3 = 1;
                if(issued==0){
                    if(group==0){
                    state_3508   = MotorApp_MoveTurns(MotorApp_GetAxis(0), 22.9218927f);    //这里算的有问题，这里的圈数是还有除以减速比的
                    state_2006_1 = MotorApp_MoveTurns(MotorApp_GetAxis(1), 34.382839f);
                    if ((state_3508   == MOTORAPP_OK) &&
                            (state_2006_1 == MOTORAPP_OK)) {
                            issued = 1u;            /* 收下了，转去等它走完 */
                        }
                        
                }
                    else {
                        state_2006_2 = MotorApp_MoveTurns(MotorApp_GetAxis(3), 85.94366926f);
                        state_2006_3 = MotorApp_MoveTurns(MotorApp_GetAxis(4), 85.94366926f);
                        if ((state_2006_2 == MOTORAPP_OK) &&
                            (state_2006_3 == MOTORAPP_OK)) {
                            issued = 1u;
                        }
                    }
                }else {
                        if (group == 0u) {
                        if ((axis_done(0u) != 0u) && (axis_done(1u) != 0u)) {
                            group  = 1u;
                            issued = 0u;
                        }
                    }else {
                        if ((axis_done(3u) != 0u) && (axis_done(4u) != 0u)) {
                            state  = second_stairs;
                            issued = 0u;
                        }
                    }
                }
                MotorApp_Update(dt);
                break;
            }

            default: {
                /* 包括 third_stairs 还没写。至少要保住 MotorApp_Update：
                 * 不调它就没有 CAN 帧发出去，电调超时会自己松力。 */
                MotorApp_Update(dt);
                break;
            }
        }
    }
}
