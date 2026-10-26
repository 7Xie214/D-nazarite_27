/**
 * 梯形速度规划器实现。核心公式的推导见 Traj.h 顶部。
 */

#include "Traj.h"
#include <math.h>       /* sqrtf / fabsf —— FPv5 上就是一条 VSQRT.F32 / VABS.F32 */

void Traj_Init(Traj_t *t, float v_max, float a_max)
{
    if (t == NULL) {
        return;
    }

    t->v_max   = v_max;
    t->a_max   = a_max;
    t->pos_ref = 0.0f;
    t->vel_ref = 0.0f;
    t->target  = 0.0f;
    t->active  = 0u;
}

void Traj_Start(Traj_t *t, float from, float to)
{
    if (t == NULL) {
        return;
    }

    /* pos_ref 对齐到当前位置，第一拍参考就是当前位置 —— 误差从 0 开始长。
     * 如果把 pos_ref 留在上一段行程的终点，起手就是一记位置阶跃。 */
    t->pos_ref = from;
    t->vel_ref = 0.0f;
    t->target  = to;
    t->active  = 1u;
}

void Traj_Stop(Traj_t *t)
{
    if (t == NULL) {
        return;
    }

    t->active  = 0u;
    t->vel_ref = 0.0f;
    /* pos_ref 不动：位置环拿它当保持点，机构就停在当前位置。 */
}

int Traj_IsActive(const Traj_t *t)
{
    return ((t != NULL) && (t->active != 0u)) ? 1 : 0;
}

void Traj_Update(Traj_t *t, float dt)
{
    float remain, v_des, dv, dv_max;

    if ((t == NULL) || (t->active == 0u) || (dt <= 0.0f)) {
        return;
    }

    remain = t->target - t->pos_ref;

    /* --- 刹得住的速度上限 ---
     * 匀减速到停：v² = 2·a·s，所以 |v| ≤ sqrt(2·a·|s|)。
     * 离得远 → 根号项超过 v_max → 顶到 v_max（加速段结束，进巡航）
     * 快到了 → 根号项掉下来 → 自动按这条曲线匀减速
     * 这一行同时把三段都算出来了，不需要状态机。 */
    v_des = sqrtf(2.0f * t->a_max * fabsf(remain));
    if (v_des > t->v_max) {
        v_des = t->v_max;
    }
    if (remain < 0.0f) {
        v_des = -v_des;                 /* 往回走 */
    }

    /* --- 加速度限幅：一拍最多变 a·dt --- */
    dv_max = t->a_max * dt;
    dv     = v_des - t->vel_ref;
    if (dv >  dv_max) { dv =  dv_max; }
    if (dv < -dv_max) { dv = -dv_max; }
    t->vel_ref += dv;

    /* --- 推进参考位置 --- */
    t->pos_ref += t->vel_ref * dt;

    /* --- 到位判决 ---
     * 位置进到 eps 内【且】速度已经低到一拍就能刹停，才算到。
     * 只看位置的话，会在还有残余速度时被硬切到 0，那一下就是个小阶跃。 */
    if ((fabsf(t->target - t->pos_ref) < TRAJ_POS_EPS) &&
        (fabsf(t->vel_ref) <= (t->a_max * dt))) {
        t->pos_ref = t->target;         /* 抹掉浮点残差，目标值才是准的 */
        t->vel_ref = 0.0f;
        t->active  = 0u;
    }
}
