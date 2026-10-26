/**
 * 位置环串级实现。详细说明见 PositionLoop.h 顶部。
 */

#include "PositionLoop.h"

void Cascade_Init(Cascade_t *c, const Cascade_Cfg_t *cfg)
{
    if ((c == NULL) || (cfg == NULL)) {
        return;
    }

    /* 位置环：策略在这里一次性钉死，调用方不用管 PID_Init 那一串参数。
     * Kd = 0、d_alpha = 1.0（Kd 是 0，微分通道整个是死的，填多少都无所谓，
     * 填 1.0 只是表明"不滤波"这个意图）、PID_AW_COND、不启用积分分离。
     * 理由见 .h 顶部。 */
    PID_Init(&c->pos,
             cfg->pos_kp, cfg->pos_ki, 0.0f,
             cfg->pos_integral_max, cfg->pos_speed_limit,
             1.0f,
             PID_AW_COND, 0.0f);

    /* 速度环：参数全部来自配置。 */
    PID_Init(&c->spd,
             cfg->spd_kp, cfg->spd_ki, cfg->spd_kd,
             cfg->spd_integral_max, cfg->spd_out_max, cfg->spd_d_alpha,
             cfg->spd_anti_windup, cfg->spd_integ_sep_thr);
}

void Cascade_Reset(Cascade_t *c, float pos_actual, float spd_actual)
{
    if (c == NULL) {
        return;
    }

    /* 用自己当前的增益重新 PID_Init 一遍 —— 这就是复位，顺带清空
     * integral / d_filt / prev_actual。
     *
     * 先把增益抄到局部变量，再调 PID_Init，免得读到自己正在被写的字段。
     * （C 里实参在函数体执行前就求值完了，直接传也不会出错，但那样看着悬，
     *   抄一遍更清楚。） */
    {
        float            kp    = c->pos.Kp;
        float            ki    = c->pos.Ki;
        float            kd    = c->pos.Kd;
        float            imax  = c->pos.integral_max;
        float            omax  = c->pos.out_max;
        float            alpha = c->pos.d_alpha;
        PID_AntiWindup_t aw    = c->pos.anti_windup;
        float            thr   = c->pos.integ_sep_thr;

        PID_Init(&c->pos, kp, ki, kd, imax, omax, alpha, aw, thr);
    }
    {
        float            kp    = c->spd.Kp;
        float            ki    = c->spd.Ki;
        float            kd    = c->spd.Kd;
        float            imax  = c->spd.integral_max;
        float            omax  = c->spd.out_max;
        float            alpha = c->spd.d_alpha;
        PID_AntiWindup_t aw    = c->spd.anti_windup;
        float            thr   = c->spd.integ_sep_thr;

        PID_Init(&c->spd, kp, ki, kd, imax, omax, alpha, aw, thr);
    }

    /* 掐掉复位后第一拍的微分冲击（见 .h 里的说明）。 */
    c->pos.prev_actual = pos_actual;
    c->spd.prev_actual = spd_actual;
}

float Cascade_Update(Cascade_t *c,
                     float pos_actual,
                     float spd_actual,
                     float pos_ref,
                     float vel_ff,
                     float dt)
{
    float sp;

    if (c == NULL) {
        return 0.0f;
    }

    /* --- 外环：位置 → 速度 ---
     * setpoint 每拍都跟着规划器走（斜坡给定），不是固定值。 */
    c->pos.setpoint = pos_ref;
    sp = PID_Update_Continuous(&c->pos, pos_actual, dt);

    /* --- 速度前馈 ---
     * 巡航速度由这里给出，位置环的 PI 只管修残差。
     * 少了这一项，加速段的速度差全压在积分器上：误差拉得大、末端容易过冲。 */
    sp += vel_ff;

    /* 前馈加进来之后可能越过速度限幅（PID_Init 只夹了自己那一部分）。
     * 再夹一次，保证喂给速度环的设定值不超限。 */
    if (sp >  c->pos.out_max) { sp =  c->pos.out_max; }
    if (sp < -c->pos.out_max) { sp = -c->pos.out_max; }

    /* --- 内环：速度 → 电流 ---
     * 级联接点就在上面这一行 sp，改接法只改这里。 */
    c->spd.setpoint = sp;
    return PID_Update_Continuous(&c->spd, spd_actual, dt);
}
