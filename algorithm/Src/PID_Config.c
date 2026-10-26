/**
 * 控制参数值。增益还全是 0，只有上台阶那两根轴的 traj_v_max / traj_a_max
 * 填了数（两轴按行程缩放，见下面那段）—— 结构先搭好，等整机通电、确定
 * 量纲之后再把增益填进去。
 *
 * ---------------------------------------------------------------------------
 * 【每根轴一份】
 *
 * 五台电机装在机构的不同位置，负载、惯量、摩擦都不一样，所以参数是分开
 * 的，各调各的。别图省事把它们调成一样 —— 那等于让几根轴互相将就：
 * 按重载调好的，轻载那根偏硬、容易振；按轻载调的，重载那根软绵绵、到位慢。
 *
 * 只有一个字段是跟【型号】走、不跟轴走的：
 *
 *   spd_out_max   电机的电流命令量程。3508/C620 是 16384，2006/C610 是
 *                 10000。填超了会把小电调顶爆 —— 改参数的时候别动它。
 *
 * 现在增益是 0，所以 PID 输出恒为 0 —— 上电跑会看到"轨迹按时序走完、
 * 状态机正常推进，但机构一动不动"。这不是坏的，是增益没填。
 * 反过来 traj_v_max = 0 时规划器不动，那种情况下轴永远到不了位。
 *
 * ---------------------------------------------------------------------------
 * 填数之前先想清楚量纲，级联里单位不统一是最难查的一类 bug：
 *
 *   整条链推荐统一用【输出轴弧度】：
 *     位置  rad       速度  rad/s       加速度  rad/s²
 *
 *   速度环：反馈 = DjiMotor.speed_rad_s
 *           Kp 单位 = 电流命令 / (rad/s)
 *           out_max = 电机的电流命令量程（3508 是 16384，2006 是 10000）
 *
 *   位置环：反馈 = DjiMotor.angle_rad（多圈连续）
 *           输出 = 速度环的 setpoint，所以单位必须是 rad/s
 *           Kp 单位 = (rad/s) / rad = 1/s
 *           speed_limit 就是这个关节允许的最大速度
 *
 *   规划器：v_max 必须【明显小于】pos_speed_limit，留出量程给位置环修误差。
 *           比如 speed_limit = 10 rad/s，v_max 取 6~8。
 *
 * 调参顺序：
 *   1. 位置环断开（vel_ff 传 0、pos_kp/ki 给 0），直接往速度环喂阶跃设定值，
 *      把速度环调好。
 *   2. 合上位置环，从纯 P 开始加，看阶跃响应。
 *   3. 最后开规划器，看整段行程的跟随误差和到位情况。
 *   反过来做的话，两环的毛病会糊在一起，根本分不清是哪一环的问题。
 *
 * 参考起点（3508 输出轴，1 圈/秒 ≈ 6.28 rad/s）：
 *   v_max = 6.28f, a_max = 20.0f
 * ---------------------------------------------------------------------------
 */

#include "PID_Config.h"

/* ====================== 上台阶两轴的时间尺度 ======================
 *
 * 两根杆必须【同时到位】，但行程不同（轴1 走 22.92 圈、轴2 走 34.38 圈，
 * 差 1.5 倍）。这个靠调 PID 拉不齐 —— 环路只负责跟踪，什么时候到是由
 * 轨迹决定的。参数给成一样只会让两轴角速度一样，行程长的必然晚到。
 *
 * 做法：两轴共用同一组时间尺度，各自的 v_max / a_max 按【行程】缩放。
 *   速度、加速度都正比于 D  ⇒  两个剖面互为"时间缩放版"
 *
 *     v_max_i = R · D_i
 *     a_max_i = Q · D_i
 *
 * 于是两个跟行程无关的量被钉死了：
 *     全程时间 T = 1/R + R/Q
 *     加速时间   = R/Q          （= v_max/a_max，与 D 无关）
 * 行程差多少都不会跑偏，两轴严格同时到位。三角形剖面同理。
 *
 * 这三个宏只被本文件用，所以放在 .c 里、不进头文件 —— 放头文件会污染
 * 所有 include 它的人（MotorApp.c 里就有一份自己的 TWO_PI，会撞车）。
 *
 * ⚠️ 单位：pos_ref / vel_ref 都是【输出轴弧度】，所以 D 必须先把圈数乘
 *    2π。下面的 _TURNS 是圈数，乘完 2π 才是 D（弧度）。
 *
 * ⚠️ _TURNS 和 My_task.c 里 MoveTurns 传的是同一组值，现在有 4 处字面量。
 *    改行程时要 4 处一起改 —— 建议后面把它们提到头文件里共用一份。 */

#define TWO_PI          6.28318530718f

#define STAIR_R         0.12f   /* 1/s  ；全程 ≈ 1/R + R/Q 秒 */
#define STAIR_Q         0.12f   /* 1/s² ；R/Q = 加速段时间 */

/* 行程（【输出轴】圈数）。
 * ⚠️ 现在这两个数还是【没除减速比】的旧值（My_task.c 里留了标记），
 *    更正时连同 My_task.c 那两处一起改。 */
#define STAIR_TURNS_1   22.9218927f
#define STAIR_TURNS_2   34.382839f

/* 行程（弧度）= 圈数 × 2π。
 * v_max 和 a_max 都从这一个数推 —— 不可能只给一个乘 2π、另一个忘了。 */
#define STAIR_D1        (STAIR_TURNS_1 * TWO_PI)    /*  144.02 rad */
#define STAIR_D2        (STAIR_TURNS_2 * TWO_PI)    /*  216.03 rad */

/* ====================== 3508 · 轴 1（反馈 ID 0x201） ======================
 *
 * 下面每根轴的块结构完全一样。把 _1/_2/_3 改成机构上的位置名（升降、
 * 大臂、小臂之类）更好认 —— 名字只在这三个文件和 MotorApp.c 里出现。 */

const Axis_Cfg_t AXIS_CFG_3508_1 = {
    .cascade = {
        /* ---- 位置环（PI，Kd 固定为 0，见 PositionLoop.h） ---- */
        .pos_kp             = 0.0f,
        .pos_ki             = 0.0f,
        .pos_integral_max   = 0.0f,     /* 位置单位 · 秒 = rad·s */
        .pos_speed_limit    = 0.0f,     /* rad/s，这个关节的速度上限 */

        /* ---- 速度环 ---- */
        .spd_kp             = 0.0f,
        .spd_ki             = 0.0f,
        .spd_kd             = 0.0f,
        .spd_integral_max   = 0.0f,
        .spd_out_max        = 16384.0f, /* C620 量程 —— 跟型号走，别按轴改 */
        .spd_d_alpha        = 1.0f,     /* 1 = 不滤波；建议 0.1~0.5 */
        .spd_anti_windup    = PID_AW_COND,
        .spd_integ_sep_thr  = 0.0f,     /* <= 0 表示不启用积分分离 */
    },
    .traj_v_max = STAIR_R * STAIR_D1,   /* ≈ 17.28 rad/s  —— 全程 9.33s */
    .traj_a_max = STAIR_Q * STAIR_D1,   /* ≈ 17.28 rad/s² —— 加速 1.00s */
};

/* ====================== 3508 · 轴 2（反馈 ID 0x202） ====================== */

const Axis_Cfg_t AXIS_CFG_3508_2 = {
    .cascade = {
        .pos_kp             = 0.0f,
        .pos_ki             = 0.0f,
        .pos_integral_max   = 0.0f,
        .pos_speed_limit    = 0.0f,

        .spd_kp             = 0.0f,
        .spd_ki             = 0.0f,
        .spd_kd             = 0.0f,
        .spd_integral_max   = 0.0f,
        .spd_out_max        = 16384.0f, /* C620 量程 —— 跟型号走，别按轴改 */
        .spd_d_alpha        = 1.0f,
        .spd_anti_windup    = PID_AW_COND,
        .spd_integ_sep_thr  = 0.0f,
    },
    .traj_v_max = STAIR_R * STAIR_D2,   /* ≈ 25.92 rad/s  —— 和轴1 同时到位 */
    .traj_a_max = STAIR_Q * STAIR_D2,   /* ≈ 25.92 rad/s² —— 加速 1.00s */
};

/* ====================== 3508 · 轴 3（反馈 ID 0x203） ====================== */

const Axis_Cfg_t AXIS_CFG_3508_3 = {
    .cascade = {
        .pos_kp             = 0.0f,
        .pos_ki             = 0.0f,
        .pos_integral_max   = 0.0f,
        .pos_speed_limit    = 0.0f,

        .spd_kp             = 0.0f,
        .spd_ki             = 0.0f,
        .spd_kd             = 0.0f,
        .spd_integral_max   = 0.0f,
        .spd_out_max        = 16384.0f, /* C620 量程 —— 跟型号走，别按轴改 */
        .spd_d_alpha        = 1.0f,
        .spd_anti_windup    = PID_AW_COND,
        .spd_integ_sep_thr  = 0.0f,
    },
    .traj_v_max = 0.0f,
    .traj_a_max = 0.0f,
};

/* ====================== 2006 · 轴 4（反馈 ID 0x204） ======================
 *
 * 注意 spd_out_max 是 10000 而不是 16384 —— C610 的量程比 C620 小。
 * 填成 16384 也能编过、也能动，但电调会长期被顶在量程边界上。 */

const Axis_Cfg_t AXIS_CFG_2006_1 = {
    .cascade = {
        .pos_kp             = 0.0f,
        .pos_ki             = 0.0f,
        .pos_integral_max   = 0.0f,
        .pos_speed_limit    = 0.0f,

        .spd_kp             = 0.0f,
        .spd_ki             = 0.0f,
        .spd_kd             = 0.0f,
        .spd_integral_max   = 0.0f,
        .spd_out_max        = 10000.0f, /* C610 量程，比 C620 小 —— 别按轴改 */
        .spd_d_alpha        = 1.0f,
        .spd_anti_windup    = PID_AW_COND,
        .spd_integ_sep_thr  = 0.0f,
    },
    .traj_v_max = 0.0f,
    .traj_a_max = 0.0f,
};

/* ====================== 2006 · 轴 5（反馈 ID 0x205） ====================== */

const Axis_Cfg_t AXIS_CFG_2006_2 = {
    .cascade = {
        .pos_kp             = 0.0f,
        .pos_ki             = 0.0f,
        .pos_integral_max   = 0.0f,
        .pos_speed_limit    = 0.0f,

        .spd_kp             = 0.0f,
        .spd_ki             = 0.0f,
        .spd_kd             = 0.0f,
        .spd_integral_max   = 0.0f,
        .spd_out_max        = 10000.0f, /* C610 量程，比 C620 小 —— 别按轴改 */
        .spd_d_alpha        = 1.0f,
        .spd_anti_windup    = PID_AW_COND,
        .spd_integ_sep_thr  = 0.0f,
    },
    .traj_v_max = 0.0f,
    .traj_a_max = 0.0f,
};

/* ============================== 应用 ============================== */

void PID_Config_Apply(Cascade_t *c, Traj_t *t, const Axis_Cfg_t *cfg)
{
    if (cfg == NULL) {
        return;
    }

    Cascade_Init(c, &cfg->cascade);
    Traj_Init(t, cfg->traj_v_max, cfg->traj_a_max);
}
