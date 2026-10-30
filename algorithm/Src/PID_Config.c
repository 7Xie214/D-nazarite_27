/**
 * 控制参数值。上层三台 3508 已填约 2s 同步运动的试跑参数，需按负载调参。
 * 速度环使用减速后输出轴 rad/s；轨迹时间在 Task/Src/My_task.c 配置。
 * 此处的试跑增益不代表已经完成实物标定。
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
 * 如果增益或位置环速度限幅为 0，可能不会输出驱动力。参考轨迹可以走，
 * 但状态机还要等实际位置、速度稳定到位，因此不能靠规划结束推进步骤。
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

/* No assumed travel speed or tuned gains. C620 current range is fixed. */
const Axis_Cfg_t CHASSIS_FRONT_MOVE_CFG[2] = {
    { /* FL */
        .cascade = {
            .pos_kp = 0.0f, .pos_ki = 0.0f,
            .pos_integral_max = 0.0f, .pos_speed_limit = 0.0f,
            .spd_kp = 0.0f, .spd_ki = 0.0f, .spd_kd = 0.0f,
            .spd_integral_max = 0.0f, .spd_out_max = 16384.0f,
            .spd_d_alpha = 1.0f, .spd_anti_windup = PID_AW_COND,
            .spd_integ_sep_thr = 0.0f
        },
        .traj_v_max = 0.0f, .traj_a_max = 0.0f
    },
    { /* FR */
        .cascade = {
            .pos_kp = 0.0f, .pos_ki = 0.0f,
            .pos_integral_max = 0.0f, .pos_speed_limit = 0.0f,
            .spd_kp = 0.0f, .spd_ki = 0.0f, .spd_kd = 0.0f,
            .spd_integral_max = 0.0f, .spd_out_max = 16384.0f,
            .spd_d_alpha = 1.0f, .spd_anti_windup = PID_AW_COND,
            .spd_integ_sep_thr = 0.0f
        },
        .traj_v_max = 0.0f, .traj_a_max = 0.0f
    }
};

/* Four chassis wheels: no tuned gains or assumed steering speed limit. */
const Chassis_PID_Cfg_t CHASSIS_PID_CFG[4] = {
    { /* FL: tune independently; steering position speed limit is rad/s. */
        .steer = {
            .pos_kp = 0.0f, .pos_ki = 0.0f,
            .pos_integral_max = 0.0f, .pos_speed_limit = 0.0f,
            .spd_kp = 0.0f, .spd_ki = 0.0f, .spd_kd = 0.0f,
            .spd_integral_max = 0.0f, .spd_out_max = 30000.0f,
            .spd_d_alpha = 1.0f, .spd_anti_windup = PID_AW_COND,
            .spd_integ_sep_thr = 0.0f
        },
        .drive = {
            .kp = 0.0f, .ki = 0.0f, .kd = 0.0f,
            .integral_max = 0.0f, .out_max = 16384.0f,
            .d_alpha = 1.0f, .anti_windup = PID_AW_COND,
            .integ_sep_thr = 0.0f
        }
    },
    { /* FR: tune independently; steering position speed limit is rad/s. */
        .steer = {
            .pos_kp = 0.0f, .pos_ki = 0.0f,
            .pos_integral_max = 0.0f, .pos_speed_limit = 0.0f,
            .spd_kp = 0.0f, .spd_ki = 0.0f, .spd_kd = 0.0f,
            .spd_integral_max = 0.0f, .spd_out_max = 30000.0f,
            .spd_d_alpha = 1.0f, .spd_anti_windup = PID_AW_COND,
            .spd_integ_sep_thr = 0.0f
        },
        .drive = {
            .kp = 0.0f, .ki = 0.0f, .kd = 0.0f,
            .integral_max = 0.0f, .out_max = 16384.0f,
            .d_alpha = 1.0f, .anti_windup = PID_AW_COND,
            .integ_sep_thr = 0.0f
        }
    },
    { /* RL: tune independently; steering position speed limit is rad/s. */
        .steer = {
            .pos_kp = 0.0f, .pos_ki = 0.0f,
            .pos_integral_max = 0.0f, .pos_speed_limit = 0.0f,
            .spd_kp = 0.0f, .spd_ki = 0.0f, .spd_kd = 0.0f,
            .spd_integral_max = 0.0f, .spd_out_max = 30000.0f,
            .spd_d_alpha = 1.0f, .spd_anti_windup = PID_AW_COND,
            .spd_integ_sep_thr = 0.0f
        },
        .drive = {
            .kp = 0.0f, .ki = 0.0f, .kd = 0.0f,
            .integral_max = 0.0f, .out_max = 16384.0f,
            .d_alpha = 1.0f, .anti_windup = PID_AW_COND,
            .integ_sep_thr = 0.0f
        }
    },
    { /* RR: tune independently; steering position speed limit is rad/s. */
        .steer = {
            .pos_kp = 0.0f, .pos_ki = 0.0f,
            .pos_integral_max = 0.0f, .pos_speed_limit = 0.0f,
            .spd_kp = 0.0f, .spd_ki = 0.0f, .spd_kd = 0.0f,
            .spd_integral_max = 0.0f, .spd_out_max = 30000.0f,
            .spd_d_alpha = 1.0f, .spd_anti_windup = PID_AW_COND,
            .spd_integ_sep_thr = 0.0f
        },
        .drive = {
            .kp = 0.0f, .ki = 0.0f, .kd = 0.0f,
            .integral_max = 0.0f, .out_max = 16384.0f,
            .d_alpha = 1.0f, .anti_windup = PID_AW_COND,
            .integ_sep_thr = 0.0f
        }
    }
};

/* 上层单轴 MoveTurns 的默认规划限幅，统一使用输出轴 rad / rad/s。
 * MoveTurns 的输入现在是转子圈数，所以默认行程换算也除一次减速比。
 * 三台 3508 同步运动另用一个 0~1 进度规划器，行程和时间尺度在
 * Task/Src/My_task.c 填写，不依赖这里两份旧行程字面量保持同步。 */
#define TWO_PI          6.28318530718f
#define STAIR_R         0.12f
#define STAIR_Q         0.12f
#define STAIR_TURNS_1   22.9218927f   /* 转子圈数 */
#define STAIR_TURNS_2   34.382839f    /* 转子圈数 */
#define STAIR_D1        (STAIR_TURNS_1 * TWO_PI / (3591.0f / 187.0f))
#define STAIR_D2        (STAIR_TURNS_2 * TWO_PI / (3591.0f / 187.0f))
/* ====================== 3508 · 轴 1（CAN3 反馈 ID 0x205） ======================
 *
 * 下面每根轴的块结构完全一样。把 _1/_2/_3 改成机构上的位置名（升降、
 * 大臂、小臂之类）更好认 —— 名字只在这三个文件和 MotorApp.c 里出现。 */

/* 上层 3508 试跑 PI：
 * 速度单位换算 k = (3591/187)*60/(2*pi) = 183.3771。
 * 原转子 rpm 的 Kp=8 -> 当前输出 rad/s 的 Kp=1467.0171。
 * Ki=2200.5256 是当前连续式 PI 的新试跑初值（积分时间约 0.667s），
 * 不是对未知旧控制周期的离散 Ki=12 做等价移植。
 * spd_integral_max 限制积分累积量，积分指令上限=Ki*spd_integral_max，
 * 以各轴当前配置为准。位置环 P 按 CAN3 ID5/ID2/ID1 为24/24/18 (1/s)。
 * pos_integral_max 为 0 时，位置积分不生效，即使 pos_ki 非零。
 * 这些是带载试调值；实际到位时间和末端稳定性仍需实机确认。
 */
const Axis_Cfg_t AXIS_CFG_3508_1 = {
    .cascade = {
        /* ---- 位置环（PI，Kd 固定为 0，见 PositionLoop.h） ---- */
        .pos_kp             = 24.0f,
        .pos_ki             = 4.5f,
        .pos_integral_max   = 15.0f,     /* 位置单位 · 秒 = rad·s */
        .pos_speed_limit    = 30.0f,     /* 输出轴rad/s；快轨迹峰值约20，留余量纠偏 */

        /* ---- 速度环 ---- */
        .spd_kp             = 2467.0171f,
        .spd_ki             = 4200.5256f,
        .spd_kd             = 0.0f,
        .spd_integral_max   = (8000.0f / 2200.5256f),
        .spd_out_max        = 16384.0f, /* C620 量程 —— 跟型号走，别按轴改 */
        .spd_d_alpha        = 1.0f,     /* 1 = 不滤波；建议 0.1~0.5 */
        .spd_anti_windup    = PID_AW_COND,
        .spd_integ_sep_thr  = 0.0f,     /* <= 0 表示不启用积分分离 */
    },
    .traj_v_max = 20.0f,                /* ID5独立触发：输出轴rad/s；20转子圈理论0.527s */
    .traj_a_max = 100.0f,               /* 输出轴rad/s²；35转子圈理论0.773s */
};

/* ====================== 3508 · 轴 2（反馈 ID 0x202） ====================== */

const Axis_Cfg_t AXIS_CFG_3508_2 = {
    .cascade = {
        .pos_kp             = 24.0f,    /* CAN3 ID2：增强末端位置纠偏 */
        .pos_ki             = 4.5f,
        .pos_integral_max   = 15.0f,
        .pos_speed_limit    = 30.0f,     /* 输出轴rad/s，快轨迹及位置纠偏共用上限 */

        .spd_kp             = 1467.0171f,
        .spd_ki             = 2200.5256f,
        .spd_kd             = 0.0f,
        .spd_integral_max   = (8000.0f / 2200.5256f), /* ID2 带载到位试调 */
        .spd_out_max        = 16384.0f, /* C620 量程 —— 跟型号走，别按轴改 */
        .spd_d_alpha        = 0.5f,
        .spd_anti_windup    = PID_AW_COND,
        .spd_integ_sep_thr  = 0.0f,
    },
    .traj_v_max = 30.0f,   /* 单轴MoveTurns：输出轴rad/s */
    .traj_a_max = 30.0f,   /* 输出轴rad/s² */
};

/* ====================== 3508 · 轴 3（CAN3 反馈 ID 0x201） ====================== */

const Axis_Cfg_t AXIS_CFG_3508_3 = {
    .cascade = {
        .pos_kp             = 24.0f,
        .pos_ki             = 4.5f,
        .pos_integral_max   = 15.0f,
        .pos_speed_limit    = 30.0f,     /* 输出轴rad/s；当前快轨迹峰值约19.1 */

        .spd_kp             = 1467.0171f,
        .spd_ki             = 2200.5256f,
        .spd_kd             = 0.0f,
        .spd_integral_max   = (8000.0f / 2200.5256f),
        .spd_out_max        = 16384.0f, /* C620 量程 —— 跟型号走，别按轴改 */
        .spd_d_alpha        = 1.0f,
        .spd_anti_windup    = PID_AW_COND,
        .spd_integ_sep_thr  = 0.0f,
    },
    .traj_v_max = 30.0f,   /* 单轴MoveTurns：输出轴rad/s */
    .traj_a_max = 30.0f,
};

/* ====================== 2006 · 轴 4（反馈 ID 0x204） ======================
 *
 * 注意 spd_out_max 是 10000 而不是 16384 —— C610 的量程比 C620 小。
 * 填成 16384 也能编过、也能动，但电调会长期被顶在量程边界上。 */

/* 两份 AXIS_CFG 的串级PID用于双轮组；traj_v/a仅用于单轴MoveTurns。
 * 第三步的公共轨迹速度、加速度在下面 MOTORPAIR_CFG_2006 中修改。
 * 2006带载起步约需1000原始电流指令：速度PI先试300/150，单位为
 * 输出轴rad/s；积分贡献限于±3000。这是新试调值，需实机确认响应。 */
const Axis_Cfg_t AXIS_CFG_2006_1 = {
    .cascade = {
        .pos_kp             = 18.0f,
        .pos_ki             = 0.0f,
        .pos_integral_max   = 0.0f,
        .pos_speed_limit    = 50.0f,    /* 输出轴 rad/s */

        .spd_kp             = 300.0f,
        .spd_ki             = 150.0f,
        .spd_kd             = 0.0f,
        .spd_integral_max   = (3000.0f / 150.0f), /* 积分累积量上限，I项±3000 */
        .spd_out_max        = 10000.0f, /* C610 量程，比 C620 小 —— 别按轴改 */
        .spd_d_alpha        = 1.0f,
        .spd_anti_windup    = PID_AW_COND,
        .spd_integ_sep_thr  = 0.0f,
    },
    .traj_v_max = 15.0f,                /* 输出轴 rad/s */
    .traj_a_max = 15.0f,                /* 输出轴 rad/s² */
};

/* ====================== 2006 · 轴 5（CAN3 反馈 ID 0x203） ====================== */

const Axis_Cfg_t AXIS_CFG_2006_2 = {
    .cascade = {
         .pos_kp             = 18.0f,
        .pos_ki             = 0.0f,
        .pos_integral_max   = 0.0f,
        .pos_speed_limit    = 50.0f,    /* 输出轴 rad/s */

        .spd_kp             = 300.0f,
        .spd_ki             = 150.0f,
        .spd_kd             = 0.0f,
        .spd_integral_max   = (3000.0f / 150.0f), /* 积分累积量上限，I项±3000 */
        .spd_out_max        = 10000.0f, /* C610 量程，比 C620 小 —— 别按轴改 */
        .spd_d_alpha        = 1.0f,
        .spd_anti_windup    = PID_AW_COND,
        .spd_integ_sep_thr  = 0.0f,
    },
    .traj_v_max = 15.0f,                /* 输出轴 rad/s */
    .traj_a_max = 15.0f,                /* 输出轴 rad/s² */
};

/* 双2006直行组初测值：位置/同步误差按转子圈；速度/加速度按输出轴。
 * 轮径和传动比假定相同。增益尚需带载标定；不再配置跟随减速/超时阈值。 */
const MotorPair_Cfg_t MOTORPAIR_CFG_2006 = {
    .traj_v_max = 30.0f,
    .traj_a_max = 30.0f,
    .sync_kp = 2.0f,
    .sync_speed_max = 1.0f,
    .command_a_max = 60.0f,
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
