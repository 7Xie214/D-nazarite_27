# 舵轮运动学库

`Swerve.h/.c` 只做底盘逆运动学：底盘速度 → 四个轮组的目标舵角和轮速。
不依赖 HAL、RTOS、`DjiMotor_t` 或具体电机型号，不初始化电机、不运行 PID、不发 CAN。

数据流：

```text
整车速度 vx/vy/wz + 实际舵角
        ↓
Swerve_Calculate：轮组解算 → 范围内选角/轮速反转 → 等比例限速
        ↓
每轮 target_angle_rad / target_speed_mps
        ↓
APP：单位与方向适配、舵向位置/速度环、驱动速度环
        ↓
Device：电流命令组帧 → BSP：CAN 发送
```

## 配置和统一初始化

在 `Chassis/Inc/Chassis_Conf.h` 中填写参数。当前选择为：

```c
#define CHASSIS_ENABLE_MECANUM    0
#define CHASSIS_ENABLE_OMNIWHEEL  0
#define CHASSIS_ENABLE_SWERVE     1
```

选择宏用 0 关闭、1 启用，同时最多启用一种轮系；全部为 0 时不初始化任何轮系。
`Chassis_Init()` 用 `#if / #elif` 分支处理，麦轮、全向轮目前只有预留位置，
没有新增对应结构体或解算实现。现有 `Swerve_Calculate()` 仍是舵轮解算入口。

启用舵轮时，`Chassis.h` 提供四元素数组 `g_swerve_modules`。`main.c` 已在
启动 RTOS 前调用一次 `Chassis_Init()`，将宏参数装入该数组。

公共参数 `CHASSIS_SWERVE_WHEEL_RADIUS_M`、`CHASSIS_SWERVE_HALF_WHEEL_BASE_M`、
`CHASSIS_SWERVE_HALF_TRACK_WIDTH_M` 都以 m 为单位，
`CHASSIS_SWERVE_MAX_WHEEL_SPEED_MPS` 单位为 m/s。这四个参数必须是有限正数。
由于尚未提供实测尺寸与速度上限，默认值为 0，普通四轮速度模式会报配置故障并输出零命令。
当前任务使用前轮编码器相对行程模式，不依赖这四项几何/线速度参数。
默认值不代表实机几何。

每轮前缀为 `CHASSIS_SWERVE_FL`、`CHASSIS_SWERVE_FR`、`CHASSIS_SWERVE_RL`、
`CHASSIS_SWERVE_RR`，后缀如下：

| 后缀 | 作用 | 默认值 |
| --- | --- | --- |
| `_ZERO_OFFSET` | 舵向编码器零偏计数 | 0 |
| `_STEER_ENCODER_DIRECTION` | 舵向反馈方向 | +1 |
| `_STEER_ELECTRIC_DIRECTION` | 舵向电流方向 | +1 |
| `_DRIVE_ENCODER_DIRECTION` | 驱动反馈方向 | +1 |
| `_DRIVE_ELECTRIC_DIRECTION` | 驱动电流方向 | +1 |
| `_ANGLE_RANGE` | 舵角范围 | -90°～90° |
| `_INITIAL_ANGLE_RAD` | 初始软件角度及保持目标，rad | 0 |

例如 `CHASSIS_SWERVE_FL_STEER_ENCODER_DIRECTION` 控制左前轮的舵向反馈方向
配置值。初始化只保存这些系数，仍由应用层在反馈/电流适配时应用它们。
参数直接在配置头文件里修改，不再为每个参数添加编译选项覆盖分支。

`Chassis_Init()` 会清零速度指令和目标轮速，将实际角度字段及保持目标设为
`_INITIAL_ANGLE_RAD`，重复调用会重新加载并重置这些状态。它是启动配置加载，
不会读取编码器或完成机械回零；开始控制前应装入实际标定后的反馈与保持角度。

## 坐标、顺序与单位

- 车头为 +x，车左为 +y，俯视逆时针为 +wz；速度指令使用车体坐标系。
- 数组顺序固定为 `SWERVE_FRONT_LEFT`、`SWERVE_FRONT_RIGHT`、
  `SWERVE_REAR_LEFT`、`SWERVE_REAR_RIGHT`。
- `half_wheel_base` 是前后方向半轴距，`half_track_width` 是左右方向半轮距，单位 m。
- `actual_angle_rad` 是经过零偏、方向和传动比处理后的实际舵角，朝前为 0，
  向左转为正。有限范围模式要求它位于选定范围内；连续模式支持多圈角度。
- `target_angle_rad` 使用同一机械坐标系，默认限制在 [-pi/2, pi/2]。
  选择 [0, pi] 模式时也严格限制范围；仅连续模式可以输出跨圈目标值。
- `target_speed_mps` 是轮缘线速度，负值表示轮子反转；不是电机 RPM 或电流。
- `V_x`、`V_y`、`W_z` 在成功解算后记录本次请求的底盘速度，方便调试。
  发生限速时，实际可实现的底盘速度也随各轮一起等比例减小。

## 调用示例

以下代码展示应用层如何准备数据。`steer_angle_rad` 由应用层提供，已完成机械标定。
机械尺寸在配置头文件按实测值填写，示例中的速度上限按实际需要调整。

```c
#include "Chassis.h"

#if CHASSIS_ENABLE_SWERVE
/* 标定完成后调用一次，输入是真实的机械舵角。 */
void Chassis_SetInitialFeedback(const float steer_angle_rad[SWERVE_MODULE_COUNT])
{
    size_t i;
    for (i = 0u; i < SWERVE_MODULE_COUNT; i++) {
        g_swerve_modules[i].actual_angle_rad = steer_angle_rad[i];
        g_swerve_modules[i].target_angle_rad = steer_angle_rad[i];
    }
}

Swerve_Status_t Chassis_Solve(
    const float steer_angle_rad[SWERVE_MODULE_COUNT],
    float vx_mps, float vy_mps, float wz_rad_s)
{
    size_t i;
    for (i = 0u; i < SWERVE_MODULE_COUNT; i++) {
        g_swerve_modules[i].actual_angle_rad = steer_angle_rad[i];
    }
    return Swerve_Calculate(g_swerve_modules, SWERVE_MODULE_COUNT,
                            vx_mps, vy_mps, wz_rad_s, 2.0f);
}
#endif
```

在配置头文件填写机械参数，启动时初始化；标定后设置初始保持舵角，
此后每拍刷新实际舵角并调用解算。
每个轮组可以独立选择角度范围，`Swerve_Calculate` 的调用方式不变。

## 舵角范围

| `angle_range` 配置 | 目标范围 | 用途 |
| --- | --- | --- |
| `SWERVE_ANGLE_MINUS_90_TO_90` | -90°～90° | 枚举值为 0，默认模式 |
| `SWERVE_ANGLE_0_TO_180` | 0°～180° | 机械半圈范围 |
| `SWERVE_ANGLE_CONTINUOUS` | 连续角度 | 显式保留原有连续转向策略 |

所有接口角度仍然使用弧度。有限范围模式会在等效朝向中筛选合法角度，并选择
距离实际舵角最近的一个；选择相反朝向时同步反转轮速。端点也属于合法范围，
两种表示与实际角度精确等距时优先正轮速表示。例如：

| 范围 | 实际舵角 | 所需运动方向 | 目标舵角 | 轮速符号 |
| --- | --- | --- | --- | --- |
| -90°～90° | 80° | 100° | -80° | 反转 |
| 0°～180° | 170° | -10° | 170° | 反转 |
| 0°～180° | 170° | 0° | 180° | 反转 |
| 0°～180° | 170° | 10° | 10° | 正转 |

生成候选舵角时，仅将距离端点不超过 `4*FLT_EPSILON` rad 的数值对齐到端点，
消除三角函数与浮点加减的舍入误差；实际反馈和上次保持目标仍严格检查范围。

最后一行需要转动 160°：等效的 190° 超过机械上限，不能为了转得少就输出它。
机械限位优先于最短转向，因此有限范围模式不保证每次舵向变化都不超过 90°。
应用层的位置环应使用实际差值，不能对有限范围的机械误差跨端点取模。

有限范围模式下，实际反馈与上次保持目标都必须位于相同的选定范围内。
反馈越界、保持目标越界或枚举非法时，整次解算返回 `SWERVE_ERR_PARAM`，不会
自动折回反馈或部分更新轮组。切换模式前应同步校准反馈和保持目标。
本库只约束计算出的目标角，不读取限位开关或保证机械运动过程不超调。

## 最短转向与静止策略

每轮计算 `vx_i = vx - wz*y_i`、`vy_i = vy + wz*x_i`。转向目标与轮速符号的
等效替换保持轮子产生的速度向量不变。
显式选择连续模式时，目标方向离当前舵角超过 90° 就翻转 180° 并反转轮速，
保证连续目标与实际角度之差不超过约 90°；例如实际 179°、方向 -179° 可输出 181°。

轮组请求速度不超过 `SWERVE_SPEED_EPS_MPS` 时，输出零轮速，保持该轮上次的
目标舵角。这里按每个轮组判断，所以以某一轮为旋转中心时，该轮也可以保持原角度。
首次调用前，应将 `target_angle_rad` 设为实际舵角，避免静止时追逐默认零角度。

任一轮超出 `max_wheel_speed_mps` 时，所有轮速乘以同一个系数。速度上限必须
为有限正数。当前库没有加速度规划、余弦补偿、世界坐标转换和里程计功能。

## 与电机和 PID 对接

保留的 `zero_offset`、`encoder_direction`、`electirc_direction`（沿用现有拼写）
以及驱动方向字段由应用层解释，解算不使用它们，以免反馈和控制命令重复翻转。

`RPM_to_Thraed_speed()` 保留现有函数名，输入必须是车轮自身的 RPM，半径单位 m。
若使用 `DjiMotor.speed_rad_s` 且电机减速器输出轴与车轮直连，线速度就是
`speed_rad_s * R`；需要的输出轴角速度则是 `target_speed_mps / R`。
有额外皮带或齿轮传动时，在应用层继续处理额外传动比。

舵向目标用于位置环，驱动轮速目标用于速度环。`ChassisApp` 将解算结果接到四套
独立的舵向串级 PID 和驱动速度 PID。Device/BSP 完成组帧发送：GM6020 使用电压
命令，M3508 使用电流命令。`MotorApp_MoveTurns()` 仍用于上层机构的相对行程。

## 底盘控制应用与任务

启动顺序为：外设初始化 → `DjiMotor_Init()` 注册全部 13 台电机 → 启动 CAN →
`Chassis_Init()` 加载底盘配置 → 启动 RTOS。注册只由 `main` 调用一次；
`MotorApp_Init()` 绑定上层五轴，`ChassisApp_Init()` 绑定底盘八台，均不重复注册。
反馈时间戳和 50 ms 在线判据使用 `HAL_GetTick()` 毫秒时钟，允许 RTOS 启动前接收反馈。

启用舵轮时，独立 `ChassisTask` 在入口初始化一次 `ChassisApp`，栈为 1536 字节，
优先级与上层控制任务相同（`osPriorityHigh2`）。任务用 `osDelayUntil()` 保持标称
1 ms 周期，以内核 tick 的实际间隔换算 `dt`；超期后重设下一次唤醒，避免连续补跑。
沿用 CubeMX 的 `Chassis_Task` 入口与创建点，`.ioc` 同步保存栈大小和优先级。
麦轮、全向轮与全部关闭的配置在任务入口立即退出，不运行舵轮控制。

当前 `ChassisTask` 每拍只调用 `ChassisApp_UpdateFrontMove(dt)`：保持四个舵向目标，
后轮驱动命令为零，左前/右前驱动各运行独立的位置/速度串级和轨迹规划器。
有效反馈到来前不启动；首次有效反馈和故障恢复时，以实际位置重新建立保持目标。
没有 START 请求时只保持，不自动执行行程。前轮模式仍要求八台电机反馈有效、
舵向标定和方向正确；活动行程发生反馈、配置、周期或输出故障时，八台全部停输出，
并以 ERROR 结束该请求，恢复后不继续旧行程。
前轮定圈模式不依赖轮径、轴距、轮距或线速度上限；这些参数用于普通四轮速度模式。

`Chassis/Inc/Chassis_Conf.h` 的两个固定行程宏为电机转子圈数：

```c
#define CHASSIS_FRONT_LEFT_MOVE_ROTOR_TURNS 0.0f
#define CHASSIS_FRONT_RIGHT_MOVE_ROTOR_TURNS 0.0f
```

它们按用户要求保持 0，须根据实机行程填写；两者同时为零返回 ERROR，不能当成到位。
目标在接受请求时分别锁定为各自实际位置加 `rotor_turns * 2*pi / gear_ratio`，
不是四轮平均圈数。反馈已经是减速器输出轴单位，不能再次除以减速比。
运行中重复 START 被消费并忽略，不重启或修改目标；新请求从当时实际位置计算。

`algorithm/Inc/PID_Config.h` 和 `algorithm/Src/PID_Config.c` 中的
`CHASSIS_FRONT_MOVE_CFG[2]` 按 FL/FR 排列，使用现有 `Axis_Cfg_t` 配置独立串级
和轨迹 `v_max/a_max`。位置/速度单位为带反馈方向的减速器输出轴 rad、rad/s，
加速度为 rad/s²。运行对象为 `g_chassis_front_axes[2]`。
增益、位置环速度限幅和轨迹速度/加速度默认仍为 0，须按实机整定；
行程要求正的限幅和规划参数，零增益不能证明电机运动或实际到位。

DONE 要求两路轨迹都结束，且两路实际位置误差、实际速度连续满足容差，
不以规划器结束替代实际到位。软件默认容差为输出轴位置 0.01 rad、
速度 0.05 rad/s，连续稳定 0.02 s，可通过 `CHASSIS_FRONT_MOVE_POS_TOL_RAD`、
`CHASSIS_FRONT_MOVE_SPEED_TOL_RAD_S`、`CHASSIS_FRONT_MOVE_SETTLE_S` 调整；
这些是软件默认值，尚未经过实机标定。完成后继续保持最终目标。

上层任务通过 `ChassisTask.h` 请求并非阻塞轮询。START=1 发给 Chassis_Task，
DONE=2、ERROR=4 回复现有 Upstairs_Task。`ChassisTask_RequestFrontMove()` 仅由
Upstairs 调用，先清除旧回复，再发 START；调用者同时只保留一个未完成请求。
反馈尚未准备好时保留待启动请求；回复发布失败时下拍重试，期间不接受新行程。
轮询返回零表示暂无回复，两个 1 kHz 控制循环均不阻塞等待。

以下仅展示如何接入，用户自行选择在 Upstairs 状态机的哪个位置请求和处理结果；
没有自动插入任何调用点。`dt` 沿用 Upstairs 实测周期：

```c
#include "ChassisTask.h"
#include "MotorApp.h"

/* Upstairs 任务内持久保存；进入用户选定的请求分支时仅调用一次。 */
static uint8_t front_waiting;

/* 用户选择的请求位置： */
if (front_waiting == 0u) {
    if (ChassisTask_RequestFrontMove() == osOK) {
        front_waiting = 1u;
    } else {
        /* 用户处理请求发布失败。 */
    }
}

/* Upstairs 每拍继续运行；等待回复期间也必须执行。 */
MotorApp_Update(dt);
if (front_waiting != 0u) {
    uint32_t reply = ChassisTask_PollReply();
    if ((reply & CHASSIS_TASK_ERROR_FLAG) != 0u) {
        front_waiting = 0u;
        /* 用户选择故障处理分支。 */
    } else if ((reply & CHASSIS_TASK_DONE_FLAG) != 0u) {
        front_waiting = 0u;
        /* 用户选择到位后的状态转换。 */
    }
}
```

请求片段只放在一次性的阶段入口，不能放在每拍无条件执行的位置。
麦轮、全向轮及全部关闭时，任务立即退出，请求接口返回错误，轮询返回零。

普通四轮速度接口 `ChassisApp_SetVelocity(vx, vy, wz)` 和
`ChassisApp_Update(dt)` 仍保留，单位 m/s、m/s、rad/s；当前任务不调用这一模式。
速度指令保留到下一条命令，没有新增超时。若另行切回普通速度模式，须填写正的
几何/速度上限并整定四轮 PID。同一周期只能选择一个 Update 入口。
底盘任务独占 CAN1/2 的命令发送，
`MotorApp_Update(dt)` 独占 CAN3；其他任务不能分别发送同一物理帧，否则槽位会互相覆盖。

`algorithm/Inc/PID_Config.h` 和 `algorithm/Src/PID_Config.c` 中的
`CHASSIS_PID_CFG[4]` 按 FL/FR/RL/RR 排列。每轮各有舵向位置/速度串级配置与
驱动速度配置，运行状态分别保存在 `g_chassis_wheels[4]` 中，调试时可独立观察。
舵向电压限幅为 30000，驱动电流限幅为 16384；所有增益默认 0，没有预置调好值。
默认速度命令为零；使用前必须标定编码器零偏及反馈/电气方向，并分别整定四轮 PID。

首次有效反馈及掉线恢复时，应用先将保持舵角与控制器历史对齐到实际反馈。
任一轮反馈缺失、掉线、越过所选机械角范围，或配置/周期/解算非法时，
整套底盘输出零命令并清除控制器历史。`g_chassis_app_status` 提供故障原因。
连续舵向恢复只可利用单圈编码器选择最接近上次机械角的等效圈数，无法观测掉线期间的整圈运动。

## 参数错误与验证

数组必须提供四个有效元素；尺寸非负，角度与指令有限。参数非法或中间计算
溢出时返回 `SWERVE_ERR_PARAM`，不部分更新四轮数据。调用方应检查返回值，
并自行决定停止驱动等处理，不能忽略错误继续发送旧目标。

在工程根目录运行：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File tests/run_swerve_tests.ps1
powershell -NoProfile -ExecutionPolicy Bypass -File tests/run_chassis_tests.ps1
powershell -NoProfile -ExecutionPolicy Bypass -File tests/run_chassis_app_tests.ps1
powershell -NoProfile -ExecutionPolicy Bypass -File tests/run_chassis_task_tests.ps1
powershell -NoProfile -ExecutionPolicy Bypass -File tests/run_dji_motor_tests.ps1
```

需要 GCC。测试生成的可执行文件放在临时目录，测试不访问硬件。
数值测试与编译检查不能证明真实底盘的方向标定和机械响应。

解算和转向优化参考：
[WPILib Swerve Drive Kinematics](https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-kinematics.html)。
