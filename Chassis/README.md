# 舵轮运动学库

`Swerve.h/.c` 只做底盘逆运动学：底盘速度 → 四个轮组的目标舵角和轮速。
不依赖 HAL、RTOS、`DjiMotor_t` 或具体电机型号，不初始化电机、不运行 PID、不发 CAN。

数据流：

```text
整车速度 vx/vy/wz + 实际舵角
        ↓
Swerve_Calculate：轮组解算 → 最短转向 → 等比例限速
        ↓
每轮 target_angle_rad / target_speed_mps
        ↓
APP：单位与方向适配、舵向位置/速度环、驱动速度环
        ↓
Device：电流命令组帧 → BSP：CAN 发送
```

## 坐标、顺序与单位

- 车头为 +x，车左为 +y，俯视逆时针为 +wz；速度指令使用车体坐标系。
- 数组顺序固定为 `SWERVE_FRONT_LEFT`、`SWERVE_FRONT_RIGHT`、
  `SWERVE_REAR_LEFT`、`SWERVE_REAR_RIGHT`。
- `half_wheel_base` 是前后方向半轴距，`half_track_width` 是左右方向半轮距，单位 m。
- `actual_angle_rad` 是经过零偏、方向和传动比处理后的实际舵角，朝前为 0，
  向左转为正，支持连续多圈角度。
- `target_angle_rad` 使用同一机械坐标系，输出靠近实际角度的连续目标值，
  不限制在 [-pi, pi]。例如实际 179°、目标方向 -179°，可输出 181°。
- `target_speed_mps` 是轮缘线速度，负值表示轮子反转；不是电机 RPM 或电流。
- `V_x`、`V_y`、`W_z` 在成功解算后记录本次请求的底盘速度，方便调试。
  发生限速时，实际可实现的底盘速度也随各轮一起等比例减小。

## 调用示例

以下代码展示应用层如何准备数据。`steer_angle_rad` 由应用层提供，已完成机械标定。
示例尺寸仅用于说明接口，实际参数按底盘测量填写。

```c
#include "Swerve.h"

static Swerve_Module_t wheel[SWERVE_MODULE_COUNT];

void Chassis_Prepare(const float steer_angle_rad[SWERVE_MODULE_COUNT])
{
    size_t i;
    for (i = 0u; i < SWERVE_MODULE_COUNT; i++) {
        wheel[i].half_wheel_base = 0.30f;
        wheel[i].half_track_width = 0.20f;
        wheel[i].steering_gear.R = 0.075f;
        wheel[i].actual_angle_rad = steer_angle_rad[i];
        wheel[i].target_angle_rad = steer_angle_rad[i];
    }
}

Swerve_Status_t Chassis_Solve(
    const float steer_angle_rad[SWERVE_MODULE_COUNT],
    float vx_mps, float vy_mps, float wz_rad_s)
{
    size_t i;
    for (i = 0u; i < SWERVE_MODULE_COUNT; i++) {
        wheel[i].actual_angle_rad = steer_angle_rad[i];
    }
    return Swerve_Calculate(wheel, SWERVE_MODULE_COUNT,
                            vx_mps, vy_mps, wz_rad_s, 2.0f);
}
```

先设置一次机械参数和初始保持舵角，此后每拍刷新实际舵角并调用解算。
解算的最短转向优化假定舵向可连续旋转，没有机械角度限位。

## 最短转向与静止策略

每轮计算 `vx_i = vx - wz*y_i`、`vy_i = vy + wz*x_i`。目标方向离当前舵角
超过 90° 时，将目标方向翻转 180°，并对轮速取反，保持轮子产生的速度向量不变。
这会使舵向目标与实际角度之差不超过约 90°。

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

舵向目标用于位置环，驱动轮速目标用于速度环。PID 计算电流命令，Device/BSP
完成发送。现有 `MotorApp_MoveTurns()` 是相对行程接口，不能直接替代持续更新的
舵向绝对目标和驱动轮速接口；本次没有改动或接通该应用层控制链。

## 参数错误与验证

数组必须提供四个有效元素；尺寸非负，角度与指令有限。参数非法或中间计算
溢出时返回 `SWERVE_ERR_PARAM`，不部分更新四轮数据。调用方应检查返回值，
并自行决定停止驱动等处理，不能忽略错误继续发送旧目标。

在工程根目录运行：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File tests/run_swerve_tests.ps1
```

需要 GCC。测试生成的可执行文件放在临时目录，测试不访问硬件。
数值测试与编译检查不能证明真实底盘的方向标定和机械响应。

解算和转向优化参考：
[WPILib Swerve Drive Kinematics](https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-kinematics.html)。
