# Chassis application control

The user approved a separate ChassisApp and 1 ms task. Preserve the portable
Swerve library, existing upstairs controller and all thirteen motor registrations.
No flashing, physical motion, or hardware calibration is part of this change.

## Control and ownership

- Main registers motors once before RTOS startup. MotorApp_Init only initializes
  the five upstairs controllers. Chassis_Init continues loading wheel geometry.
- Motor receive timestamps/50 ms online checks use the HAL millisecond tick:
  early CAN interrupts must not call FreeRTOS ISR APIs before its port starts.
- ChassisApp owns CAN1/2 commands; MotorApp owns CAN3 commands. Each task sends
  complete physical frames only after updating its motor commands.
- FL: drive3508_4/steer6020_1; FR: drive3508_5/steer6020_4;
  RL: drive3508_6/steer6020_2; RR: drive3508_7/steer6020_3.
- Four ChassisApp wheel objects link g_swerve_modules to motor pointers, steering
  Cascade_t and drive PID_Controller_t. Expose these objects for tuning/debugging.
- APIs: ChassisApp_Init(), ChassisApp_SetVelocity(vx, vy, wz),
  ChassisApp_Update(dt). SI units, existing chassis coordinates and module order.
- A short PRIMASK-preserving section makes three-component command transfer
  coherent between a command task and the control task.

## Feedback and outputs

- Update eight motors each cycle. Drive feedback is encoder_direction times
  motor output rad/s times wheel radius; speed PID works in m/s.
- Steering limited modes use absolute raw encoder counts, zero offset, encoder
  direction and the configured mechanical interval, not power-on-relative angle.
  Centered mode maps to [-pi,pi]; positive mode maps to [0,2pi). An angle outside
  the selected mechanical interval is invalid, never clamped into that interval.
- Continuous steering uses an absolute zero-calibrated initial angle plus ongoing
  accumulated motor angle. After feedback interruption, reseed from absolute
  feedback using the equivalent turn nearest the last mechanical angle; turns
  made while offline cannot be observed by a single-turn encoder.
- First valid feedback/recovery seeds held targets to measured steering angles
  and resets controller histories to measured values before PID execution.
- Call Swerve_Calculate once for all four wheels. Feed target angles to steering
  cascades and signed wheel speeds to four drive speed PIDs. Apply electric
  direction once at the motor command boundary. Clamp before int16 conversion.
- Missing/offline feedback, invalid dt/configuration, range violations or failed
  solve produce zero commands for the complete chassis and clear PID histories.
  Retain the last commanded velocity; no new remote-control timeout is added.

## Configuration

- Chassis_Conf.h retains geometry, zero offsets and per-wheel directions; add
  CHASSIS_SWERVE_MAX_WHEEL_SPEED_MPS, default zero until physically specified.
- Require positive radius, half wheelbase, half track and maximum wheel speed.
- PID_Config.h/.c holds CHASSIS_PID_CFG[4], each with steering Cascade_Cfg_t
  and a plain drive PID configuration. Gains default zero; command limits are
  30000 steering voltage and 16384 drive current. No fabricated tuned gains.
- Non-swerve selections compile reserved no-op application branches; the CubeMX
  Chassis_Task exits immediately and does not run an active control loop.

## Validation

Native tests use actual application, solver, motor driver and PID code with only
HAL/RTOS transport stubs and a temporary configuration header. Verify bindings,
signed feedback/output conversions, zero hold, all steering ranges, offline
stop/recovery, independent PID histories, physical CAN slots and CAN3 ownership.
Run existing affected checks and an actual ARMCC5 Keil build. Report compilation
and native evidence separately from hardware execution.
