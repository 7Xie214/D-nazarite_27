# Project context

- Target: STM32H723ZET6, HAL, FreeRTOS CMSIS-RTOS2, Keil ARMCC 5.
- Layers: BSP transports bytes; Device handles motor protocols; algorithm contains
  controllers/planners; APP binds devices to controllers; Chassis contains wheel kinematics.
- Swerve scope: four independently steered/driven modules; pure C99 calculations.
- Swerve coordinates: +x forward, +y left, +wz counterclockwise; SI units.
- Swerve module order: front left, front right, rear left, rear right.
- Motor initialization, feedback calibration, PID updates and CAN transmission stay
  outside the swerve kinematics library.
- Swerve verification (2026-10-26): 595 native checks passed, ARMCC 5 compile passed,
  Keil build reported 0 errors and 2 pre-existing newline warnings. No board run.
