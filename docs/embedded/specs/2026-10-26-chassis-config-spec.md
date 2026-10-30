# Chassis macro configuration and initialization

User follow-up preference: configuration values are now edited directly as plain
#define entries. Per-parameter #ifndef overrides and the nonboolean flag check
were removed; only the wheel-selection mutual-exclusion check remains. Tests
exercise alternate settings using temporary configuration header copies.

Implement the user's requested Chassis_Init and configuration header. Preserve
pure kinematics and existing motor-independent types. Other wheel systems have
reserved branches only; do not add their structs, calculations or driver hooks.

- Create Chassis/Inc/Chassis_Conf.h, Chassis/Inc/Chassis.h, Chassis/Src/Chassis.c.
- CHASSIS_ENABLE_MECANUM=0, CHASSIS_ENABLE_OMNIWHEEL=0,
  CHASSIS_ENABLE_SWERVE=1 by default. Flags must be 0/1 and at most one is enabled.
  All zero is valid. Use #ifndef defaults to allow build/test overrides.
- Expose void Chassis_Init(void). Its #if/#elif branches select the active wheel
  type; mecanum/omni branches are explicit reserved locations with no fake types.
- Under swerve selection, expose g_swerve_modules[SWERVE_MODULE_COUNT] in standard
  FL/FR/RL/RR order. Init copies macro-configured per-wheel calibration/directions/
  angle range/initial hold angle and common wheel radius/half dimensions. It resets
  command velocities and wheel speeds to zero. It is deterministic on repeat calls.
- Per-wheel macro prefix: CHASSIS_SWERVE_FL, FR, RL, RR. Suffixes:
  ZERO_OFFSET, STEER_ENCODER_DIRECTION, STEER_ELECTRIC_DIRECTION,
  DRIVE_ENCODER_DIRECTION, DRIVE_ELECTRIC_DIRECTION, ANGLE_RANGE, INITIAL_ANGLE_RAD.
  Defaults: zero=0u, signs=+1, range=SWERVE_ANGLE_MINUS_90_TO_90, initial angle=0.
- Common macros: CHASSIS_SWERVE_WHEEL_RADIUS_M,
  CHASSIS_SWERVE_HALF_WHEEL_BASE_M, CHASSIS_SWERVE_HALF_TRACK_WIDTH_M.
  Defaults are 0 because physical dimensions are unknown; comments require real
  measurements before using geometry/wheel speed conversion. No guessed hardware.
- Map STEER_ELECTRIC_DIRECTION to the existing electirc_direction field, preserving
  spelling compatibility. Assign signs but do not apply them inside the solver.
- Initialization loads software configuration only; it does not measure actual
  angles, home hardware, start motors, bind PID, or send CAN. Real calibrated
  feedback and initial hold angle must be supplied before controlling hardware.
- Add Chassis_Init to main's user initialization section before the RTOS starts,
  and register the three new files in the existing Keil Chassis group.
- Keep Swerve_Calculate as the existing solver API; no new generic solver wrapper.

Verify default assignment, asymmetric override assignment, reset/reinit, independent
module storage, enabled/disabled/reserved builds, invalid flag rejection, existing
2670 kinematics checks, ARMCC compilation and full Keil build. No board test/commit.
