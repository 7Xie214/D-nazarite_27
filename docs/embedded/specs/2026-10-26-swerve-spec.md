# Four-module swerve kinematics

## Authorized scope

Implement the kinematics requested after the interface discussion. Keep the user's
four-element Swerve_Module_t array and direction/calibration configuration. Do not
add motor model dependencies, initialization, PID calls or hardware transmission.

## Contract

- Body coordinates: +x forward, +y left, +wz counterclockwise; m, m/s, rad, rad/s.
- Module order: front left, front right, rear left, rear right.
- Each module provides nonnegative half_wheel_base (longitudinal) and
  half_track_width (lateral), actual_angle_rad, and the previous target_angle_rad.
- Resolve signed module positions from the array index. Wheel velocity components:
  vx_i = vx - wz * y_i; vy_i = vy + wz * x_i.
- Compute speed magnitude and atan2 angle. Choose the equivalent heading requiring
  at most 90 degrees steering and negate wheel speed when the heading is flipped.
- Return a continuous target angle near the current actual angle so a position
  controller can subtract feedback directly without a 2*pi boundary jump.
- For requested wheel speed <= 1e-4 m/s, output zero speed and preserve the previous
  target angle. The caller seeds the previous target on first use.
- When any wheel exceeds the supplied positive speed limit, scale all four signed
  speeds by the same factor; preserve the intended chassis motion ratios.
- Swerve_Calculate takes pointer, count (must be 4), body command and speed limit.
  Invalid pointers/counts, nonfinite numbers, negative geometry, or intermediate
  overflow return an error without partially changing module fields.
- Preserve RPM_to_Thraed_speed as a wheel-side RPM conversion. Reject null,
  nonpositive radius and nonfinite/overflowing results by returning zero.
- Encoder zero/sign and current sign fields are retained but not applied in this
  layer: feedback and targets must share the calibrated mechanical coordinate frame.
- Steering is assumed to permit continuous rotation. No steering hard-stop model,
  field-relative transform, odometry, acceleration planning or cosine compensation.

## Verification

Native C tests cover translations, rotation, combined motion, shortest steering,
multi-turn feedback, zero-speed hold, common saturation, invalid inputs, and RPM
conversion. Compile with GCC warnings as errors and the STM32 ARMCC toolchain.
Run a full Keil build. No board actuation is required or claimed for this pure
library; hardware initialization and task integration remain outside this change.

Reference: https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-kinematics.html
