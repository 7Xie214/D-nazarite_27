# Swerve mechanical steering angle limits

The user requests steering targets constrained to either [0, 180] degrees or
[-90, 90] degrees, reversing wheel speed when using the opposite wheel heading.
This supersedes the continuous-only assumption in the initial swerve spec.

- Append a per-module Swerve_AngleRange_t angle_range configuration. Enum zero
  selects [-pi/2, pi/2] by default; also support [0, pi] and the previous explicit
  continuous mode for existing applications and regression checks.
- Keep SI units, four-wheel ordering, and the public Swerve_Calculate signature.
- Limited modes evaluate equivalent headings theta + k*pi, with speed reversed
  for odd k. Consider only headings inside the selected closed interval and choose
  the smallest absolute difference from actual mechanical angle. On an exact tie,
  prefer the original positive-speed representation.
- Limits take priority over the <=90-degree continuous-mode optimization. A
  180-degree-wide mechanical interval can require >90 degrees steering near an
  endpoint; do not wrap the error across a forbidden mechanical stop.
- actual_angle_rad and previous target_angle_rad must be in the selected range.
  Invalid mode or out-of-range feedback/held target rejects the entire calculation
  without changing any module. Do not clamp feedback or silently normalize it.
- Zero-speed hold, common speed scaling, overflow handling and RPM conversion
  retain their existing behavior. Limits are expressed in the same calibrated
  mechanical frame as actual feedback and target output, not raw encoder counts.
- No initialization, motor bindings, PID, hardware commands, Git commit or push.

Verification includes regression tests for continuous mode, both limited ranges,
all quadrant directions, inclusive endpoints, reversal/vector preservation,
nearest legal endpoint, long legal moves, stationary holds, bad mode/feedback/
previous target and transactional failure. Compile natively and with ARMCC 5,
then run a full Keil build. Do not claim physical mechanism verification.
