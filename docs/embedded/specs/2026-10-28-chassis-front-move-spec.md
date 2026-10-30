# Front-wheel step motion

User's latest instruction supersedes earlier velocity-only/average-count ideas:
use existing Traj_t and independent position/speed cascades for FL and FR M3508.
Maintain the 1 kHz chassis task. On an Upstairs request, advance both front
motors by configured fixed rotor turns and reply only after actual arrival.
The user explicitly wants turn macros left unset and will choose the Upstairs
notification call site. Do not edit the upstairs stage machine or add triggers.

## Application control

- Preserve ChassisApp_Update and SetVelocity for ordinary four-wheel kinematics.
  Add ChassisApp_UpdateFrontMove(dt) as the task's exclusive new cyclic entry.
  Never call both updates in one task cycle or send overlapping command frames.
- Two public front controller objects, FL then FR, each bind the existing drive
  motor and own Cascade_t cascade, Traj_t traj and a seed flag. Initialize them
  in ChassisApp_Init. Add CHASSIS_FRONT_MOVE_CFG[2] of Axis_Cfg_t to PID_Config.
  Position and speed loop/planner units are signed gearbox-output rad/rad_s.
- Start API: ChassisApp_MoveFrontTurns(left_rotor_turns,right_rotor_turns).
  Returns OK, NOT_READY, BAD_CONFIG or BUSY. Each target is captured once from
  measured position plus rotor_turns * 2pi / model.gear_ratio. Normalize measured
  position/speed with drive encoder direction; apply electric direction once
  when converting PID output to raw current. Do not divide the target twice.
- Expose front state HOLD, MOVING, DONE, FAULT. Before a request, latch measured
  front positions on first valid feedback and hold them. After completion hold
  final targets. Requests while MOVING do not replace targets. A new accepted
  request starts relative to actual current position, not an old target.
- Front mode holds the four current steering targets and writes zero rear-drive
  commands. It does not invoke inverse kinematics or require geometry/radius/
  linear-speed limits, because both front targets use motor encoder position.
  Still require valid calibrated steering, directions and all eight feedbacks.
- Abort active motion on existing feedback/dt/config/output faults, never report
  DONE on a fault, stop all eight commands and reseed holds on recovery instead
  of resuming an obsolete target.
- Completion requires both planners inactive, both actual position errors and
  actual speeds within configurable tolerances continuously for a settle time.
  Planner completion alone is not physical arrival.

## Configuration

- Chassis_Conf.h: CHASSIS_FRONT_LEFT_MOVE_ROTOR_TURNS and
  CHASSIS_FRONT_RIGHT_MOVE_ROTOR_TURNS default 0 until the user chooses travel.
  Both zero is an unconfigured move and returns BAD_CONFIG, not a false DONE.
- Existing Axis_Cfg_t holds each front cascade and trajectory v_max/a_max.
  Gains, position speed limits and trajectory limits default zero for tuning.
- Direct configurable completion defaults: output-position tolerance 0.01 rad,
  output-speed tolerance 0.05 rad/s, continuous settle time 0.02 s. Document these
  are software defaults, not measured calibration. Motion requires positive
  planning speed, acceleration and position speed limit; zero gains do not count
  as proof of motion or arrival.

## Task handshake

- Use CMSIS-RTOS2 thread flags: START=1 on Chassis_Task; DONE=2 and ERROR=4 on
  Upstairs_Task. Poll start flags with timeout zero and test osFlagsError before
  interpreting flag bits. Do not block either 1 kHz control loop.
- ChassisTask_RequestFrontMove() is called by Upstairs only; clears stale reply
  flags and posts START. ChassisTask_PollReply() consumes DONE/ERROR nonblocking,
  returning zero when none. The caller keeps one request outstanding.
- Task retains a START arriving before feedback readiness, starts exactly once,
  discards duplicate START while busy, and sends exactly one result for that
  request. NOT_READY is pending; BAD_CONFIG/FAULT produce ERROR, never DONE.
  Retry a failed result publication; do not start another move in that gap.
- Serialize busy-START draining and result publication with a short scheduler
  lock against the Upstairs-only sender. Restore the prior scheduler state using
  this CMSIS port's actual behavior; do not mask interrupts around thread flags.
- Reply destination is the existing Upstairs_TaskHandle; reuse existing CubeMX
  thread creation, priority, stack and period logic. Reserved modes remain inert.

No physical runs, flash, automatic nonzero travel, guessed Upstairs call sites,
or changes to CAN3 bindings/control parameters are authorized by this change.
