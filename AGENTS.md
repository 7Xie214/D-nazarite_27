# Project context

- Target: STM32H723ZET6, HAL, FreeRTOS CMSIS-RTOS2, Keil ARMCC 5.
- Layers: BSP transports bytes; Device handles motor protocols; algorithm contains
  controllers/planners; APP binds devices to controllers; Chassis contains wheel kinematics.
- Swerve scope: four independently steered/driven modules; pure C99 calculations.
- Swerve coordinates: +x forward, +y left, +wz counterclockwise; SI units.
- Swerve module order: front left, front right, rear left, rear right.
- Motor initialization, feedback calibration, PID updates and CAN transmission stay
  outside the swerve kinematics library.
- Swerve steering ranges: per-module [-90, 90] degrees by default, [0, 180], or
  explicit continuous mode. Limited modes prioritize mechanical bounds over a
  <=90-degree steering move. Actual/held angles must be in the selected range.
- Swerve verification (2026-10-26): 2670 native checks passed with and without -O2,
  ARMCC 5 strict compile passed, latest Keil build reported 0 errors/0 warnings.
  No board run.
- Chassis/Inc/Chassis_Conf.h selects one wheel system (swerve by default) and holds
  per-wheel settings. Chassis_Init loads g_swerve_modules before RTOS startup;
  mecanum/omni branches are reserved. Unknown geometry/radius macros stay zero.
- Prefer direct configuration macros and straightforward embedded C. Avoid
  per-parameter override scaffolding and duplicated validation helpers.
- Chassis init verification: all 6 configuration build cases passed, default and
  asymmetric override each passed 123 checks, ARMCC strict compile and Keil
  build/link passed (0 errors/0 warnings). No hardware calibration performed.
- ChassisApp now binds the four swerve modules to eight motors and owns steering
  cascades / drive speed PIDs. ChassisTask reuses CubeMX Chassis_Task at High2,
  1536-byte stack, nominal 1 ms with measured dt. MotorApp owns the five CAN3 axes.
- DjiMotor_Init registers all 13 motors once in main before CAN / RTOS startup.
  Receive timestamps and 50 ms online checks use HAL milliseconds, not RTOS ticks.
- Chassis application requires positive geometry / wheel-speed limit and real
  encoder calibration. These macros and all new PID gains intentionally remain
  zero. ChassisApp_SetVelocity uses m/s, m/s, rad/s; no command timeout is added.
- ChassisApp verification (2026-10-27): 367 native checks passed at default
  optimization and -O2; 67 task timing/reserved checks and 668 motor checks passed.
  Existing solver/config/calibration suites passed; Keil ARMCC5 build/link reported
  0 errors/0 warnings. No flashing, board run or physical calibration performed.
- Front step mode (2026-10-28): ChassisTask now cycles UpdateFrontMove, using
  independent FL/FR Cascade_t + Traj_t and holding all steering targets; rear
  drive commands stay zero. Ordinary velocity Update remains a separate API.
- Front travel macros are signed motor-rotor turns before reduction, converted
  once to output rad. Travel and front PID/planner settings remain zero for the
  user to fill. Front encoder-position mode does not need wheel geometry.
- Upstairs integration is deliberately not inserted in My_task.c: user chooses
  the call site. RequestFrontMove posts START; PollReply consumes DONE/ERROR
  without blocking. DONE requires both real positions/speeds settled after both
  planners finish. Busy/late requests do not restart motion; faults abort it.
- Front-step verification (2026-10-28): 473 application checks passed at default
  optimization and -O2, including an ideal encoder plant with actual planner
  completion. 542 task/timing/notification checks and 668 motor checks passed.
  Keil ARMCC5 build/link: 0 errors/0 warnings. No board run or calibration.
- Bare-metal current-test startup fix (2026-10-28): BSP_USART_Init creates
  FreeRTOS semaphores. The local RVDS port retains BASEPRI before scheduler
  start, blocking TIM23/HAL_Delay and CAN RX interrupts if RTOS stays disabled.
  UART BSP initialization now belongs to Communicate_Task, not main. Keep
  bare-metal motor tests free of RTOS object creation. Startup regression
  reproduced the blocked delay with the port side effect modeled; fixed
  startup/UART task tests, 40 packet, 34 encoder and 668 DJI checks passed.
  Keil ARMCC5 build/link: 0 errors/0 warnings. No board run performed.
- CAN current-test investigation (2026-10-28): user reports the startup fix
  still does not turn the motor. A separate F103 board with
  C:/Users/86181/Desktop/ZET6-C610/ZET6-C610Plus drives it; this is not an
  on-board verification of H723. Reference packet layout matches our helper;
  its checked-in main currently sends ID2/1000 while H723 sends CAN1 ID1/2000.
  F103 is disconnected/stopped during H723 tests. H723 board oscillator and
  connector mapping are not confirmed. User reports HSI/PLL clock-source
  experiment also did not turn the motor; main and .ioc were restored to the
  original HSE configuration. Temporary HSI characterization tests removed.
  The motor-not-turning root cause is still unconfirmed. Do not claim the
  startup fix or clock experiment solved actual motor operation.
- Follow-up CAN investigation (2026-10-28): user confirmed CAN1 TXD->PA12,
  RXD->PA11. Native execution of real Motor_Test + BSP + STM32 HAL verified
  0x200 classic DLC8 message RAM bytes and TXBAR request for ID1/2000; this
  does not prove physical transmission. main.c changed during investigation:
  RTOS startup was restored and raw-current loop removed. A temporary CAN1-only
  startup mode was added, then replaced at the user's next request (below).
  Physical motion remains unconfirmed; do not call this an established
  root-cause fix.
- Latest current test (2026-10-28): user corrected CAN2 pins to PB5 RX/PB6 TX
  (AF9), CAN3 to PD12 RX/PD13 TX (AF5), in fdcan.c and .ioc; preserve them.
  Main again initializes all three CAN buses and registers all 13 motors.
  MOTOR_CURRENT_TEST_ONLY=1 now sends CAN2 ID2/2000 every 10 ms using unchanged
  Test_MotorCurrent before starting RTOS. Set 0 for normal RTOS startup.
  This is a temporary open-loop test: permanent right-front registration stays
  CAN2 ID4 and CAN2 feedback filters stay at the normal chassis IDs.
  Both startup paths/UART task checks, 40 packet and 34 calibration checks pass;
  ARMCC5 build/link 0 errors/0 warnings. No motor motion verified by the agent.
- CAN1 hardware evidence (user Watch screenshot, 2026-10-28): init HAL_OK,
  send HAL_ERROR, loop count 0x217e, PSR 0x7f7 with BusOff=1, ECR 0xf8,
  TXBTO=0; hfdcan1.ErrorCode=0x200 is HAL_FDCAN_ERROR_FIFO_FULL.
  Bus-Off / blocked transmission is confirmed, but the initiating bus error is
  not identified. User's live test call currently sends CAN1 ID2/2000.
- BSP now enables protocol-error and Bus-Off interrupts for all three buses.
  g_can_diagnostics[0..2] preserves first/last valid nominal LEC (1..6),
  Bus-Off entry count and fault PSR/ECR snapshots. No-change LEC=7 and LEC=0
  do not erase prior errors. On Bus-Off entry, request cancellation of pending
  Tx buffers before clearing CCCR.INIT to let hardware perform recovery;
  an exit notification must not restart recovery or cancel new traffic.
  Native tests exercise the real BSP with mapped STM32 register layouts and
  modeled HAL/hardware effects, all three buses, error retention and post-
  recovery submission. Recovery, 40 packet and 34 calibration tests pass;
  ARMCC5 build/link 0 errors/0 warnings. Recovery has not yet been verified on
  the board. HAL ErrorCode is cumulative; use PSR.BusOff and fresh send status
  to assess current state rather than expecting old ErrorCode bits to vanish.
- Follow-up board Watch evidence (2026-10-28): CAN1 RX count increased to
  0x3f2e, but send count and Bus-Off entry count both reached 0x7e1. First and
  last retained nominal LEC are 5 (Bit0 error); ErrorCode is 0x08000000
  (HAL_FDCAN_ERROR_PROTOCOL_ARBT). PSR=0x708 / ECR=0 / BusOff=0 are snapshots
  between repeated failures and recoveries, not evidence of successful Tx.
  TXBTO snapshot remains zero. PA12 runtime MODER=2, AFRH=9, OTYPER=0 confirms
  alternate-function AF9 push-pull configuration. Recovery is acting on the
  board, but the initiating transmit fault remains unresolved. User identifies
  the board as custom STM32H723ZET6 with TJA1050T-JSM transceiver. User's CAN1
  schematic shows U12 VCC pin 3 on 5V, GND pin 2 grounded, C14 100nF supply
  decoupling, S pin 8 pulled down through R21 10k (not MCU-controlled),
  CANH/CANL through L2 ACT45B and R35 120-ohm across the connector-side pair.
  No software S-pin initialization is needed for this design. Actual supply
  and S-pin voltages, MCU pad supply and transmit waveforms remain unmeasured;
  do not equate the schematic or GPIO configuration with verified board wiring.
- Board replacement result (user report, 2026-10-28): replacing the controller
  made CAN1 work, and the user has tested all M3508 motors. The failing board's
  exact faulty component/path remains unidentified; do not continue treating
  the previous Bus-Off snapshots as the state of the replacement board.
- GM6020 open-loop test: Test/Inc/Motor_Test.h and Test/Src/Motor_Test.c now
  expose Test_GM6020Voltage(handle, id, voltage), for the already-confirmed
  voltage mode. ID1..4 -> 0x1ff, ID5..7 -> 0x2ff, big-endian signed command,
  other slots zero. Caller supplies raw voltage in +/-30000. Existing
  Test_MotorCurrent remains the C620/C610 helper. main's bare-metal test now
  calls GM6020 CAN1 ID1 / 2000 every 10 ms; RTOS remains disabled for testing.
  Fresh verification: 40 current + 9 literal GM6020 voltage packet cases,
  34 encoder checks, both startup paths/UART task, CAN recovery and 668 DJI
  checks passed. ARMCC5 Keil build/link: 0 errors/0 warnings. GM6020 board
  motion has not yet been verified; no flashing performed by the agent.
- Upstairs debug start (2026-10-28): user has set MOTOR_CURRENT_TEST_ONLY=0
  for RTOS operation; preserve their raw-test target edits. My_task exports
  volatile uint8_t g_upstairs_start, initially 0. Watch value 1 releases the
  sequence once. Waiting still updates feedback/controllers; later writing 0
  is not a pause/reset. First and second stair rotor-turn arrays stay in
  Task/Src/My_task.c, ordered CAN3 ID5/ID2/ID1. Existing first two values are
  22.9218927 / 34.382839; ID1 remains 0 pending user input. The legacy 2006
  steps remain after each 3508 group; per-axis issued bits prevent reissuing
  an already accepted move. Stage transitions reset the group and eventually
  reach the reserved third stage instead of looping second stage forever.
- User clarified MotorApp_MoveTurns inputs are PRE-reduction rotor turns,
  including 2006. Convert to output radians exactly once inside the APP API.
  Default independent 3508 planner limits were converted consistently; PID
  gains/speed limits remain zero, and 2006 independent planner limits remain
  zero, for the user to tune. Do not claim physical holding/motion with zero
  gains. MotorApp_Move3508TurnsSync uses one normalized 0..1 Traj for all
  three upper 3508 references (task defaults rate/accel 0.12/0.12), independent
  of their travel lengths. Preflight all feedback before committing; any
  active-group feedback loss aborts the group and latches the task fault.
- AXIS_IDLE after a commanded move now requires planner completion PLUS
  actual error <=0.01 rotor turns and actual speed <=10 rotor rpm continuously
  for 50ms; macros are in MotorApp.h. Position control continues holding the
  final reference after arrival. Fresh O0/O2 tests cover rotor units, signed
  and zero travel, shared reference completion, stationary/fast actual motor
  not falsely completing, settle reset, atomic start, dropouts, debug gating,
  no repeated moves, and both stair stages. 668 DJI, 473 chassis application
  and 542 chassis task/notification checks also passed. Final ARMCC5 build
  linked with 0 errors/0 warnings. No board run, gain tuning or flashing.
- User readability preference: keep frist_stairs and second_stairs as separate,
  explicit case blocks. Do not stack labels into a shared branch or obscure
  the step sequence with extraction. Each case now contains its own rotor-turn
  array and actions, so the user edits a stair locally. Debug start, shared
  motion planning and completion semantics are unchanged. O0/O2 upstairs
  regression tests and fresh ARMCC5 build/link passed (0 errors/0 warnings).
- Latest user edits: first stair rotor turns are now +22.9218927 / -34.382839 /
  -22.9218927 (CAN3 ID5/ID2/ID1), and second_stairs is commented out. Preserve
  these edits. 3508 PID values are now pos_kp=0.6, spd_kp=8, spd_ki=12, but
  spd_integral_max remains 0 (also on 2006), so integral action is clamped out.
  ID5 pos_speed_limit was still 0, definitively suppressing its speed command;
  changed only that limit to 2.0 output rad/s (planned peak ~0.90). Other
  user gains/limits were not retuned. A native probe of the real controllers
  with stationary feedback reproduced pre-fix raw commands 0/-14/-9 at 1s
  and 0/-53/-35 at 10s; post-fix ID5 produces ~9.36 at the same 1s reference.
  These are modeled outputs, not board measurements or proof of motion.
  Actual three cmd values and the tuning unit origin (rotor rpm vs output
  rad/s) were requested before adjusting gains. First-step/motion tests pass;
  the existing two-stage flow case has 3 assertion failures because it still
  expects the user's now-disabled second stage. Do not restore their second
  stage merely to satisfy that test. Latest Keil build: 0 errors/0 warnings.
- User confirmed the old 3508 speed gains were tuned in rotor rpm, then
  requested a direct initial parameter set for about 2s travel. First-group
  shared progress now uses rate=2/3 and accel=4/3: theoretical 0.5s acceleration,
  1s cruise, 0.5s deceleration. The real discrete Traj measured 2.032s at 1ms
  in native tests; this is reference completion, not measured board arrival.
  Current first-stage signed rotor turns and commented second case preserved.
- Three upper 3508 trial controllers now use position P=4 (1/s), speed
  Kp=1467.0171 (8 rotor-rpm gain times 183.3771), continuous Ki=2200.5256,
  and integral accumulator cap=2000/Ki (integral command contribution +/-2000).
  Ki is a NEW continuous-PI trial setting, not an equivalent transfer of old
  discrete Ki=12 with unconfirmed source/control period. Output speed caps
  are 6/9/6 rad/s for CAN3 ID5/ID2/ID1, above planned peaks ~5/7.5/5 rad/s.
  Existing protocol output caps retained. 2006 gains were not converted.
- PID.c continuous/discrete gain-conversion COMMENTS were reversed and now
  corrected: Ki_cont=Ki_disc/Ts; Kd_cont=Kd_disc*Ts. No PID algorithm change.
  New behavior tests reproduce a 130 rotor-rpm error as 1040 raw P output,
  check no clipping of the 2s cruise speeds, and execute the task's actual
  rate/accel through Traj. Focused upper-motion and wait/start/fault task cases
  passed at O0/O2 (TaskCases parameter selects cases; default still includes
  the two-stage flow case). 668 DJI and 473 chassis checks passed. Keil final
  build/link: 0 errors/0 warnings. No flashing, board arrival-time validation
  or measured-load tuning; these values are initial trial parameters.
- First stair is now explicitly three sequential groups inside its own case:
  group 0 moves 3508 CAN3 ID1/ID2 with ID5 held (array order stays ID5/ID2/ID1,
  values 0/-22.9218927/-34.382839 rotor turns, preserving latest user edits).
  Group 1 has a separate three-3508 array, all zero placeholders for the user.
  Group 2 has a two-2006 array ordered CAN3 ID3/ID4, retaining 85.94366926
  rotor turns each. ID3 maps to APP axis4; ID4 maps to axis3. Each target is
  relative to its own step start. Zero second-step travel means hold and
  continue after settling, not an unconfigured-step interlock. After the
  third group settles, enter the user's still-commented second_stairs case.
- ID1's latest first-step travel is the longer 34.382839 rotor turns, so its
  2s reference peak is ~7.5 output rad/s. Its position speed cap increased
  from 6 to 9 rad/s; an actual Cascade test first reproduced clipping at 6,
  then passed at 9. Other trial PID values retained. Three-step task tests
  now cover both 3508 groups before 2006, correct ID3/ID4 binding, no repeat
  commands, staggered completion, late 2006 feedback and faults in either
  3508 group. The former flow test now tests the complete FIRST stair's three
  groups, not the disabled second stair. All upstairs native cases passed
  at O0/O2. Latest ARMCC5 build/link: 0 errors/0 warnings. No board run.
- Keil visibility fix: task locals/static axes were inconvenient to watch.
  Task/Inc/My_task.h now declares the global volatile g_upstairs_debug
  (68 bytes, linked in the Keil map). Task updates it while waiting and after
  each state-machine iteration. Fields: updates heartbeat, task state,
  1-based step, issued/started, and three axes in CAN3 ID5/ID2/ID1 order.
  Each axis exposes motor ID, state, planner_active, raw command, rotor rpm,
  target-minus-actual rotor-turn error, settled milliseconds and RX count.
  It is an observation copy; writing it does not control the task.
- Latest concurrent user edits have commented FIRST-stair steps 2 and 3 as
  well as second_stairs. Retained their step2 array values in comments
  (+22.9218927/-22.9218927/-22.9218927). Fixed only brace pairing around the
  disabled code; did not re-enable motion. Thus first-step completion reaches
  debug.step=2, debug.issued=0 and holds, with no second-step motion currently.
  Focused wait/fault/first_only tests validate exported debug values, rotor
  conversion, IDs, heartbeat and state transitions at O0/O2. Full three-step
  tests require re-enabling user-reserved actions. Fresh Keil build/link:
  0 errors/0 warnings; no flashing or board verification performed.
- Paired 2006 control (2026-10-29, supersedes independent third-step motion):
  first-stair steps 2/3 are enabled by the user. Third step now requests one
  MotorApp_Move2006Pair(75.19518086f), positive common travel means ID3 negative
  and ID4 positive. Both feedbacks must be ready before either starts. Pair
  owns its common trajectory, per-wheel cascades, limited cross coupling and
  coordinated holding; single-axis APIs cannot interrupt it. Reference virtual
  time slows with slower-wheel tracking lag; FF uses the same clock scale.
  Current simultaneous errors/speeds/sync tolerance must settle50ms to DONE;
  no stale per-axis done latch. Disturbance while holding re-evaluates arrival.
  Offline/excess skew/20s timeout latch pairFAULT with both commands zero.
  MOTORPAIR_CFG_2006 in PID_Config.c holds trial group trajectory/sync settings;
  per-axis PID gains retained. See docs/embedded/specs/2026-10-29-2006-pair-control.md.
  Watch g_upstairs_debug.pair_2006 (state0idle/1moving/2done/3fault); axis[3]=ID4,
  axis[4]=ID3 include feedback current/counts. main raw-test flag restored to0
  so RTOS runs; raw current-test call edits preserved. Upper7 + pair11 + task7
  cases passed at O0/O2; full RTOS ARMCC5 build/link0errors0warnings. No flashing,
  board verification or physical tuning. Equal wheel radius/ratio assumed.
- Third-step halfway action (2026-10-29): when BOTH 2006 actual normalized
  rotor travels reach half their common target, request CAN3 ID5 axis0 to move
  -35 rotor turns relative to its trigger position using its existing single-axis
  planner/PID. Reference-only or one-side crossing does not trigger; accept once,
  retry rejected requests, and suppress a new action if pair already faulted.
  Step3 waits for pairDONE and ID5 idle. issued3=pair accepted, issued7=ID5 also
  accepted. Watch id5_half_started / move_id5_result expose acceptance/results.
  O0/O2 upper7+pair12+task8 cases passed, including concurrent pair/ID5 updates,
  exact-half trigger, no repeat after recrossing, retry and waiting for ID5.
  ARMCC5 full RTOS build/link0errors0warnings. No flashing/board verification.
- ID5 trigger updated (2026-10-30): threshold is now BOTH 2006 actual travels
  reaching75%, without waiting for pairDONE. ID5 and pair run concurrently;
  step completion still waits for both pairDONE and ID5 idle, in either order.
  Preserved concurrent user edit of ID5 relative travel from -35 to -55 rotor
  turns. Old Watch id5_half_started name retained for compatibility. Task tests
  cover75% boundary, one-side-only crossing, retry, no repeat and both finish
  orders; all existing upper/pair/task cases pass O0/O2. Keil0errors0warnings.
  No flashing or physical validation.
- ID5 lift speed (2026-10-30): AXIS_CFG_3508_1 single-axis trajectory v/a
  increased from about0.9/0.9 to6/6 output rad/s and rad/s². Actual Traj native
  execution for -55 rotor turns measured reference duration21.038s ->4.044s.
  This is reference timing only; user PID gains and15rad/s position speed cap
  retained. 75% trigger unchanged. O0/O2 suites passed; Keil0errors0warnings.
  No flashing or board validation.
- 2006 load-start trial (2026-10-30): user reports about1000 raw command is
  needed to start. Stationary-feedback real-code probe showed old speed PI12/6
  gives only98 at0.5s. Both 2006 speed loops now trial Kp300/Ki150 (outputrad/s),
  integral accumulator cap3000/150=20 (I contribution±3000), output cap10000.
  This is a new trial, not an inferred equivalent conversion of unknown RPM
  gains. No fixed current floor. Same probe reaches±1000 at0.226s,±2450 at0.5s;
  this is modeled command buildup, not measured physical startup. Upper/pair/
  task suites passO0/O2, Keil0errors0warnings. No flashing or load validation.
- Offline investigation (2026-10-30): new board screenshot pair.state3/fault1
  means feedback loss, unlike previous timeout3; ID5 request was already accepted.
  Reproduced an IsOnline software race: original HAL_GetTick()-volatile rx_tick
  may read now, then CAN ISR refreshes rx_tick1ms newer, unsigned subtraction
  underflows and falsely reports offline. Fixed by snapshotting rx_tick BEFORE
  reading HAL time. Native injected-ISR and wraparound cases failed before and
  pass after;50ms timeout retained. Not proven as cause of this board event.
  Pair status now latches fault_motor_id, fault_rx_age_ms[2], fault_rx_count[2],
  fault_command[2] (array orderID3/ID4) before stopping; recovery retains evidence.
  Pair tests cover either wheel offline, command snapshot and retention after
  recovery.672 DJI checks, upper/pair/taskO0/O2 and full ARMCC5 link pass0errors/
  0warnings. Unit-conversion test uses fixed known gain rather than asserting
  user tunable ID5 gain still equals old1467.0171. User gain edits retained.
  No flashing or physical verification.
- Split final 2006 travel (2026-10-30): first leg now user-configured50.19518086
  rotor turns; KEEP both-actual75% trigger for ID5 (current user value-35 turns).
  When first pair legDONE AND ID5 idle, enter group3/debug.step4 and request
  another common+20 rotor turns relative to the new start. Only second legDONE
  sets g_upstairs_debug.first_stair_done=1 and enters reserved second_stairs.
  Both legs keep shared governor/sync/fault semantics. O0/O2 tests cover two
  pair requests exactly once,75% trigger, both finish orders, retries and final
  leg faults/not falsely completing; CAN recovery/49packets/34calibration pass.
  Full ARMCC5 compile/link0errors0warnings; no flashing or hardware validation.
- CAN timing evidence (2026-10-30): continuous-run screenshots alternate offline
  ID3/4 with both feedback ages50/51ms. CAN3 error records1Stuff/3ACK, PSR0x77b,
  ECR0x880, noBusOff; records may be historical. Added diagnostic
  error_record_count/last_error_tick_ms and pair fault_tick_ms to correlate
  effective error snapshots with stopping. No CAN timeout/gain changes in this
  step. Actual bus/power/receive failure root cause remains unconfirmed.
- Pair timeout changed at user request (2026-10-30): MOTORPAIR_CFG_2006.timeout_s
  is now100s instead of20s, each leg starts its own timer. Arrival, sync and
  50ms feedback timeout unchanged. Native timeout tests derive duration from
  configuration; O0/O2 upstairs suites pass, Keil0errors0warnings. No board run.
- Simplified pair arrival (2026-10-30, user rejects long3->4 wait): preserve
  user MOTORAPP_POSITION_TOLERANCE_ROTOR_TURNS=0.1 and SETTLE_TIME_S=0. PairDONE
  now requires only common planner ended and BOTH actual endpoint errors within
  position tolerance. Remove speed gate and separate sync_done_rotor_turns
  config; cross coupling and sync/feedback/timeout fault handling remain active.
  Handoff may occur with residual wheel speed. Single-axis/3508 speed checks
  unchanged. ID5 arrival still required at task level. Preserve user's current
  extra travel35 (previously40/20). Tests adapt to tunable position/timing and
  travel; new position_finish case proves speed no longer delays pair arrival.
  Old tests must not restore50ms or strict0.04 configuration just to pass them.
  Latest O0/O2 upper/pair/task suites pass. Board behavior not yet verified.
- Further simplification (2026-10-30, user explicitly requests removing pair
  protections after observed30s step3->4 delay): supersedes previous arrival
  and governor notes. Pair reference advances with actualdt, time_scale=1.
  Removed follow_limit_rotor_turns, sync_fault_rotor_turns and timeout_s config
  and their governor/sync-abort/timeout logic. No extra terminal speed cap:
  sync_speed_max limits correction ONLY, axis speed/current limits still apply.
  Both actual endpoint errors within0.1 rotor turns immediately snap reference
  to endpoint and reportDONE, regardless of planner/speed/settle timer.
  Step3 also checks ID5 actual target error directly, bypassing its state,
  planner/speed/settle gates; ID5 controller continues independently. Existing
  3508 first/second step semantics unchanged. Preserve75% trigger,50.19518086
  first pair travel,-35ID5,35extra, and user config30/30/60,axis speed50.
  Feedback offline50ms/zero+latch and protocol current bounds remain. Without
  motion timeout, a blocked motor keeps trying; do not claim timed completion.
  Added early-position, terminal-speed, normal-clock, no-timeout, sync-reporting
  and ID5-position regressions. O0/O2 upper/pair/task tests pass; no board run.
- Per-step timing (2026-10-30): g_upstairs_debug.timing now exposes step1_ms,
  step2_ms, step3_ms, step4_ms and total_ms for the first stair's four steps.
  Timing starts when g_upstairs_start releases the task, includes request retries
  and arrival waits, and excludes initial standby. Each step updates live then
  locks at transition; all timing freezes on completion or task fault. HAL
  unsigned millisecond subtraction handles tick wrap. Existing elapsed_ms keeps
  its original step3+step4 meaning; use total_ms for all four steps. Motion
  parameters and transition conditions are unchanged. O0/O2 regressions verify
  live/completed/fault-frozen intervals, sum of steps, standby and tick wrap.
- 2006 phase profiling (2026-10-30): MotorApp_2006Timing_t is observational only,
  embedded in pair status. HAL ms phase buckets accel/cruise/decel (reference
  velocity) and follow (reference inactive, actual not jointly arrived) sum to
  total_ms. Captures planner exit, per-ID3/ID4 first0.01rotor motion, both75%,
  first endpoint tolerance entry, and per-motor <=10rpm/outside-tolerance time.
  Milestones are relative to accepted request; UINT32_MAX means not yet seen.
  Freeze on FIRST jointDONE or offline fault, even if subsequent holding moves
  again; new accepted request resets current profile. Task pair_timing[0/1]
  preserves step3/4 separately before the second request overwrites current
  status. timing.step3_after_pair_ms measures first joint arrival ->step4,
  including ID5 waits/renewed disturbance. Profiles do not alter motion logic.
  O0/O2 tests cover stationary then delayed actual arrival, signed reverse,
  acceleration/cruise/deceleration/follow sum, HAL wrap, fault freeze, new-move
  reset, and preservation of both task snapshots. No board measurements yet.
- Any-side completion (2026-10-30, explicit user change): supersedes all prior
  BOTH-position arrival and ID5-wait rules. Either ID3 OR ID4 within0.1 rotor
  turns ends each pair leg. Both axes rebase hold targets to their real current
  positions, clear cascaded integrals/old speed commands and disable cross
  coupling. Encoder feedback and original-goal timing are not fabricated.
  DONE stays latched despite later disturbances; hold each axis independently,
  next pair request captures fresh actual origins. Timing finish_motor_id gives
  the trigger ID; losing first_in_tol_ms staysUINT32_MAX unless actually reached.
  Task advances step3->4 once pairDONE and ID5 request accepted, without waiting
  for ID5 arrival. User explicitly confirms: if either2006 arrives before both
  reach75%, still triggerID5 at leg end. Accepted moves never repeat; transient
  request failures retry before advancing. Zero pair travel keeps no-ID5 policy.
  Step4 also ends on either side; first_stair_done can be1 while ID5 is moving.
  Added all winner/direction tests plus ID5 fallback/retry and immediate handoff.
  O0/O2 upper/pair/task regressions pass. No board verification or flashing.
- Faster3508 reference (2026-10-30, user requests within1s): latest user correction
  sets ID5 triggered travel to-35rotor turns; preserve it, do not restore-20.
  Shared task progress
  rate=5/3 and accel=25/3 replace2/3,4/3: theoretical0.8s, realTraj at1ms=0.820s
  (previous2.032s). Each35rotor travel peaks~19.1outputrad/s, within existing30
  speed caps. ID5 single traj_v_max=20, traj_a_max=100 replace30/30; realTraj
  +/-20turn reference0.554s vs0.971s, +/-35turn0.799s vs1.278s, peaks20rad/s.
  Faster ID5 comes from increased acceleration despite the lower cruise cap.
  User position gains24/24/18 and speed PI/integral settings are untouched.
  Add lift_fast regressions for both travel magnitudes/signs, speed headroom,
  and actual motion not falsely completed by reference end. Actual task macros
  tested via realTraj for0.8s reference. O0/O2 suites and Keil0errors0warnings
  pass. No board run; do not promise actual1s arrival without measured feedback.
- Step2 wait diagnosis (2026-10-30): user observes lift visually arrived before
  third step. Step2 still requires ALL three3508 IDLE (ID5+35,ID2hold,ID1-35),
  planner inactive, <=0.1rotor error and <=10rotorrpm; settle0. No specific motor
  or gate is proven from total-duration screenshot. Added observation-only
  g_upstairs_debug.step2_axis[0/1/2] for ID5/ID2/ID1: mutually exclusive priority
  planner_ms/position_ms/speed_ms/settle_ms while MOVING, done_ms relative to step2
  entry, plus last_reason/error_turns/speed_rpm/current command at final blocked
  sample. Counters start after accepted request and freeze per-axis on IDLE;
  task fault also stops sampling. Highest done_ms identifies last arrival.
  No controller/config/transition changes. Task regression covers distinct
  planner/position/speed waits and preserved results after later steps, including
  HAL wrap. O0/O2 full upper/pair/task suites pass; no board verification yet.
- Second stair enabled (2026-10-30, user reports first stair working): replaces
  earlier commented-second-stairs notes. second_stairs has its own explicit
  three-case switch. Only group0 implemented: CAN3 ID5+35, ID2hold0, ID1-35,
  relative PRE-reduction rotor turns. Shared rate5/3,accel25/3 unchanged. Original
  3508 actual position/speed arrival applies to all3; success goes to group1
  placeholder and holds indefinitely. group1/group2 reserved, never auto-skipped
  or armed with zero target actions. Prior first-stair ID5 motion must finish
  before atomic shared API accepts this new group; BUSY/NOFB retry while normal
  controls continue. No extra2006 moves and no third-stair transition yet.
  Watch state1/step1 executing, state1/step2/issued0 means first step finished.
  New second_stair debug step1_start_ms/step1_ms/move_result/status captures
  entry through completion (includes retry), freezes on completion/fault; all
  first-stair timing/profile snapshots stay preserved. User first-stair extra
  pair travel is now42turns, preserve it; ID5 triggered travel-35 preserved.
  O0/O2 upper/pair/task cases pass including second preflight retry, earlier-ID5
  busy wait, staggered completion, fault and post-completion holding. No flash
  or physical second-stair test performed.
- Complete second stair (2026-10-30): supersedes previous reserved group1/2.
  Own explicit cases: group0 ID5+35/ID2hold/ID1-35; group1 pair+50.19518086
  rotor turns and ID5-35 when BOTH actual sides>=75%, with accepted user's
  fallback trigger if either reaches endpoint first. No repeated lift requests;
  transient request failures retry. PairDONE + ID5 request accepted advances
  directly, without waiting for lift arrival. group2 pair+42 rotor turns from
  each motor's fresh actual origin. Both pair legs use ANY-side completion and
  independent rebased holding in MotorApp. End goes to third_stairs reserved
  state2/group0, controls continue, no physical third-stair moves implemented.
  second_stair.status now means whole stair (1active/2complete/3fault), not only
  step1. Added step2/3/total timing, per-stair ID5 request metadata and pair
  phase histories; old first-stair observations stay intact. Separate full
  second-stair mock fixture covers flow, wrap, pair retry,75%/fallback/retry,
  pair/extra/lift faults, pending extra leg and no restart after completion.
  Existing task fixture stops at second step1 boundary; full O0/O2 suites pass.
  No flashing or physical second-stair verification by the agent.
