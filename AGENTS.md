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
- Second-stair wait diagnostics (2026-10-30): second_stair.step1_axis[3]
  records ID5/ID2/ID1 waiting in the lift step. Uses the existing first-stair
  step2 gate classifier, with separate buffers/start clocks; each axis freezes
  at first IDLE. last_reason: 1planner/2position/3speed/4settle/5fault;
  done_ms=0 means not yet complete. Actual second-stair entry still requires
  all three upper axes IDLE, including ID2 holding. pair_request_count[0/1]
  and pair_move_result[0/1] distinguish unrequested legs from rejected/accepted
  second-stair 2006 commands; result starts at1, OK0, errors follow MotorApp.
  No changes to travel, PID, completion gates or motor control. O0/O2 tests
  cover lift planner/position/speed blocking and persistent NOFB requests,
  alongside prior full motion/task cases. ARMCC5 build/link0errors0warnings.
  User's reported stop is not yet diagnosed on hardware; need current Watch
  state/step, second_stair records and live axes/pair status for accepted moves.
- Second-stair fourth step (2026-10-30): after its extra2006 pair+42 rotor
  turns reports DONE, enter explicit group3/Watch step4 and command CAN3 ID2
  M3508 (APP axis1) -35 rotor turns from its own current position. No repeated
  accepted request. Use existing transient retry and PARAM/FAULT handling;
  wait for this axis IDLE before setting second_stair.status2 and entering
  third_stairs. The third step now freezes its duration and starts step4,
  rather than declaring stair completion. Added second_stair.step4_start_ms,
  step4_ms and id2_move_result (initial1, OK0); total_ms includes all four
  steps. Existing first-stair actions and PID/planner parameters preserved.
  O0/O2 full motion/task tests pass, including ID2 busy retry, pending arrival,
  request/active faults, direction/binding, no repeat, timing and HAL wrap.
  Keil ARMCC5 build/link0errors0warnings. No flashing or board verification.
- Second-stair fifth step (2026-10-30): latest user edit sets step4 ID2
  travel to -22.8 rotor turns; preserve it. ID2 IDLE now freezes step4 timing
  and advances to explicit group4/Watch step5, commanding CAN3 ID1 M3508
  (APP axis2) +35 rotor turns from its current actual origin. Only ID1 IDLE
  completes second_stair/status2 and enters the reserved third_stairs state.
  Existing accepted-once, transient retry and PARAM/FAULT patterns retained.
  Added step5_start_ms, step5_ms, id1_move_result (initial1/OK0) to second_stair;
  total_ms includes all five steps. Watch axis[2] for this ID1 motion. O0/O2
  regression cases cover ID1 busy/pending/request-fault/active-fault, correct
  order and sign, no repeat, five-step timing and HAL wrap. Original motion/
  task cases pass; ARMCC5 build/link0errors0warnings. No flashing or board run.
- First-stair wait-for-lift handoff (2026-10-30): supersedes the old
  request-accepted-only handoff for the selected ACTIVE first-stair group2.
  Require pair DONE and accepted ID5 request plus axis0 IDLE before group3;
  zero pair travel retains its no-lift exception. ID5 faults stop progression.
  User now configures 60% trigger, ID5 -36.5 rotor turns and extra pair45;
  preserve these. User has commented out the whole second_stairs case; do
  not re-enable it. A void-use keeps its reserved clock warning-free without
  changing the commented code. Existing Watch axis[0].state shows lift
  readiness; step3_after_pair_ms now includes lift-arrival waiting. New
  standalone first-stair handoff script runs7 cases at O0/O2 (14passed),
  covering either completion order, MOVING despite position arrival, stall,
  fault, fallback, retry and clock wrap. Old full-stair fixtures still assume
  enabled second-stair actions and old no-wait behavior; they are not a pass
  claim for this current user-disabled configuration. Keil ARMCC5 build/link
  0errors0warnings. No flashing or physical verification by the agent.
- ID2-to-ID1 current-arrival gate (2026-10-30): user has RE-ENABLED the
  second_stairs case. Latest second targets: +36/0/-36, ID5-36, pair extra45,
  ID2-22.8, ID1+36 rotor turns; preserve these user edits. Original group3
  checked only latched AXIS_IDLE. Real MotorApp tests confirm an IDLE axis
  remains IDLE after0.5rotor-turn drift/30rpm and outputs holding correction;
  task regressions reproduced premature ID1 acceptance for stale-IDLE inputs.
  This is a proven software edge case, not a confirmed board root cause.
  Group3 now saves its original accepted ID2 target privately and requires
  IDLE plus inactive planner, current original-goal error<=0.1rotor turns and
  speed<=10rotor rpm before group4. Existing tolerances/PIDs are unchanged.
  New second_stair.id2_actual_turns/id2_error_turns/id2_speed_rpm update in
  group3 and freeze at handoff; id2_handoff_done1 and step5_start_ms identify
  that snapshot. Debug writes do not affect the private target/controller.
  Distinguish actual stage5 command (step5/id1_move_result0) from ID1 holding
  motion while still step4/result1; user was asked for these board values.
  Native upper/pair/23second-task cases pass O0/O2 with TaskCases=wait, plus
  standalone first-handoff14runs; old first full-task fixture is not claimed
  passing because it retains obsolete targets/handoff expectations. Keil
  ARMCC5 build/link0errors0warnings. No flash or board-run verification.
- FIFO0 drain fix (2026-10-30): latest user snapshot confirms pairFAULT3 /
  faultOFFLINE1 / sourceID4 at HAL9391ms, ID3 rx age1ms vs ID4 age51ms.
  Pre-fault raw commands ID3-2787/ID4+5256; leg duration5702ms. This proves
  an ID4 timestamp gap, not a damaged motor or a proven physical cause.
  Found original BSP HAL_FDCAN_RxFifo0Callback consumed only one queued frame
  although local HAL clears RF0N before callback. Now drain FIFO0 to empty on
  NEW_MESSAGE or MESSAGE_LOST, return on read failure, avoid empty reads.
  Enable lost-message notification and add per-bus rx_fifo0_lost_count and
  last_rx_loss_tick_ms; the count is IRQ observations, not exact lost frames.
  Existing CAN3 FIFO8, 50ms offline threshold, protocol history, Bus-Off
  recovery, pins/timing, stage targets and PIDs retained. Native real-BSP
  queue regression reproduced24pre-fix failures across3buses; O0/O2 now pass
  queued bursts, lost/mixed events, empty FIFO, unrelated flags and read error
  exit, alongside previous recovery tests. 672DJI checks pass; ARMCC5 build/
  link0errors0warnings. No flash or board run; the software queue defect is
  fixed but whether it caused this51ms gap requires fresh board evidence.
  Compare CAN3 diag[2] loss/error event times with new pair fault timestamps.
- Full feedback-gap observation (2026-10-30): user reports BOTH2006 live
  rx_count values keep increasing after fault; CAN3 loss/error diagnostics0.
  This establishes resumed feedback, while the group/task fault stays latched;
  the triggering age51ms does NOT give the full interruption duration. Added
  per-DjiMotor pure diagnostics rx_max_gap_ms, rx_long_gap_count,
  rx_last_long_gap_ms and rx_resume_tick. First frame is excluded; valid
  recovery frames close the interval and increment count only if>50ms;
  later normal frames preserve the last long-gap record. Times are Device
  dispatch HAL timestamps, not direct wire timestamps. Registration clears
  all4fields. No change to offline threshold, fault latch, motor commands or
  stair actions. Watch ID4 g_motor_2006_1 vs ID3 g_motor_2006_2 after leaving
  program running through feedback recovery; correlate resume/fault times.
  692checks pass at O0 and O2 (original672 +20gap checks), including 50/51ms,
  retained history, first-frame ignore, independent nodes, invalid frames and
  uint32wrap. Keil ARMCC5 build/link0errors0warnings. No flashing or board run;
  underlying source of the short feedback outage remains unconfirmed.
- Online-snapshot preemption race (2026-10-30): new screenshots show ID4
  maxgap2ms/longgapcount0 and active command3887, but do not simultaneously
  show pair state/fault. Asked whether same faulted boot and for same-time
  group values; do not assert the pictures prove that earlier board fault
  was false or that these were true51ms wire outages. Earlier timestamp-gap
  interpretation was too strong: IsOnline loaded rx_tick before reading now;
  task preemption60ms with perfectly continuous1ms CAN updates caused old
  snapshot to return OFFLINE although actual maxgap1ms. Reproduced before
  fixing. Now read rx_tick/now and retry if rx_tick changed during sampling;
  avoids stale-age and prior unsigned-underflow races without IRQ masking or
  threshold change. Pair-fault age snapshot uses the same consistent read.
  Genuine source stop55ms still fails, continuous updates through uint32wrap
  pass. Each O0/O2 runs705DJI checks; upper/pair/23second cases with TaskCases
  wait and first-handoff14runs pass. User now sets ID2 step4=-23.8 (not-22.8);
  preserved, fixture expected value updated. ARMCC5 build/link0errors0warnings.
  No flash/board run. Re-download/reset for testing; do not label this proven
  resolution of the user's physical symptom before fresh board evidence.
- Fresh post-fix board evidence (2026-10-30): user explicitly labels new run.
  ID4 maxgap86ms/4long gaps/last61ms/resume13382ms; ID3 maxgap125ms/
  1long gap/resume22948ms. Both commands0. PairFAULT3/OFFLINE1/source4
  at12209ms; fault age ID3=0ms, ID4=51ms. CAN3 firstLEC1Stuff, lastLEC2Form,
  error snapshots6 (not exact bad-frame count), busoff0, FIFO-loss0. Last
  protocol error22961ms is13ms after ID3 recovery but10752ms AFTER first
  pair fault; do not claim it directly caused that earlier event. ID4's last
  recovery is1173ms after fault, so not necessarily the triggering interval.
  Actual Device dispatch gaps now established, beyond prior false-online
  race; wire/node versus MCU receipt root cause still unconfirmed. Current
  FIRST extra travel is user-edited42rotor turns (not45), preserved. Pair
  FAULT branch prevents DONE transition to second stair. No new runtime
  changes made for this analysis. Next discriminating evidence is CAN3
  0x203/0x204 traffic/waveform and node ID uniqueness/power/link conditions;
  do not enlarge FIFO or relax threshold as an established root-cause fix.
- Authorized timeout trial (2026-10-31): user explicitly asks to increase
  the time because feedback later resumes. ONLY canonical DJI_MODEL_2006
  online threshold is now200ms (DJI_MOTOR_2006_TIMEOUT_MS in DjiMotor.c);
  3508/GM6020 retain50ms via DJI_MOTOR_TIMEOUT_MS. Keep long-gap diagnostics
  counting>50ms, so86/125ms gaps remain observable despite the trial. At
  exactly200ms online,201ms offline; no-feedback startup still offline and
  fault latch still requires reset. No root-cause or hardware fix claimed.
  User has AGAIN commented out entire second_stairs; preserve this. Local
  reserved second_id2_target_rad is now unused, so full native task compile
  with-Werror fails in that user-disabled configuration. Added MotionOnly
  test selection for upper/pair suites without changing task code. Each O0/O2
  passes715DJI checks and upper/pair motion including two125ms gap-recovery
  cases and201ms true-fault zero/latch cases. No changes to PID/travel/FIFO.
  Test strategy documents that stale feedback continues being used within
  the widened window. New board test still required; no flashing performed.
  Latest header-triggered Keil compile/link:0errors/1warning (#177 unused
  second_id2_target_rad from user-commented second case). Task source left
  untouched; this is not an error in the timeout trial. Fresh binary produced.
- Front steering zero hold (2026-10-31): user explicitly chooses ChassisApp /
  ChassisTask ownership, separate from upstairs_task, for FL CAN1 GM6020 ID1
  and FR CAN2 GM6020 ID4. ChassisTask runs calibrated mechanical zero hold
  immediately at nominal 1ms, before/independent of g_upstairs_start. New
  CHASSIS_TASK_FRONT_MOVE_ENABLE defaults0 (hold only);1 retains legacy front
  drive/notification mode. Hold-only move requests return osErrorResource.
  ChassisApp_UpdateFrontSteerHold uses existing g_chassis_wheels[0/1].steer_pid
  and g_swerve_modules[0/1], sharing extracted absolute-angle readback with the
  old APP flow. No geometry or other six motors' feedback needed. Per-axis
  offline/invalid range clears output and resets control, while healthy side
  continues; recovery targets calibrated zero rather than startup heading.
  CAN1/2 transmit only0x1ff; unused rear-steer slots stay0. Upper CAN3/task,
  travel, pins, 2006 trial timeout and kinematics untouched. FL/FR .steer in
  CHASSIS_PID_CFG now trial posKp6, speedKp1500, speed cap1rad/s, I/D0,
  voltage cap30000. These are initial P/P trial values, not measured tuning.
  FL/FR ZERO_OFFSET still0 and must be filled with actual calibration counts.
  New real-task/APP/Device/PID regression failed14checks before implementation,
  then379checks each O0/O2 pass, covering two-node-only startup, absolute wrap,
  reversed installation, zero hold, disturbance, loss/recovery, bounds, packets,
  voltage clipping and invaliddt. Legacy APP467+reserved6 checks each O0/O2
  pass; task hold65/legacy84/notifications454/reserved4 pass. Upper/pair motion
  O0/O2 regressions pass. Keil ARMCC5 compile/link0errors0warnings; fresh HEX
  produced. No board run, flashing or physical holding claim.
- Front steer tuning follow-up (2026-10-31): user changes FL/FR position speed
  caps to25rad/s (not rpm). At posKp6, a90deg error requests9.425rad/s, so
  cap25 is inactive throughout the default +/-90deg return-to-zero range.
  Authorized FL/FR speed-loop voltage command caps now24000 (80% of protocol
  range30000); Device model cap remains30000. All gains retained. Ideal
  perfect-velocity-following model gives95% error decay0.499s,98%0.652s and
  90deg->1deg0.750s; these are conditional estimates, not board timings.
  Front hold regression380checks each O0/O2 pass, including24000 braking cap
  and separate30000 protocol clamp. Fresh Keil0errors0warnings. No flashing.
- Third stair front-drive pair (2026-10-31): user requests FL CAN1 M3508 ID1
  +12.2969213 and FR CAN2 M3508 ID4 -12.2969213 PRE-reduction rotor turns.
  User explicitly confirms ANY-side completion like current2006: either
  original goal error<=0.1rotor turns ends leg, both rebase actual positions,
  clear prior controller state/cross coupling, latchDONE while independently
  holding. No planner/speed/settle/timeout gate. ChassisApp adds distinct
  UpdateFrontPair/MoveFrontPairTurns; legacy independent FrontMove APIs remain.
  Raw motor signs are independent of swerve encoder-direction coordinates;
  staged endpoints divide by ratio once and are validated before either start.
  One common positive-output-rad Traj, per-wheel cascades, normalized bounded
  cross coupling and final speed slew. Motion reference cannot reverse solely
  from coupling. Only four front nodes needed, no rear feedback/geometry.
  Front GM6020 zero hold continues independently; rear old commands cleared
  before0x1ff/0x200 submission. Active or DONE-hold front feedback loss zeros
  drives and latchesFAULT; healthy steering continues. Pair fault status and
  sourceID remain visible after feedback recovery; no automatic restart.
  CHASSIS_TASK_FRONT_MOVE_ENABLE now1. Task still1ms with actualdt; START/DONE/
  ERROR handshake retained but uses new pair APIs. Third_stairs explicitgroup0
  sends request once after second completes, retries rejected flag publication,
  waits reply, errors latch taskfault; success group1/debugstep2 is reserved.
  g_upstairs_debug.third_stair exposes status/request/reply/timing;
  g_chassis_front_pair exposes signed actual/error rotor turns, normalized
  sync error, common reference, signed rpm commands, planner and finish/faultID.
  Config travel/tolerance in Chassis_Conf; CHASSIS_FRONT_MOVE_CFG[2].cascade
  trial posP18, speedPI1467.0171/2200.5256, Icommandcap4000, speedcap12rad/s;
  MOTORPAIR_CFG_CHASSIS_FRONT v6/a12/syncP2/syncmax1/commandA60. Per-axis
  trajv/a stay0 (only old independent API uses them); new pair uses groupcfg.
  User current calibration FL4628/FR5657/RL1495/RR6690, front steer gains
  pos16/2 and speed3000/100, voltage24000 and Device6020cap25000 preserved.
  First extra2006 travel now38.5; second lift+/-36.5, ID5-36.5, extra40,
  ID2-24.2, ID1+36.5; no preceding stair motion values changed by this feature.
  Main RTOS flag0 and user's inactive raw encoder-calibration branch preserved.
  New real upstairs third-step fixture failed3assertions before implementation,
  flow/retry/rejection/error cases passO0/O2 after. Pair tests4737checks each
  default/reversed direction atO0/O2 (18948total), covering genuine wrapped CAN
  feedback, atomic/preflight, signs, shared refs/slew/sync, either-side rebase,
  no120s/reference-only false completion, dropout/status retention, stale rear
  slots, bounds and numeric staging. Legacy APP467+reserved6 checks eachO0/O2,
  hold380eachO0/O2, task65/84/notifications454/reserved4, first-handoff14runs,
  upper9/pair26/second-task23 and firstwait atO0/O2 pass. Legacy fixtures use
  synthetic zero offsets/local P gains so user measured calibration/PI tuning
  isn't overwritten. Full old first-stair task case selection still contains
  obsolete configuration expectations; use isolated handoff/newthird plus
  run_upstairs_tests -TaskCases wait for current supported regression set.
  Two-stage read-only review findings (finite endpoint staging and fault status)
  resolved and rechecked. Fresh ARMCC5 Keil compile/link0errors0warnings.
  No flash, board run or physical tuning; trial values do not promise arrival.
- First-stair-only selection (2026-10-31): user explicitly requests commenting
  out both later stairs. Task/Src/My_task.c wraps complete second_stairs and
  third_stairs case blocks in #if0, preserving all source inside. First-stair
  actions/targets/conditions unchanged; its existing transition reaches now-
  disabled second state and default branch keeps updating/holding without new
  later-stage commands. Void-use preserves reserved ID2 target variable without
  unused warnings. ChassisTask startup front drive/steer holding remains active;
  no change to motor ownership, PID, CAN or notifications. Do not re-enable later
  stairs to satisfy full-stage tests. Existing first handoff7cases passO0/O2;
  fresh Keil ARMCC5 compile/link0errors0warnings. No flashing performed.
- Fix premature front3508 holding (2026-10-31): user reports chassis wheels
  resist climbing and clarifies only front6020 should control before third-
  stair request. Root defect: UpdateFrontPair used Cascade_Update in initial
  CHASSIS_FRONT_HOLD as well as DONE, braking externally rolled wheels before
  permission. New real-feedback regression produced100failing assertions on
  old code. Initial HOLD now bypasses BOTH drive PIDs, forces both3508 cmd0,
  speed-command metadata0, and still updates encoder feedback/readiness.
  Successful MoveFrontPairTurns alone starts drive control from latest actual
  origins; latched DONE still holds after an accepted move. Front6020 zero
  control, drive gains/targets, upper axes and stair steps unchanged. Both
  later stairs remain #if0, so this first-only selection never arms chassis
  driving. Keep mode1 notification capability; do not restore startup drive
  holding. Pair suite5002checks each default/reversed atO0/O2 (20008total)
  passes, including external wheel travel/300,-400rpm producing zero CAN
  current before request, no integral accumulation, steer control still active,
  rejected requests not arming, and fresh-start relative12.2969213rotor targets.
  First handoff14runs, task65/84+notifications454+reserved4, steer hold380each
  O0/O2 also pass. Fresh ARMCC5 compile/link0errors0warnings. No flashing or
  board verification; user must download fresh firmware to test this correction.
- Re-enable later stairs (2026-10-31): user now explicitly asks to remove the
  temporary comment. Removed only #if0/#endif around second_stairs/third_stairs
  and restored default-branch explanatory comment. All travel/PID/calibration
  and first-stair actions unchanged. Crucial prior fix retained: front3508
  commands remain0 and both drive PIDs inactive until accepted third-stair
  request; front6020 zero hold starts at power-on. Second/third task fixtures,
  upper/pair motion and selected firstwait passO0/O2. Fresh Keil ARMCC5 build/
  link0errors0warnings; no flashing performed.
- Third-stair lift and2006 follow-up (2026-10-31): after chassis DONE,
  explicit third group1/Watchstep2 calls existing upper3508 sync API once with
  ID5/ID2/ID1 rotor-turn array +36.5/0/-36.5; ID2 no added travel. Requests
  BUSY/NOFB retry without partial starts; PARAM/FAULT locks task fault. Wait
  only active ID5/ID1: IDLE plus current target error/speed within existing
  tolerances and planner inactive, avoiding stale-IDLE handoff after drift.
  Then group2/Watchstep3 calls Move2006Pair(+50.19518086rotor turns) once;
  existing any-side arrival applies. No75%/other ID5 action in this leg. DONE
  enters group3/Watchstep4 reserved, keeps holding. First/second stair cases,
  all gains/calibration and front-drive no-PID-before-request fix untouched.
  Third debug adds step2/3 timestamps/durations,total,lift/pair results;status2
  now means implemented3-step sequence complete, not merely chassis step done.
  Async task fixture first failed4checks before implementation,13cases each
  O0/O2 pass including staggered/wait either3508, stale-IDLE actual drift,
  lift/pair request retries/rejections/faults, pair waiting, no repeats and
  timing sum/freeze. First handoff14runs and upper/pair/second23 regressions
  pass. Fresh Keil ARMCC5 compile/link0errors0warnings. No board run or flash.
- Third stair final ID1 step (2026-10-31): user says existing motion now works
  and asks one last action. After third group2 dual2006 DONE, group3/Watchstep4
  commands CAN3 ID1 upper3508 (APPaxis2) +36.5 rotor turns relative to actual
  position via MoveTurns. Accepted once, transient request retry, permanent or
  active fault stops sequence. Only ID1 IDLE now marks third.status2 and
  enters group4/Watchstep5 holding;2006 DONE alone no longer marks completion.
  Third debug adds step4_start_ms/step4_ms/id1_move_result, total includes all4.
  No prior stage motion/PID/Chassis/Device changes. New absence test failed2
  checks first; expanded17task cases passO0/O2 including final retry, rejection,
  active fault, pending actual arrival, no repeats and four-step timing sum.
  First handoff14runs, upper/pair and second23 casesO0/O2 also pass. Fresh
  ARMCC5 Keil compile/link0errors0warnings. No flash or board testing by agent.
- Parallel third-stair final lift (2026-10-31): user adds CAN3 ID5 -36.5
  rotor turns to the end, then immediately specifies it can run simultaneously
  with final CAN3 ID1 +36.5. Supersedes ID1-only final step; no serial fifth
  production step installed. Thirdgroup3/Watchstep4 now calls existing
  Move3508TurnsSync once with ID5/ID2/ID1=-36.5/0/+36.5, same shared progress
  rate/accel, ID2 no added travel. Wait BOTH axes0/2 IDLE, inactive planner,
  live endpoint error and speed within existing tolerances. Old latched IDLE
  cannot finish if an axis has drifted. Any relevant axis fault stops flow.
  Only both arrived set third.status2/group4/Watchstep5 holding. Existing
  id1_move_result now records group acceptance and new id5_move_result records
  the same result; fourth-step timing covers concurrent moves. Earlier stages,
  other third steps, PID and chassis startup drive-current gate unchanged.
  Aborted-turn serial test edits replaced by final-sync expectations. Old
  ID1-only code failed4checks;20async task cases passO0/O2, including atomic
  final request/directions, either-side waiting/fault, stale-IDLE position,
  retry/rejection, no repeats and four-step timing. First handoff14runs,
  upper/pair motion and second23cases passO0/O2. No flashing or board run.
- Fourth stair step1 (2026-10-31): after third final BOTH upper3508 arrivals,
  enter fourth_stairs (enum4, fault remains3). Explicit group0 requests front
  CAN1 ID1 +13.8070695 and CAN2 ID4 -13.8070695 PRE-reduction rotor turns.
  Reuses existing shared front-pair planner/controllers/any-side completion,
  then holds at fourth group1/Watchstep2 reserved. Third defaults remain
  +/-12.2969213; earlier stair targets/PIDs and startup drive-zero gate retained.
  RequestFrontMoveTurns publishes travel+START under scheduler lock, rejects
  busy without overwrite, preserves private travel through readiness retries,
  releases reservation only after successful reply. Noarg API keeps defaults.
  fourth_stair debug records request/reply/status/step1 time.24flow scenarios
  and65/84/674/4chassis checks passO0/O2, plus first handoff/upper/pair/second
  regressions. Fresh Keil ARMCC5 link0errors0warnings. No flash or board run.
- Fourth stair steps2/3 (2026-10-31): after front pair DONE, group1 requests
  upper CAN3 ID5/ID2/ID1=+36.5/0/-36.5 rotor turns in one shared3508 move.
  BOTH requested axes0/2 must be IDLE with inactive planner, live endpoint
  error and speed inside existing tolerances before group2 requests dual2006
  +50.19518086 rotor turns. Existing pair any-sideDONE ends this implemented
  sequence at group3/Watchstep4 reserved, fourth.status2. No lift trigger is
  added during this pair move. Retries precede acceptance; never repeat accepted
  moves; permanent/active faults stop flow. fourth debug adds lift/pair results,
  step2/3 time and total, frozen on completion/fault. Preserve user fourth front
  travel +/-15.2453059 and second ID2 -23.8 tuning (old test expectations updated).
  SHA256 confirms earlier three case blocks and PID/Chassis configs unchanged.
  New flow failed4checks first;34third/fourth scenariosO0/O2 and first handoff,
  upper9/pair26/second23/wait regressionsO0/O2 pass. Fresh ARMCC5 Keil link
  0errors0warnings. No flashing or physical fourth-stair verification.
- Fourth stair step4 (2026-10-31): after group2 pair DONE, group3 first calls
  MoveTurns(axis2/CAN3 ID1,+36.5 rotor turns). Wait for live IDLE+inactive
  planner+endpoint position/speed within existing limits; only then request
  front FL+17.402660 / FR-17.4026606 rotor turns through the existing task pair
  API. Keep both actions in Watchstep4: issued0 requestID1,1 waitID1,2 request
  front,3 waitreply. Final frontDONE alone sets fourth.status2 and group4/
  Watchstep5 reserved; group2 DONE no longer marks fourth complete. Retries
  cannot repeat accepted moves; faults stop further requests. Added fourth
  step4 duration, ID1 result and separate final-front request/reply fields.
  No controller/PID/earlier stage motion changes. User second ID2 is now-24.0;
  tests read that local constant instead of imposing the previous-23.8 value.
  New absent-flow failed5checks,45async scenariosO0/O2 and first handoff/upper/
  pair/second/wait regressions pass. Fresh Keil ARMCC5 link0errors0warnings.
  No flashing or board verification performed.
- Fourth stair temporary pause (2026-10-31): user asks to comment out its
  last three steps. Only fourth_stairs cases1/2/3 are enclosed in #if0;
  original code/targets remain intact. Fourth step1 stays active, then group1/
  Watchstep2 enters default holding without new moves. Prior stair cases and
  PID settings unchanged. The full-flow test expects disabled actions and is
  not applicable while this temporary block remains disabled. No flashing.
- Corrected temporary-disable scope (2026-10-31): user meant stair levels2/3/4,
  not fourth-level steps2/3/4. Removed that inner disable and placed one outer
  #if0 around entire second_stairs through fourth_stairs cases. First stair
  actions remain active; its normal handoff reaches default holding with no
  later-stage requests. All later code/targets retained. Full-flow tests that
  require later stages are inapplicable while disabled. No flashing.
- Restored full stair flow (2026-10-31): user asks to remove the disable.
  Removed the outer #if0/#endif around levels2/3/4; all four implemented levels
  are active again. Existing targets, PID settings and completion gates retained.
  No flashing performed.
- Four-steering zero hold (2026-10-31): user adds rear6020 holding. ChassisTask
  mode0 now calls UpdateSteerHold for all4; front-pair mode also runs4steer
  cascades. Rear RL CAN1ID2 / RR CAN2ID3 use existing ZERO_OFFSET/directions.
  Rear steer gains copy front trial16/2,3000/100, speed25, voltage cap24000;
  front/upper/drive gains and calibration macros retained. Shared private loop
  returns front-only status for pair readiness/fault checks while steering-only
  mode reports all4 aggregate. Rear faults stop only affected rear; no new front
  travel dependencies. Old UpdateFrontSteerHold retains front-only API behavior.
  Rear drive commands stay0, front drive gate before accepted request unchanged.
  Both0x1ffframes now carry rear steer slots too, no extra traffic/CAN3 ownership.
  New tests first failed5checks, then387holdchecksO0/O2 and5004pairchecks each
  normal/reversed directionsO0/O2 pass;task65/84/674/4 bothopts pass. Fresh Keil
  ARMCC5 link0errors0warnings. No flashing or physical rear tuning performed.
- Fifth stair steps1/2 (2026-10-31): after fourth final frontDONE, record fourth
  completion and enter fifth_stairs(enum5;fault remains3). Own group0 requests
  front CAN1ID1 +15.2453059 / CAN2ID4 -15.2453059 PRE-reduction rotor turns,
  using existing front-pair any-side completion; wait its DONE reply. Group1
  then MoveTurns(APPaxis1, upper CAN3ID2,+7.6812 rotor turns). Wait liveIDLE,
  inactive planner and endpoint/speed tolerances before fifth.status2/group2/
  Watchstep3 reserved. Retries precede acceptance; no accepted move repeats;
  errors/faults latch. New fifth debug results/reply/two-step/total timing freezes
  on completion/fault. Earlier targets/PIDs/four-steer holds unchanged; fourth
  only changes its next-stage transition. Six checks failed before implementation,
  then56async scenariosO0/O2 plus first handoff/upper/pair/second/wait regressions
  pass. Fresh Keil ARMCC5 link0errors0warnings. No flashing or physical test.
- Fourth final simultaneous ID5/ID1 update (2026-10-31): user adds ID5-36.5
  after fourth dual2006DONE, concurrently with existing ID1+36.5, then next
  action. Replaced fourth group3 single MoveTurns with one Move3508TurnsSync
  ID5/ID2/ID1=-36.5/0/+36.5, same progress limits. Both axes0/2 must be IDLE,
  planner inactive and current endpoint/speed within tolerance before issued2
  permits original front17.402660/-17.4026606 move. Either axis fault aborts;
  accepted group never reissued. New fourth.id5_move_result matches shared
  id1 result. Original bottom request/reply, fifth transition, other targets,
  PID and four-steer holds unchanged. Former code failed5checks first;59async
  scenariosO0/O2 pass including ID5 late/stale-IDLE/fault and prior ID1 gates,
  plus first handoff/upper/pair/second/wait regressions. No flashing/board test.
- Fourth35% ID2 linkage replaces duplicate fifth stair (2026-10-31): user had
  commented fifthcase; removed its case/enum/debug/transition. Fourth final
  FL+17.402660/FR-17.4026606 now triggers CAN3ID2+7.6812 raw rotor turns when
  BOTH signed ACTUAL travels reach35%; latch/retry/acceptonce. APP move_sequence
  distinguishes prior15.245 progress. Task publishes a complete pair+sequence
  snapshot under scheduler lock after APP update/start and before reply; getter
  reads under same lock. Mid-APP preemption test failed15checks with directreads,
  then715notificationchecksO0/O2 pass. Early frontDONE supplements untriggered
  ID2 to avoiddeadlock, with missed75 recorded. Latch consumedDONE; wait BOTH
  frontDONE and current ID2 idle/inactive/liveendpoint/speed before fourthdone/
  group4reserved. PID_Config.h per-action20rad/s,150rad/s2 applied only after
  acceptedID2request, originals restored when reference inactive/faultstopped;
  defaultAxis2 PID/30/30 and earlier targets unchanged. RealTraj1ms: front35=
  582ms,75=962ms/window380ms, oldID2=609ms,new=281ms/margin99ms (reference only).
  fourth debug front35/75/ID2trigger/done timestamps, normalizedactuals,
  id2_before75 and deadline_missed expose actual result; no autopause/timeout.
  57async scenariosO0/O2,5006frontpair eachdirection/opt,387steer,task65/84/715/4
  and upper/pair/second/wait/firsthandoff suites pass. No flashing/physical test.
- Fourth final40%-wait scheme (2026-10-31): user explicitly replaces timing-only
  method and cancels further rootcause investigation. Keep BOTH actual35%
  triggerID2+7.6812, but request frontWithPause(...,0.4). APP preplans braking
  to40%, retains original full endpoints/origins/sequence, holds position and
  suppresses DONE while armed. paused requires common reference inactive, BOTH
  actual stop errors within existing tolerance and BOTHspeeds<=0.05outputrad/s;
  disturbance clearspaused. Upstairs captures original ID2goal, validates live
  arrival against it, then posts matchingsequence RESUME once/retry. Task API
  accepts resume only when coherent publishedpaused; owner recheckspaused and
  continues SAME trajectory40->100. No newfull relativeleg, no fifthstage, no
  defaultPID/travel changes. APP paused/pause_ms/resume_ms and fourthresume
  result/originalID2error expose status. Old75% timing is observational only.
  58async casesO0/O2 pass inclhold40, retargeted-halfgoalnotaccepted,resumeretry;
  5032APPchecks eachdirection/O0/O2 incl1600cyclehold, armednoDONE, originalend,
  negative travel/disturbance/dropout;Task65/84/789/4,387steer and upper/pair/
  second/wait/handoff regressionspass. Fresh ARMCC5 link0errors0warnings.
  No flashing or physical verification performed.
- Fourth final ID2 travel update (2026-10-31): requested CAN3ID2 move is now
  +31.6812 rotor turns instead of+7.6812. Earlier ID2 travels-7.6812/-24 give
  configured net0 (absolute63.3624). Retain35% trigger,40% stop/wait/resume,
  captured actual endpoint checks and existing PID/limits. CAN3ID1 configured
  stage1=-34.430979-36.5; stages2/3/4 each-36.5+36.5, hence net-70.930979
  and absolute289.930979 rotor turns. Updated existing async request assertion
  failed once with old target, then58casesO0/O2 and upper/pair regressions
  passed. RealTraj new ID2 reference674ms with current per-action limits,
  not physical arrival time. No flashing or board verification performed.
- Final upper ID1 return (2026-10-31): fourth group4/Watchstep5 now requests
  CAN3 ID1(axis2)+70.930979 rotor turns, only after final frontDONE AND live
  ID2arrival. Actual start is captured by MoveTurns; save original target for
  live arrival (IDLE/inactive planner/position0.1rotor/speed10rpm). Acceptonce,
  transient requests retry, faults latch. Fourth.status2 now occurs AFTER this
  move, group5/Watchstep6 is reserved holding. New fourth step5_start_ms/ms,
  id1_return_move_result/error_turns expose timing/result/original-goal error;
  total includes the new step. No PID/controller changes. Preserve user's
  current final front+19.402660/-19.4026606, ID2+31.6812 and40%-wait flow.
  ID1 configured net becomes0 after this relative return. Test fake now derives
  signed final front targets from accepted request, preserving user tuning.
  Missing-step regression failed4checks, then66async cases eachO0/O2 pass,
  including returnretry/reject/wait/stale/speed/planner/fault/goalreset;72upper/
  pair runs also pass. Fresh ARMCC5 link0errors0warnings. No flash/board test.
- First-stair-only and RR release (2026-11-01): user requests disabling stair2
  onward AND releasing right-rear position hold. One outer#if0 wraps entire
  second/third/fourth cases; related fourth-only locals also#if0 to keep compile
  clean. All retained targets/parameters/ID1final return remain in source.
  First stair and its completion flag are unchanged; normal handoff selects
  disabled second case and falls into default holding, without later moves.
  Full later-stage async tests are inapplicable until the block is restored.
  Current ChassisTask steer-only and paired-front paths use zero-hold count3
  (FL/FR/RL), so RR GM6020 CAN2ID3 skips cascade and helper forces its command
  and CAN voltage slot0 each cycle. Other3 holds/calibration/PIDs retained;
  rear3508 commands remain0. Existing legacy/general movement APIs unchanged.
  RR-zero regression failed59checks first, then444holdchecksO0/O2 and7first
  handoff cases eachO0/O2 passed;5032pair checks eachnormal/reversed direction
  eachO0/O2 pass, including overwriting stale RRcommand while fronts move.
  Fresh Keil ARMCC5 link0errors0warnings. No flashing/board verification.
- Restore through third stair (2026-11-01): user requests third enabled while
  retaining fourth disabled. Move outer#if0 from second_stairs to fourth_stairs,
  enabling second as required by sequential first->second->third flow. Fourth
  case and its exclusive local variables remain#if0, with all targets retained.
  Third completion enters disabled fourth/default holding without new requests.
  Remove obsolete unused-variable casts now second locals are used. No motion
  target/PID/steering edits; RR GM6020 remains released in current task modes.
  120upper/pair/standby/second task runs and14first-handoff runs passedO0/O2.
  Fresh ARMCC5 link0errors0warnings. No flashing/physical verification.
- Restore fourth stair (2026-11-01): user requests enabling fourth as well.
  Remove both fourth-only#if0 blocks (local variables and entire case); all
  stairs1/2/3/4 run sequentially again. Preserve current fourth initial front
  +/-16.2453059, finalfront+19.402660/-19.4026606, ID2+31.6812 and final
  ID1+70.930979; completion remains fourth.status2 AFTER final ID1 arrival.
  RR GM6020 remains released in current ChassisTask modes. Async fixture now
  accepts user-tunable signed fourth initial targets and publishes their real
  requested values for old-sequence progress. All66async cases eachO0/O2 pass;
  fresh Keil ARMCC5 link0errors0warnings. No flashing/board verification.
