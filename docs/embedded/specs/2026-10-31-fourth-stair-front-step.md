# Fourth stair, steps 1-5

Current user selection (2026-11-01): stairs1/2/3/4 are enabled. Both temporary
fourth-level #if 0 blocks (locals and case) are removed. All configured travel,
including the final ID1 return, is retained. RR GM6020 remains released.

- Enter only after the third stair's final concurrent ID5/ID1 move actually
  arrives at both targets. Keep earlier motion targets and PID settings.
- Front left CAN1 ID1: +16.2453059 motor-rotor turns; front right CAN2 ID4:
  -16.2453059 motor-rotor turns (user tuning, preserved). Targets are relative to this request's actual
  starting positions, before gear reduction.
- Reuse ChassisApp_MoveFrontPairTurns: one common trajectory, two cascaded
  controllers, bounded synchronization correction, and the existing any-side
  arrival rule. On completion both wheels hold their current positions.
- Keep the third stair's default travel at +/-12.2969213 rotor turns. A new
  ChassisTask_RequestFrontMoveTurns API carries the fourth stair's own travel.
  Publish the pair and START together under the scheduler lock; reserve until
  the completion/error reply is successfully sent. Reject busy requests without
  replacing the queued or active travel. Capture a task-private copy for retries.
- Step 2: one Move3508TurnsSync request, CAN3 ID5/ID2/ID1 = +36.5/0/-36.5
  rotor turns, using the existing shared progress rate/acceleration. Wait for
  BOTH requested motors: IDLE, inactive planner, live endpoint error and speed
  within the existing arrival tolerances. Stale IDLE alone cannot advance.
- Step 3: request Move2006Pair(+50.19518086 rotor turns), using its existing
  shared trajectory, signs and any-side completion rule. No ID5 trigger added.
- Step 4: after pair DONE, one Move3508TurnsSync request moves upper CAN3
  ID5/ID2/ID1 = -36.5/0/+36.5 rotor turns relative to the actual start positions.
  ID5 and ID1 share the existing progress trajectory; ID2 adds no travel. Wait
  for BOTH ID5 and ID1: IDLE, inactive planner and live endpoint error/speed
  within the existing limits; then request front left +19.402660
  and front right -19.4026606 rotor turns (current user settings). Reuse the front pair controller and
  any-side completion. During this final front leg, BOTH actual wheel travels
  reaching35% triggers upper CAN3 ID2 +31.6812 rotor turns concurrently. The
  front reference stops at40%, then holds until actual ID2 arrival before
  resuming the SAME original endpoint. See the40%-wait spec.75% is only observed.
  All actions remain in group 3 / Watch step 4: issued0 request lift pair,
  1 wait BOTH, 2 request front pair, 3 wait frontDONE AND live ID2 arrival.
- Step 5: only after step4 frontDONE AND live ID2 arrival, CAN3 ID1 (APPaxis2)
  moves +70.930979 PRE-reduction rotor turns relative to its actual start.
  Accept once/retry temporary request failures; permanent errors or active
  faults abort. Capture its original goal separately. Completion requires
  IDLE, inactive planner, actual original-goal error within0.1rotor turns and
  speed within10rotorrpm; a retargeted half-way hold cannot fabricate arrival.
  Keep existing PID/planner settings. Configured net ID1 travel is now0.
- Keep fault state value 3 and fourth_stairs value 4. Accepted moves are not
  repeated; transient errors retry and permanent/active faults latch failure.
  Final ID1 arrival marks fourth.status2, then hold at group5/Watchstep6
  reserved. Watchstep5 is now the ID1 return, not an independent fifth stair.
  The duplicate fifth stage remains removed. Step5 timing is included in total.
- Watch g_upstairs_debug.fourth_stair for request_result, reply,
  lift_move_result, pair_move_result, id1_move_result, id5_move_result, front_final_request_result,
  front_final_reply, step1/2/3/4_ms, total_ms and status
  (0 waiting, 1 running, 2 implemented sequence done, 3 error). Timing includes
  retries and waits, freezing on each transition, completion or fault.

Concurrent ID5 update verification: the former ID1-only implementation failed
five new flow checks. 59 async scenarios pass O0/O2, including ID5-late/stale-IDLE/
fault, ID1-late/stale/speed/planner/fault, atomic acceptance, retry/reject and
unchanged subsequent front/fifth flow. Prior first handoff, upper, paired2006,
second-stage and standby regressions pass O0/O2. No flashing or board run.

Initial step-4 verification: missing implementation failed five new flow checks first.
45 async task scenarios pass at O0/O2, including ID1 pending/stale-IDLE position,
active planner/high speed, retries/rejections/faults, front waiting/error reply,
single acceptance and final timing freeze. Upper, 2006 pair, first handoff and
second-stair regressions pass at O0/O2. Second ID2 test reads the current user's
local travel constant rather than requiring an obsolete tuned value. Fresh
ARMCC5 build/link: 0 errors, 0 warnings. No flashing or physical validation.

Extension verification: old code failed the new flow's four checks before the
extension. All 34 third/fourth scenarios now pass at O0/O2, covering both lift
arrivals, stale-IDLE actual drift, request retries/rejections, faults, delayed
2006 completion, timing sum/freeze and no repeated moves. SHA256 comparison
confirms the three earlier stair case blocks, PID and Chassis configuration
are unchanged. Fresh ARMCC5 build/link: 0 errors, 0 warnings. No flashing or
physical fourth-stair validation.

Initial step-1 verification: 24 third/fourth task scenarios passed at O0/O2; chassis tests
passed 65 hold, 84 timing, 674 notification/publication and 4 reserved checks at
both optimization levels. First-stair handoff, upper motion, paired 2006 and
second-stair regressions also passed at O0/O2. Fresh Keil ARMCC5 build/link:
0 errors, 0 warnings. No flashing or physical fourth-stair validation.
