# Fourth final front leg, ID2 parallel action

Current override: plan a stop at40%, wait for actual ID2 arrival, then resume
the same original endpoint. See 2026-10-31-fourth-forty-percent-wait.md. The
35%-to75% unpaused estimates below are historical baseline measurements; there
is no longer a deadline requiring the chassis to keep moving while ID2 catches up.

- Delete the commented duplicate fifth_stairs branch, enum, transition and
  debug block. Keep earlier stair motion targets and all default PID settings.
- Fourth final front travel remains FL+17.402660 / FR-17.4026606 raw rotor
  turns. BOTH signed actual wheel travels reaching35% latches the ID2 trigger.
  One wheel, reference travel, or the prior completed leg cannot trigger it.
- Request upper CAN3 ID2 +7.6812 raw rotor turns once; retry rejected requests
  even after recrossing below35%. Armed40% pause suppresses finalDONE until
  the matching resume command; there is no early-DONE trigger fallback.
- APP accepted-move sequence identifies the new leg. ChassisTask publishes a
  complete wheel-pair/sequence snapshot under the scheduler lock AFTER all APP
  updates/start processing and BEFORE replies. Getter uses the same lock, so an
  Upstairs preemption mid-APP update cannot expose a mixed-cycle threshold pair.
- Apply PID_Config.h limits20 rad/s,150 rad/s2 only AFTER this ID2 request is
  accepted. Save its original limits and restore when its reference ends or
  stops, including faults. Axis default limits30/30 and PID gains remain intact.
- Real Traj at1ms: front35%=582ms,75%=962ms, window380ms; old ID2 reference609ms,
  new reference281ms, margin99ms. This estimates reference timing only. Actual
  load, tracking and scheduling can consume the margin; no physical guarantee.
- Log front35_ms, front75_ms (first wheel), id2_trigger_ms, id2_done_ms relative
  to front request acceptance. UINT32_MAX means not yet seen. id2_before75=1
  only if first real ID2 arrival precedes first75%; deadline_missed reports a
  miss. Current40% pause makes these historical timing fields observational;
  no fabricated arrival or new timeout is added.
- Latch frontDONE locally because PollReply consumes it. Fourth completes only
  when frontDONE AND current ID2 IDLE/inactive planner/live position+speed pass.

Verification: absent linkage failed4flow checks first.57 async scenarios pass
O0/O2, including stale prior leg, delayed publication, each-one-side35%, recross
retry, one-shotDONE, both finish orders, late/missed75 metrics, earlyDONE fallback,
fault/reject/wait and restored limits. Mid-APP preemption regression failed15
checks with direct reads;715 notification checks pass O0/O2 after snapshot fix.
5006 front-pair checks pass each normal/reversed drive configuration O0/O2;
387 steering checks, task65/84/4 and upper/pair/second/standby/first-handoff suites
pass. No flashing or physical timing validation performed.
