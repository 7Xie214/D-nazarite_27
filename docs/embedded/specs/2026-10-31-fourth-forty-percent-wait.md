# Fourth final leg: stop at40%, wait for ID2

- User replaces the timing-only35%-to75% approach; no further investigation of
  the reported half-travel stop is requested. Keep BOTH actual35% trigger for
  CAN3 ID2 +31.6812 rotor turns (latest requested travel) and all existing PID
  settings. Together with earlier-7.6812/-24 travels, the configured net is0.
- Only the fourth final front request uses pause_fraction0.4. APP plans its
  first reference endpoint at40% from the START, so it decelerates in advance.
  All other request APIs default to pause0 and retain their behavior.
- Preserve full original origins, per-wheel final targets, accepted sequence
  and signed actual progress. At40% retain MOVING and position control; do not
  emit DONE. paused=1 only after the reference ends, BOTH actual stop errors
  are within existing rotor tolerance and BOTH speeds within0.05outputrad/s.
  Re-evaluate paused every cycle so disturbance prevents premature resume.
- Upstairs validates ID2 against the separately captured ORIGINAL goal, current
  IDLE/inactive planner/position/speed. Retargeting an idle axis to a half-way
  feedback position cannot fake completion. ID2 not ready means no resume.
- Resume command is accepted only for the matching active sequence whose
  coherent published snapshot is paused. Payload/RESUME flag publication uses
  the existing scheduler lock. ChassisTask rechecks paused with fresh APP data
  and retains a pending accepted resume if disturbed. Clear stale flags on reply.
- Resume extends the SAME common trajectory from40% to original100%; no new
  relative origin, sequence, extra full leg or fabricated encoder position.
  Existing any-side final completion and feedback-loss faults remain active.
- APP status exposes paused/pause_ms/resume_ms. Fourth debug records resume
  request result and original-goal ID2 error. 75% timings remain observational;
  the old380ms window is now only an unpaused baseline, not a completion deadline.

Verification: prior code failed the new pause-request expectation.58 async task
cases pass O0/O2, including nonarrival holding40%, original-target reset not
resuming, resume retry and joint completion. Real APP5032 checks pass each
normal/reversed drive configuration O0/O2, including1600cycles stopped at40%,
no armed finalDONE, disturbance rejection, same-sequence original100% endpoint,
reverse direction and paused dropout/fault. Task65/84/789/4 checks,387steering
checks and upper/pair/second/wait/first-handoff regressions pass. Fresh Keil
ARMCC5 build/link:0errors0warnings. No flashing or physical test performed.
