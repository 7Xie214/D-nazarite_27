# Four GM6020 mechanical-zero holds

Current override (2026-11-01): ChassisTask steering-only and front-pair modes
hold FL/FR/RL only. RR CAN2 ID3 is released: its cascade is not updated and
the existing shared send helper writes zero voltage every cycle. Other motor
gains, calibration, drive ownership and first-stair actions are retained.
The four-hold design below describes the retained earlier implementation.

- ChassisTask owns all steering updates and CAN1/2 voltage frames. Both task
  modes run independent mechanical-zero cascades after RTOS startup; no
  Upstairs start notification is needed for steering hold.
- Mapping: FL CAN1 ID1, FR CAN2 ID4, RL CAN1 ID2, RR CAN2 ID3. Targets use each
  module's existing ZERO_OFFSET and encoder/electrical directions. Calibration
  values and steering angle bounds are retained.
- Rear CHASSIS_PID_CFG[2/3].steer starts with the front trial values:
  position PI16/2, position integral cap20, speed limit25 rad/s; speed PI3000/100,
  integral cap18000, voltage output cap24000. Front gains and drive gains are
  unchanged. These are initial parameters, not measured rear load tuning.
- One shared private steering loop serves count2/count4. Existing public
  UpdateFrontSteerHold retains its two-front behavior; new UpdateSteerHold
  drives all four and is used in steering-only task mode.
- Front-pair mode runs all four steering holds but retains its existing FRONT
  readiness/fault dependencies. Rear failures clear/reset only the affected
  rear steering output and expose feedback_ready=0; they do not newly block
  front travel. Steering-only mode reports aggregate four-steer status.
- CAN1 0x1ff slots1/2 (zero-based0/1) carry FL/RL; CAN2 slots3/4
  (zero-based2/3) carry RR/FR. No extra frames, CAN3 control, or rear drive
  position loops are added. Front drive remains zero before an accepted request.

Verification: new rear expectations failed five checks before implementation;
387 real task/APP/Device steering checks pass at O0/O2, including rear independent
hold, dropouts/recovery/range rejection and CAN slots. Front-pair tests pass
5004 checks each for normal/reversed drive directions at O0/O2, including rear
dropout isolation and zero rear drive commands. Task65/84/674/4 checks pass at
both optimization levels. Fresh Keil ARMCC5 build/link: 0 errors, 0 warnings.
No flashing or physical rear-steering validation.
