# Chassis Configuration Implementation Plan

> **For agentic workers:** REQUIRED: Use embedded-implementation skill to execute this plan task-by-task.

**Goal:** Centralize chassis configuration macros and initialize the selected wheel system.

**Architecture:** Chassis_Conf.h selects one wheel system and supplies scalar
configuration; Chassis.h exposes Chassis_Init and the active array; Chassis.c
initializes via #if/#elif. Main calls it before scheduling. Existing solvers remain
separate portable C libraries.

**Constraints:** No other wheel types or motor drivers; preserve existing code.
Unknown dimensions stay zero, default directions +1, no hardware operation.

1. Write tests/test_chassis_init.c and tests/run_chassis_tests.ps1 for a real
   Chassis_Init contract. Establish failing baseline before implementation.
2. Add configuration defaults/selection guards, active swerve array and macro-driven
   initialization. Tests override all four wheel configurations independently.
3. Register Chassis.c/h/Conf.h in Keil, add main initialization call, and update
   README with actual macro names and the distinction from feedback calibration.
4. Run default/override/disabled/mecanum/omni build cases plus expected rejection
   of multi-selected or nonboolean flags. Run existing kinematics tests and ARMCC
   strict compilation, then full Keil build.
5. Review specs/hardware boundaries and C quality, record fresh verification.

## Verification results

- Initial test baseline rejected the missing Chassis_Init interface.
- Seven build cases passed: default configuration (123 checks), asymmetric macro
  overrides (123 checks), disabled/mecanum/omni (one check each), and expected
  compiler rejection of conflicting/nonboolean selection flags.
- Existing swerve regression suite: 2670 checks, 0 failures.
- ARMCC 5 strict C99 Cortex-M7 compile passed for default and disabled modes.
- Complete Keil build/link: 0 errors, 0 warnings; AXF/HEX generated.
- Native geometry checks use finite approximate comparison to avoid MinGW x87
  decimal-macro excess-precision differences when comparing a stored float.
- No board calibration, flash, motor actuation, commit or push performed.
