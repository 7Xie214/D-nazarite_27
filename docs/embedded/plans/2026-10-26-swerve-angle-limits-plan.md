# Swerve Steering Limits Implementation Plan

> **For agentic workers:** REQUIRED: Use embedded-implementation skill to execute this plan task-by-task.

**Goal:** Add per-module half-turn steering limits with equivalent wheel reversal.

**Architecture:** Keep all geometry and heading selection inside the pure C99
Swerve library. The application supplies calibrated feedback in the selected range
and consumes bounded target angles with signed wheel speeds.

**Constraints:** Preserve the user's existing source comment and public function
signature. No hardware initialization, task changes, motor dependencies or upload.

1. Add a regression requesting 100 degrees from actual 80 degrees: default
   [-90, 90] mode must return -80 degrees and negative speed. Run the existing
   library and confirm failure before implementation.
2. Add range enum/field and tests for [0, 180], [-90, 90], edge/tie selection,
   vector preservation, out-of-range feedback/target, invalid mode and zero hold.
   Existing continuous tests explicitly select continuous mode.
3. Implement range validation and bounded candidate selection before publishing
   any targets. Preserve the existing continuous calculation as an explicit mode.
4. Run tests/run_swerve_tests.ps1 with GCC warnings treated as errors and ARMCC 5
   Cortex-M7 C99 compilation. Run complete Keil build and inspect its final result.
5. Review geometry/units and memory/control flow, update Chassis/README.md and
   AGENTS.md, report fresh evidence and absence of board testing.

Baseline evidence: 612 checks, 12 failures. The original code returns 100 degrees
instead of the expected -80 degrees, confirming the missing bounded behavior.

## Verification results

- Native GCC C99 warnings-as-errors runner: 2670 checks, 0 failures.
- Optimized GCC -O2 build/run: 2670 checks, 0 failures.
- ARMCC 5 strict Cortex-M7 C99 compile: passed, warnings treated as errors.
- Complete Keil build/link: 0 errors, 0 warnings; AXF/HEX generated.
- Endpoints use direct +/-pi/2 constants in tests; generated candidates within
  4*FLT_EPSILON radians of a boundary are aligned to the boundary to remove
  roundoff. Tests include 90.01-degree requests and one-ULP out-of-range feedback
  to distinguish actual limit crossings from generated numerical error.
- No board run, PID integration, commit or push performed.
