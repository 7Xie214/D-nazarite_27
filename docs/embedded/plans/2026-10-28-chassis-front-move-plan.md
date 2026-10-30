# Front step implementation plan

Use embedded-implementation with hardware then quality review.
Approved requirements: ../specs/2026-10-28-chassis-front-move-spec.md.

1. Add focused front-motion native checks before product code. Extend ChassisApp
   and PID_Config with two independent cascades/planners, relative rotor-turn
   conversion, hold/execute/actual-arrival state and fault abort. Add direct
   configuration macros. Preserve ordinary velocity mode. Validate native tests
   and review hardware math, then C99/code clarity.
2. Add task handshake regression checks. Extend ChassisTask.h/.c with a start
   request API and nonblocking reply API, CMSIS flags and one-result task logic.
   Switch its cyclic entry to UpdateFrontMove without changing 1 kHz cadence.
   Do not change Task/Src/My_task.c: user will insert the request call site.
   Document an Upstairs request/wait example that continues MotorApp_Update.
   Review task timing/notification semantics and code quality.
3. Run front application and handshake/timing tests, existing affected suites,
   optimized native checks and Keil ARMCC5 compile/link. Record software evidence
   separately from board execution. Leave physical travel and PID settings zero,
   retain all existing user changes, and do not commit or flash.
