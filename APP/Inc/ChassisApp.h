/** @brief Chassis motor control. Initialize after Chassis_Init/DjiMotor_Init. */
#ifndef CHASSIS_APP_H
#define CHASSIS_APP_H

#include "Chassis.h"

typedef enum {
    CHASSIS_FRONT_HOLD = 0,
    CHASSIS_FRONT_MOVING,
    CHASSIS_FRONT_DONE,
    CHASSIS_FRONT_FAULT
} ChassisFront_State_t;

typedef enum {
    CHASSIS_FRONT_START_OK = 0,
    CHASSIS_FRONT_START_NOT_READY,
    CHASSIS_FRONT_START_BAD_CONFIG,
    CHASSIS_FRONT_START_BUSY
} ChassisFront_StartResult_t;

extern ChassisFront_State_t g_chassis_front_state;

#if CHASSIS_ENABLE_SWERVE
#include "DjiMotor.h"
#include "PositionLoop.h"
#include "Traj.h"

typedef struct {
    Swerve_Module_t *module;
    DjiMotor_t *drive_motor;
    DjiMotor_t *steer_motor;
    Cascade_t steer_pid;
    PID_Controller_t drive_pid;
    float drive_speed_mps;
    float steer_speed_rad_s;
    float steer_angle_bias_rad; /* Continuous angle = bias + signed motor angle. */
    uint8_t angle_seeded;       /* Retain last mechanical turn across faults. */
    uint8_t feedback_ready;     /* Clear on fault; reseed before running PIDs. */
} ChassisApp_Wheel_t;

extern ChassisApp_Wheel_t g_chassis_wheels[SWERVE_MODULE_COUNT];

/* FL then FR. Position/planner feedback uses signed gearbox-output rad. */
typedef struct {
    DjiMotor_t *motor;
    Cascade_t cascade;
    Traj_t traj;
    uint8_t seeded;
} ChassisFront_Axis_t;

extern ChassisFront_Axis_t g_chassis_front_axes[2];

/* FL/FR observation: signed travel/errors are PRE-reduction rotor turns. */
typedef struct {
    float actual_turns[2];
    float error_turns[2];
    float speed_command_rpm[2];
    float reference_turns;
    float sync_error_turns;
    float correction_rad_s;
    uint32_t start_ms;
    uint32_t elapsed_ms;
    uint8_t planner_active;
    uint8_t finish_motor_id;
    uint8_t fault_motor_id;
    uint32_t move_sequence;      /* 每次成功接受新行程递增，区分上一次的进度。 */
    float pause_fraction;        /* 0=不停；指定比例处规划停稳，终点仍为原目标。 */
    uint8_t paused;
    uint32_t pause_ms;
    uint32_t resume_ms;
} ChassisFrontPair_Status_t;
extern ChassisFrontPair_Status_t g_chassis_front_pair;
#endif

typedef enum {
    CHASSIS_APP_READY = 0,
    CHASSIS_APP_FAULT_CONFIG,
    CHASSIS_APP_FAULT_FEEDBACK,
    CHASSIS_APP_FAULT_DT,
    CHASSIS_APP_FAULT_SOLVE,
    CHASSIS_APP_FAULT_OUTPUT
} ChassisApp_Status_t;

extern ChassisApp_Status_t g_chassis_app_status;

void ChassisApp_Init(void);
/* Coherent command publication; m/s, m/s, rad/s. Retained until replaced. */
void ChassisApp_SetVelocity(float vx, float vy, float wz);
/* Called by the chassis task only; dt in seconds. Sends CAN1/2 groups. */
void ChassisApp_Update(float dt);
/* Alternative task entry: never call both updates in the same cycle. */
void ChassisApp_UpdateFrontMove(float dt);
/* ChassisTask only: FL/FR mechanical zero hold, no drive/rear feedback required. */
void ChassisApp_UpdateFrontSteerHold(float dt);
/* ChassisTask steering-only mode: FL/FR/RL zero holds; RR voltage stays zero. */
void ChassisApp_UpdateSteerHold(float dt);
/* Paired front-drive alternative; holds FL/FR/RL GM6020 zero, releases RR.
 * Rear-steering faults stay local and do not add front-drive readiness gates.
 * Before the first accepted request, drive feedback updates but drive PID is
 * inactive and both 3508 commands stay zero. DONE holds after an actual move.
 * Request signs follow RAW motor rotor feedback, before reduction. Equal
 * absolute travel only; either side in tolerance ends the pair and rebases holds. */
void ChassisApp_UpdateFrontPair(float dt);
ChassisFront_StartResult_t ChassisApp_MoveFrontPairTurns(float left_rotor_turns,
                                                       float right_rotor_turns);
ChassisFront_StartResult_t ChassisApp_MoveFrontPairTurnsWithPause(float left_rotor_turns,
                                                               float right_rotor_turns,
                                                               float pause_fraction);
int ChassisApp_ResumeFrontPair(void); /* 仅停稳暂停时恢复，成功返回1。 */
/* Chassis task only. Relative motor-rotor turns, before gearbox reduction. */
ChassisFront_StartResult_t ChassisApp_MoveFrontTurns(float left_rotor_turns,
                                                   float right_rotor_turns);

#endif /* CHASSIS_APP_H */
