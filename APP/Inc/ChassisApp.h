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
/* Chassis task only. Relative motor-rotor turns, before gearbox reduction. */
ChassisFront_StartResult_t ChassisApp_MoveFrontTurns(float left_rotor_turns,
                                                   float right_rotor_turns);

#endif /* CHASSIS_APP_H */
