
#ifndef TASK_MAN_H
#define TASK_MAN_H

/**
 * @brief Identifiers for application tasks.
 */
typedef enum {
    TASK_ID_GIMBAL_CONTROL = 0, /** NVS storage service task */
    TASK_ID_STEPPER_CONTROL,    /** Gimbal control service task */
    TASK_ID_COMMS,              /** AP manager service task */
    TASK_ID_ACCEL,              /** Miscellaneous or unassigned task */
    TASK_ID_MAX                 /** Sentinel value for number of task IDs */
} task_id_t;

#endif // TASK_MAN_H
