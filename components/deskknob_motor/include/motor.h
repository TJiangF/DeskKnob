/*
 * DeskKnob haptic motor service.
 *
 * Wraps the ESP-IDF port of SimpleFOC (espressif/esp_simplefoc) driving a
 * TMC6300 6-PWM gimbal motor with an AS5600 magnetic encoder. Three control
 * modes reproduce the original Arduino firmware:
 *
 *   MOTOR_MODE_ANGLE            - absolute position hold (used on the start page)
 *   MOTOR_MODE_TORQUE_INFINITE  - endless magnetic detents, unbounded travel
 *   MOTOR_MODE_TORQUE_BOUNDED   - detents clamped to [0, gear_limit-1] (menus)
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    MOTOR_MODE_ANGLE = 0,
    MOTOR_MODE_TORQUE_INFINITE,
    MOTOR_MODE_TORQUE_BOUNDED,
} motor_mode_t;

/* delta: gears moved since last callback, gear: absolute current gear. */
typedef void (*motor_gear_cb_t)(int delta, int gear, void *user);

esp_err_t motor_init(void);
esp_err_t motor_start_task(void);

void motor_set_mode(motor_mode_t mode, int gear_limit);
motor_mode_t motor_get_mode(void);

/* Number of detents over a full revolution (2..60). */
void motor_set_detents(int n);
int  motor_get_detents(void);

/* Detent strength, in the same "torque_k" units as the original firmware. */
void  motor_set_stiffness(float k);
float motor_get_stiffness(void);

/* Live-tunable drive parameters (also persisted by the caller). */
void  motor_set_voltage_limit(float v);
float motor_get_voltage_limit(void);
void  motor_set_current_limit(float a);
float motor_get_current_limit(void);
void  motor_set_velocity_limit(float v);
float motor_get_velocity_limit(void);
void  motor_set_pid_velocity(float p, float i, float d, float ramp);
void  motor_get_pid_velocity(float *p, float *i, float *d, float *ramp);
void  motor_set_kp_angle(float p);
float motor_get_kp_angle(void);

/* Angle mode target (radians). */
void motor_set_target_angle(float rad);

float motor_get_angle(void);            /* absolute accumulated sensor angle */
float motor_get_relative_angle(void);   /* angle - zero reference */
float motor_get_velocity(void);

int motor_get_gear(void);
int motor_get_gear_limit(void);

/* Short haptic "shake" used to acknowledge a selection. */
void motor_shake(float strength, uint32_t duration_ms);

void motor_set_gear_callback(motor_gear_cb_t cb, void *user);

bool motor_is_ready(void);
void motor_set_enabled(bool enabled);

#ifdef __cplusplus
}
#endif
