#include "motor.h"
#include "board_pins.h"
#include "settings.h"

#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "esp_simplefoc.h"

static const char *TAG = "motor";

/* -------- SimpleFOC objects (hardware pins from board_pins.h) -------- */
static BLDCDriver6PWM driver = BLDCDriver6PWM(
    BOARD_MOTOR_PWM_AH, BOARD_MOTOR_PWM_AL,
    BOARD_MOTOR_PWM_BH, BOARD_MOTOR_PWM_BL,
    BOARD_MOTOR_PWM_CH, BOARD_MOTOR_PWM_CL,
    BOARD_MOTOR_EN, 0);
static BLDCMotor motor = BLDCMotor(BOARD_MOTOR_POLE_PAIRS);
static AS5600 sensor = AS5600(BOARD_I2C_PORT, BOARD_I2C_SCL, BOARD_I2C_SDA);

/* -------- shared control state -------- */
static volatile motor_mode_t s_mode = MOTOR_MODE_ANGLE;
static volatile int s_gear_limit = 0;
static volatile int s_num_positions = 24;
static volatile float s_torque_k = 5.0f;
static volatile float s_target_angle = 0.0f;
static volatile bool s_ready = false;
static volatile bool s_enabled = true;

static float s_zero_angle = 0.0f;
static float s_attractor = 0.0f;
static int s_gear = 0;
static int s_last_gear = 0;

static motor_gear_cb_t s_gear_cb = nullptr;
static void *s_gear_cb_user = nullptr;

static volatile bool s_shake_active = false;
static float s_shake_strength = 0.0f;
static int64_t s_shake_until_us = 0;

static void update_attractor(void)
{
    if (s_num_positions < 1) {
        s_num_positions = 1;
    }
    s_attractor = 2.0f * PI / (float)s_num_positions;
}

esp_err_t motor_init(void)
{
    sensor.init();
    motor.linkSensor(&sensor);

    driver.dead_zone = 0.02f;
    driver.voltage_power_supply = 3.3f;
    driver.voltage_limit = 3.3f;
    if (!driver.init()) {
        ESP_LOGE(TAG, "driver.init failed");
        return ESP_FAIL;
    }

    motor.linkDriver(&driver);
    motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
    motor.LPF_velocity.Tf = 0.05f;
    motor.controller = MotionControlType::angle;

    /* Load persisted drive parameters (fall back to the historical defaults). */
    driver.voltage_limit = (float)settings_get_int(SET_MOTOR_VOLTAGE, 33) / 10.0f;
    motor.voltage_limit = driver.voltage_limit;
    motor.current_limit = (float)settings_get_int(SET_MOTOR_CURRENT, 50) / 100.0f;
    motor.velocity_limit = (float)settings_get_int(SET_MOTOR_VEL_LIMIT, 60);
    motor.PID_velocity.P = (float)settings_get_int(SET_MOTOR_PID_P, 150) / 1000.0f;
    motor.PID_velocity.I = (float)settings_get_int(SET_MOTOR_PID_I, 500) / 100.0f;
    motor.PID_velocity.D = (float)settings_get_int(SET_MOTOR_PID_D, 100) / 100000.0f;
    motor.PID_velocity.output_ramp = (float)settings_get_int(SET_MOTOR_PID_RAMP, 200);
    motor.P_angle.P = (float)settings_get_int(SET_MOTOR_KP_ANGLE, 70) / 10.0f;

    motor.init();
    motor.initFOC();
    motor.enable();

    s_zero_angle = sensor.getAngle();
    update_attractor();
    s_gear = 0;
    s_last_gear = 0;
    s_ready = true;
    ESP_LOGI(TAG, "motor ready (pp=%d, detents=%d)", BOARD_MOTOR_POLE_PAIRS, s_num_positions);
    return ESP_OK;
}

void motor_set_mode(motor_mode_t mode, int gear_limit)
{
    s_mode = mode;
    s_gear_limit = gear_limit;
    s_zero_angle = sensor.getAngle();
    s_gear = 0;
    s_last_gear = 0;
    s_target_angle = 0.0f;
    switch (mode) {
    case MOTOR_MODE_ANGLE:
        motor.controller = MotionControlType::angle;
        break;
    case MOTOR_MODE_TORQUE_INFINITE:
    case MOTOR_MODE_TORQUE_BOUNDED:
        motor.controller = MotionControlType::torque;
        break;
    }
}

motor_mode_t motor_get_mode(void) { return s_mode; }

void motor_set_detents(int n)
{
    if (n < 2) n = 2;
    if (n > 60) n = 60;
    s_num_positions = n;
    update_attractor();
}

int motor_get_detents(void) { return s_num_positions; }

void motor_set_stiffness(float k)
{
    if (k < 0.5f) k = 0.5f;
    if (k > 15.0f) k = 15.0f;
    s_torque_k = k;
}

float motor_get_stiffness(void) { return s_torque_k; }

void motor_set_voltage_limit(float v)
{
    if (v < 0.5f) v = 0.5f;
    if (v > 3.3f) v = 3.3f;
    motor.voltage_limit = v;
    driver.voltage_limit = v;
}

float motor_get_voltage_limit(void) { return motor.voltage_limit; }

void motor_set_current_limit(float a)
{
    if (a < 0.05f) a = 0.05f;
    if (a > 1.0f) a = 1.0f;
    motor.current_limit = a;
}

float motor_get_current_limit(void) { return motor.current_limit; }

void motor_set_velocity_limit(float v)
{
    if (v < 1.0f) v = 1.0f;
    motor.velocity_limit = v;
}

float motor_get_velocity_limit(void) { return motor.velocity_limit; }

void motor_set_pid_velocity(float p, float i, float d, float ramp)
{
    motor.PID_velocity.P = p;
    motor.PID_velocity.I = i;
    motor.PID_velocity.D = d;
    motor.PID_velocity.output_ramp = ramp;
}

void motor_get_pid_velocity(float *p, float *i, float *d, float *ramp)
{
    if (p) *p = motor.PID_velocity.P;
    if (i) *i = motor.PID_velocity.I;
    if (d) *d = motor.PID_velocity.D;
    if (ramp) *ramp = motor.PID_velocity.output_ramp;
}

void motor_set_kp_angle(float p)
{
    if (p < 0.0f) p = 0.0f;
    motor.P_angle.P = p;
}

float motor_get_kp_angle(void) { return motor.P_angle.P; }

void motor_set_target_angle(float rad) { s_target_angle = rad; }

float motor_get_angle(void) { return sensor.getAngle(); }
float motor_get_relative_angle(void) { return sensor.getAngle() - s_zero_angle; }
float motor_get_velocity(void) { return sensor.getVelocity(); }

int motor_get_gear(void) { return s_gear; }
int motor_get_gear_limit(void) { return s_gear_limit; }

/* Vibration feedback disabled by request; kept as a no-op API. */
void motor_shake(float strength, uint32_t duration_ms)
{
    (void)strength;
    (void)duration_ms;
}

void motor_set_gear_callback(motor_gear_cb_t cb, void *user)
{
    s_gear_cb = cb;
    s_gear_cb_user = user;
}

bool motor_is_ready(void) { return s_ready; }

void motor_set_enabled(bool enabled)
{
    s_enabled = enabled;
    if (enabled) {
        motor.enable();
    } else {
        motor.disable();
    }
}

static void motor_task(void *arg)
{
    (void)arg;
    while (!s_ready) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    for (;;) {
        motor.loopFOC();

        float rel = sensor.getAngle() - s_zero_angle;
        float torque = 0.0f;

        if (!s_enabled) {
            motor.move(0.0f);
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }

        switch (s_mode) {
        case MOTOR_MODE_ANGLE:
            motor.move(s_target_angle + s_zero_angle);
            break;

        case MOTOR_MODE_TORQUE_INFINITE: {
            int gear = (int)lroundf(rel / s_attractor);
            if (gear != s_last_gear && s_gear_cb) {
                s_gear_cb(gear - s_last_gear, gear, s_gear_cb_user);
                s_last_gear = gear;
            }
            s_gear = gear;
            float err = (float)gear * s_attractor - rel;
            torque = s_torque_k * fminf(0.8f, err);
            motor.move(torque);
            break;
        }

        case MOTOR_MODE_TORQUE_BOUNDED: {
            int gear = (int)lroundf(rel / s_attractor);
            int limit = s_gear_limit > 0 ? s_gear_limit : 1;
            if (gear < 0) gear = 0;
            if (gear > limit - 1) gear = limit - 1;
            if (gear != s_last_gear && s_gear_cb) {
                s_gear_cb(gear - s_last_gear, gear, s_gear_cb_user);
                s_last_gear = gear;
            }
            s_gear = gear;
            float err = (float)gear * s_attractor - rel;
            torque = s_torque_k * fminf(0.7f, err);
            motor.move(torque);
            break;
        }
        }

        if (s_shake_active) {
            if (esp_timer_get_time() > s_shake_until_us) {
                s_shake_active = false;
            } else {
                /* Oscillating "tick" for tactile confirmation. */
                int phase = (int)(esp_timer_get_time() / 25000) & 1;
                motor.move(phase ? s_shake_strength : -s_shake_strength);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

esp_err_t motor_start_task(void)
{
    BaseType_t ok = xTaskCreatePinnedToCore(motor_task, "motor", 8192, NULL, 20, NULL, 0);
    return ok == pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
}
