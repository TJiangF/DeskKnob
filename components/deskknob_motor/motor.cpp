#include "motor.h"
#include "board_pins.h"

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
    /* Lower phase voltage headroom -> lower peak current / less supply droop
     * (helps avoid brownouts that blank the shared 3V3 rail). */
    driver.voltage_limit = 2.6f;
    if (!driver.init()) {
        ESP_LOGE(TAG, "driver.init failed");
        return ESP_FAIL;
    }

    motor.linkDriver(&driver);
    motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
    motor.PID_velocity.P = 0.15f;
    motor.PID_velocity.I = 5.0f;
    motor.PID_velocity.D = 0.001f;
    /* Gentler ramps reduce di/dt and instantaneous current draw. */
    motor.PID_velocity.output_ramp = 100.0f;
    motor.LPF_velocity.Tf = 0.05f;
    motor.voltage_limit = 2.6f;
    motor.current_limit = 0.25f;
    motor.P_angle.P = 7.0f;
    motor.velocity_limit = 40.0f;
    motor.controller = MotionControlType::angle;

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

void motor_set_target_angle(float rad) { s_target_angle = rad; }

float motor_get_angle(void) { return sensor.getAngle(); }
float motor_get_relative_angle(void) { return sensor.getAngle() - s_zero_angle; }
float motor_get_velocity(void) { return sensor.getVelocity(); }

int motor_get_gear(void) { return s_gear; }
int motor_get_gear_limit(void) { return s_gear_limit; }

void motor_shake(float strength, uint32_t duration_ms)
{
    s_shake_strength = strength;
    s_shake_until_us = esp_timer_get_time() + (int64_t)duration_ms * 1000;
    s_shake_active = true;
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
    /* Prime the FOC state at zero torque first, so initFOC()'s alignment
     * sweep and the first control frames do not slam full voltage. */
    motor.move(0.0f);
    while (!s_ready) {
        motor.loopFOC();
        vTaskDelay(pdMS_TO_TICKS(5));
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
                /* Gentle oscillating "tick" - keep the amplitude small so the
                 * torque step does not spike supply current. */
                int phase = (int)(esp_timer_get_time() / 25000) & 1;
                float a = s_shake_strength * 0.5f;
                motor.move(phase ? a : -a);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

esp_err_t motor_start_task(void)
{
    BaseType_t ok = xTaskCreatePinnedToCore(motor_task, "motor", 8192, NULL, 20, NULL, 1);
    return ok == pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
}
