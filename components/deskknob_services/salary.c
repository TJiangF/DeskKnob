#include "salary.h"
#include "settings.h"

#include "esp_timer.h"

int64_t salary_get_monthly_cents(void)
{
    return (int64_t)settings_get_int(SET_SALARY_RATE, settings_get_default(SET_SALARY_RATE));
}

void salary_set_monthly_cents(int64_t cents)
{
    if (cents < 0) cents = 0;
    settings_set_int(SET_SALARY_RATE, (int32_t)cents);
}

int salary_get_hours_x10(void)
{
    return settings_get_int(SET_SALARY_HOURS, settings_get_default(SET_SALARY_HOURS));
}

void salary_set_hours_x10(int v)
{
    if (v < 0) v = 0;
    settings_set_int(SET_SALARY_HOURS, v);
}

int salary_get_days_x10(void)
{
    return settings_get_int(SET_SALARY_DAYS, settings_get_default(SET_SALARY_DAYS));
}

void salary_set_days_x10(int v)
{
    if (v < 0) v = 0;
    settings_set_int(SET_SALARY_DAYS, v);
}

double salary_rate_cents_per_sec(void)
{
    double monthly = (double)salary_get_monthly_cents();
    double hours_x10 = (double)salary_get_hours_x10();
    double days_x10 = (double)salary_get_days_x10();
    /* total worked seconds per month = (hours/10)*3600 * (days/10) */
    double seconds = (hours_x10 / 10.0) * 3600.0 * (days_x10 / 10.0);
    if (seconds <= 0.0 || monthly <= 0.0) {
        return 0.0;
    }
    return monthly / seconds;
}

/* ---- running counter ---- */
static int64_t s_start_us;
static bool s_running;

void salary_counter_start(void)
{
    s_start_us = esp_timer_get_time();
    s_running = true;
}

void salary_counter_stop(void)
{
    s_running = false;
}

void salary_counter_reset(void)
{
    s_running = false;
    s_start_us = 0;
}

bool salary_counter_running(void) { return s_running; }

double salary_counter_elapsed_sec(void)
{
    if (!s_running) {
        return 0.0;
    }
    return (double)(esp_timer_get_time() - s_start_us) / 1e6;
}

int64_t salary_counter_earned_cents(void)
{
    double sec = salary_counter_elapsed_sec();
    return (int64_t)(sec * salary_rate_cents_per_sec());
}
