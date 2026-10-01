/*
 * Salary calculator.
 *
 * The user provides three values:
 *   - monthly salary  (yuan, x100 in storage)
 *   - hours per day   (x10)
 *   - days per month  (x10)
 *
 * From these we derive an effective "earn rate" per second, and expose a
 * running counter: money earned since a start instant. The counter keeps
 * running across UI navigation; it can be reset (long-press).
 *
 * All internal money values are stored in "cents" (1/100 yuan) as int64.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Monthly salary in cents (stored). */
int64_t salary_get_monthly_cents(void);
void    salary_set_monthly_cents(int64_t cents);

int  salary_get_hours_x10(void);
void salary_set_hours_x10(int v);

int  salary_get_days_x10(void);
void salary_set_days_x10(int v);

/* Effective earning rate in cents per second (0 if inputs invalid). */
double salary_rate_cents_per_sec(void);

/* ---- running counter ---- */
void    salary_counter_start(void);   /* (re)start counting from zero now */
void    salary_counter_stop(void);
void    salary_counter_reset(void);   /* stop + clear */
bool    salary_counter_running(void);

/* Seconds elapsed since the counter started (0 if not running). */
double  salary_counter_elapsed_sec(void);

/* Money earned since start, in cents. */
int64_t salary_counter_earned_cents(void);

#ifdef __cplusplus
}
#endif
