/*
 * Network time (SNTP) helper.
 *
 * Time is only meaningful once Wi-Fi is connected. Start SNTP after the first
 * GOT_IP; until it syncs, clock_time_valid() stays false and the UI shows
 * "--:--".
 */
#pragma once

#include <stdbool.h>
#include <time.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t clock_time_init(void);         /* register IP event, auto-start SNTP */
void      clock_time_start_sync(void);   /* (re)start SNTP */
bool      clock_time_valid(void);

/* Fills hh:mm (24h). Returns false if time not synced yet. */
bool clock_time_hm(int *hour, int *minute);
/* "HH:MM" or "--:--" when not synced. */
void clock_time_str(char *out, int out_len);

#ifdef __cplusplus
}
#endif
