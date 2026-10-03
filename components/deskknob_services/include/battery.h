/*
 * Battery level provider.
 *
 * Currently returns a simulated percentage. When the real fuel-gauge / ADC
 * read is wired up, only battery_get() needs to change.
 */
#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BATTERY_LEVEL_UNKNOWN (-1)

/* Returns 0..100, or BATTERY_LEVEL_UNKNOWN if not available. */
int battery_get(void);

#ifdef __cplusplus
}
#endif
