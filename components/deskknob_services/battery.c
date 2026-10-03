#include "battery.h"

/* Simulated battery percentage for now (see header). */
static int s_mock_level = 100;

int battery_get(void)
{
    return s_mock_level;
}
