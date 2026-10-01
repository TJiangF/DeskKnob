/*
 * DeepSeek account balance service.
 *
 * GET https://api.deepseek.com/user/balance with the configured API key.
 * The request runs in a small worker task so the UI never blocks.
 */
#pragma once

#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bool valid;            /* true when last fetch succeeded */
    bool busy;             /* request in flight */
    bool available;        /* is_available from the API */
    double total;          /* total_balance */
    char currency[8];      /* e.g. "CNY" */
    char message[64];      /* error/info text */
} balance_info_t;

esp_err_t balance_service_init(void);
void balance_request_refresh(void);       /* async */
void balance_get(balance_info_t *out);    /* copy cached result */

/* API key persistence (NVS). */
void balance_set_api_key(const char *key);
const char *balance_get_api_key(void);

#ifdef __cplusplus
}
#endif
