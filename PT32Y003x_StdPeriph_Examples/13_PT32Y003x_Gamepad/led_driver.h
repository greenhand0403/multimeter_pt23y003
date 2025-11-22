#include "system_config.h"

static uint32_t last_blink_time = 0;
static uint8_t led_state = 0;  // 0=off, 1=on
static uint8_t blink_mode = 0; // 0=off, 1=fast, 2=slow
static uint32_t blink_interval = BLINK_FAST_INTERVAL_MS;

void led_init(void);
void led_set_on(void);
void led_set_off(void);
void led_set_blink_fast(void);
void led_update(void);