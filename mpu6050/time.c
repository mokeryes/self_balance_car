#include "time.h"

void sleep_time_us(uint32_t us) { ets_delay_us(us); }

uint64_t get_time_us(void) { return (uint64_t)esp_timer_get_time(); }
