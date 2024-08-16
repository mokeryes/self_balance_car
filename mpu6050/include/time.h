#include <stdint.h>

#include "esp_timer.h"
#include "rom/ets_sys.h"

void sleep_time_us(uint32_t us);

uint64_t get_time_us(void);
