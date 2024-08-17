#include <stdint.h>
#include "rom/ets_sys.h"
#include "esp_timer.h"

void inv_imu_sleep_us(uint32_t us) { ets_delay_us(us); }

uint64_t inv_imu_get_time_us(void) { return (uint64_t)esp_timer_get_time(); }
