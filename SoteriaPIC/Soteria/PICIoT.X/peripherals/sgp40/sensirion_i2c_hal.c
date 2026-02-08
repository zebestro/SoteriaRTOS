#include "sensirion_i2c_hal.h"
#include "sensirion_common.h"
#include "i2c_simple_master.h"
#include <stdint.h>
#include <stddef.h>
#include "FreeRTOS.h"
#include "task.h"


int16_t sensirion_i2c_hal_select_bus(uint8_t bus_idx) {
    (void)bus_idx;
    return 0;
}

void sensirion_i2c_hal_init(void) {}

void sensirion_i2c_hal_free(void) {}

int8_t sensirion_i2c_hal_write(uint8_t address, const uint8_t* data, uint16_t count) {
    if (data == NULL || count == 0)
        return -1;

    i2c_writeNBytes(address, (void*)data, count);
    return 0;
}

int8_t sensirion_i2c_hal_read(uint8_t address, uint8_t* data, uint16_t count) {
    if (data == NULL || count == 0)
        return -1;

    i2c_readNBytes(address, data, count);
    return 0;
}

void sensirion_i2c_hal_sleep_usec(uint32_t useconds) {
    vTaskDelay(pdMS_TO_TICKS((useconds+999)/1000));
}

