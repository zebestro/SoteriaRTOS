#ifndef SENSORS_H
#define SENSORS_H

#include <stdint.h>
#include <stdbool.h>

#define SENSORS_TASK_STACK_SIZE 512


typedef struct {
    uint16_t sgp40_raw_voc;
    int32_t sht40_raw_temp;
    int32_t sht40_raw_hum;
    uint16_t scd40_raw_co2;
    int32_t scd40_raw_temp;
    int32_t scd40_raw_hum;
    int32_t mcp9808_raw_temp;
    uint16_t temt6000_raw_light;
} sensorData_t;


void vSensorsTask(void *pvParameters);

#endif
