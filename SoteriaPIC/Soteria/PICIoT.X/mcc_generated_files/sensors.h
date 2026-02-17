#ifndef SENSORS_H
#define SENSORS_H

#include <stdint.h>
#include <stdbool.h>

#define SGP_TASK_STACK_SIZE 512
#define SENSORS_TASK_STACK_SIZE 512
#define SCD_TASK_STACK_SIZE 512


typedef struct {
    int32_t temperature_mC;
    int32_t humidity_mRH;
    uint16_t light;
    uint16_t co2;
    uint16_t voc;
    bool valid;
} sensorData_t;


void vSGPTask(void *pvParameters);
void vSensorsTask(void *pvParameters);

#endif
