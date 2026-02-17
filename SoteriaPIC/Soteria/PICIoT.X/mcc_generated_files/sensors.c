#include <string.h>
#include <stdio.h>
#include "sensors.h"
#include "debug_print.h"
#include "sgp40_i2c.h"
#include "sht4x_i2c.h"
#include "scd4x_i2c.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"


static sensorData_t sensorData;

static SemaphoreHandle_t sensorMutex;
static StaticSemaphore_t sensorMutexBuffer;

//static StaticTask_t xSGPTaskTCB;
//static StackType_t xSGPTaskStack[SGP_TASK_STACK_SIZE];

static StaticTask_t xSensorsTaskTCB;
static StackType_t xSensorsTaskStack[SENSORS_TASK_STACK_SIZE];


void SENSOR_DATA_init(void)
{
    sensorMutex = xSemaphoreCreateMutex();
    sensorMutex = xSemaphoreCreateMutexStatic(&sensorMutexBuffer);
    if(sensorMutex == NULL) {
        debug_printError("Mutex wasn't created");
        return;
    }
    
    xTaskCreateStatic(vSensorsTask, "SNRS", SENSORS_TASK_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, xSensorsTaskStack, &xSensorsTaskTCB);

    sensorData.sgp40_raw_voc = 0;
    sensorData.sht40_raw_temp = 0;
    sensorData.sht40_raw_hum = 0;
    sensorData.scd40_raw_co2 = 0;
    sensorData.scd40_raw_temp = 0;
    sensorData.scd40_raw_hum = 0;
    sensorData.mcp9808_raw_temp = 0;
    sensorData.temt6000_raw_light = 0;
}

static void SENSOR_DATA_updateVocSGP40(uint16_t raw_voc)
{
    if(xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        sensorData.sgp40_raw_voc = raw_voc;
        xSemaphoreGive(sensorMutex);
    }
}

static void SENSOR_DATA_updateTemperatureSHT40(int32_t raw_temp)
{
    if(xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        sensorData.sht40_raw_temp = raw_temp;
        xSemaphoreGive(sensorMutex);
    }
}

static void SENSOR_DATA_updateHumiditySHT40(int32_t raw_hum)
{
    if(xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        sensorData.sht40_raw_hum = raw_hum;
        xSemaphoreGive(sensorMutex);
    }
}

static void SENSOR_DATA_updateCo2SCD40(uint16_t raw_co2)
{
    if(xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        sensorData.scd40_raw_co2 = raw_co2;
        xSemaphoreGive(sensorMutex);
    }
}

static void SENSOR_DATA_updateTemperatureSCD40(int32_t raw_temp)
{
    if(xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        sensorData.scd40_raw_temp = raw_temp;
        xSemaphoreGive(sensorMutex);
    }
}

static void SENSOR_DATA_updateHumiditySCD40(int32_t raw_hum)
{
    if(xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        sensorData.scd40_raw_hum = raw_hum;
        xSemaphoreGive(sensorMutex);
    }
}

static void SENSOR_DATA_updateTemperatureMCP9808(int32_t raw_temp)
{
    if(xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        sensorData.mcp9808_raw_temp = raw_temp;
        xSemaphoreGive(sensorMutex);
    }
}

static void SENSOR_DATA_updateLightTEMT6000(uint16_t raw_light)
{
    if(xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        sensorData.temt6000_raw_light = raw_light;
        xSemaphoreGive(sensorMutex);
    }
}

bool SENSOR_DATA_getSnapshot(sensorData_t *out)
{
    if (xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        *out = sensorData;
        xSemaphoreGive(sensorMutex);
        return true;
    }

    return false;
}

int SENSOR_DATA_snapshotToJson(const sensorData_t *s, char *json, size_t jsonSize)
{
    if (!s || !json)
        return 0;

    int len = snprintf(
        json,
        jsonSize,
        "{"
        "\"VOC (SGP40)\":%u,"
        "\"Temp (SHT40)\":%ld.%03ld,"
        "\"Hum (SHT40)\":%ld.%03ld,"
        "\"CO2 (SCD40)\":%u,"
        "\"Temp (SCD40)\":%ld.%03ld,"
        "\"Hum (SCD40)\":%ld.%03ld,"
        "\"Temp (MCP9808)\":%ld.%03ld,"
        "\"Light (TEMT6000)\":%u"
        "}",
        
        s->sgp40_raw_voc,

        s->sht40_raw_temp / 1000,
        labs(s->sht40_raw_temp) % 1000,

        s->sht40_raw_hum / 1000,
        labs(s->sht40_raw_hum) % 1000,

        s->scd40_raw_co2,

        s->scd40_raw_temp / 1000,
        labs(s->scd40_raw_temp) % 1000,

        s->scd40_raw_hum / 1000,
        labs(s->scd40_raw_hum) % 1000,

        s->mcp9808_raw_temp / 100,
        labs(s->mcp9808_raw_temp) % 100,

        s->temt6000_raw_light
    );

    if (len < 0 || len >= jsonSize)
        return 0;

    return len;
}


void vSensorsTask(void *pvParameters)
{
    (void)pvParameters;
    
    int mcp9808_temp = 0;
    int temt6000_light = 0;
    
    /* SGP Section */
    uint16_t sgp40_voc = 0;

    uint16_t t_sgp40  = 0x8000;
    uint16_t rh_sgp40 = 0x8000;
    
    
    /* SHT Section */
    int32_t sht40_temp= 0;
    int32_t sht40_hum = 0;

    sht4x_init(SHT40_I2C_ADDR_45);

    int16_t ret = sht4x_soft_reset();

    if(ret != 0)
        debug_print("[INIT] Soft reset FAILED (%d)", ret);

    vTaskDelay(pdMS_TO_TICKS(10));
    
    
    
    
    /*SCD Section */
    uint16_t scd40_co2 = 0;
    int32_t scd40_temp = 0;
    int32_t scd40_hum = 0;

    scd4x_init(SCD40_I2C_ADDR_62);

    // --- Serial number ---
    uint16_t serial[3] = {0, 0, 0};

    ret = scd4x_get_serial_number(serial, 3);
    if(ret) {
        debug_printError("scd4x_stop_periodic_measurement(): error, return code: (%d)", ret);
    }
    
    ret = scd4x_set_temperature_offset_raw(1495);
    if(ret) {
        debug_printError("scd4x_set_temperature_offset_raw(): error, return code: (%d)", ret);
    }
    
    uint16_t t = 0;
    
    ret = scd4x_get_temperature_offset_raw(&t);
    if(ret) {
        debug_printError("scd4x_get_temperature_offset_raw(): error, return code: (%d)", ret);
    }
    
    
    ret = scd4x_set_sensor_altitude(540);
    if(ret) {
        debug_printError("scd4x_set_temperature_offset_raw(): error, return code: (%d)", ret);
    }
    
    uint16_t a = 0;
    
    ret = scd4x_get_sensor_altitude(&a);
    if(ret) {
        debug_printError("scd4x_get_sensor_altitude(): error, return code: (%d)", ret);
    }
    

    // --- Start measurement ---
    ret = scd4x_start_periodic_measurement();
    if(ret) {
        debug_printError("scd4x_start_periodic_measurement(): error, return code: (%d)", ret);
    }
    
    uint16_t scd40_status = 0;    
    

    while(1)
    {
        /* MCP9808 section */
        mcp9808_temp = SENSORS_getTempValue();
        SENSOR_DATA_updateTemperatureMCP9808(mcp9808_temp);
        
        /* TEMT6000 section */
        temt6000_light = SENSORS_getLightValue();
        SENSOR_DATA_updateLightTEMT6000(temt6000_light);
        
        /* SHT Section */

        sht40_temp = 0;
        sht40_hum   = 0;

         int16_t ret = sgp40_measure_raw_signal(rh_sgp40, t_sgp40, &sgp40_voc);
        if(ret != 0) {
            debug_printError("SGP40: Failed to read VOC signal! Error: %d", ret);
        } else {
            SENSOR_DATA_updateVocSGP40(sgp40_voc);
            //debug_printInfo("SGP40 VOC raw: %d", sgp40_voc);
        }

         
        /* SGP Section */
         
        ret = sht4x_measure_high_precision(&sht40_temp, &sht40_hum);

        if(ret != 0)
        {
            debug_printError("[ERROR] Measurement failed");
        } else {
            SENSOR_DATA_updateTemperatureSHT40(sht40_temp);
            SENSOR_DATA_updateHumiditySHT40(sht40_hum);
            //debug_print("[RAW] Temp mC: %ld", sht40_temp);
            //debug_print("[RAW] Hum  mRH: %ld", sht40_hum);
        }
        
        
        
        
        /* SCD Section */
        
        ret = scd4x_get_data_ready_status_raw(&scd40_status);

        if(scd40_status & 0x07FF)
        {            
            ret = scd4x_read_measurement(&scd40_co2, &scd40_temp, &scd40_hum);

            if(ret != 0)
            {
                debug_print("[ERROR] Read failed (%d)", ret);
            }
            else
            {
                SENSOR_DATA_updateCo2SCD40(scd40_co2);
                SENSOR_DATA_updateTemperatureSCD40(scd40_temp);
                SENSOR_DATA_updateHumiditySCD40(scd40_hum);
                //debug_print("Carbon dioxide concentration: %u", scd40_co2);
                //debug_print("Environment temperature: %ld.%03ld C", scd40_temp/1000, labs(scd40_temp)%1000);
                //debug_print("Relative humidity: %ld.%03ld %%RH", scd40_hum/1000, labs(scd40_hum)%1000);
            }
        }
        
        
        ret = scd4x_start_periodic_measurement();
        if(ret) {
            debug_print("scd4x_start_periodic_measurement(): error, return code: (%d)", ret);
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
