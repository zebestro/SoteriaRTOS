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
    
    //xTaskCreateStatic(vSGPTask, "SGP", SGP_TASK_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, xSGPTaskStack, &xSGPTaskTCB);
    xTaskCreateStatic(vSensorsTask, "SNRS", SENSORS_TASK_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, xSensorsTaskStack, &xSensorsTaskTCB);

    sensorData.temperature_mC = 0;
    sensorData.humidity_mRH = 0;
    sensorData.light = 0;
    sensorData.co2 = 0;
    sensorData.voc = 0;
    sensorData.valid = false;
}

void SENSOR_DATA_updateTemperature(int32_t temp_mC)
{
    if(xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        sensorData.temperature_mC = temp_mC;
        sensorData.valid = true;
        xSemaphoreGive(sensorMutex);
    }
}

void SENSOR_DATA_updateHumidity(int32_t hum_mRH)
{
    if(xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        sensorData.humidity_mRH = hum_mRH;
        sensorData.valid = true;
        xSemaphoreGive(sensorMutex);
    }
}

void SENSOR_DATA_updateCO2(uint16_t co2)
{
    if(xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        sensorData.co2 = co2;
        sensorData.valid = true;
        xSemaphoreGive(sensorMutex);
    }
}

void SENSOR_DATA_updateVOC(uint16_t voc)
{
    if(xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        sensorData.voc = voc;
        sensorData.valid = true;
        xSemaphoreGive(sensorMutex);
    }
}

void SENSOR_DATA_updateLight(uint16_t light)
{
    if(xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        sensorData.light = light;
        sensorData.valid = true;
        xSemaphoreGive(sensorMutex);
    }
}

sensorData_t SENSOR_DATA_getSnapshot(void)
{
    sensorData_t snapshot;
    if(xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        snapshot = sensorData;
        xSemaphoreGive(sensorMutex);
    }
    else
    {
        snapshot.valid = false;
    }
    return snapshot;
}


// ================= TASKS =================

void vSGPTask(void *pvParameters)
{
    (void)pvParameters;
    debug_print("SGP Task Started!");
    uint16_t sraw_voc = 0;

    uint16_t t_sgp40  = 0x8000;
    uint16_t rh_sgp40 = 0x8000;

    while(1)
    {
        int16_t ret = sgp40_measure_raw_signal(rh_sgp40, t_sgp40, &sraw_voc);
        if(ret != 0) {
            debug_printError("SGP40: Failed to read VOC signal! Error: %d", ret);
        } else {
            debug_printInfo("SGP40 VOC raw: %d", sraw_voc);
        }

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}





void vSensorsTask(void *pvParameters)
{
    //application_init();
    (void)pvParameters;
    
    /* SGP Section */
    uint16_t sgp40_raw_voc = 0;

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
    uint16_t scd40_co2;
    int32_t scd40_temp;
    int32_t scd40_hum;

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
    uint8_t counter = 0;
    
    

    while(1)
    {
        
        /* SHT Section */

        sht40_temp = 0;
        sht40_hum   = 0;

         int16_t ret = sgp40_measure_raw_signal(rh_sgp40, t_sgp40, &sgp40_raw_voc);
        if(ret != 0) {
            debug_printError("SGP40: Failed to read VOC signal! Error: %d", ret);
        } else {
            debug_printInfo("SGP40 VOC raw: %d", sgp40_raw_voc);
        }

         
        /* SGP Section */
         
        ret = sht4x_measure_high_precision(&sht40_temp, &sht40_hum);

        if(ret != 0)
        {
            debug_printError("[ERROR] Measurement failed");
        }
        
        
        
        else
        {
            // RAW values
            debug_print("[RAW] Temp mC: %ld", sht40_temp);
            debug_print("[RAW] Hum  mRH: %ld", sht40_hum);
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
                debug_print("Carbon dioxide concentration: %u", scd40_co2);
                debug_print("Environment temperature: %ld.%03ld C", scd40_temp/1000, labs(scd40_temp)%1000);
                debug_print("Relative humidity: %ld.%03ld %%RH", scd40_hum/1000, labs(scd40_hum)%1000);
            }
        }
        
        
        ret = scd4x_start_periodic_measurement();
        if(ret) {
            debug_print("scd4x_start_periodic_measurement(): error, return code: (%d)", ret);
        }
        
        counter++;
        
        if (counter == 10) {
            //mainDataTask();
            counter = 0;
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
