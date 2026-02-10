#define FCY     16000000UL
#include <xc.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "FreeRTOS.h"
#include "logger.h"
#include "task.h"


static uint8_t buffer[LOGGER_MESSAGE_SIZE];
static volatile bool dmaBusy = false;
static uint8_t ucQueueStorage[LOGGER_QUEUE_LEN * sizeof(xLoggerMessage)];
static StaticQueue_t xQueueBuffer;

QueueHandle_t xLoggerQueue;

static StaticTask_t xIdleTaskTCB;
static StackType_t uxIdleTaskStack[configMINIMAL_STACK_SIZE];


void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer,
                                    StackType_t **ppxIdleTaskStackBuffer,
                                    uint32_t *pulIdleTaskStackSize )
{
    *ppxIdleTaskTCBBuffer = &xIdleTaskTCB;
    *ppxIdleTaskStackBuffer = uxIdleTaskStack;
    *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}


void vSomeTask(void *pvParameters);


QueueHandle_t xStartLoggerTask( void )
{
	/* Create the queue used by the Logger task */
        //xLoggerQueue = xQueueCreate( LOGGER_QUEUE_LEN, sizeof(xLoggerMessage) );
        xLoggerQueue = xQueueCreateStatic( LOGGER_QUEUE_LEN, sizeof(xLoggerMessage), ucQueueStorage, &xQueueBuffer);
        dmaInit();
        uartInit();
        xTaskCreate(vSomeTask, "Task", LOGGER_TASK_STACK_SIZE, NULL, tskIDLE_PRIORITY + 2, NULL );
       
	xTaskCreate(vLoggerTask, "Log", LOGGER_TASK_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL);
	return xLoggerQueue;
}


void dmaInit(void) {
    DMACONbits.DMAEN = 1;
    DMACONbits.PRSSEL = 0;
    //DMAL = 0x0000;
    //DMAH = 0xFFFF;
    DMAL = (uint16_t)&buffer;
    DMAH = (uint16_t)&buffer + sizeof(buffer) - 1;
}


void uartInit(void) {
    U1MODE = 0;
    U1MODEbits.UARTEN = 1;
    U1MODEbits.UEN = 0;
    U1MODEbits.BRGH = 1;

    U1STA = 0;
    U1STAbits.UTXISEL1 = 0;
    U1STAbits.UTXISEL0 = 0;
    U1STAbits.URXEN = 1;
    U1STAbits.UTXEN = 1;
    U1STAbits.URXISEL = 0;

    U1BRG = FCY / (4UL * 115200);
    //U1BRG = FCY / (4UL * 9600);

    _U1TXIF = 0;

    DMACH0 = 0;
    DMACH0bits.SAMODE = 1;
    DMACH0bits.DAMODE = 0;
    DMACH0bits.TRMODE = 1;
    DMACH0bits.SIZE = 1;

    DMAINT0 = 0;
    DMAINT0bits.CHSEL = 68;

    DMADST0 = (uint16_t)&U1TXREG;

    _DMA0IF = 0;
    _DMA0IE = 1;
    _DMA0IP = 3;

    _U1RXIF = 0;
    _U1RXIE = 1;
    _U1RXIP = 3;
}


void uart_startTransfer(uint8_t *memory, uint16_t size) {
    DMASRC0 = (uint16_t)memory;
    DMACNT0 = size;
    DMACH0bits.CHEN = 1;
    DMACH0bits.CHREQ = 1;
}


void __attribute__((interrupt,no_auto_psv)) _DMA0Interrupt(void) {
    _DMA0IF = 0;
    dmaBusy = false;
}


void logger_printf(const char *format, ...)
{
    xLoggerMessage logMsg;
    va_list args;

    va_start(args, format);

    int len = vsnprintf(
        (char *)logMsg.pcMessage,
        LOGGER_MESSAGE_SIZE - 2,   // ????????? ????? ??? \r\n
        format,
        args
    );

    va_end(args);

    if (len < 0)
        return;

    if (len > LOGGER_MESSAGE_SIZE - 2)
        len = LOGGER_MESSAGE_SIZE - 2;

    logMsg.pcMessage[len++] = '\r';
    logMsg.pcMessage[len++] = '\n';

    logMsg.size = len;

    xQueueSend(xLoggerQueue, &logMsg, 0);
}


void vLoggerTask(void *pvParameters)
{
    xLoggerMessage rxMsg;

    for (;;)
    {
        if (xQueueReceive(xLoggerQueue, &rxMsg, portMAX_DELAY) == pdPASS)
        {
            while (dmaBusy)
                taskYIELD();

            memset(buffer, 0, sizeof(buffer));
            memcpy(buffer, rxMsg.pcMessage, rxMsg.size);

            dmaBusy = true;
            uart_startTransfer(buffer, rxMsg.size);
        }
    }
}


#include "sensors_handling.h"
#include<stdlib.h>
/*
void vSomeTask(void *pvParameters)
{
    int16_t temp = 0;
    uint16_t light = 0;
    while(1)
    {
        temp =  SENSORS_getTempValue();
        light = SENSORS_getLightValue();
        //logger_printf("LOG - temperature %d, lightning %d", temp, light);
        logger_printf("Light: %d, Temp: %d.%02d", light, temp/100, abs(temp)%100);
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
*/

/*
#include "sgp40_i2c.h"
#include "logger.h"

void vSomeTask(void *pvParameters)
{
    (void)pvParameters;

    uint16_t sraw_voc = 0;

    // ??? SGP40: "no temperature/humidity compensation"
    uint16_t t_sgp40  = 0x8000;
    uint16_t rh_sgp40 = 0x8000;

    while(1)
    {
        int16_t ret = sgp40_measure_raw_signal(rh_sgp40, t_sgp40, &sraw_voc);
        if(ret != 0) {
            logger_printf("SGP40: Failed to read VOC signal! Error: %d", ret);
        } else {
            logger_printf("SGP40 VOC raw: %d", sraw_voc);
        }

        vTaskDelay(pdMS_TO_TICKS(1000)); // ????????? ?????? ???????
    }
}
*/



/*
#include "sht4x_i2c.h"
#include "logger.h"

void vSomeTask(void *pvParameters)
{
    (void)pvParameters;

    logger_printf("==== SHT40 TASK START ====");

    int32_t temperature_mC = 0;
    int32_t humidity_mRH   = 0;

    // ---------- INIT ----------
    logger_printf("[INIT] Setting I2C address...");
    sht4x_init(SHT40_I2C_ADDR_45);
    logger_printf("[INIT] Address set: 0x45");

    // ---------- RESET ----------
    logger_printf("[INIT] Performing soft reset...");
    int16_t ret = sht4x_soft_reset();
    //int16_t ret = 0;

    if(ret != 0)
        logger_printf("[INIT] Soft reset FAILED (%d)", ret);
    else
        logger_printf("[INIT] Soft reset OK");

    logger_printf("[INIT] Waiting sensor recovery...");
    vTaskDelay(pdMS_TO_TICKS(10));

    uint32_t cycle = 0;

    while(1)
    {
        cycle++;
        logger_printf("\n--- Measurement cycle #%lu ---", cycle);

        // Clear buffers
        temperature_mC = 0;
        humidity_mRH   = 0;

        logger_printf("[MEASURE] Sending high precision command...");

        ret = sht4x_measure_high_precision(&temperature_mC, &humidity_mRH);

        logger_printf("[MEASURE] Return code: %d", ret);

        if(ret != 0)
        {
            logger_printf("[ERROR] Measurement failed");
        }
        else
        {
            // RAW values
            logger_printf("[RAW] Temp mC: %ld", temperature_mC);
            logger_printf("[RAW] Hum  mRH: %ld", humidity_mRH);

            // Human readable
            logger_printf(
                "[DATA] Temp: %ld.%03ld C",
                temperature_mC / 1000,
                labs(temperature_mC) % 1000
            );

            logger_printf(
                "[DATA] Hum : %ld.%03ld %%RH",
                humidity_mRH / 1000,
                labs(humidity_mRH) % 1000
            );
        }

        logger_printf("[TASK] Sleeping 1000 ms...");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
*/



#include "scd4x_i2c.h"
#include "logger.h"


void vSomeTask(void *pvParameters)
{
    logger_printf("\n========== SCD4x TASK START ==========");

    int16_t ret;
    uint16_t co2;
    int32_t temp;
    int32_t hum;

    // ---------------- INIT ----------------
    logger_printf("[INIT] scd4x_init()");
    scd4x_init(SCD40_I2C_ADDR_62);


    // --- Serial number ---
    uint16_t serial[3] = {0, 0, 0};
    logger_printf("[INIT] Empty Serial: %04X %04X %04X",
                      serial[0], serial[1], serial[2]);
    logger_printf("Getting serial number...");

    ret = scd4x_get_serial_number(serial, 3);
    if(ret) {
        logger_printf("scd4x_stop_periodic_measurement(): error, return code: (%d)", ret);
    } else {
        logger_printf("(Performed): scd4x_stop_periodic_measurement() =====> (OK)");
        logger_printf("[INIT] Serial: %04X %04X %04X", serial[0], serial[1], serial[2]);
    }

    
    
    
    ret = scd4x_stop_periodic_measurement();
    if(ret) {
        logger_printf("scd4x_stop_periodic_measurement(): error, return code: (%d)", ret);
    } else {
        logger_printf("(Performed): scd4x_stop_periodic_measurement() =====> (OK)");
    }
    
    
    
    
    ret = scd4x_set_temperature_offset_raw(1495);
    if(ret) {
        logger_printf("scd4x_set_temperature_offset_raw(): error, return code: (%d)", ret);
    } else {
        logger_printf("(Performed): scd4x_set_temperature_offset_raw() =====> (OK)");
    }
    
    uint16_t t = 0;
    
    ret = scd4x_get_temperature_offset_raw(&t);
    if(ret) {
        logger_printf("scd4x_get_temperature_offset_raw(): error, return code: (%d)", ret);
    } else {
        logger_printf("The current temperature compensation value: %d C", t);
    }
    
    
    
    ret = scd4x_set_sensor_altitude(540);
    if(ret) {
        logger_printf("scd4x_set_temperature_offset_raw(): error, return code: (%d)", ret);
    } else {
        logger_printf("(Performed): scd4x_set_sensor_altitude() =====> (OK)");
    }
    
    uint16_t a = 0;
    
    ret = scd4x_get_sensor_altitude(&a);
    if(ret) {
        logger_printf("scd4x_get_sensor_altitude(): error, return code: (%d)", ret);
    } else {
        logger_printf("Set the current sensor altitude: %d C", a);
    }
    

    // --- Start measurement ---
    ret = scd4x_start_periodic_measurement();
    if(ret) {
        logger_printf("scd4x_start_periodic_measurement(): error, return code: (%d)", ret);
    } else {
        logger_printf("(Performed): scd4x_start_periodic_measurement() =====> (OK)");
    }

    // ---------------- LOOP ----------------

    uint16_t status = 0;
    
    while(1)
    {
        ret = scd4x_get_data_ready_status_raw(&status);

        //logger_printf("[POLL] Ready raw: 0x%04X", status);

        if(status & 0x07FF)
        {            
            logger_printf("[POLL] Ready raw: 0x%04X", status);
            logger_printf("[READ] scd4x_read_measurement()");
            ret = scd4x_read_measurement(&co2, &temp, &hum);

            if(ret != 0)
            {
                logger_printf("[ERROR] Read failed (%d)", ret);
            }
            else
            {
                logger_printf("Carbon dioxide concentration: %u", co2);
                logger_printf("Environment temperature: %ld.%03ld C", temp/1000, labs(temp)%1000);
                logger_printf("Relative humidity: %ld.%03ld %%RH", hum/1000, labs(hum)%1000);
            }
        } else {
            logger_printf("Waiting...");
        }
        
        // ----- READ DATA -----
        
        
        ret = scd4x_start_periodic_measurement();
        if(ret) {
            logger_printf("scd4x_start_periodic_measurement(): error, return code: (%d)", ret);
        }

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}


