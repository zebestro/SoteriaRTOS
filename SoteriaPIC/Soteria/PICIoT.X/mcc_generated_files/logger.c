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


void vSomeTask(void *pvParameters)
{
    while(1)
    {
        logger_printf("LOG - int %d char - %c, str - %s", 102, 'Y', "Hello world!");
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}


