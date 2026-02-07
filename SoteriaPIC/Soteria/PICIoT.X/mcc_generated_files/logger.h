#ifndef LOGGER_H
#define	LOGGER_H

#include "FreeRTOS.h"
#include "queue.h"

#define LOGGER_QUEUE_LEN		12
#define LOGGER_MESSAGE_SIZE  128
#define LOGGER_TASK_STACK_SIZE 1024


typedef struct {
    char pcMessage[LOGGER_MESSAGE_SIZE];
    uint16_t size;
} xLoggerMessage;

void dmaInit(void);
void uartInit(void);
void uart_startTransfer(uint8_t *memory, uint16_t size);
void __attribute__((interrupt,no_auto_psv)) _DMA0Interrupt(void);
void logger_print(char * msg);
void vLoggerTask(void *pvParameters);
QueueHandle_t xStartLoggerTask( void );
    
#endif

