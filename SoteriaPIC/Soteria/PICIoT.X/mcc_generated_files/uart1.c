/**
  UART1 Generated Driver File 

  @Company
    Microchip Technology Inc.

  @File Name
    uart1.c

  @Summary
    This is the generated source file for the UART1 driver using Foundation Services Library

  @Description
    This source file provides APIs for driver for UART1. 

    Generation Information : 
        Product Revision  :  Foundation Services Library - pic24-dspic-pic32mm : v1.26
        Device            :  PIC24FJ128GA705
    The generated drivers are tested against the following:
        Compiler          :  XC16 1.30
        MPLAB 	          :  MPLAB X 3.45
*/

/*
    (c) 2016 Microchip Technology Inc. and its subsidiaries. You may use this
    software and any derivatives exclusively with Microchip products.

    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
    WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
    PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION
    WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION.

    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
    BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
    FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
    ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
    THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.

    MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE
    TERMS.
*/

/**
  Section: Included Files
*/
#include <stdbool.h>
#include <stdint.h>
#include "xc.h"
#include "uart1.h"

/**
  Section: Data Type Definitions
*/

/** UART Driver Queue Status

  @Summary
    Defines the object required for the status of the queue.
*/

static uint8_t * volatile rxTail;
static uint8_t *rxHead;
static uint8_t *txTail;
static uint8_t * volatile txHead;
static bool volatile rxOverflowed;

/** UART Driver Queue Length

  @Summary
    Defines the length of the Transmit and Receive Buffers

*/

#define UART1_CONFIG_TX_BYTEQ_LENGTH (128+1)
#define UART1_CONFIG_RX_BYTEQ_LENGTH (128+1)

/** UART Driver Queue

  @Summary
    Defines the Transmit and Receive Buffers

*/

static uint8_t txQueue[UART1_CONFIG_TX_BYTEQ_LENGTH] ;
static uint8_t rxQueue[UART1_CONFIG_RX_BYTEQ_LENGTH] ;

void (*UART1_TxDefaultInterruptHandler)(void);
void (*UART1_RxDefaultInterruptHandler)(void);

/**
  Section: Driver Interface
*/


void UART1_Initialize (void)
{
   IEC0bits.U1TXIE = 0;
   IEC0bits.U1RXIE = 0;
   
   // STSEL 1; IREN disabled; PDSEL 8N; UARTEN enabled; RTSMD disabled; USIDL disabled; WAKE disabled; ABAUD disabled; LPBACK disabled; BRGH enabled; URXINV disabled; UEN TX_RX; 
   U1MODE = (0x8008 & ~(1<<15));  // disabling UART
   // UTXISEL0 TX_ONE_CHAR; UTXINV disabled; URXEN disabled; OERR NO_ERROR_cleared; URXISEL RX_ONE_CHAR; UTXBRK COMPLETED; UTXEN disabled; ADDEN disabled; 
   U1STA = 0x0000;
   // BaudRate = 9600; Frequency = 16000000 Hz; U1BRG 416; 
   U1BRG = 0x01A0;
   // ADMADDR 0; ADMMASK 0; 
   U1ADMD = 0x0000;
   
   txHead = txQueue;
   txTail = txQueue;
   rxHead = rxQueue;
   rxTail = rxQueue;
   
   rxOverflowed = 0;

   UART1_SetTxInterruptHandler(UART1_Transmit_ISR);

   UART1_SetRxInterruptHandler(UART1_Receive_ISR);
   IEC0bits.U1RXIE = 1;
   
    //Make sure to set LAT bit corresponding to TxPin as high before UART initialization
   U1MODEbits.UARTEN = 1;  // enabling UART ON bit
   U1STAbits.UTXEN = 1;
}

/**
    Maintains the driver's transmitter state machine and implements its ISR
*/
void UART1_SetTxInterruptHandler(void* handler){
    UART1_TxDefaultInterruptHandler = handler;
}

void __attribute__ ( ( interrupt, no_auto_psv ) ) _U1TXInterrupt ( void )
{
    (*UART1_TxDefaultInterruptHandler)();
}

void UART1_Transmit_ISR(void)
{ 
    if(txHead == txTail)
    {
        IEC0bits.U1TXIE = 0;
        return;
    }

    IFS0bits.U1TXIF = 0;

    while(!(U1STAbits.UTXBF == 1))
    {
        U1TXREG = *txHead++;

        if(txHead == (txQueue + UART1_CONFIG_TX_BYTEQ_LENGTH))
        {
            txHead = txQueue;
        }

        // Are we empty?
        if(txHead == txTail)
        {
            break;
        }
    }
}

void UART1_SetRxInterruptHandler(void* handler){
    UART1_RxDefaultInterruptHandler = handler;
}

void __attribute__ ( ( interrupt, no_auto_psv ) ) _U1RXInterrupt( void )
{
    (*UART1_RxDefaultInterruptHandler)();
}

void UART1_Receive_ISR(void)
{
    while((U1STAbits.URXDA == 1))
    {
        *rxTail = U1RXREG;

        // Will the increment not result in a wrap and not result in a pure collision?
        // This is most often condition so check first
        if ( ( rxTail    != (rxQueue + UART1_CONFIG_RX_BYTEQ_LENGTH-1)) &&
             ((rxTail+1) != rxHead) )
        {
            rxTail++;
        } 
        else if ( (rxTail == (rxQueue + UART1_CONFIG_RX_BYTEQ_LENGTH-1)) &&
                  (rxHead !=  rxQueue) )
        {
            // Pure wrap no collision
            rxTail = rxQueue;
        } 
        else // must be collision
        {
            rxOverflowed = true;
        }

    }

    IFS0bits.U1RXIF = 0;
}

void __attribute__ ( ( interrupt, no_auto_psv ) ) _U1ErrInterrupt ( void )
{
    if ((U1STAbits.OERR == 1))
    {
        U1STAbits.OERR = 0;
    }

    IFS4bits.U1ERIF = 0;
}

/**
  Section: UART Driver Client Routines
*/

uint8_t UART1_Read( void)
{
    uint8_t data = 0;

    while (rxHead == rxTail )
    {
    }
    
    data = *rxHead;

    rxHead++;

    if (rxHead == (rxQueue + UART1_CONFIG_RX_BYTEQ_LENGTH))
    {
        rxHead = rxQueue;
    }
    return data;
}

void UART1_Write( uint8_t byte)
{
    while(UART1_IsTxReady() == 0)
    {
    }

    *txTail = byte;

    txTail++;
    
    if (txTail == (txQueue + UART1_CONFIG_TX_BYTEQ_LENGTH))
    {
        txTail = txQueue;
    }

    IEC0bits.U1TXIE = 1;
}

bool UART1_IsRxReady(void)
{    
    return !(rxHead == rxTail);
}

bool UART1_IsTxReady(void)
{
    uint16_t size;
    uint8_t *snapshot_txHead = (uint8_t*)txHead;
    
    if (txTail < snapshot_txHead)
    {
        size = (snapshot_txHead - txTail - 1);
    }
    else
    {
        size = ( UART1_CONFIG_TX_BYTEQ_LENGTH - (txTail - snapshot_txHead) - 1 );
    }
    
    return (size != 0);
}

bool UART1_IsTxDone(void)
{
    if(txTail == txHead)
    {
        return (bool)U1STAbits.TRMT;
    }
    
    return false;
}

int __attribute__((__section__(".libc.write"))) write(int handle, void *buffer, unsigned int len) {
    unsigned int i;
    uint8_t *data = buffer;

    for(i=0; i<len; i++)
    {
        while(UART1_IsTxReady() == false)
        {
        }

        UART1_Write(*data++);
    }
  
    return(len);
}

/* !!! Deprecated API - This function may not be supported in a future release !!! */
static uint8_t UART1_RxDataAvailable(void)
{
    uint16_t size;
    uint8_t *snapshot_rxTail = (uint8_t*)rxTail;
    
    if (snapshot_rxTail < rxHead) 
    {
        size = ( UART1_CONFIG_RX_BYTEQ_LENGTH - (rxHead-snapshot_rxTail));
    }
    else
    {
        size = ( (snapshot_rxTail - rxHead));
    }
    
    if(size > 0xFF)
    {
        return 0xFF;
    }
    
    return size;
}

/* !!! Deprecated API - This function may not be supported in a future release !!! */
uint8_t __attribute__((deprecated)) UART1_is_rx_ready(void)
{
    return UART1_RxDataAvailable();
}

/* !!! Deprecated API - This function may not be supported in a future release !!! */
static uint8_t UART1_TxDataAvailable(void)
{
    uint16_t size;
    uint8_t *snapshot_txHead = (uint8_t*)txHead;
    
    if (txTail < snapshot_txHead)
    {
        size = (snapshot_txHead - txTail - 1);
    }
    else
    {
        size = ( UART1_CONFIG_TX_BYTEQ_LENGTH - (txTail - snapshot_txHead) - 1 );
    }
    
    if(size > 0xFF)
    {
        return 0xFF;
    }
    
    return size;
}

/* !!! Deprecated API - This function may not be supported in a future release !!! */
uint8_t __attribute__((deprecated)) UART1_is_tx_ready(void)
{
    return UART1_TxDataAvailable();
}

/* !!! Deprecated API - This function may not be supported in a future release !!! */
bool __attribute__((deprecated)) UART1_is_tx_done(void)
{
    return UART1_IsTxDone();
}

/* !!! Deprecated API - This function may not be supported in a future release !!! */
unsigned int __attribute__((deprecated)) UART1_ReadBuffer( uint8_t *buffer ,  unsigned int numbytes)
{
    unsigned int rx_count = UART1_RxDataAvailable();
    unsigned int i;
    
    if(numbytes < rx_count)
    {
        rx_count = numbytes;
    }
    
    for(i=0; i<rx_count; i++)
    {
        *buffer++ = UART1_Read();
    }
    
    return rx_count;    
}

/* !!! Deprecated API - This function may not be supported in a future release !!! */
unsigned int __attribute__((deprecated)) UART1_WriteBuffer( uint8_t *buffer , unsigned int numbytes )
{
    unsigned int tx_count = UART1_TxDataAvailable();
    unsigned int i;
    
    if(numbytes < tx_count)
    {
        tx_count = numbytes;
    }
    
    for(i=0; i<tx_count; i++)
    {
        UART1_Write(*buffer++);
    }
    
    return tx_count;  
}

/* !!! Deprecated API - This function may not be supported in a future release !!! */
UART1_TRANSFER_STATUS __attribute__((deprecated)) UART1_TransferStatusGet (void )
{
    UART1_TRANSFER_STATUS status = 0;
    uint8_t rx_count = UART1_RxDataAvailable();
    uint8_t tx_count = UART1_TxDataAvailable();
    
    switch(rx_count)
    {
        case 0:
            status |= UART1_TRANSFER_STATUS_RX_EMPTY;
            break;
        case UART1_CONFIG_RX_BYTEQ_LENGTH:
            status |= UART1_TRANSFER_STATUS_RX_FULL;
            break;
        default:
            status |= UART1_TRANSFER_STATUS_RX_DATA_PRESENT;
            break;
    }
    
    switch(tx_count)
    {
        case 0:
            status |= UART1_TRANSFER_STATUS_TX_FULL;
            break;
        case UART1_CONFIG_RX_BYTEQ_LENGTH:
            status |= UART1_TRANSFER_STATUS_TX_EMPTY;
            break;
        default:
            break;
    }

    return status;    
}

/* !!! Deprecated API - This function may not be supported in a future release !!! */
uint8_t __attribute__((deprecated)) UART1_Peek(uint16_t offset)
{
    uint8_t *peek = rxHead + offset;
    
    while(peek > (rxQueue + UART1_CONFIG_RX_BYTEQ_LENGTH))
    {
        peek -= UART1_CONFIG_RX_BYTEQ_LENGTH;
    }
    
    return *peek;
}

/* !!! Deprecated API - This function may not be supported in a future release !!! */
bool __attribute__((deprecated)) UART1_ReceiveBufferIsEmpty (void)
{
    return (UART1_RxDataAvailable() == 0);
}

/* !!! Deprecated API - This function may not be supported in a future release !!! */
bool __attribute__((deprecated)) UART1_TransmitBufferIsFull (void)
{
    return (UART1_TxDataAvailable() == 0);
}

/* !!! Deprecated API - This function may not be supported in a future release !!! */
UART1_STATUS __attribute__((deprecated)) UART1_StatusGet (void )
{
    return U1STA;
}




// THIS IS

#define FCY     16000000UL
#include <xc.h>
#include <stdint.h>


void dmaInit(void) {
    DMACONbits.DMAEN = 1;         // Включить DMA
    DMACONbits.PRSSEL = 0;        // Использовать фиксированную схему приоритетов
    DMAL = 0x0000;                // Установить диапазон DMA. Разрешить работу со всей памятью
    DMAH = 0xFFFF;
}


void uartInit(void) {
    U1MODE = 0;
    U1MODEbits.UARTEN = 1;        // Включить периферийное устройство перед его настройкой
    U1MODEbits.UEN = 0;           // Отключить управление потоком
    U1MODEbits.BRGH = 1;          // Установить высокую скорость BRG (x4)

    U1STA = 0;
    U1STAbits.UTXISEL1 = 0;       // Генерировать прерывание TX, когда в FIFO есть место
    U1STAbits.UTXISEL0 = 0;
    U1STAbits.URXEN = 1;          // Включить приёмник
    U1STAbits.UTXEN = 1;          // Включить передатчик
    U1STAbits.URXISEL = 0;        // Генерировать RX-прерывание при приёме любого символа

    U1BRG = FCY / (4UL * 115200); // Установить скорость передачи

    _U1TXIF = 0;                  // Очистить флаг UART TX, чтобы предотвратить ошибочную работу DMA

    // Настройка DMA канала
    DMACH0 = 0;
    DMACH0bits.SAMODE = 1;        // Адрес источника инкрементируется после передачи
    DMACH0bits.DAMODE = 0;        // Адрес приёмника не меняется после передачи
    DMACH0bits.TRMODE = 1;        // Режим One Shot
    DMACH0bits.SIZE = 1;          // Размер передачи — 1 байт

    DMAINT0 = 0;
    DMAINT0bits.CHSEL = 68;       // Триггер DMA — прерывание U5 TX

    DMADST0 = (uint16_t)&U1TXREG;

    // Настройка прерываний и приоритетов
    _DMA0IF = 0;
    _DMA0IE = 1;
    _DMA0IP = 3;

    _U1RXIF = 0;
    _U1RXIE = 1;
    _U1RXIP = 3;
}


void uart_startTransfer(uint8_t *memory, uint16_t size) {
    DMASRC0 = (uint16_t)memory;   // Адрес источника данных
    DMACNT0 = size;               // Количество байт для передачи
    DMACH0bits.CHEN = 1;          // Включить DMA канал
    DMACH0bits.CHREQ = 1;         // Запустить операцию DMA
}

volatile bool dmaBusy = false;

void __attribute__((interrupt,no_auto_psv)) _DMA0Interrupt(void) {
    _DMA0IF = 0;
    dmaBusy = false; // передача завершена
}

static bool flag = true;

int some_func(void)
{
    if(flag) {
        dmaInit();
        uartInit();
        flag = false;
    }

    uint8_t message[] = "Hello DMA UART!\r\n";

    if(!dmaBusy) {
        dmaBusy = true;
        uart_startTransfer(message, sizeof(message)-1);
    }
}
