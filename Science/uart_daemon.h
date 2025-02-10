/* ************************************************************************** */
/** Descriptive File Name

  @Company
    Company Name

  @File Name
    filename.h

  @Summary
    Brief description of the file.

  @Description
    Describe the purpose of this file.
 */
/* ************************************************************************** */

#ifndef _UART_DAEMON_H    /* Guard against multiple inclusion */
#define _UART_DAEMON_H


#include "PIC32Arch.h"
#include <xc.h>
    

#include <stdint.h>

#define MAX_UART_RX_LEN (82)

#define UART_RX_QUEUE_LENGTH 255
#define UART_TX_QUEUE_LENGTH 255

#define ONE_INT32_MAX_LEN     12

static QueueHandle_t uartRxQueue;
static QueueHandle_t uartTxQueue;

static TaskHandle_t xUARTrx_Task_Handle = NULL;

    typedef enum {
        LOW = 9600,
        NORMAL = 115200
    } _baudrate_t;

    typedef struct {
        uint16_t fifoOverruns;
        uint16_t badParity;
        uint16_t framingErrors;
        uint16_t queueFull;
    } uart_stats_t;
    
    void uart2_Init(void);
    void uart2_println(const char *str);
    void uart2_print(const char *str);
    void uart2_enable(void );
//    void _uart2_TxTask(void *pvParameters);
//    void _uart2_RxTask(void *pvParameters);
    
    void        _uart2_SendString(const char *str);
    uint16_t    _calcUartBaudRate(uint32_t fpb, uint32_t baud);
    void        _uart2_rx_handle( void );
    
    
    void vLoggingPrintf(const char *pcFormat, ...);
    
    extern uart_stats_t uart_rx_stats;
    
#endif /* _UART_DAEMON_H */

/* *****************************************************************************
 End of File
 */
