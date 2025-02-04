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

#ifndef _UART_RX_TX_H    /* Guard against multiple inclusion */
#define _UART_RX_TX_H


#include "PIC32Arch.h"
#include <xc.h>
    

#include <stdint.h>

#define MAX_UART_RX_LEN (82)

#define UART_RX_QUEUE_LENGTH 10
#define UART_TX_QUEUE_LENGTH 10

QueueHandle_t uartRxQueue;
QueueHandle_t uartTxQueue;


    typedef enum {
        LOW = 9600,
        NORMAL = 115200
    } _baudrate_t;

    typedef struct {
        uint8_t header;
        uint8_t data[256];
        uint16_t length;
        uint8_t checksum;
    } UartPacket;
    //int ExampleFunction(int param1, int param2);
//    void uart_rx_tx(_rx_msg_t rx_msg, _tx_msg_t tx_msg, _baudrate_t baudrate, void* params);
//    void UART_Task(void *pvParameters);
//    void UART_imalive_Task(void *pvParameters);
//    void UART_Init()
    
//    
//    void task_UART_Init(void);
//    void UART_TxTask(void *pvParameters);
//    void UART_RxTask(void *pvParameters);
    
    void UART2_Init(void);
    void UART2_TxTask(void *pvParameters);
    void UART2_RxTask(void *pvParameters);
    void vLoggingPrintf(const char *pcFormat, ...);
    
#endif /* _EXAMPLE_FILE_NAME_H */

/* *****************************************************************************
 End of File
 */
