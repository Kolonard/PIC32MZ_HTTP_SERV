
// RTOS
#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>
// C Runtime
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <xc.h>


#include <xc.h>
#include <p32xxxx.h>
#include <sys/attribs.h>
#include "uart_rx_tx.h" 

#define ClearAllInterrupts()    IFS4CLR = _IFS4_U2EIF_MASK | _IFS4_U2RXIF_MASK | _IFS4_U2TXIF_MASK
#define ClearErrorInterrupt()   IFS4CLR = _IFS4_U2EIF_MASK
#define ClearRxInterrupt()      IFS4CLR = _IFS4_U2RXIF_MASK
#define EnableRxInterrupt()     IEC4SET = _IEC4_U2RXIE_MASK
#define DisableRxInterrupt()    IEC4CLR = _IEC4_U2RXIE_MASK
#define ClearTxInterrupt()      IFS4CLR = _IFS4_U2TXIF_MASK
#define EnableTxInterrupt()     IEC4SET = _IEC4_U2TXIE_MASK
#define DisableTxInterrupt()    IEC4CLR = _IEC4_U2TXIE_MASK

#define _XTAL_FREQ 200000000UL
#define BAUDRATE    115200

void vLoggingPrintf(const char *pcFormat, ...)
{
    va_list ap;
    va_start(ap, pcFormat);
    vprintf(pcFormat, ap);
    va_end(ap);
}



void UART2_RxTask(void *pvParameters) {
//    UartPacket rxPacket;
//    while (1) {
//        if (xQueueReceive(uartRxQueue, &rxPacket, portMAX_DELAY)) {
////            processReceivedPacket(&rxPacket);
//        }
//    }
}
void UART2_TxTask(void *pvParameters) {
//    UartPacket txPacket;
//
//    // Пример пакета
//    txPacket.header = 0xAA;
//    txPacket.length = 3;
//    txPacket.data[0] = 0x01;
//    txPacket.data[1] = 0x02;
//    txPacket.data[2] = 0x03;
//    txPacket.checksum = txPacket.header + txPacket.length + txPacket.data[0] + txPacket.data[1] + txPacket.data[2];
//
//    while (1) {
//        xQueueSend(uartTxQueue, &txPacket, portMAX_DELAY);
//        EnableTxInterrupt(); // Включаем прерывание TX
//        vTaskDelay(pdMS_TO_TICKS(1000));
//    }
}

uint16_t CalcUartBaudRate(uint32_t fpb, uint32_t baud)
{
    return ((fpb / (16.0f * baud)) - 0.5f);
}

void UART2_SendByteFromISR(uint8_t byte) {
    while (U2STAbits.UTXBF); // Ждём, если буфер переполнен
    U2TXREG = byte;
}

void uart2_println(const char *str){
    UART2_SendString(str);
    UART2_SendString("\r\n");
}

void UART2_SendString(const char *str) {
    while (*str) {
        xQueueSend(uartTxQueue, str++, portMAX_DELAY);
    }  
    EnableTxInterrupt(); // Включаем прерывание TX (старт передачи)
}

void UART2_Init() {
    // Создание очередей
    uartRxQueue = xQueueCreate(UART_RX_QUEUE_LENGTH, sizeof(uint8_t));
    uartTxQueue = xQueueCreate(UART_TX_QUEUE_LENGTH, sizeof(uint8_t));

    // Отключаем UART перед конфигурацией
//    U2MODEbits.ON = 0;
    
    
    U2BRG = CalcUartBaudRate(configPERIPHERAL_CLOCK_HZ, BAUDRATE);
    //    return ((fpb / (16.0f * baud)) - 0.5f);
//    U2BRG = ((configCPU_CLOCK_HZ / (16.0f * BAUDRATE)) - 0.5f); // UxBRG(HIGH SPEED) = Fpb / 4 / Baudrate - 1
    U2MODEbits.PDSEL = 0; //8bit no parity   
    U2MODEbits.STSEL = 0; //1 stop bit       
    U2STAbits.UTXISEL = 0;    
//    IPC36bits.U2TXIP = configKERNEL_INTERRUPT_PRIORITY;
//    IPC36bits.U2RXIP = configKERNEL_INTERRUPT_PRIORITY;
//    IPC36bits.U2EIP = configKERNEL_INTERRUPT_PRIORITY;
    
            
    U2STAbits.URXEN = 1; //enable receiver
    U2STAbits.UTXEN = 1; //enable transmitter           

    __builtin_disable_interrupts();
    IPC36bits.U2TXIP = configKERNEL_INTERRUPT_PRIORITY;
    IPC36bits.U2RXIP = configKERNEL_INTERRUPT_PRIORITY;
    IPC36bits.U2EIP  = configKERNEL_INTERRUPT_PRIORITY;
    
    ClearAllInterrupts();
    EnableRxInterrupt();
    __builtin_enable_interrupts();

    U2STASET  = _U2STA_UTXEN_MASK | _U2STA_URXEN_MASK;
    U2MODESET = _U2MODE_ON_MASK;
//    U2MODEbits.ON = 1;
    
    
//    U2STASET = _U2STA_UTXEN_MASK | _U2STA_URXEN_MASK;

//    U2TXREG = 0xC0;
//    U2TXREG = 0xC1;
//    U2TXREG = 0xC2;
    
    
//    xTaskCreate(UART2_RxTask, "UART2 RX Task", 1024, NULL, 2, NULL);
//    xTaskCreate(UART2_TxTask, "UART2 TX Task", 1024, NULL, 2, NULL);
}


void __ISR(_UART2_RX_VECTOR, IPL1SRS) UART2_RX_Handler(void) {
//    static UartPacket rxPacket;
//    static uint16_t rxIndex = 0;
//    uint8_t received_byte;
//    
//    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
//
////    ClearTxInterrupt();
//    
//    while (U2STAbits.URXDA) {  // Пока есть данные в буфере RX
//        received_byte = U2RXREG;
//
//        if (rxIndex == 0) {
//            rxPacket.header = received_byte;
//        } else if (rxIndex < sizeof(rxPacket.data) + 1) {
//            rxPacket.data[rxIndex - 1] = received_byte;
//        } else if (rxIndex == sizeof(rxPacket.data) + 1) {
//            rxPacket.length = received_byte;
//        } else {
//            rxPacket.checksum = received_byte;
//            xQueueSendFromISR(uartRxQueue, &rxPacket, &xHigherPriorityTaskWoken);
//            rxIndex = 0;
//            continue;
//        }
//
//        rxIndex++;
//    }
//
//    IFS4bits.U2RXIF = 0; // Сброс флага прерывания
//    portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}

void __ISR(_UART2_TX_VECTOR, IPL1AUTO) UART2_TX_Handler(void) {
//    static UartPacket txPacket;
//    static uint16_t txIndex = 0;
//    
    
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint8_t byte;
    ClearTxInterrupt();
    
    while( !U2STAbits.UTXBF )
    {
        if( !xQueueReceiveFromISR(uartTxQueue, &byte, &xHigherPriorityTaskWoken) )
        {
            DisableTxInterrupt();
            break;          
        }
//        UART2_SendByteFromISR(byte);//U2TXREG = ch;
        U2TXREG = byte;
    }
    
    
//    
//    if( xQueueReceiveFromISR(uartTxQueue, &byte, &xHigherPriorityTaskWoken) == pdTRUE )
//    {
//        UART2_SendByteFromISR(byte);
//    } else {
//        DisableTxInterrupt();
//    }
//    
    
    portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}

