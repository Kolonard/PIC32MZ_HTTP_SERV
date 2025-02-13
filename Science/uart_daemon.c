
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
#include "uart_daemon.h" 

#define LED_PIN LATHbits.LATH2
#define LED_TRIS TRISHbits.TRISH2

//void __attribute__(( interrupt(IPL0AUTO), vector(_UART2_FAULT_VECTOR) ))    Uart2FaultInterruptWrapper(void);
//void __attribute__(( interrupt(IPL0AUTO), vector(_UART2_TX_VECTOR) ))       Uart2TxInterruptWrapper(void);
//void __attribute__(( interrupt(IPL0AUTO), vector(_UART2_RX_VECTOR) ))       Uart2RxInterruptWrapper(void);

#define ClearAllInterrupts()    IFS4CLR = _IFS4_U2EIF_MASK | _IFS4_U2RXIF_MASK | _IFS4_U2TXIF_MASK
#define ClearErrorInterrupt()   IFS4CLR = _IFS4_U2EIF_MASK
#define ClearRxInterrupt()      IFS4CLR = _IFS4_U2RXIF_MASK
#define EnableRxInterrupt()     IEC4SET = _IEC4_U2RXIE_MASK
#define DisableRxInterrupt()    IEC4CLR = _IEC4_U2RXIE_MASK
#define ClearTxInterrupt()      IFS4CLR = _IFS4_U2TXIF_MASK
#define EnableTxInterrupt()     IEC4SET = _IEC4_U2TXIE_MASK
#define DisableTxInterrupt()    IEC4CLR = _IEC4_U2TXIE_MASK

#define ULONG_MAX_  0xFFFFFFFFUL

#define BAUDRATE    115200


typedef struct {
    uint32_t timestamp;
    uint8_t data[UART_RX_QUEUE_LENGTH]; // Произвольный размер данных
    uint8_t length;   // Длина данных
} Packet_t;



uart_stats_t uart_rx_stats;

void vLoggingPrintf(const char *pcFormat, ...)
{
    va_list ap;
    va_start(ap, pcFormat);
    vprintf(pcFormat, ap);
    va_end(ap);
}

uint16_t _calcUartBaudRate(uint32_t fpb, uint32_t baud)
{
    return ((fpb / (16.0f * baud)) - 0.5f);
}

void _uart2_SendByteFromISR(uint8_t byte) {
    while (U2STAbits.UTXBF); // ????, ???? ????? ??????????
    U2TXREG = byte;
}

void uart2_println(const char *str){
    _uart2_SendString(str);
    _uart2_SendString("\r\n");
}

void uart2_print(const char *str){
    _uart2_SendString(str);
}


void _uart2_SendString(const char *str) {
    xMutex = xSemaphoreCreateMutex();
    if (xMutex != NULL) {
        if (xSemaphoreTake(xMutex, portMAX_DELAY)){
            while (*str) {
                xQueueSend(uartTxQueue, str++, NULL);
            }  
            xSemaphoreGive(xMutex);
            EnableTxInterrupt();
        }
    } 
}

void _uart2_SendByteArray(uint8_t *array, uint8_t len_array) {
    uint8_t ii = 0;
    while ( ii < len_array) {
        xQueueSend(uartTxQueue, array[ii], portMAX_DELAY);
        ii++;
    }
    EnableTxInterrupt();
}




void uart2_Init() {
    // UART2 setup & initialization
    _uart2_PPS_setup();
    
    uartRxQueue = xQueueCreate(UART_RX_QUEUE_LENGTH, sizeof(uint8_t));
    uartTxQueue = xQueueCreate(UART_TX_QUEUE_LENGTH, sizeof(uint8_t));
        
    U2BRG = _calcUartBaudRate(configPERIPHERAL_CLOCK_HZ, BAUDRATE);
    
    U2MODEbits.PDSEL = 0; //8bit no parity   
    U2MODEbits.STSEL = 0; //1 stop bit       
    U2STAbits.UTXISEL = 0;    
    
    IPC36bits.U2TXIP    = configKERNEL_INTERRUPT_PRIORITY;
    IPC36bits.U2RXIP    = configKERNEL_INTERRUPT_PRIORITY;
    IPC36bits.U2EIP     = configKERNEL_INTERRUPT_PRIORITY;
    
    ClearAllInterrupts();
    EnableRxInterrupt();

    U2STASET = _U2STA_UTXEN_MASK | _U2STA_URXEN_MASK;
    U2MODESET = _U2MODE_ON_MASK;
   
    xTaskCreate(_uart2_rx_handle,
                "uart_rx_handle",
                2048,
                NULL,
                tskIDLE_PRIORITY + 1,
                &xUARTrx_Task_Handle);
}

void uart2_enable(void ){
    U2MODEbits.ON = 1; //! enable UART
}
    
void _uart2_rx_handle( void ){
  
    char str[ONE_INT32_MAX_LEN];
    static uint8_t msg[UART_RX_QUEUE_LENGTH];
    uint32_t ul_UARTrx_time_stamp = 0;
    char byte;
    uint8_t ii;
    
    volatile UBaseType_t count = 0;
    
    Packet_t receivedPacket;
    receivedPacket.timestamp = 05005;
    receivedPacket.length = 15;
    
    UBaseType_t freeStack = uxTaskGetStackHighWaterMark(NULL);
    
    sprintf(str, "%d", freeStack);
    uart2_println(str);
    
    while(true){
        
//        
//            
////            ii = 0;
//        count =  uxQueueMessagesWaiting(uartRxQueue);
//    while (pdTRUE == xQueueReceive(uartRxQueue, &msg, portMAX_DELAY)
        
        
        
//        if (xTaskNotifyWait(0, ULONG_MAX_, &ul_UARTrx_time_stamp, portMAX_DELAY) == pdTRUE);
        ii = 0;
//        while(xQueueReceive(uartRxQueue, &byte, portMAX_DELAY) == pdTRUE){
//            
//            if (byte == '\r' || byte == '\n'){
//                break;
//            }
//            msg[ii] = byte;
//            ii++;
//        }
        xQueueReceive(uartRxQueue, &msg, portMAX_DELAY);
//        msg[ii++] = '\0';
//        uart2_print("input: ");
        uart2_println(msg);
            
//            sprintf(str, "%c", byte);
//            uart2_print(str);
//            
//            freeStack = uxTaskGetStackHighWaterMark(NULL);
//            sprintf(str, "stakksldkfsldkfskdf;lkck %d", freeStack);
//            uart2_println(str);
            
        

        

////        if (xQueueReceive(uartRxQueue, &receivedPacket, portMAX_DELAY) == pdTRUE){ 
////        while (xQueueReceive(uartRxQueue, &byte, 0) == pdTRUE) {
////            msg[ii] = byte;
////            ii++;
////        }
////?        if (xQueueReceive(uartRxQueue, &msg, portMAX_DELAY) == pdTRUE){
//            count =  uxQueueMessagesWaiting(uartRxQueue);
//            
//            sprintf(str, "%d", ul_UARTrx_time_stamp);
////            uart2_print(str);
////            uart2_print(": ");
////            sprintf(str, "%d%d%d%d", msg[0], msg[1], msg[2], msg[3]);
////            
////            uart2_println(str);
////            uart2_println(msg);
////        }        
//        }

//        _uart2_SendByteArray(msg,ii);     
    }
}

static uint8_t rxBuffer[UART_RX_QUEUE_LENGTH];
static uint16_t rxIndex = 0;

void  __ISR(_UART2_RX_VECTOR, IPL1AUTO) UART2_RX_Handler(void) {
       

    while( U2STAbits.URXDA )
    {
        
//        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
//    TickType_t ticks = xTaskGetTickCountFromISR();
        volatile uint8_t rxchar;
//        
        
        
        rxchar = U2RXREG;
        
        if (rxIndex < UART_RX_QUEUE_LENGTH - 1) // Проверка границ буфера
        {
            rxBuffer[rxIndex++] = rxchar;
            if (rxchar == 0x0A) // Если встретили конец строки (0x0A)
            {
                rxBuffer[rxIndex] = '\0'; // Завершаем строку
                if (!xQueueSendFromISR(uartRxQueue, rxBuffer, NULL) == pdPASS){
                    break;
                }; // Отправляем в очередь
                rxIndex = 0; // Сбрасываем индекс
            }
        }
        else
        {
            rxIndex = 0; // Сброс при переполнении
        }
        
        
//        while(rxchar != 0x0D && rxchar != 0x0A){
//        xQueueSendFromISR(uartRxQueue, &rxchar, &xHigherPriorityTaskWoken);
//        }
        
////        if( !xQueueSendFromISR(uartRxQueue, &rxchar, &xHigherPriorityTaskWoken) ){
//////            s_UartStats.queueFull++;
////            break;
////        }
    }
    rxIndex = 0;
    IFS4bits.U2RXIF = 0;
//    ClearRxInterrupt();
//    portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
//    xTaskNotifyFromISR(xUARTrx_Task_Handle, ticks, eSetValueWithOverwrite, NULL);
}

void __ISR(_UART2_TX_VECTOR, IPL1AUTO) UART2_TX_Handler(void) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint8_t byte;
    ClearTxInterrupt();
    while( !U2STAbits.UTXBF )
    {
        if( !xQueueReceiveFromISR(uartTxQueue, &byte, &xHigherPriorityTaskWoken ) )
        {
            DisableTxInterrupt();
            break;          
        }
        U2TXREG = byte;
    }
    portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}

void        _uart2_PPS_setup( void ){
    
    // Setup PPS to uart2rx -> RB15, uart2tx -> RB14,       
    SYSKEY_UNLOCK();
    
    ANSELBCLR = _ANSELB_ANSB14_MASK | _ANSELB_ANSB15_MASK; // analog off
    TRISBSET = _TRISB_TRISB15_MASK;// input mode for RB15
    U2RXRbits.U2RXR = 0b0011;   // uart2rx  -> RPB15
    RPB14Rbits.RPB14R = 0b0010; // RB14     -> uart2tx 
    
    SYSKEY_LOCK();

}
