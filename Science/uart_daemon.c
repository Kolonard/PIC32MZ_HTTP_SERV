
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

#define BAUDRATE    115200

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
    while (*str) {
        xQueueSend(uartTxQueue, str++, portMAX_DELAY);
    }  
    EnableTxInterrupt(); // ???????? ?????????? TX (????? ????????)
}

void uart2_Init() {
    // ???????? ????????
    
  
    uartRxQueue = xQueueCreate(UART_RX_QUEUE_LENGTH, sizeof(uint8_t));
    uartTxQueue = xQueueCreate(UART_TX_QUEUE_LENGTH, sizeof(uint8_t));
    
    
    U2BRG = _calcUartBaudRate(configPERIPHERAL_CLOCK_HZ, BAUDRATE);
    
    U2MODEbits.PDSEL = 0; //8bit no parity   
    U2MODEbits.STSEL = 0; //1 stop bit       
    U2STAbits.UTXISEL = 0;    
    
    IPC36bits.U2TXIP = configKERNEL_INTERRUPT_PRIORITY;
    IPC36bits.U2RXIP = configKERNEL_INTERRUPT_PRIORITY;
    IPC36bits.U2EIP = configKERNEL_INTERRUPT_PRIORITY;
    
    ClearAllInterrupts();
    EnableRxInterrupt();

    U2STASET = _U2STA_UTXEN_MASK | _U2STA_URXEN_MASK;
    U2MODESET = _U2MODE_ON_MASK;
//    U2MODEbits.SIDL = 0;
//    U2MODEbits.IREN = 0;
//    U2MODEbits.RTSMD = 1;
//    U2MODEbits.UEN = 0x10;
//    U2MODEbits.WAKE = 1;
//    U2MODEbits.LPBACK = 0;
//    U2MODEbits.ABAUD = 0;
//    U2MODEbits.RXINV = 1;
//    U2MODEbits.BRGH = 0;
//    U2MODEbits.PDSEL = 0x00; //!8-bit data, No parity
//    U2MODEbits.STSEL = 0; //! 1 Stop bit
//
//    U2STAbits.ADM_EN = 1;
//    U2STAbits.UTXISEL = 0x03; //! Tx. Interrupt is generated when transmit buffer becomes empty
//    U2STAbits.UTXINV = 1;
//    U2STAbits.UTXBRK = 0;
//    U2STAbits.URXISEL = 0x00; //! Rx. Interrupt flag bit is set when a char is received
//    U2STAbits.ADDEN = 1;
//    U2STAbits.OERR = 0;
////
//    IPC36bits.U2TXIP = 0b001; //! Interrupt priority of 7
//    IPC36bits.U2TXIS = 0b00; //! Interrupt sub-priority of 0
//    IPC36bits.U2RXIP = 0b111; //! Interrupt priority of 7
//    IPC36bits.U2RXIS = 0b00; //! Interrupt sub-priority of 0
//    IEC4SET = _IEC4_U2TXIE_MASK; //! Tx INT Enable
//    IEC4SET = _IEC4_U2RXIE_MASK; //! Rx INT Enable
////    U2BRG = _calcUartBaudRate(configPERIPHERAL_CLOCK_HZ, BAUDRATE);//((PBCLK2 / 115200) / 16) - 1;
//    U2MODEbits.ON = 1; //! enable UART
//    U2STAbits.UTXEN = 1; //! enable transmit pin
//    U2STAbits.URXEN = 1; //! enable receive pin
//    IFS4bits.U2TXIF = 0; //! Clear Tx flag
//    IFS4bits.U2RXIF = 0; //! Clear Rx flag    
    
    
//    U2STAbits.URXEN = 1; //enable receiver
//    U2STAbits.UTXEN = 1; //enable transmitter 
////    ClearAllInterrupts();
////    EnableRxInterrupt();
////    __builtin_enable_interrupts();
//
////    U2STASET  = _U2STA_UTXEN_MASK | _U2STA_URXEN_MASK;
////    U2MODESET = _U2MODE_ON_MASK;
//    U2MODEbits.ON = 1;
//          __builtin_disable_interrupts();        
////    IPC36bits.U2TXIP = configKERNEL_INTERRUPT_PRIORITY;
////    IPC36bits.U2TXIS = 0;
////    IPC36bits.U2RXIP = configKERNEL_INTERRUPT_PRIORITY;
////    IPC36bits.U2RXIS = 0;
////    IPC36bits.U2EIP  = configKERNEL_INTERRUPT_PRIORITY;
////    IEC4bits.U2RXIE =1;
//    U2STAbits.URXISEL = 0; //interrupt when rx buffer is not empty
//    IPC36bits.U2RXIP = 1; //priority 1 (1~7)
//    IPC36bits.U2RXIS = 0; //sub-priority 0 (0~3)
//    IFS4bits.U2RXIF = 0; //clear flag
//    IEC4bits.U2RXIE =1; //enable uart2 rx interrupt    
//    __builtin_enable_interrupts();

    
    xTaskCreate(_uart2_rx_handle,"uart_rx_handle",620,NULL,tskIDLE_PRIORITY + 1, &xUARTrx_Task_Handle);
}
void uart2_enable(void ){
//    U2MODEbits.ON = 1; //! enable UART
}
void _uart2_rx_handle( void ){
    
    char str[ONE_INT32_MAX_LEN];
    uint8_t msg[UART_RX_QUEUE_LENGTH];  
    uint32_t ul_UARTrx_time_stamp;
    
       
    while(true){
        if (xTaskNotifyWait(0, 0, &ul_UARTrx_time_stamp, portMAX_DELAY) == pdTRUE) ;
        if (xQueueReceive(uartRxQueue, msg, 0) == pdTRUE) {
            
            sprintf(str, "%d", ul_UARTrx_time_stamp);
            uart2_println(str);
            uart2_println(msg);
        }   
    }
}
    
//
//void Uart2TxInterruptHandler(void)
//{
//    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
//    uint8_t byte;
//    ClearTxInterrupt();
//    while( !U2STAbits.UTXBF )
//    {
//        if( !xQueueReceiveFromISR(uartTxQueue, &byte, &xHigherPriorityTaskWoken) )
//        {
//            DisableTxInterrupt();
//            break;          
//        }
//        U2TXREG = byte;
//    }
//    portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
//}
//
//void Uart2FaultInterruptHandler(void)
//{
//    ClearErrorInterrupt();
//}


//void Uart2RxInterruptHandler(void)
//{
//    if (U2STAbits.OERR) {
//        U2STAbits.OERR = 0; // ????? ?????? ????????????
//    }
//    
//    if (IFS4bits.U2RXIF) {
//        char received_char = U2RXREG; // ?????? ???????? ??????
//        U2TXREG = received_char;      // ?????????? ??? ??????? (???)
//                 // ?????????? ???? ??????????
//    }
//    IFS4bits.U2RXIF = 0; 
//}


void  __ISR(_UART2_RX_VECTOR, IPL1AUTO) UART2_RX_Handler(void) {
//(_UART2_RX_VECTOR), interrupt(IPL7AUTO), nomips16)) UART2RxISR(void){//void __ISR(_UART2_RX_VECTOR, IPL7AUTO) UART2_RX_ISR(void) {
    
    if (U2STAbits.OERR) {
        U2STAbits.OERR = 0; // ????? ?????? ????????????
    }
    
    if (IFS4bits.U2RXIF) {
        char received_char = U2RXREG; // ?????? ???????? ??????
        U2TXREG = received_char;      // ?????????? ??? ??????? (???)
                 // ?????????? ???? ??????????
    }
    IFS4bits.U2RXIF = 0; 
}

void __ISR(_UART2_TX_VECTOR, IPL1AUTO) UART2_TX_Handler(void) {
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
        U2TXREG = byte;
    }
    portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}

