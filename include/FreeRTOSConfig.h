/*
    FreeRTOS V8.2.3 - Copyright (C) 2015 Real Time Engineers Ltd.
    All rights reserved

    VISIT http://www.FreeRTOS.org TO ENSURE YOU ARE USING THE LATEST VERSION.

    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation >>>> AND MODIFIED BY <<<< the FreeRTOS exception.

    ***************************************************************************
    >>!   NOTE: The modification to the GPL is included to allow you to     !<<
    >>!   distribute a combined work that includes FreeRTOS without being   !<<
    >>!   obliged to provide the source code for proprietary components     !<<
    >>!   outside of the FreeRTOS kernel.                                   !<<
    ***************************************************************************

    FreeRTOS is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
    FOR A PARTICULAR PURPOSE.  Full license text is available on the following
    link: http://www.freertos.org/a00114.html

    ***************************************************************************
     *                                                                       *
     *    FreeRTOS provides completely free yet professionally developed,    *
     *    robust, strictly quality controlled, supported, and cross          *
     *    platform software that is more than just the market leader, it     *
     *    is the industry's de facto standard.                               *
     *                                                                       *
     *    Help yourself get started quickly while simultaneously helping     *
     *    to support the FreeRTOS project by purchasing a FreeRTOS           *
     *    tutorial book, reference manual, or both:                          *
     *    http://www.FreeRTOS.org/Documentation                              *
     *                                                                       *
    ***************************************************************************

    http://www.FreeRTOS.org/FAQHelp.html - Having a problem?  Start by reading
    the FAQ page "My application does not run, what could be wrong?".  Have you
    defined configASSERT()?

    http://www.FreeRTOS.org/support - In return for receiving this top quality
    embedded software for free we request you assist our global community by
    participating in the support forum.

    http://www.FreeRTOS.org/training - Investing in training allows your team to
    be as productive as possible as early as possible.  Now you can receive
    FreeRTOS training directly from Richard Barry, CEO of Real Time Engineers
    Ltd, and the world's leading authority on the world's leading RTOS.

    http://www.FreeRTOS.org/plus - A selection of FreeRTOS ecosystem products,
    including FreeRTOS+Trace - an indispensable productivity tool, a DOS
    compatible FAT file system, and our tiny thread aware UDP/IP stack.

    http://www.FreeRTOS.org/labs - Where new FreeRTOS products go to incubate.
    Come and try FreeRTOS+TCP, our new open source TCP/IP stack for FreeRTOS.

    http://www.OpenRTOS.com - Real Time Engineers ltd. license FreeRTOS to High
    Integrity Systems ltd. to sell under the OpenRTOS brand.  Low cost OpenRTOS
    licenses offer ticketed support, indemnification and commercial middleware.

    http://www.SafeRTOS.com - High Integrity Systems also provide a safety
    engineered and independently SIL3 certified version for use in safety and
    mission critical applications that require provable dependability.

    1 tab == 4 spaces!
*/

#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#include <xc.h>

/*-----------------------------------------------------------
 * Application specific definitions.
 *
 * These definitions should be adjusted for your particular hardware and
 * application requirements.
 *
 * THESE PARAMETERS ARE DESCRIBED WITHIN THE 'CONFIGURATION' SECTION OF THE
 * FreeRTOS API DOCUMENTATION AVAILABLE ON THE FreeRTOS.org WEB SITE.
 *
 * See http://www.freertos.org/a00110.html.
 *----------------------------------------------------------*/

#if defined(__32MZ2048EFM144__)
#include "MZ_EFM\FreeRTOSConfig.h"
#elif defined(__32MZ2048ECM144__)
#include "MZ_ECM\FreeRTOSConfig.h"
#elif defined(__32MX795F512L__)
#include "MX_795\FreeRTOSConfig.h"
#elif defined(__32MZ2064DAB288__)
#include "MZ_DAB\FreeRTOSConfig.h"
#else
#error "Unsupported processor"
#endif

#define configUSE_PREEMPTION                            1
#define configUSE_PORT_OPTIMISED_TASK_SELECTION         1
#define configUSE_QUEUE_SETS                            1
#define configUSE_IDLE_HOOK                             1
#define configUSE_TICK_HOOK                             0
#define configTICK_RATE_HZ                              ( ( TickType_t ) 1000 )

#ifdef __MPLAB_DEBUGGER_SIMULATOR
#define configPERIPHERAL_CLOCK_HZ                       configCPU_CLOCK_HZ
#endif

#define configMAX_PRIORITIES                            ( 5UL )
#define configMINIMAL_STACK_SIZE                        ( 80 )
//#define configISR_STACK_SIZE                            ( 400 )
#define configISR_STACK_SIZE                            ( 512 )
#define configTOTAL_HEAP_SIZE                           ( ( size_t ) 131072 )
#define configMAX_TASK_NAME_LEN                         ( 8 )
#define configUSE_TRACE_FACILITY                        1
#define configUSE_16_BIT_TICKS                          0
#define configIDLE_SHOULD_YIELD                         0
#define configUSE_MUTEXES                               1
#define configCHECK_FOR_STACK_OVERFLOW                  3 /* Three also checks the system/interrupt stack. */
#define configQUEUE_REGISTRY_SIZE                       4
#define configUSE_RECURSIVE_MUTEXES                     1
#define configUSE_MALLOC_FAILED_HOOK                    0
#define configUSE_LIST_DATA_INTEGRITY_CHECK_BYTES       0
#define configUSE_APPLICATION_TASK_TAG                  0
#define configNUM_THREAD_LOCAL_STORAGE_POINTERS         3
#define configUSE_COUNTING_SEMAPHORES                   1

/* Run time stats gathering configuration options. */
#ifndef __LANGUAGE_ASSEMBLY
    extern unsigned long ulGetRunTimeCounterValue(void);    /* Prototype of function that returns run time counter. */
    extern void vConfigureTimerForRunTimeStats(void);       /* Prototype of function that initialises the run time counter. */
#endif

#define configGENERATE_RUN_TIME_STATS                   1
#define configUSE_STATS_FORMATTING_FUNCTIONS            1
#define portCONFIGURE_TIMER_FOR_RUN_TIME_STATS()        vConfigureTimerForRunTimeStats()
#define portGET_RUN_TIME_COUNTER_VALUE()                ulGetRunTimeCounterValue()

/* Co-routine definitions. */
#define configUSE_CO_ROUTINES                           0
#define configMAX_CO_ROUTINE_PRIORITIES                 ( 2 )

/* Software timer definitions. */
#define configUSE_TIMERS                                1
#define configTIMER_TASK_PRIORITY                       ( 2 )
#define configTIMER_QUEUE_LENGTH                        5
#define configTIMER_TASK_STACK_DEPTH                    ( 120U )

/* Event group related definitions. */
#define configUSE_EVENT_GROUPS                          1

/* Set the following definitions to 1 to include the API function, or zero
to exclude the API function. */

#define INCLUDE_vTaskPrioritySet                        1
#define INCLUDE_uxTaskPriorityGet                       1
#define INCLUDE_vTaskDelete                             1
#define INCLUDE_vTaskCleanUpResources                   0
#define INCLUDE_vTaskSuspend                            1
#define INCLUDE_vTaskDelayUntil                         1
#define INCLUDE_vTaskDelay                              1
#define INCLUDE_uxTaskGetStackHighWaterMark             1
#define INCLUDE_xTaskGetSchedulerState                  1
#define INCLUDE_xTimerGetTimerTaskHandle                0
#define INCLUDE_xTaskGetIdleTaskHandle                  0
#define INCLUDE_xQueueGetMutexHolder                    1
#define INCLUDE_eTaskGetState                           1
#define INCLUDE_xEventGroupSetBitsFromISR               1
#define INCLUDE_xTimerPendFunctionCall                  0
#define INCLUDE_pcTaskGetTaskName                       1

/* The priority at which the tick interrupt runs.  This should probably be
kept at 1. */
#define configKERNEL_INTERRUPT_PRIORITY                 0x01

/* The maximum interrupt priority from which FreeRTOS.org API functions can
be called.  Only API functions that end in ...FromISR() can be used within
interrupts. */
#define configMAX_SYSCALL_INTERRUPT_PRIORITY            0x03

/* Prevent C specific syntax being included in assembly files. */
#ifndef __LANGUAGE_ASSEMBLY
    extern void vAssertCalled( const char * pcFile, unsigned long ulLine );
    #define configASSERT( x ) if( ( x ) == 0  ) vAssertCalled( __FILE__, __LINE__ )
#endif

#ifndef __LANGUAGE_ASSEMBLY

    /* The size of the global output buffer that is available for use when there
    are multiple command interpreters running at once (for example, one on a UART
    and one on TCP/IP).  This is done to prevent an output buffer being defined by
    each implementation - which would waste RAM.  In this case, there is only one
    command interpreter running, and it has its own local output buffer, so the
    global buffer is just set to be one byte long as it is not used and should not
    take up unnecessary RAM. */
    #define configCOMMAND_INT_MAX_OUTPUT_SIZE 1
    #define configHTTP_ROOT "/www"

    #define portINLINE inline

    #include <time.h>

#endif

#endif /* FREERTOS_CONFIG_H */