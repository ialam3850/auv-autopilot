#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

/* FreeRTOS kernel configuration for the STM32F446RE autopilot.
 * See https://www.freertos.org/a00110.html for every option. */

#include <stdint.h>
extern uint32_t SystemCoreClock;

#define configUSE_PREEMPTION                    1
#define configUSE_PORT_OPTIMISED_TASK_SELECTION 1
#define configUSE_TICKLESS_IDLE                 0
#define configCPU_CLOCK_HZ                      (SystemCoreClock)
#define configTICK_RATE_HZ                      ((TickType_t)1000)
#define configMAX_PRIORITIES                    6
#define configMINIMAL_STACK_SIZE                ((uint16_t)128)
#define configMAX_TASK_NAME_LEN                 12
#define configUSE_16_BIT_TICKS                  0
#define configIDLE_SHOULD_YIELD                 1
#define configUSE_TASK_NOTIFICATIONS            1
#define configUSE_MUTEXES                       1
#define configQUEUE_REGISTRY_SIZE               0
#define configUSE_TIME_SLICING                  1

/* Memory: heap_4 in .bss; tasks and queues are created once at boot. */
#define configSUPPORT_DYNAMIC_ALLOCATION        1
#define configSUPPORT_STATIC_ALLOCATION         0
#define configTOTAL_HEAP_SIZE                   ((size_t)(24 * 1024))

/* Hooks and diagnostics. */
#define configUSE_IDLE_HOOK                     0
#define configUSE_TICK_HOOK                     0
#define configCHECK_FOR_STACK_OVERFLOW          2
#define configUSE_MALLOC_FAILED_HOOK            1
#define configUSE_TRACE_FACILITY                0
#define configGENERATE_RUN_TIME_STATS           0

#define configUSE_TIMERS                        0
#define configUSE_CO_ROUTINES                   0

/* Cortex-M4 interrupt priorities. The STM32F4 implements 4 priority bits.
 * Any ISR that calls a ...FromISR() API must have a numerically HIGHER (i.e.
 * logically lower) priority than configMAX_SYSCALL_INTERRUPT_PRIORITY. The
 * UART and DMA interrupts in this project use priority 6. */
#define configPRIO_BITS                                  4
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY          15
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY     5
#define configKERNEL_INTERRUPT_PRIORITY      (configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))
#define configMAX_SYSCALL_INTERRUPT_PRIORITY (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))

#define configASSERT(x)          \
  if ((x) == 0) {                \
    taskDISABLE_INTERRUPTS();    \
    for (;;) {                   \
    }                            \
  }

#define INCLUDE_vTaskDelay                1
#define INCLUDE_vTaskDelayUntil           1
#define INCLUDE_xTaskGetTickCount         1
#define INCLUDE_uxTaskGetStackHighWaterMark 1

/* Map the kernel's handlers onto the CMSIS vector names used by the startup file. */
#define vPortSVCHandler     SVC_Handler
#define xPortPendSVHandler  PendSV_Handler
#define xPortSysTickHandler SysTick_Handler

#endif /* FREERTOS_CONFIG_H */
