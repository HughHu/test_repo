/*
 * Driver_AON_TIMER.h
 *
 *  Created on: 2025年5月15日
 *      Author: USER
 */

#ifndef CHIP_VENUSA_INCLUDE_DRIVER_AON_TIMER_H_
#define CHIP_VENUSA_INCLUDE_DRIVER_AON_TIMER_H_

#include "Driver_Common.h"
#include "chip.h"

#define HAL_AON_TIMER_MODE_Pos                  (0)
#define HAL_AON_TIMER_MODE_Msk                  (3UL << HAL_AON_TIMER_MODE_Pos)
#define HAL_AON_TIMER_MODE_Wrapping             (0UL << HAL_AON_TIMER_MODE_Pos)
#define HAL_AON_TIMER_MODE_Repeat               (1UL << HAL_AON_TIMER_MODE_Pos)
#define HAL_AON_TIMER_MODE_Normal               (2UL << HAL_AON_TIMER_MODE_Pos)

#define HAL_AON_TIMER_INTERRUPT_Pos             (2)
#define HAL_AON_TIMER_INTERRUPT_Msk             (1UL << HAL_AON_TIMER_INTERRUPT_Pos)
#define HAL_AON_TIMER_INTERRUPT_Enabled         (0UL << HAL_AON_TIMER_INTERRUPT_Pos)
#define HAL_AON_TIMER_INTERRUPT_Disabled        (1UL << HAL_AON_TIMER_INTERRUPT_Pos)

#define AON_TIMER_LOAD_VALUE_MASK               (0xFFFFFF)
#define HAL_AON_TIMER_EVENT_COMPLETE            (1UL << 0)

typedef void (*HAL_AON_TIMER_SignalEvent_t) (uint32_t event, void* workspace);

typedef struct {
    HAL_AON_TIMER_SignalEvent_t cb_event;
    void *workspace;
    uint32_t reload_cnt;
    uint32_t run_mode;
} AON_TIMER_Info_t;

typedef enum {
    AON_TIMER_UNINITIALIZED = 0,
    AON_TIMER_INITIALIZED   = (1U << 0),
    AON_TIMER_POWERED       = (1U << 1),
} AON_TIMER_State_t;

typedef struct {
    AON_TIMER_RegDef* reg;
    uint32_t irq_num;
    void (*irq_handler)(void);
    AON_TIMER_Info_t* info;
    AON_TIMER_State_t state;
} AON_TIMER_Resources_t;

#define CHECK_RESOURCES(res)    do{ \
    if(res != &aon_timer_resources){ \
        return CSK_DRIVER_ERROR_PARAMETER; \
    } \
} while(0)

int32_t AON_TIMER_Initialize(void* res, HAL_AON_TIMER_SignalEvent_t cb_event, void* workspace);

int32_t AON_TIMER_Uninitialize(void* res);

int32_t AON_TIMER_PowerControl(void* res, CSK_POWER_STATE state);

int32_t AON_TIMER_Control(void* res, uint32_t control);

int32_t AON_TIMER_SetTimerPeriodByCount(void* res, uint32_t count);

int32_t AON_TIMER_StartTimer(void* res);

int32_t AON_TIMER_StopTimer(void* res);

int32_t AON_TIMER_ReadTimerCount(void* res, uint32_t *count);

void* AON_TIMER(void);

#endif /* CHIP_VENUSA_INCLUDE_DRIVER_AON_TIMER_H_ */
