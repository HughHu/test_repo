/*
 * aon_timer.c
 *
 *  Created on: 2025年5月15日
 *      Author: USER
 */

#include "Driver_AON_TIMER.h"
#include "PowerManager.h"
#include "chip.h"

static void aon_timer_irq_handler(void);

static AON_TIMER_Info_t aon_timer_info = {0};

static AON_TIMER_Resources_t aon_timer_resources = {
    .reg = IP_AON_TIMER,
    .irq_num = IRQ_AON_TIMER_VECTOR,
    .irq_handler = aon_timer_irq_handler,
    .info = &aon_timer_info,
    .state = AON_TIMER_UNINITIALIZED,
};

int32_t AON_TIMER_Initialize(void* res, HAL_AON_TIMER_SignalEvent_t cb_event, void* workspace) {
    CHECK_RESOURCES(res);
    AON_TIMER_Resources_t *aon_timer = (AON_TIMER_Resources_t *)res;

    if(aon_timer->state & AON_TIMER_INITIALIZED) {
        return CSK_DRIVER_OK;
    }

    aon_timer->info->cb_event = cb_event;
    aon_timer->info->workspace = workspace;
    aon_timer->info->run_mode = HAL_AON_TIMER_MODE_Normal;
    aon_timer->info->reload_cnt = 0;

    aon_timer->state |= AON_TIMER_INITIALIZED;

    return CSK_DRIVER_OK;
}

int32_t AON_TIMER_PowerControl(void* res, CSK_POWER_STATE state) {
    CHECK_RESOURCES(res);
    AON_TIMER_Resources_t *aon_timer = (AON_TIMER_Resources_t *)res;

    switch (state)
    {
    case CSK_POWER_OFF:
        if ((aon_timer->state & AON_TIMER_INITIALIZED) == 0U) {
            return CSK_DRIVER_ERROR;
        }

        // Disable irq
        disable_IRQ(aon_timer->irq_num);
        register_ISR(aon_timer->irq_num, NULL, NULL);
        aon_timer->state &= ~AON_TIMER_POWERED;
        break;
    case CSK_POWER_LOW:
        return CSK_DRIVER_ERROR_UNSUPPORTED;
    case CSK_POWER_FULL:
        if ((aon_timer->state & AON_TIMER_INITIALIZED) == 0U) {
            return CSK_DRIVER_ERROR;
        }

        IP_AON_CTRL->REG_AON_CLK_CTRL.bit.ENA_AON_TIMER_CLK = 0x1;
        IP_AON_CTRL->REG_AON_RST_CTRL.bit.AON_TIMER_RESET = 0x1;

        // __HAL_PMU_AON_TIMER_ENABLE();
        // __HAL_PMU_AON_TIMER_RESET();

        register_ISR(aon_timer->irq_num, aon_timer->irq_handler, NULL);
        enable_IRQ(aon_timer->irq_num);
        
        aon_timer->state |= AON_TIMER_POWERED;
        break;
    default:
        return CSK_DRIVER_ERROR_UNSUPPORTED;
    }

    return CSK_DRIVER_OK;
}

int32_t AON_TIMER_Control(void* res, uint32_t control) {
    CHECK_RESOURCES(res);
    AON_TIMER_Resources_t *aon_timer = (AON_TIMER_Resources_t *)res;

    if((aon_timer->state & AON_TIMER_POWERED) == 0){
        return CSK_DRIVER_ERROR;
    }

    aon_timer->info->run_mode = control & HAL_AON_TIMER_MODE_Msk;
    switch(control & HAL_AON_TIMER_MODE_Msk) 
    {
    case HAL_AON_TIMER_MODE_Wrapping:
        aon_timer->reg->REG_OS_TIMER_CTRL.bit.WRAP_MODE = 0x1;
        aon_timer->reg->REG_OS_TIMER_CTRL.bit.REPEAT_MODE = 0x0;
        break;
    case HAL_AON_TIMER_MODE_Repeat:
        aon_timer->reg->REG_OS_TIMER_CTRL.bit.WRAP_MODE = 0x0;
        aon_timer->reg->REG_OS_TIMER_CTRL.bit.REPEAT_MODE = 0x1;
        break;
    case HAL_AON_TIMER_MODE_Normal:
        aon_timer->reg->REG_OS_TIMER_CTRL.bit.WRAP_MODE = 0x0;
        aon_timer->reg->REG_OS_TIMER_CTRL.bit.REPEAT_MODE = 0x0;
        break;
    default:
        return CSK_DRIVER_ERROR_UNSUPPORTED;
    }

    switch (control & HAL_AON_TIMER_INTERRUPT_Msk)
    {
    case HAL_AON_TIMER_INTERRUPT_Enabled:
        aon_timer->reg->REG_OS_TIMER_IRQ_MASK.all = 0x1;
        break;
    case HAL_AON_TIMER_INTERRUPT_Disabled:
        aon_timer->reg->REG_OS_TIMER_IRQ_MASK.all = 0x0;
        break;
    default:
        return CSK_DRIVER_ERROR_UNSUPPORTED;
    }

    return CSK_DRIVER_OK;
}

int32_t AON_TIMER_SetTimerPeriodByCount(void* res, uint32_t count) {
    CHECK_RESOURCES(res);
    AON_TIMER_Resources_t *aon_timer = (AON_TIMER_Resources_t *)res;

    if((aon_timer->state & AON_TIMER_POWERED) == 0){
        return CSK_DRIVER_ERROR;
    }

    aon_timer->reg->REG_OS_TIMER_CTRL.bit.LOADVAL = count & AON_TIMER_LOAD_VALUE_MASK;
    aon_timer->reg->REG_OS_TIMER_CTRL.bit.LOADER = 0x1;

    return CSK_DRIVER_OK;
}

int32_t AON_TIMER_StartTimer(void* res) {
    CHECK_RESOURCES(res);
    AON_TIMER_Resources_t *aon_timer = (AON_TIMER_Resources_t *)res;

    if((aon_timer->state & AON_TIMER_POWERED) == 0){
        return CSK_DRIVER_ERROR;
    }

    aon_timer->reg->REG_OS_TIMER_CTRL.bit.ENABLE = 0x1;
    while(!aon_timer->reg->REG_OS_TIMER_CTRL.bit.ENABLED);

    return CSK_DRIVER_OK;
}

int32_t AON_TIMER_Uninitialize(void* res) {
    CHECK_RESOURCES(res);
    AON_TIMER_Resources_t *aon_timer = (AON_TIMER_Resources_t*)res;

    aon_timer->info->cb_event = NULL;
    aon_timer->info->run_mode = HAL_AON_TIMER_MODE_Normal;
    aon_timer->info->reload_cnt = 0;

    aon_timer->state = AON_TIMER_UNINITIALIZED;

    return CSK_DRIVER_OK;
}

int32_t AON_TIMER_ReadTimerCount(void* res, uint32_t *count) {
    CHECK_RESOURCES(res);
    AON_TIMER_Resources_t *aon_timer = (AON_TIMER_Resources_t*)res;

    *count = aon_timer->reg->REG_OS_TIMER_CURVAL.all & AON_TIMER_LOAD_VALUE_MASK;

    return CSK_DRIVER_OK;
}

void *AON_TIMER() {
    return (void*)&aon_timer_resources;
}

int32_t AON_TIMER_StopTimer(void* res) {
    CHECK_RESOURCES(res);
    AON_TIMER_Resources_t *aon_timer = (AON_TIMER_Resources_t*)res;

    if((aon_timer->state & AON_TIMER_POWERED) == 0){
        return CSK_DRIVER_ERROR;
    }

    aon_timer->reg->REG_OS_TIMER_CTRL.bit.ENABLE = 0x0;
    while(aon_timer->reg->REG_OS_TIMER_CTRL.bit.ENABLED);

    return CSK_DRIVER_OK;
}

static void aon_timer_irq_handler() {
    aon_timer_resources.reg->REG_OS_TIMER_IRQ_CLR.all = 0x1;

    if (aon_timer_resources.info->cb_event){
    	aon_timer_resources.info->cb_event(HAL_AON_TIMER_EVENT_COMPLETE, aon_timer_resources.info->workspace);
    }

    while(aon_timer_resources.reg->REG_OS_TIMER_IRQ_CAUSE.bit.OSTIMER_STATUS);
}
