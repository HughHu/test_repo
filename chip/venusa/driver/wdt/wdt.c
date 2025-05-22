/*
 * wdt.c
 *
 *  Created on: 2025年5月19日
 *      Author: USER
 */

#include "wdt.h"

static WDT_Info_t wdt0_info = { 0 };
static void WDT_IRQ_Handler(void);
static const WDT_Resources_t wdt0_resources = {
#if(BOOT_HARTID == 0)
    CORE0_WDT_BASE,
    IRQ_CORE0_WDT_VECTOR,
#else
    CORE1_WDT_BASE,
    IRQ_CORE1_WDT_VECTOR,
#endif
    WDT_IRQ_Handler,
    &wdt0_info,
};

void* WDT(void) {
    return (void*)&wdt0_resources;
}

int32_t WDT_Initialize(void* res, HAL_WDT_SignalEvent_t callback, void* workspace) {
    CHECK_RESOURCES(res);
    WDT_Resources_t *wdt = (WDT_Resources_t *)res;

    if (wdt->info->state & WDT_INITIALIZED) {
        return CSK_DRIVER_OK;
    }

    wdt->info->busy = 0x0;
    wdt->info->callback = callback;
    wdt->info->int_stage = 0x0;
    wdt->info->reset_stage = 0x0;
    wdt->info->state |=  WDT_INITIALIZED;
    wdt->info->workspace = workspace;

    return CSK_DRIVER_OK;
}

int32_t WDT_Uninitialize(void* res) {
    CHECK_RESOURCES(res);
    WDT_Resources_t *wdt = (WDT_Resources_t *)res;

    // Reset WDT status flags
    wdt->info->int_stage = 0x0;
    wdt->info->reset_stage = 0x0;
    wdt->info->state = 0U;
    wdt->info->busy = 0x0;
    wdt->info->callback = NULL;
    wdt->info->workspace = NULL;

    return CSK_DRIVER_OK;
}

int32_t WDT_PowerControl(void* res, CSK_POWER_STATE state) {
    CHECK_RESOURCES(res);
    WDT_Resources_t *wdt = (WDT_Resources_t *)res;

    switch (state)
    {
    case CSK_POWER_OFF:
        if ((wdt->info->state & WDT_INITIALIZED) == 0U) {
            return CSK_DRIVER_ERROR;
        }

        register_ISR(wdt->irq_num, NULL, NULL);
        disable_IRQ(wdt->irq_num);

        wdt->reg->REG_CTRL.bit.EN = 0x0;

        wdt->info->state &= ~WDT_POWERED;
        break;
    case CSK_POWER_LOW:
        return CSK_DRIVER_ERROR_UNSUPPORTED;
    case CSK_POWER_FULL:
        if ((wdt->info->state & WDT_INITIALIZED) == 0U) {
            return CSK_DRIVER_ERROR;
        }
        register_ISR(wdt->irq_num, wdt->irq_handler, NULL);
        enable_IRQ(wdt->irq_num);

        wdt->reg->REG_CTRL.bit.EN = 0x1;
        
        wdt->info->state |= WDT_POWERED; 
        break;
    default:
        break;
    }

    return CSK_DRIVER_OK;
}

int32_t WDT_Control(void* res, HAL_DRIVER_WDT_Cfg_t* cfg) {
    CHECK_RESOURCES(res);
    WDT_Resources_t *wdt = (WDT_Resources_t *)res;

    if ((wdt->info->state & WDT_STATE_POWERED) == 0U) {
        return CSK_DRIVER_ERROR;
    }

    if (wdt->info->busy) {
        return CSK_DRIVER_ERROR_BUSY;
    }

    uint32_t cfg_value = 0;

    // Clock source
    cfg_value = __RV_INSERT_FIELD(cfg_value, WDT_CTRL_CLKSEL_Msk, cfg->clk_src);
    // interrupt time
    cfg_value = __RV_INSERT_FIELD(cfg_value, WDT_CTRL_INTTIME_Msk, cfg->int_time);
    // reset timer
    cfg_value = __RV_INSERT_FIELD(cfg_value, WDT_CTRL_RSTTIME_Msk, cfg->rst_time);

    wdt->info->int_stage   = cfg->int_time;
    wdt->info->reset_stage = cfg->rst_time;

    wdt->reg->REG_WREN.all = WDT_MAGIC_WRITE_PROTECTION;
    wdt->reg->REG_CTRL.all = cfg_value;

    // Set configured flag
    wdt->info->state |= WDT_STATE_CONFIGURED;
    
    return CSK_DRIVER_OK;
}

int32_t WDT_Enable(void *res) {
    CHECK_RESOURCES(res);
    WDT_Resources_t *wdt = (WDT_Resources_t *)res;

    uint32_t cfg_value = 0U;

    if ((wdt->info->state & WDT_STATE_POWERED) == 0U) {
        return CSK_DRIVER_ERROR;
    }

    if (wdt->info->busy == 1) {
        return CSK_DRIVER_ERROR_BUSY;
    }

    // rst enable
    cfg_value = __RV_INSERT_FIELD(cfg_value, WDT_CTRL_RSTEN_Msk, 0x1);
    // int enable
    cfg_value = __RV_INSERT_FIELD(cfg_value, WDT_CTRL_INTEN_Msk, 0x1);
    // enable
    cfg_value = __RV_INSERT_FIELD(cfg_value, WDT_CTRL_EN_Msk, 0x1);
    
    wdt->info->busy = 0x1;

    // enable write and write control register
    wdt->reg->REG_WREN.all = WDT_MAGIC_WRITE_PROTECTION;
    wdt->reg->REG_CTRL.all = (wdt->reg->REG_CTRL.all & (~(WDT_CTRL_RSTEN_Msk | WDT_CTRL_INTEN_Msk | WDT_CTRL_EN_Msk))) | cfg_value;

    return CSK_DRIVER_OK;
}

int32_t WDT_Disable(void *res) {
    CHECK_RESOURCES(res);
    WDT_Resources_t *wdt = (WDT_Resources_t *)res;

    if((wdt->info->state & WDT_STATE_POWERED) == 0U) {
    	return CSK_DRIVER_ERROR;
    }

    wdt->reg->REG_WREN.all = WDT_MAGIC_WRITE_PROTECTION;
    wdt->reg->REG_CTRL.bit.EN = 0x0;

    wdt->info->busy = 0x0;

    return CSK_DRIVER_OK;
}


int32_t WDT_Feed(void *res) {
    CHECK_RESOURCES(res);
    WDT_Resources_t *wdt = (WDT_Resources_t *)res;

    wdt->reg->REG_WREN.all = WDT_MAGIC_WRITE_PROTECTION;  // 0x5AA5
    wdt->reg->REG_RESTART.all = WDT_MAGIC_RESTART_VALUE;  // 0xCAFE

    return CSK_DRIVER_OK;
}


static void
WDT_IRQ_Handler(void) {
    if (wdt0_resources.info->callback != NULL){
        wdt0_resources.info->callback(wdt0_resources.info->workspace);
    }
}
