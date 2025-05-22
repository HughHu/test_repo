/*
 * Driver_WDT.h
 *
 *  Created on: 2025年5月19日
 *      Author: USER
 */

#ifndef CHIP_VENUSA_INCLUDE_DRIVER_WDT_H_
#define CHIP_VENUSA_INCLUDE_DRIVER_WDT_H_

#include "Driver_Common.h"
#include "chip.h"

// WDT flags
#define WDT_STATE_INITIALIZED                (1U << 0)
#define WDT_STATE_POWERED                    (1U << 1)
#define WDT_STATE_CONFIGURED                 (1U << 2)

#define WDT_MAGIC_WRITE_PROTECTION           (0x5AA5)
#define WDT_MAGIC_RESTART_VALUE              (0xCAFE)

typedef void (*HAL_WDT_SignalEvent_t)(void* workspace);

typedef enum {
    WDT_UNINITIALIZED = 0,
    WDT_INITIALIZED   = (1U << 0),
    WDT_POWERED       = (1U << 1),
    WDT_CONFIGURED    = (1u << 2),
} WDT_State_t;

typedef struct {
    WDT_State_t state;
    uint8_t busy;
    uint8_t int_stage;
    uint8_t reset_stage;
    HAL_WDT_SignalEvent_t callback;
    void* workspace;
} WDT_Info_t;

typedef struct {
    WDT_RegDef *reg;
    uint32_t irq_num;
    void (*irq_handler)(void);
    WDT_Info_t *info;
} WDT_Resources_t;

#define CHECK_RESOURCES(res)    do{ \
    if(res != &wdt0_resources){ \
        return CSK_DRIVER_ERROR_PARAMETER;  \
    }   \
} while(0)

typedef enum {
    hal_driver_wdt_clk_src_32K = 0x0,
    hal_driver_wdt_clk_src_apb = 0x1,
} HAL_DRIVER_WDT_Clk_Src_t;

typedef enum {
    hal_driver_wdt_rst_time_7 = 0x0, // clock_period * 2^7
    hal_driver_wdt_rst_time_8,
    hal_driver_wdt_rst_time_9,
    hal_driver_wdt_rst_time_10,
    hal_driver_wdt_rst_time_11,
    hal_driver_wdt_rst_time_12,
    hal_driver_wdt_rst_time_13,
    hal_driver_wdt_rst_time_14,
} HAL_DRIVER_WDT_Rst_Time_t;

typedef enum {
    hal_driver_wdt_int_time_6 = 0x0, // clock_period * 2^6
    hal_driver_wdt_int_time_8 = 0x1,
    hal_driver_wdt_int_time_10 = 0x2,
    hal_driver_wdt_int_time_11,
    hal_driver_wdt_int_time_12,
    hal_driver_wdt_int_time_13,
    hal_driver_wdt_int_time_14,
    hal_driver_wdt_int_time_15,
    hal_driver_wdt_int_time_17,
    hal_driver_wdt_int_time_19,
    hal_driver_wdt_int_time_21,
    hal_driver_wdt_int_time_23,
    hal_driver_wdt_int_time_25,
    hal_driver_wdt_int_time_27,
    hal_driver_wdt_int_time_29,
    hal_driver_wdt_int_time_31, // clock_period * 2^31
} HAL_DRIVER_WDT_Int_Time_t;

typedef struct {
    HAL_DRIVER_WDT_Clk_Src_t  clk_src;
    HAL_DRIVER_WDT_Rst_Time_t rst_time;
    HAL_DRIVER_WDT_Int_Time_t int_time;    
} HAL_DRIVER_WDT_Cfg_t;

void* WDT(void);

int32_t WDT_Initialize(void* res, HAL_WDT_SignalEvent_t callback, void* workspace);

int32_t WDT_Uninitialize(void* res);

int32_t WDT_PowerControl(void* res, CSK_POWER_STATE state);

int32_t WDT_Control(void* res, HAL_DRIVER_WDT_Cfg_t* cfg);

int32_t WDT_Enable(void* res);

int32_t WDT_Disable(void* res);

int32_t WDT_Feed(void *res);

#endif /* CHIP_VENUSA_INCLUDE_DRIVER_WDT_H_ */
