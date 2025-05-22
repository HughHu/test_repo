/*
 * Driver_GPT_COMMON.h
 *
 *  Created on: 2025年4月24日
 *      Author: USER
 */

#ifndef CHIP_VENUSA_INCLUDE_DRIVER_GPT_COMMON_H_
#define CHIP_VENUSA_INCLUDE_DRIVER_GPT_COMMON_H_

#include "Driver_Common.h"
#include "Driver_GPT_Common.h"

#define GPT_NUMBER_OF_CHANNELS          2

#define CSK_GPT_EVENT_PWM_CYCLE_DONE                   (1UL << 0)
#define CSK_GPT_EVENT_LEDC_COMPLETE                    (1UL << 1)
#define CSK_GPT_EVENT_LEDC_TX_FIFO_UNDERFLOW           (1UL << 2)
#define CSK_GPT_EVENT_BREATH_PORT0                     (1UL << 3)
#define CSK_GPT_EVENT_BREATH_PORT1                     (1UL << 4)
#define CSK_GPT_EVENT_BREATH_PORT2                     (1UL << 5)
#define CSK_GPT_EVENT_16BITS_TIMER_COMPLETE            (1UL << 6)
#define CSK_GPT_EVENT_8BITS_TIMER0_COMPLETE            (1UL << 7)
#define CSK_GPT_EVENT_8BITS_TIMER1_COMPLETE            (1UL << 8)


typedef enum {
    GPT_CHANNEL_0 = 0,
    GPT_CHANNEL_1,
} GPT_Channel_Num_t;

typedef enum {
    GPT_CLK_SRC_T0 = 0,
    GPT_CLK_SRC_EXCLK,
    GPT_CLK_SRC_PCLK,
} GPT_Clk_Src_t;

typedef enum {
    GPT_CLK_DIV_1 = 0,
    GPT_CLK_DIV_2,
    GPT_CLK_DIV_4,
    GPT_CLK_DIV_8,
    GPT_CLK_DIV_16,
    GPT_CLK_DIV_32,
    GPT_CLK_DIV_64,
    GPT_CLK_DIV_128,
} GPT_Clk_Div_t;

typedef struct 
{
    GPT_Clk_Src_t clk_src;
    uint16_t prediv;
    GPT_Clk_Div_t clk_div;
} GPT_Config_Para_t;

typedef enum {
    HARDWARE_CHANNEL_STAT_IDLE = 0,
    HARDWARE_CHANNEL_STAT_USED_BY_TIM16,
    HARDWARE_CHANNEL_STAT_USED_BY_TIM8,
    HARDWARE_CHANNEL_STAT_USED_BY_PWM = 4,
} Hardware_Channel_Type_t;

typedef void
(*CSK_GPT_SignalEvent_t)(uint32_t event, void *param);

typedef struct {
    volatile Hardware_Channel_Type_t channel_type[GPT_NUMBER_OF_CHANNELS];
	CSK_GPT_SignalEvent_t cb_event[GPT_NUMBER_OF_CHANNELS];
} GPT_Info_t;

int32_t HAL_GPT_Initialize(void* res);

int32_t HAL_GPT_Uninitialize(void* res);

int32_t HAL_GPT_PowerControl(void* res, CSK_POWER_STATE state);

int32_t HAL_GPT_Control(void* res, GPT_Channel_Num_t channel, GPT_Config_Para_t* para);

int32_t HAL_GPT_DisableChannel(void* res, GPT_Channel_Num_t channel);

int32_t HAL_GPT_RegisterChannel(void* res, GPT_Channel_Num_t channel, Hardware_Channel_Type_t type);

int32_t HAL_GPT_EnableChannel(void* res, GPT_Channel_Num_t channel);

void* GPT0(void);

#endif /* CHIP_VENUSA_INCLUDE_DRIVER_GPT_COMMON_H_ */


