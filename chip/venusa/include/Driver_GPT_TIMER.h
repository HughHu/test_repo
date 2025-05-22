/*
 * Driver_GPT_TIMER.h
 *
 *  Created on: 2025年4月24日
 *      Author: USER
 */

#ifndef CHIP_VENUSA_INCLUDE_DRIVER_GPT_TIMER_H_
#define CHIP_VENUSA_INCLUDE_DRIVER_GPT_TIMER_H_

#include "Driver_Common.h"
#include "Driver_GPT_Common.h"

#define HAL_GPT_TIMER_16BITS_INDEX_0				0
#define HAL_GPT_TIMER_8BITS_INDEX_0					3
#define HAL_GPT_TIMER_8BITS_INDEX_1					5

typedef enum {
	GPT_TIMER_COUNTMODE_UP = 0,
	GPT_TIMER_COUNTMODE_DOWN,
	GPT_TIMER_COUNTMODE_UPDOWM
} GPT_TIMER_CountMode_t;

typedef enum {
	GPT_TIMER_RUNMODE_SINGLE  = 0,
	GPT_TIMER_RUNMODE_REPEAT,
	GPT_TIMER_RUNMODE_FREE_RUN, //Count up (down) continuously, without reload, without stopping
	GPT_TIMER_RUNMODE_KPPEGO
} GPT_TIMER_RunMode_t;

typedef struct {
	uint8_t index;
	GPT_TIMER_RunMode_t run_mode;
	GPT_TIMER_CountMode_t cnt_mode;
} GPT_TIMER_Config_Para_t;

typedef struct {
    void* workspace[GPT_NUMBER_OF_CHANNELS];
}GPT_TIMER_Info_t;

int32_t HAL_GPT_TimerControl(void* res, GPT_Channel_Num_t channel, GPT_TIMER_Config_Para_t* para);

int32_t HAL_GPT_ReadTimerCount(void *res, GPT_Channel_Num_t channel, uint8_t ch_mode, uint16_t *count);

int32_t HAL_GPT_RegisterTimerCallback(void* res, GPT_Channel_Num_t channel, CSK_GPT_SignalEvent_t cb_event, void* workspace);

int32_t HAL_GPT_SetTimerPeriodByCount(void* res, GPT_Channel_Num_t channel, uint8_t ch_mode, uint16_t count);

int32_t HAL_GPT_StartTimer(void* res, GPT_Channel_Num_t channel, uint8_t ch_mode);

#endif /* CHIP_VENUSA_INCLUDE_DRIVER_GPT_TIMER_H_ */
