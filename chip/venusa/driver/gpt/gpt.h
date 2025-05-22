/*
 * gpt.h
 *
 *  Created on: 2025年4月24日
 *      Author: USER
 */

#ifndef CHIP_VENUSA_DRIVER_GPT_GPT_H_
#define CHIP_VENUSA_DRIVER_GPT_GPT_H_

#include "chip.h"
#include "Driver_GPT_Common.h"
#include "Driver_Common.h"
#include "ClockManager.h"

#include "Driver_GPT_TIMER.h"
#include "Driver_GPT_PWM.h"

#define GPT_NUMBER_OF_CHANNELS        2
#define HAL_GPT_TIMER_BITS_DIFF       0x1
#define HAL_GPT_TIMER_8BITS_MASK      0x6
#define HAL_GPT_TIMER_8BITS_POS       1

typedef struct {
    GPT_RegDef *reg;
    uint32_t irq_num;
    void (*irq_handler)(void);
    GPT_Info_t *info;
    GPT_PWM_Info_t* pwm_info;
    GPT_TIMER_Info_t* timer_info;
} GPT_Resources_t;

extern GPT_Resources_t  gpt0_resources;
extern GPT_TIMER_Info_t __timer_info;
extern GPT_PWM_Info_t   __pwm_info;

#define CHECK_RESOURCES(res) do{ \
    if (res != &gpt0_resources) { \
        return  CSK_DRIVER_ERROR_PARAMETER; \
    } \
} while(0)

#endif /* CHIP_VENUSA_DRIVER_GPT_GPT_H_ */
