/*
 * Driver_GPT_PWM.h
 *
 *  Created on: 2025年5月12日
 *      Author: USER
 */

#ifndef CHIP_VENUSA_INCLUDE_DRIVER_GPT_PWM_H_
#define CHIP_VENUSA_INCLUDE_DRIVER_GPT_PWM_H_

#include "Driver_Common.h"
#include "Driver_GPT_Common.h"

typedef enum {
    GPT_PWM_PORT_0 = 0,
    GPT_PWM_PORT_1,
    GPT_PWM_PORT_2,
    GPT_PWM_PORT_3,
} GPT_PWM_Port_t;

typedef enum {
	GPT_PWM_PORT_0_MASK = (1 << 0),
	GPT_PWM_PORT_1_MASK = (1 << 1),
	GPT_PWM_PORT_2_MASK = (1 << 2),
	GPT_PWM_PORT_3_MASK = (1 << 3),
} GPT_PWM_PortBitMask_t;

// Init Level
typedef enum {
    GPT_PWM_INIT_LEVEL_LOW = 0,
    GPT_PWM_INIT_LEVEL_HIGH
} GPT_PWM_InitPolarity_t;

// Output Polarity
typedef enum {
    GPT_PWM_OUTPUT_ACTIVE_HIGH = 0,  // 输出高电平有效
    GPT_PWM_OUTPUT_ACTIVE_LOW        // 输出低电平有效
} GPT_PWM_OutputPolarity_t;

typedef struct {
    GPT_PWM_InitPolarity_t      init_level;        // 初始电平(未开启pwm时的状态)
    GPT_PWM_OutputPolarity_t    output_polarity;   // PWM输出极性
} GPT_PWM_Config_t;

typedef struct {
    void* workspace;
} GPT_PWM_Info_t;

int32_t HAL_GPT_DisablePWM(void *res, GPT_Channel_Num_t channel, GPT_PWM_PortBitMask_t port_mask);

int32_t HAL_GPT_EnablePWM(void *res, GPT_Channel_Num_t channel, GPT_PWM_PortBitMask_t port_mask);

int32_t HAL_GPT_SetPWMDuty(void* res, GPT_Channel_Num_t channel, GPT_PWM_Port_t port, uint16_t h_duty);

int32_t HAL_GPT_SetPWMDelayPhase(void* res, GPT_Channel_Num_t channel, GPT_PWM_Port_t port, uint16_t dly);

int32_t HAL_GPT_PWMControl(void *res, GPT_Channel_Num_t channel, GPT_PWM_Port_t port, GPT_PWM_Config_t* para);

int32_t HAL_GPT_SetPWMFrequence(void* res, GPT_Channel_Num_t channel, uint16_t freq);

int32_t HAL_GPT_RegsterPWMCallback(void* res, GPT_Channel_Num_t channel, CSK_GPT_SignalEvent_t cb_event, void *workspace);

#endif /* CHIP_VENUSA_INCLUDE_DRIVER_GPT_PWM_H_ */
