/*
 * gpt_pwm.c
 *
 *  Created on: 2025年5月12日
 *      Author: USER
 */

#include "gpt_pwm.h"

GPT_PWM_Info_t __pwm_info = {
	0,
};

int32_t HAL_GPT_DisablePWM(void *res, GPT_Channel_Num_t channel, GPT_PWM_PortBitMask_t port_mask) {
    CHECK_RESOURCES(res);

    GPT_Resources_t* gpt = (GPT_Resources_t*)res;

    volatile uint32_t *gpt_ch_ctrl = (uint32_t*)&gpt->reg->REG_CH0_CTRL.all + channel;
    volatile uint32_t gpt_ch_ctrl_data = *gpt_ch_ctrl;

    *gpt_ch_ctrl = gpt_ch_ctrl_data &  ~(port_mask << GPT_CH0_CTRL_CH0_PWM_OUT_EN_Pos);

    HAL_GPT_DisableChannel(res, channel);

    return CSK_DRIVER_OK;
}

int32_t HAL_GPT_EnablePWM(void *res, GPT_Channel_Num_t channel, GPT_PWM_PortBitMask_t port_mask) {
    CHECK_RESOURCES(res);

    GPT_Resources_t* gpt = (GPT_Resources_t*)res;

    volatile uint32_t *gpt_ch_ctrl = &gpt->reg->REG_CH0_CTRL.all + channel;

    *gpt_ch_ctrl |= port_mask << GPT_CH0_CTRL_CH0_PWM_OUT_EN_Pos;

    gpt->reg->REG_IMR_ISR_GPT.bit.PWM_CYCLE_INT_MASK &= ~(1U << channel);

    HAL_GPT_EnableChannel(res, channel);

    // PWM out Start
    gpt->reg->REG_CH_CNT_CTRL.bit.TIMER1_PWM_EN |= 1U << channel;

    return CSK_DRIVER_OK;
}

int32_t HAL_GPT_SetPWMDelayPhase(void* res, GPT_Channel_Num_t channel, GPT_PWM_Port_t port, uint16_t dly) {
    CHECK_RESOURCES(res);
    GPT_Resources_t* gpt = (GPT_Resources_t*)res;

    if (channel > GPT_CHANNEL_1 || port > GPT_PWM_PORT_3) {
        return CSK_DRIVER_ERROR;
    }

    switch (channel)
    {
    case GPT_CHANNEL_0:
        switch (port)
        {
            case GPT_PWM_PORT_0:
                gpt->reg->REG_CH0_PWM_CFG3.bit.CH0_PWM0_PH_DLY = dly;
                break;
            case GPT_PWM_PORT_1:
                gpt->reg->REG_CH0_PWM_CFG3.bit.CH0_PWM1_PH_DLY = dly;
                break;
            case GPT_PWM_PORT_2:
                gpt->reg->REG_CH0_PWM_CFG4.bit.CH0_PWM2_PH_DLY = dly;
                break;
            case GPT_PWM_PORT_3:
                gpt->reg->REG_CH0_PWM_CFG4.bit.CH0_PWM3_PH_DLY = dly;
                break;
            default:
                return CSK_DRIVER_ERROR;
        }
        break;
    
    case GPT_CHANNEL_1:
        switch (port)
        {
            case GPT_PWM_PORT_0:
                gpt->reg->REG_CH1_PWM_CFG3.bit.CH1_PWM0_PH_DLY = dly;
                break;
            case GPT_PWM_PORT_1:
                gpt->reg->REG_CH1_PWM_CFG3.bit.CH1_PWM1_PH_DLY = dly;
                break;
            case GPT_PWM_PORT_2:
                gpt->reg->REG_CH1_PWM_CFG4.bit.CH1_PWM2_PH_DLY = dly;
                break;
            case GPT_PWM_PORT_3:
                gpt->reg->REG_CH1_PWM_CFG4.bit.CH1_PWM3_PH_DLY = dly;
                break;
            default:
                return CSK_DRIVER_ERROR;
        }
        break;

    default:
        return CSK_DRIVER_ERROR;
    }

    gpt->reg->REG_SHADOW_LOAD.all |= 0x1 << channel;

    return CSK_DRIVER_OK;
}

int32_t HAL_GPT_SetPWMDuty(void *res, GPT_Channel_Num_t channel, GPT_PWM_Port_t port, uint16_t h_duty) {
    CHECK_RESOURCES(res);
    GPT_Resources_t* gpt = (GPT_Resources_t*)res;

    switch (channel) {
        case GPT_CHANNEL_0:
            switch (port) {
                case GPT_PWM_PORT_0:
                    gpt->reg->REG_CH0_PWM_CFG0.bit.CH0_PWM0_DUTY_HIGH = h_duty;
                    break;
                case GPT_PWM_PORT_1:
                    gpt->reg->REG_CH0_PWM_CFG0.bit.CH0_PWM1_DUTY_HIGH = h_duty;
                    break;
                case GPT_PWM_PORT_2:
                    gpt->reg->REG_CH0_PWM_CFG1.bit.CH0_PWM2_DUTY_HIGH = h_duty;
                    break;
                case GPT_PWM_PORT_3:
                    gpt->reg->REG_CH0_PWM_CFG1.bit.CH0_PWM3_DUTY_HIGH = h_duty;
                    break;
                default:
                    return CSK_DRIVER_ERROR_PARAMETER;
            }
            break;

        case GPT_CHANNEL_1:
            switch (port) {
                case GPT_PWM_PORT_0:
                    gpt->reg->REG_CH1_PWM_CFG0.bit.CH1_PWM0_DUTY_HIGH = h_duty;
                    break;
                case GPT_PWM_PORT_1:
                    gpt->reg->REG_CH1_PWM_CFG0.bit.CH1_PWM1_DUTY_HIGH = h_duty;
                    break;
                case GPT_PWM_PORT_2:
                    gpt->reg->REG_CH1_PWM_CFG1.bit.CH1_PWM2_DUTY_HIGH = h_duty;
                    break;
                case GPT_PWM_PORT_3:
                    gpt->reg->REG_CH1_PWM_CFG1.bit.CH1_PWM3_DUTY_HIGH = h_duty;
                    break;
                default:
                    return CSK_DRIVER_ERROR_PARAMETER;
            }
            break;

        default:
            return CSK_DRIVER_ERROR_PARAMETER;
    }

    gpt->reg->REG_SHADOW_LOAD.all |= 0x1 << channel;

    return CSK_DRIVER_OK;
}


int32_t HAL_GPT_PWMControl(void *res, GPT_Channel_Num_t channel, GPT_PWM_Port_t port, GPT_PWM_Config_t* para) {
    CHECK_RESOURCES(res);

    GPT_Resources_t* gpt = (GPT_Resources_t*)res;

    // Disable channel before configuring
    HAL_GPT_DisableChannel(res, channel);

    // Access channel control register
    volatile uint32_t *gpt_pwm_ch_ctrl = &(gpt->reg->REG_CH0_CTRL.all) + channel;
    volatile uint32_t pwm_ch_ctrl = *gpt_pwm_ch_ctrl;

    // Set PWM initialize polarity
    pwm_ch_ctrl &= ~((0x1 << port) << GPT_CH0_CTRL_CH0_PWM_POLARITY_INIT_Pos);
    pwm_ch_ctrl |= ((para->init_level << port) << GPT_CH0_CTRL_CH0_PWM_POLARITY_INIT_Pos);

    // Set PWM polarity
    pwm_ch_ctrl &= ~((0x1 << port) << GPT_CH0_CTRL_CH0_PWM_POLARITY_Pos);
    pwm_ch_ctrl |= ((para->output_polarity << port) << GPT_CH0_CTRL_CH0_PWM_POLARITY_Pos);

    // Write all updates at once
    *gpt_pwm_ch_ctrl = pwm_ch_ctrl;

    // Register channel
    HAL_GPT_RegisterChannel(res, channel, HARDWARE_CHANNEL_STAT_USED_BY_PWM);

    return CSK_DRIVER_OK;
}

int32_t HAL_GPT_SetPWMFrequence(void* res, GPT_Channel_Num_t channel, uint16_t freq) {
    CHECK_RESOURCES(res);

    GPT_Resources_t* gpt = (GPT_Resources_t*)res;

    volatile uint32_t *gpt_ch_reload = &gpt->reg->REG_CH0_RELOAD.all + channel;

    *gpt_ch_reload = freq;

    // Load active value from frequnce register
    gpt->reg->REG_SHADOW_LOAD.all |= 0x1 << (channel);

    return CSK_DRIVER_OK;
}

int32_t HAL_GPT_RegsterPWMCallback(void* res, GPT_Channel_Num_t channel, CSK_GPT_SignalEvent_t cb_event, void *workspace) {
    CHECK_RESOURCES(res);

    GPT_Resources_t* gpt = (GPT_Resources_t*)res;

    gpt->info->cb_event[channel] = cb_event;
    gpt->pwm_info->workspace = workspace;

    return CSK_DRIVER_OK;
}
