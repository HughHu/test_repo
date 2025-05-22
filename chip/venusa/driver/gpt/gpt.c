/*
 * gpt.c
 *
 *  Created on: 2025年4月24日
 *      Author: USER
 */

#include "gpt.h"
#include "log_print.h"

static void gpt0_irq_handler(void);

static GPT_Info_t gpt0_info = {
	.cb_event = {0},
	.channel_type = {0},
};

GPT_Resources_t gpt0_resources = {
    .reg = IP_GPT,
    .irq_num = IRQ_GPT_VECTOR,
    .irq_handler = gpt0_irq_handler,
    .info = &gpt0_info,
	.pwm_info = &__pwm_info,
    .timer_info = &__timer_info,
};

static void gpt_irq_handler(GPT_Resources_t* gpt){
	uint32_t isr_status = gpt->reg->REG_IMR_ISR_GPT.all & 0xffff0000;

	gpt->reg->REG_IRSR_ICR_GPT.all = isr_status;

	uint8_t channel = 0;
	uint32_t irq_flag = 0;

	// timer0 interrupt
	channel = 0;
	irq_flag = isr_status & GPT_IMR_ISR_GPT_TIMER0_INT_ISR_Msk;
	irq_flag >>= GPT_IMR_ISR_GPT_TIMER0_INT_ISR_Pos;

	while(irq_flag) {
		if(irq_flag & 0x1) {
			if (gpt->info->channel_type[channel] == HARDWARE_CHANNEL_STAT_USED_BY_TIM16) {
				// 16bits-Timer trigger complete
				if (gpt->info->cb_event[channel]) {
					gpt->info->cb_event[channel](CSK_GPT_EVENT_16BITS_TIMER_COMPLETE, gpt->timer_info->workspace[channel]);
				}
			}
			else if (gpt->info->channel_type[channel] == HARDWARE_CHANNEL_STAT_USED_BY_TIM8) {
				// 8bits-Timer trigger complete
				if (gpt->info->cb_event[channel]) {
					gpt->info->cb_event[channel](CSK_GPT_EVENT_8BITS_TIMER0_COMPLETE, gpt->timer_info->workspace[channel]);
				}
			}
			else {
				// pass
				// TODO
			}
		}
		// Channel increase
		channel++;

		irq_flag = irq_flag >> 1;
	}

	channel = 0;
	irq_flag = isr_status & GPT_IMR_ISR_GPT_TIMER1_INT_ISR_Msk;
	irq_flag >>= GPT_IMR_ISR_GPT_TIMER1_INT_ISR_Pos;

	while (irq_flag){
		if (irq_flag & 0x1){
			// Only 8bits index1 timer complete
			if (gpt->info->cb_event[channel]){
				gpt->info->cb_event[channel](CSK_GPT_EVENT_8BITS_TIMER1_COMPLETE, gpt->timer_info->workspace[channel]);
			}
		}
		// Channel increase
		channel++;

		irq_flag = irq_flag >> 1;
	}
}

void* GPT0(void) {
    return &gpt0_resources;
}

static void gpt0_irq_handler(void) {
	gpt_irq_handler(GPT0());
}

int32_t HAL_GPT_Initialize(void *res) {
    CHECK_RESOURCES(res);

    GPT_Resources_t *gpt = (GPT_Resources_t *)res;

    {
    	uint8_t i = 0;
    	for(i = 0; i < GPT_NUMBER_OF_CHANNELS; i++) {
    		gpt->info->cb_event[i] = NULL;
    		gpt->info->channel_type[i] = HARDWARE_CHANNEL_STAT_IDLE;
    	}
    }

    {
    	gpt->pwm_info->workspace = NULL;
    }

	return CSK_DRIVER_OK;
}

int32_t HAL_GPT_PowerControl(void* res, CSK_POWER_STATE state) {
    CHECK_RESOURCES(res);

    GPT_Resources_t *gpt = (GPT_Resources_t *)res;

    switch (state)
    {
    case CSK_POWER_OFF:
        disable_IRQ(gpt->irq_num);
        register_ISR(gpt->irq_num, NULL, NULL);
        break;
    case CSK_POWER_LOW:
        break;
    case CSK_POWER_FULL:
        // Nodify
        // Enable gpt clk
    	IP_SYSCTRL->REG_PERI_CLK_CFG4.bit.ENA_GPT_CLK = 0x1;

        // Reset gpt
        IP_SYSCTRL->REG_SW_RESET_CFG2.bit.GPT_RESET = 0x1;
        // clear interrupt
        gpt->reg->REG_IRSR_ICR_GPT.all |= 0xFFFF0000;
        register_ISR(gpt->irq_num,gpt->irq_handler, NULL);
        enable_IRQ(gpt->irq_num);
        break;
    default:
        break;
    }
    return CSK_DRIVER_OK;
}

int32_t HAL_GPT_Control(void* res, GPT_Channel_Num_t channel, GPT_Config_Para_t* para) {
    CHECK_RESOURCES(res);

    GPT_Resources_t* gpt = (GPT_Resources_t*)res;
    volatile uint32_t* ch_clk_ctrl = &gpt->reg->REG_CH0_CLK_CTRL.all + channel;

    uint32_t clk_ctrl = *ch_clk_ctrl;

    // Step 1: Gate the clock (write 1 close clock before changing settings)
    clk_ctrl |= GPT_CH0_CLK_CTRL_CH0_CLK_GATE_Msk;
    *ch_clk_ctrl = clk_ctrl;

    // Step 2: Set pre-divider value
    clk_ctrl &= ~GPT_CH0_CLK_CTRL_CH0_CLK_PREDIV_Msk;
    clk_ctrl |= ((para->prediv) << GPT_CH0_CLK_CTRL_CH0_CLK_PREDIV_Pos);
    *ch_clk_ctrl = clk_ctrl;
    *ch_clk_ctrl |= GPT_CH0_CLK_CTRL_CH0_CLK_PREDIV_LD_Msk;

    // Step 3: Set clock dividers
    clk_ctrl &= ~GPT_CH0_CLK_CTRL_CH0_CLK_DIV_Msk;
    clk_ctrl |= ((para->clk_div) << GPT_CH0_CLK_CTRL_CH0_CLK_DIV_Pos);
    *ch_clk_ctrl = clk_ctrl;
    *ch_clk_ctrl |= GPT_CH0_CLK_CTRL_CH0_CLK_DIV_LD_Msk;

    // Step 4: Set clock source
    clk_ctrl &= ~GPT_CH0_CLK_CTRL_CH0_CLK_SEL_Msk;
    clk_ctrl |= ((para->clk_src) << GPT_CH0_CLK_CTRL_CH0_CLK_SEL_Pos);
    *ch_clk_ctrl = clk_ctrl;
    
    // Step 5: Ungate the clock (write 0 enable clock to start working)
    clk_ctrl &= ~GPT_CH0_CLK_CTRL_CH0_CLK_GATE_Msk;
    *ch_clk_ctrl = clk_ctrl;

    return CSK_DRIVER_OK;
}

int32_t HAL_GPT_RegisterChannel(void* res, GPT_Channel_Num_t channel, Hardware_Channel_Type_t type) {
	CHECK_RESOURCES(res);

    GPT_Resources_t* gpt = (GPT_Resources_t*)res;

//    Hardware_Channel_Type_t pre_type = gpt->info->channel_type[channel];

	gpt->info->channel_type[channel] = type;

	// Get the channel control register pointer
	volatile uint32_t* ch_clk_ctrl = &gpt->reg->REG_CH0_CTRL.all + channel;
	volatile uint32_t ch_clk_ctrl_data = *ch_clk_ctrl;

	// Clear CH_MODE and OPERATION bits
	ch_clk_ctrl_data &= ~(GPT_CH0_CTRL_CH0_CH_MODE_Msk | GPT_CH0_CTRL_CH0_OPERATION_Msk);

    // Clear CLK_CNT_GATE bit to open clock gate(write 0 enable)
    // Note: this bit is active high, clearing it enables the clock
	ch_clk_ctrl_data &= ~(GPT_CH0_CTRL_CH0_CLK_CNT_GATE_Msk);

	// Set CH_MODE and OPERATION according to the usage type
	switch (type){
		case HARDWARE_CHANNEL_STAT_IDLE:
			// No mode set when idle
			break;
		case HARDWARE_CHANNEL_STAT_USED_BY_TIM8:
			// Set CH_MODE to 0x2 for 8-bit timer
			ch_clk_ctrl_data |= (0x2 << GPT_CH0_CTRL_CH0_CH_MODE_Pos);
			break;
		case HARDWARE_CHANNEL_STAT_USED_BY_TIM16:
			// Set CH_MODE to 0x1 for 16-bit timer
			ch_clk_ctrl_data |= (0x1 << GPT_CH0_CTRL_CH0_CH_MODE_Pos);
			break;
		case HARDWARE_CHANNEL_STAT_USED_BY_PWM:
			// Set OPERATION to 0x3 and CH_MODE to 0x4 for PWM
			ch_clk_ctrl_data |= (0x3 << GPT_CH0_CTRL_CH0_OPERATION_Pos) |
								(0x4 << GPT_CH0_CTRL_CH0_CH_MODE_Pos);
			break;
	}

	// Write back the updated control value
	*ch_clk_ctrl = ch_clk_ctrl_data;

	// Return previous type
	return CSK_DRIVER_OK;
}

int32_t HAL_GPT_DisableChannel(void* res, GPT_Channel_Num_t channel) {
    CHECK_RESOURCES(res);

    GPT_Resources_t* gpt = (GPT_Resources_t *)res;

    volatile uint32_t* gpt_ch_ctrl = &gpt->reg->REG_CH0_CTRL.all + channel;

    *gpt_ch_ctrl |= 0x1 << GPT_CH0_CTRL_CH0_STOP_Pos;

    return CSK_DRIVER_OK;
}

int32_t HAL_GPT_EnableChannel(void* res, GPT_Channel_Num_t channel) {
	CHECK_RESOURCES(res);

    GPT_Resources_t* gpt = (GPT_Resources_t*)res;

    volatile uint32_t* gpt_ch_ctrl = &gpt->reg->REG_CH0_CTRL.all + channel;

    *gpt_ch_ctrl |= 0x1 << GPT_CH0_CTRL_CH0_START_Pos;

	// Count start
	gpt->reg->REG_CH_CNT_CTRL.bit.CNT_START = 0x1 << channel;

	return CSK_DRIVER_OK;
}
