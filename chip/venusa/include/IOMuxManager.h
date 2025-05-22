#ifndef __CSK_IOMUX_MANAGER_H
#define __CSK_IOMUX_MANAGER_H

#include <stdint.h>

#include "Driver_Common.h"


#define CSK_IOMUX_PAD_A                 0
// PIN NUM: 0-31
#define CSK_IOMUX_PAD_B                 1
// PIN NUM: 0-5
#define CSK_IOMUX_PAD_C                 2
// PIN NUM: 0-3

#define CSK_IOMUX_PAD_A_MAX_PIN			31
#define CSK_IOMUX_PAD_B_MAX_PIN			5
#define CSK_IOMUX_PAD_C_MAX_PIN			3

/************** NORMAL IOMUX********************/
#define CSK_IOMUX_FUNC_DEFAULT                 (0U)
#define CSK_IOMUX_FUNC_ALTER1                  (1U)
#define CSK_IOMUX_FUNC_ALTER2                  (2U)
#define CSK_IOMUX_FUNC_ALTER3                  (3U)
#define CSK_IOMUX_FUNC_ALTER4                  (4U)
#define CSK_IOMUX_FUNC_ALTER5                  (5U)
#define CSK_IOMUX_FUNC_ALTER6                  (6U)
#define CSK_IOMUX_FUNC_ALTER7                  (7U)
#define CSK_IOMUX_FUNC_ALTER8                  (8U)
#define CSK_IOMUX_FUNC_ALTER9                  (9U)
#define CSK_IOMUX_FUNC_ALTER10                 (10U)
#define CSK_IOMUX_FUNC_ALTER11                 (11U)
#define CSK_IOMUX_FUNC_ALTER12                 (12U)
#define CSK_IOMUX_FUNC_ALTER13                 (13U)
#define CSK_IOMUX_FUNC_ALTER14                 (14U)
#define CSK_IOMUX_FUNC_ALTER15                 (15U)
#define CSK_IOMUX_FUNC_ALTER16                 (16U)
#define CSK_IOMUX_FUNC_ALTER17                 (17U)
#define CSK_IOMUX_FUNC_ALTER18                 (18U)
#define CSK_IOMUX_FUNC_ALTER19                 (19U)
#define CSK_IOMUX_FUNC_ALTER20                 (20U)
#define CSK_IOMUX_FUNC_ALTER21                 (21U)
#define CSK_IOMUX_FUNC_ALTER22                 (22U)
#define CSK_IOMUX_FUNC_ALTER23                 (23U)
#define CSK_IOMUX_FUNC_ALTER24                 (24U)
#define CSK_IOMUX_FUNC_ALTER25                 (25U)
#define CSK_IOMUX_FUNC_ALTER26                 (26U)
#define CSK_IOMUX_FUNC_ALTER27                 (27U)
#define CSK_IOMUX_FUNC_ALTER28                 (28U)
#define CSK_IOMUX_FUNC_ALTER29                 (29U)
#define CSK_IOMUX_FUNC_ALTER30                 (30U)
#define CSK_IOMUX_FUNC_ALTER31                 (31U)

/****************** AON IOMUX********************/
#define CSK_AON_IOMUX_FUNC_DEFAULT             (0U)
#define CSK_AON_IOMUX_FUNC_ALTER1              (1U)
#define CSK_AON_IOMUX_FUNC_ALTER2              (2U)
#define CSK_AON_IOMUX_FUNC_ALTER3              (3U)
#define CSK_AON_IOMUX_FUNC_ALTER4              (4U)
#define CSK_AON_IOMUX_FUNC_ALTER5              (5U)
#define CSK_AON_IOMUX_FUNC_ALTER6              (6U)
#define CSK_AON_IOMUX_FUNC_ALTER7              (7U)
#define CSK_AON_IOMUX_FUNC_ALTER8              (8U)

/***************** ANA IOMUX**********************/
#define CSK_ANA_IOMUX_FUNC_DEFAULT             	(0U)
#define CSK_ANA_IOMUX_FUNC_ALTER1              	(1U)
#define CSK_ANA_IOMUX_FUNC_ALTER2              	(2U)
#define CSK_ANA_IOMUX_FUNC_ALTER3              	(3U)
#define CSK_ANA_IOMUX_FUNC_ALTER4              	(4U)
#define CSK_ANA_IOMUX_FUNC_ALTER5              	(5U)
#define CSK_ANA_IOMUX_FUNC_ALTER6              	(6U)
#define CSK_ANA_IOMUX_FUNC_ALTER7              	(7U)
#define CSK_ANA_IOMUX_FUNC_ALTER8             	(8U)
#define CSK_ANA_IOMUX_FUNC_ALTER9              	(9U)
#define CSK_ANA_IOMUX_FUNC_ALTER10              (10U)
#define CSK_ANA_IOMUX_FUNC_ALTER11              (11U)


/**************** MODE CONFIGURE********************/
#define HAL_IOMUX_NONE_MODE                    (0U)
#define HAL_IOMUX_PULLUP_MODE                  (1U)
#define HAL_IOMUX_PULLDOWN_MODE                (2U)

/************** AON TO NORMAL********************/
#define CSK_AON_IOMUX_FUNC_NORMAL              (6U)

/************** AON TO GPIO_OUT******************/
#define CSK_AON_GPIO_OUT_FUNC				   (2U)

/************** AON TO ANA********************/
#define CSK_AON_IOMUX_FUNC_ANA                 (3U)

/************** AON IOMUX FUNC********************/
#define CSK_AON_IOMUX_FUNC_DEFAULT             (0U)
#define CSK_AON_IOMUX_FUNC_ALTER1              (1U)
#define CSK_AON_IOMUX_FUNC_ALTER2              (2U)
#define CSK_AON_IOMUX_FUNC_ALTER3              (3U)
#define CSK_AON_IOMUX_FUNC_ALTER4              (4U)
#define CSK_AON_IOMUX_FUNC_ALTER5              (5U)
#define CSK_AON_IOMUX_FUNC_ALTER6              (6U)

//************ IOMUX FORCE DATA *****************/
#define HAL_IOMUX_FORCE_OUT_LOW                (0U)
#define HAL_IOMUX_FORCE_OUT_HIGH               (1U)
#define HAL_IOMUX_FORCE_OFF                    (2U)

int32_t IOMuxManager_PinConfigure(uint8_t pad, uint8_t pin_num, uint32_t pin_cfg);

int32_t IOMuxManager_ModeConfigure(uint8_t pad, uint8_t pin_num, uint8_t pin_mode);

int32_t IOMuxManager_PinForce(uint8_t pad, uint8_t pin_num, uint8_t data);

int32_t AON_IOMuxManager_PinConfigure(uint8_t pad, uint8_t pin_num, uint32_t pin_cfg);

int32_t AON_IOMuxManager_ModeConfigure(uint8_t pad, uint8_t pin_num, uint8_t pin_mode);

int32_t AON_IOMuxManager_PinForce(uint8_t pad, uint8_t pin_num, uint8_t data);

int32_t ANA_IOMuxManager_PinConfigure(uint8_t pad, uint8_t pin_num, uint32_t pin_cfg);

#endif /* __CSK_IOMUX_MANAGER_H */
