#if 0
#ifndef __DRIVER_I8080_INTERNAL_H
#define __DRIVER_I8080_INTERNAL_H

#include "chip.h"
#include "ClockManager.h"
#include "log_print.h"
#include "Driver_I8080.h"


#define I8080_CLK_OUT_SEL_100MHz  100000000
#define I8080_CLK_OUT_SEL_24MHz   24000000
#define I8080_CLK_OUT_DIV_MAX     15
#define I8080_CLK_OUT_DIV_MIN     1
#define I8080_CLK_OUT_HZ_MAX      (I8080_CLK_OUT_SEL_100MHz / I8080_CLK_OUT_DIV_MIN)
#define I8080_CLK_OUT_HZ_MIN      (I8080_CLK_OUT_SEL_24MHz / I8080_CLK_OUT_DIV_MAX)

/**
  * @brief  I8080 handle Structure definition
  */
typedef struct __I8080_DEV
{
    I8080_OUT_RegDef            *Instance;           /*!< I8080 Register base address  */

    I8080_InitTypeDef           Init;                /*!< I8080 parameters             */

    CSK_I8080_SignalEvent_t     cb_event;            /*!< I8080 callback               */

    volatile I8080_emState      State;               /*!< I8080 state                  */

    volatile uint32_t           ErrorCode;           /*!< I8080 Error code             */
}I8080_DEV;


#endif /* __DRIVER_I8080_INTERNAL_H */

#endif
