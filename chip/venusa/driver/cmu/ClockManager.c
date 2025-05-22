#ifndef INCLUDE_CLOCKMANAGER_C_
#define INCLUDE_CLOCKMANAGER_C_

#include "ClockManager.h"

// Retrieves the frequency of the (fixed).
static uint32_t CRM_GetRC032KFreq(void){
    return BOARD_BOOTCLOCKRUN_RC032K_CLK; // Returns predefined (fixed) clock frequency.
}

// Retrieves the frequency of the (fixed).
static uint32_t CRM_GetXTAL32KFreq(void){
    return BOARD_BOOTCLOCKRUN_XTAL32K_CLK; // Returns predefined (fixed) clock frequency.
}

// Retrieves the frequency of the (fixed).
static uint32_t CRM_GetRC024MFreq(void){
    return BOARD_BOOTCLOCKRUN_RC024M_CLK; // Returns predefined (fixed) clock frequency.
}

// Retrieves the frequency of the (fixed).
static uint32_t CRM_GetXTAL24MFreq(void){
    return BOARD_BOOTCLOCKRUN_XTAL24M_CLK; // Returns predefined (fixed) clock frequency.
}

// Determines the CORE24M core clock frequency based on the selected source.
static uint32_t CRM_GetCORE24MFreq(void){
    uint32_t src = IP_CMN_SYSCFG->REG_PERI_CLK_CFG7.bit.SEL_24M_SRC;
    switch (src){
    case CRM_IpSrcRC024M:
        return CRM_GetRC024MFreq();
    case CRM_IpSrcXTAL24M:
        return CRM_GetXTAL24MFreq();
    default:
        return 0;
    }
}

// Determines the CORE32K core clock frequency based on the selected source.
static uint32_t CRM_GetCORE32KFreq(void){
    uint32_t src = IP_AON_CTRL->REG_AON_CLK_CTRL.bit.SEL_32KDIV_SRC;
    switch (src){
    case CRM_IpSrcRC032K:
        return CRM_GetRC032KFreq();
    case CRM_IpSrcXTAL32K:
        return CRM_GetXTAL32KFreq();
    default:
        return 0;
    }
}

// Initializes the system SYSPLL CORE clock with a specified divider.
// Returns CSK_DRIVER_OK on success, CSK_DRIVER_ERROR_PARAMETER if divider is invalid.
int32_t CRM_InitSyspllCore(clock_src_syspllcore_div div){
    if (div > 14){
        return CSK_DRIVER_ERROR_PARAMETER;
    }
    IP_CMN_BUSCFG->REG_SYSPLL_CFG1.bit.SYSPLL_POSTDIV_SYSTEM_DIV_SEL = div;
    IP_CMN_BUSCFG->REG_SYSPLL_CFG0.bit.SYSPLL_POSTDIV_SYSTEM_EN = 0x1;
    return CSK_DRIVER_OK;
}

// Retrieves the system SYSPLL CORE clock frequency.
// Returns 0 if SYSPLL CORE is disabled, otherwise calculates frequency based on divider.
static uint32_t CRM_GetSyspllCoreFreq(void){
    uint32_t freq = 0;
    uint32_t div = 0;
    if (!IP_CMN_BUSCFG->REG_SYSPLL_CFG0.bit.SYSPLL_POSTDIV_SYSTEM_EN){
        return 0;
    }
    div = IP_CMN_BUSCFG->REG_SYSPLL_CFG1.bit.SYSPLL_POSTDIV_SYSTEM_DIV_SEL + 3;
    freq = BOARD_BOOTCLOCKRUN_SYSPLL_CLK / div;
    return freq;
}

// Initializes the system SYSPLL PERI clock with a specified divider.
// Returns CSK_DRIVER_OK on success, CSK_DRIVER_ERROR_PARAMETER if divider is invalid.
int32_t CRM_InitSyspllPeri(clock_src_syspllperi_div div){
    if (div > 10){
        return CSK_DRIVER_ERROR_PARAMETER;
    }
    IP_CMN_BUSCFG->REG_SYSPLL_CFG1.bit.SYSPLL_POSTDIV_PERI_DIV_SEL = div;
    IP_CMN_BUSCFG->REG_SYSPLL_CFG0.bit.SYSPLL_POSTDIV_PERI_EN = 0x1;
    return CSK_DRIVER_OK;
}

// Retrieves the system SYSPLL PERI clock frequency.
// Returns 0 if SYSPLL PERI is disabled, otherwise calculates frequency based on divider.
static uint32_t CRM_GetSyspllPeriFreq(void){
    uint32_t freq = 0;
    uint32_t div = 0;
    if (!IP_CMN_BUSCFG->REG_SYSPLL_CFG0.bit.SYSPLL_POSTDIV_PERI_EN){
        return 0;
    }
    div = IP_CMN_BUSCFG->REG_SYSPLL_CFG1.bit.SYSPLL_POSTDIV_PERI_DIV_SEL + 6;
    freq = BOARD_BOOTCLOCKRUN_SYSPLL_CLK / div;
    return freq;
}

// Initializes the system SYSPLL FLASH clock with a specified divider.
// Returns CSK_DRIVER_OK on success, CSK_DRIVER_ERROR_PARAMETER if divider is invalid.
int32_t CRM_InitSyspllFlash(clock_src_syspllflash_div div){
    if (div > 7){
        return CSK_DRIVER_ERROR_PARAMETER;
    }
    IP_CMN_BUSCFG->REG_SYSPLL_CFG1.bit.SYSPLL_POSTDIV_FLASH_DIV_SEL = div;
    IP_CMN_BUSCFG->REG_SYSPLL_CFG0.bit.SYSPLL_POSTDIV_FLASH_EN = 0x1;
    return CSK_DRIVER_OK;
}

// Retrieves the system SYSPLL FLASH clock frequency.
// Returns 0 if SYSPLL FLASH is disabled, otherwise calculates frequency based on divider.
static uint32_t CRM_GetSyspllFlashFreq(void){
    uint32_t freq = 0;
    uint32_t div = 0;
    if (!IP_CMN_BUSCFG->REG_SYSPLL_CFG0.bit.SYSPLL_POSTDIV_FLASH_EN){
        return 0;
    }
    div = IP_CMN_BUSCFG->REG_SYSPLL_CFG1.bit.SYSPLL_POSTDIV_FLASH_DIV_SEL + 8;
    freq = BOARD_BOOTCLOCKRUN_SYSPLL_CLK / div;
    return freq;
}

// Initializes the system SYSPLL PSRAM clock with a specified divider.
// Returns CSK_DRIVER_OK on success, CSK_DRIVER_ERROR_PARAMETER if divider is invalid.
int32_t CRM_InitSyspllPsram(clock_src_syspllpsram_div div){
    if (div > 7){
        return CSK_DRIVER_ERROR_PARAMETER;
    }
    IP_CMN_BUSCFG->REG_SYSPLL_CFG2.bit.SYSPLL_POSTDIV_PSRAM_DIV_SEL = div;
    IP_CMN_BUSCFG->REG_SYSPLL_CFG0.bit.SYSPLL_POSTDIV_PSRAM_EN = 0x1;
    return CSK_DRIVER_OK;
}

// Retrieves the system SYSPLL PSRAM clock frequency.
// Returns 0 if SYSPLL PSRAM is disabled, otherwise calculates frequency based on divider.
static uint32_t CRM_GetSyspllPsramFreq(void){
    uint32_t freq = 0;
    uint32_t div = 0;
    if (!IP_CMN_BUSCFG->REG_SYSPLL_CFG0.bit.SYSPLL_POSTDIV_PSRAM_EN){
        return 0;
    }
    div = IP_CMN_BUSCFG->REG_SYSPLL_CFG2.bit.SYSPLL_POSTDIV_PSRAM_DIV_SEL + 3;
    freq = BOARD_BOOTCLOCKRUN_SYSPLL_CLK / div;
    return freq;
}

// Initializes the system SYSPLL SDIO clock with a specified divider.
// Returns CSK_DRIVER_OK on success, CSK_DRIVER_ERROR_PARAMETER if divider is invalid.
int32_t CRM_InitSyspllSdio(clock_src_syspllsdio_div div){
    if (div > 5){
        return CSK_DRIVER_ERROR_PARAMETER;
    }
    IP_CMN_BUSCFG->REG_SYSPLL_CFG1.bit.SYSPLL_POSTDIV_SDIO_DIV_SEL = div;
    IP_CMN_BUSCFG->REG_SYSPLL_CFG0.bit.SYSPLL_POSTDIV_SDIO_EN = 0x1;
    return CSK_DRIVER_OK;
}

// Retrieves the system SYSPLL SDIO clock frequency.
// Returns 0 if SYSPLL SDIO is disabled, otherwise calculates frequency based on divider.
static uint32_t CRM_GetSyspllSdioFreq(void){
    uint32_t freq = 0;
    uint32_t div = 0;
    if (!IP_CMN_BUSCFG->REG_SYSPLL_CFG0.bit.SYSPLL_POSTDIV_SDIO_EN){
        return 0;
    }
    div = IP_CMN_BUSCFG->REG_SYSPLL_CFG1.bit.SYSPLL_POSTDIV_SDIO_DIV_SEL + 3;
    freq = BOARD_BOOTCLOCKRUN_SYSPLL_CLK / div;
    return freq;
}

uint32_t CRM_GetSyspllAudFreq(void){
    return BOARD_BOOTCLOCKRUN_XTAL24M_CLK;
}

uint32_t CRM_GetUsbUtmiFreq(void){
    return 30000000;
}

uint32_t CRM_GetVicPixelFreq(void){
    return BOARD_BOOTCLOCKRUN_XTAL24M_CLK;
}

// Retrieves the frequency of a specified clock source.
// Returns the frequency based on the source, or 0 for unsupported sources.
uint32_t CRM_GetSrcFreq(clock_src_name_t src){
    switch (src){
    case CRM_IpSrcRC032K:
        return CRM_GetRC032KFreq();
    
    case CRM_IpSrcXTAL32K:
        return CRM_GetXTAL32KFreq();
    
    case CRM_IpSrcRC024M:
        return CRM_GetRC024MFreq();
    
    case CRM_IpSrcXTAL24M:
        return CRM_GetXTAL24MFreq();
    
    case CRM_IpSrcCORE32K:
        return CRM_GetCORE32KFreq();
    
    case CRM_IpSrcCORE24M:
        return CRM_GetCORE24MFreq();
    
    case CRM_IpSrcSyspllCore:
        return CRM_GetSyspllCoreFreq();
    
    case CRM_IpSrcSyspllPeri:
        return CRM_GetSyspllPeriFreq();
    
    case CRM_IpSrcSyspllFlash:
        return CRM_GetSyspllFlashFreq();
    
    case CRM_IpSrcSyspllPsram:
        return CRM_GetSyspllPsramFreq();
    
    case CRM_IpSrcSyspllSdio:
        return CRM_GetSyspllSdioFreq();
    
    case CRM_IpSrcSyspllAud:
        return CRM_GetSyspllAudFreq();
    
    case CRM_IpSrcUsbUtmi:
        return CRM_GetUsbUtmiFreq();
    
    case CRM_IpSrcVicPixel:
        return CRM_GetVicPixelFreq();
    
    default:
        return 0;
    }
}

/**
  * @brief  Function to configure the SYSCLK clock source
  * @param  src specifies which source that SYSCLK can choose
  *         This parameter can be one of the following values:
  *         @arg 0: SYSCLK using CORE24M to source clock
  *         @arg 1: SYSCLK using RC024M to source clock
  *         @arg 2: SYSCLK using SyspllCore to source clock
  * @return CSK HAL status
  */
int32_t HAL_CRM_SetSysclkClkSrc(clock_src_name_t src){
    if (src == CRM_IpSrcCORE24M){
        IP_CMN_BUSCFG->REG_BUS_CLK_CFG0.bit.SEL_HCLK = 0;
    }if (src == CRM_IpSrcRC024M){
        IP_CMN_BUSCFG->REG_BUS_CLK_CFG0.bit.SEL_HCLK = 1;
    }if (src == CRM_IpSrcSyspllCore){
        IP_CMN_BUSCFG->REG_BUS_CLK_CFG0.bit.SEL_HCLK = 2;
    }
    return CSK_DRIVER_OK;
}

// Retrieves the SYSCLK clock frequency.
// Returns 0 if SYSCLK is disabled, otherwise calculates frequency based on source and dividers.
uint32_t CRM_GetSysclkFreq(void){
    uint32_t freq = 0;
    
    clock_src_name_t src = 0;
    HAL_CRM_GetSysclkClkConfig(&src );    
    freq = CRM_GetSrcFreq(src);

    return freq;
}

/**
  * @brief  Function to configure the HCLK clock divider
  * @param div_n must less than or equal to 15
  * @param div_m must less than or equal to 31
  */
int32_t HAL_CRM_SetHclkClkDiv(
    uint32_t div_n, uint32_t div_m){
    IP_CMN_BUSCFG->REG_BUS_CLK_CFG0.bit.DIV_HCLK_LD = 0x0;
    if (div_n > 15){
        return CSK_DRIVER_ERROR_PARAMETER;
    }
    IP_CMN_BUSCFG->REG_BUS_CLK_CFG0.bit.DIV_HCLK_N = div_n;
    
    if (div_m > 31){
        return CSK_DRIVER_ERROR_PARAMETER;
    }
    IP_CMN_BUSCFG->REG_BUS_CLK_CFG0.bit.DIV_HCLK_M = div_m-1;
    

    IP_CMN_BUSCFG->REG_BUS_CLK_CFG0.bit.DIV_HCLK_LD = 0x1;
    return CSK_DRIVER_OK;
}
    

// Retrieves the HCLK clock frequency.
// Returns 0 if HCLK is disabled, otherwise calculates frequency based on source and dividers.
uint32_t CRM_GetHclkFreq(void){
    uint32_t freq = 0;
    uint32_t div_n = 1;
    uint32_t div_m = 1;
    
    
    HAL_CRM_GetHclkClkConfig(&div_n, &div_m);    
    freq = CRM_GetSysclkFreq();

    return freq*div_n/div_m;
}

/**
  * @brief  Function to configure the CMN_PCLK clock divider
  * @param div_n must less than or equal to 15
  * @param div_m must less than or equal to 31
  */
int32_t HAL_CRM_SetCmn_pclkClkDiv(
    uint32_t div_n, uint32_t div_m){
    IP_CMN_BUSCFG->REG_BUS_CLK_CFG1.bit.DIV_CMN_PERI_PCLK_LD = 0x0;
    if (div_n > 15){
        return CSK_DRIVER_ERROR_PARAMETER;
    }
    IP_CMN_BUSCFG->REG_BUS_CLK_CFG1.bit.DIV_CMN_PERI_PCLK_N = div_n;
    
    if (div_m > 31){
        return CSK_DRIVER_ERROR_PARAMETER;
    }
    IP_CMN_BUSCFG->REG_BUS_CLK_CFG1.bit.DIV_CMN_PERI_PCLK_M = div_m-1;
    

    IP_CMN_BUSCFG->REG_BUS_CLK_CFG1.bit.DIV_CMN_PERI_PCLK_LD = 0x1;
    return CSK_DRIVER_OK;
}
    

// Retrieves the CMN_PCLK clock frequency.
// Returns 0 if CMN_PCLK is disabled, otherwise calculates frequency based on source and dividers.
uint32_t CRM_GetCmn_pclkFreq(void){
    uint32_t freq = 0;
    uint32_t div_n = 1;
    uint32_t div_m = 1;
    
    
    HAL_CRM_GetCmn_pclkClkConfig(&div_n, &div_m);    
    freq = CRM_GetHclkFreq();

    return freq*div_n/div_m;
}

/**
  * @brief  Function to configure the AON_CFG_PCLK clock divider
  * @param div_n must less than or equal to 31
  * @param div_m must less than or equal to 63
  */
int32_t HAL_CRM_SetAon_cfg_pclkClkDiv(
    uint32_t div_n, uint32_t div_m){
    IP_CMN_BUSCFG->REG_BUS_CLK_CFG1.bit.DIV_AON_CFG_PCLK_LD = 0x0;
    if (div_n > 31){
        return CSK_DRIVER_ERROR_PARAMETER;
    }
    IP_CMN_BUSCFG->REG_BUS_CLK_CFG1.bit.DIV_AON_CFG_PCLK_N = div_n;
    
    if (div_m > 63){
        return CSK_DRIVER_ERROR_PARAMETER;
    }
    IP_CMN_BUSCFG->REG_BUS_CLK_CFG1.bit.DIV_AON_CFG_PCLK_M = div_m-1;
    

    IP_CMN_BUSCFG->REG_BUS_CLK_CFG1.bit.DIV_AON_CFG_PCLK_LD = 0x1;
    return CSK_DRIVER_OK;
}
    

// Retrieves the AON_CFG_PCLK clock frequency.
// Returns 0 if AON_CFG_PCLK is disabled, otherwise calculates frequency based on source and dividers.
uint32_t CRM_GetAon_cfg_pclkFreq(void){
    uint32_t freq = 0;
    uint32_t div_n = 1;
    uint32_t div_m = 1;
    
    
    HAL_CRM_GetAon_cfg_pclkClkConfig(&div_n, &div_m);    
    freq = CRM_GetHclkFreq();

    return freq*div_n/div_m;
}

// Retrieves the CORE0 clock frequency.
// Returns 0 if CORE0 is disabled, otherwise calculates frequency based on source and dividers.
uint32_t CRM_GetCore0Freq(void){
    uint32_t freq = 0;
    
    
        
    freq = CRM_GetHclkFreq();

    return freq;
}

// Retrieves the CORE1 clock frequency.
// Returns 0 if CORE1 is disabled, otherwise calculates frequency based on source and dividers.
uint32_t CRM_GetCore1Freq(void){
    uint32_t freq = 0;
    
    
        
    freq = CRM_GetHclkFreq();

    return freq;
}

/**
  * @brief  Function to configure the PSRAM clock divider
  * @param div_m must less than or equal to 1023
  */
int32_t HAL_CRM_SetPsramClkDiv(
    uint32_t div_m){
    IP_CMN_SYSCFG->REG_PERI_CLK_CFG0.bit.DIV_PSRAM_CLK_LD = 0x0;
    if (div_m > 1023){
        return CSK_DRIVER_ERROR_PARAMETER;
    }
    IP_CMN_SYSCFG->REG_PERI_CLK_CFG0.bit.DIV_PSRAM_CLK_M = div_m-1;
    

    IP_CMN_SYSCFG->REG_PERI_CLK_CFG0.bit.DIV_PSRAM_CLK_LD = 0x1;
    return CSK_DRIVER_OK;
}
    

/**
  * @brief  Function to configure the PSRAM clock source
  * @param  src specifies which source that PSRAM can choose
  *         This parameter can be one of the following values:
  *         @arg 0: PSRAM using CORE24M to source clock
  *         @arg 1: PSRAM using SyspllPsram to source clock
  * @return CSK HAL status
  */
int32_t HAL_CRM_SetPsramClkSrc(clock_src_name_t src){
    if (src == CRM_IpSrcCORE24M){
        IP_CMN_SYSCFG->REG_PERI_CLK_CFG0.bit.SEL_PSRAM_CLK = 0;
    }if (src == CRM_IpSrcSyspllPsram){
        IP_CMN_SYSCFG->REG_PERI_CLK_CFG0.bit.SEL_PSRAM_CLK = 1;
    }
    return CSK_DRIVER_OK;
}

// Retrieves the PSRAM clock frequency.
// Returns 0 if PSRAM is disabled, otherwise calculates frequency based on source and dividers.
uint32_t CRM_GetPsramFreq(void){
    uint32_t freq = 0;
    uint32_t div_m = 1;
    if (!HAL_CRM_PsramClkIsEnabled()){
        return 0;
    }
    clock_src_name_t src = 0;
    HAL_CRM_GetPsramClkConfig(&src, &div_m);    
    freq = CRM_GetSrcFreq(src);

    return freq/div_m;
}

/**
  * @brief  Function to configure the MTIME clock divider
  * @param div_m must less than or equal to 63
  */
int32_t HAL_CRM_SetMtimeClkDiv(
    uint32_t div_m){
    IP_CMN_SYSCFG->REG_PERI_CLK_CFG0.bit.MTIME_TOGGLE_LD = 0x0;
    if (div_m > 63){
        return CSK_DRIVER_ERROR_PARAMETER;
    }
    IP_CMN_SYSCFG->REG_PERI_CLK_CFG0.bit.DIV_MTIME_TOGGLE_M = div_m-1;
    

    IP_CMN_SYSCFG->REG_PERI_CLK_CFG0.bit.MTIME_TOGGLE_LD = 0x1;
    return CSK_DRIVER_OK;
}
    

// Retrieves the MTIME clock frequency.
// Returns 0 if MTIME is disabled, otherwise calculates frequency based on source and dividers.
uint32_t CRM_GetMtimeFreq(void){
    uint32_t freq = 0;
    uint32_t div_m = 1;
    if (!HAL_CRM_MtimeClkIsEnabled()){
        return 0;
    }
    
    HAL_CRM_GetMtimeClkConfig( &div_m);    
    
    freq = CRM_GetCORE24MFreq();

    return freq/div_m;
}

/**
  * @brief  Function to configure the FLASH clock divider
  * @param div_m must less than or equal to 31
  */
int32_t HAL_CRM_SetFlashClkDiv(
    uint32_t div_m){
    IP_CMN_SYSCFG->REG_PERI_CLK_CFG0.bit.DIV_FLASHC_CLK_LD = 0x0;
    if (div_m > 31){
        return CSK_DRIVER_ERROR_PARAMETER;
    }
    IP_CMN_SYSCFG->REG_PERI_CLK_CFG0.bit.DIV_FLASHC_CLK_M = div_m-1;
    

    IP_CMN_SYSCFG->REG_PERI_CLK_CFG0.bit.DIV_FLASHC_CLK_LD = 0x1;
    return CSK_DRIVER_OK;
}
    

/**
  * @brief  Function to configure the FLASH clock source
  * @param  src specifies which source that FLASH can choose
  *         This parameter can be one of the following values:
  *         @arg 0: FLASH using CORE24M to source clock
  *         @arg 1: FLASH using SyspllFlash to source clock
  * @return CSK HAL status
  */
int32_t HAL_CRM_SetFlashClkSrc(clock_src_name_t src){
    if (src == CRM_IpSrcCORE24M){
        IP_CMN_SYSCFG->REG_PERI_CLK_CFG0.bit.SEL_FLASHC_CLK = 0;
    }if (src == CRM_IpSrcSyspllFlash){
        IP_CMN_SYSCFG->REG_PERI_CLK_CFG0.bit.SEL_FLASHC_CLK = 1;
    }
    return CSK_DRIVER_OK;
}

// Retrieves the FLASH clock frequency.
// Returns 0 if FLASH is disabled, otherwise calculates frequency based on source and dividers.
uint32_t CRM_GetFlashFreq(void){
    uint32_t freq = 0;
    uint32_t div_m = 1;
    if (!HAL_CRM_FlashClkIsEnabled()){
        return 0;
    }
    clock_src_name_t src = 0;
    HAL_CRM_GetFlashClkConfig(&src, &div_m);    
    freq = CRM_GetSrcFreq(src);

    return freq/div_m;
}

/**
  * @brief  Function to configure the SPI0 clock divider
  * @param div_n must less than or equal to 7
  * @param div_m must less than or equal to 15
  */
int32_t HAL_CRM_SetSpi0ClkDiv(
    uint32_t div_n, uint32_t div_m){
    IP_CMN_SYSCFG->REG_PERI_CLK_CFG1.bit.DIV_SPI0_CLK_LD = 0x0;
    if (div_n > 7){
        return CSK_DRIVER_ERROR_PARAMETER;
    }
    IP_CMN_SYSCFG->REG_PERI_CLK_CFG1.bit.DIV_SPI0_CLK_N = div_n;
    
    if (div_m > 15){
        return CSK_DRIVER_ERROR_PARAMETER;
    }
    IP_CMN_SYSCFG->REG_PERI_CLK_CFG1.bit.DIV_SPI0_CLK_M = div_m-1;
    

    IP_CMN_SYSCFG->REG_PERI_CLK_CFG1.bit.DIV_SPI0_CLK_LD = 0x1;
    return CSK_DRIVER_OK;
}
    

/**
  * @brief  Function to configure the SPI0 clock source
  * @param  src specifies which source that SPI0 can choose
  *         This parameter can be one of the following values:
  *         @arg 0: SPI0 using CORE24M to source clock
  *         @arg 1: SPI0 using SyspllPeri to source clock
  * @return CSK HAL status
  */
int32_t HAL_CRM_SetSpi0ClkSrc(clock_src_name_t src){
    if (src == CRM_IpSrcCORE24M){
        IP_CMN_SYSCFG->REG_PERI_CLK_CFG1.bit.SEL_SPI0_CLK = 0;
    }if (src == CRM_IpSrcSyspllPeri){
        IP_CMN_SYSCFG->REG_PERI_CLK_CFG1.bit.SEL_SPI0_CLK = 1;
    }
    return CSK_DRIVER_OK;
}

// Retrieves the SPI0 clock frequency.
// Returns 0 if SPI0 is disabled, otherwise calculates frequency based on source and dividers.
uint32_t CRM_GetSpi0Freq(void){
    uint32_t freq = 0;
    uint32_t div_n = 1;
    uint32_t div_m = 1;
    if (!HAL_CRM_Spi0ClkIsEnabled()){
        return 0;
    }
    clock_src_name_t src = 0;
    HAL_CRM_GetSpi0ClkConfig(&src,&div_n, &div_m);    
    freq = CRM_GetSrcFreq(src);

    return freq*div_n/div_m;
}

/**
  * @brief  Function to configure the UART0 clock divider
  * @param div_n must less than or equal to 511
  * @param div_m must less than or equal to 1023
  */
int32_t HAL_CRM_SetUart0ClkDiv(
    uint32_t div_n, uint32_t div_m){
    IP_CMN_SYSCFG->REG_PERI_CLK_CFG1.bit.DIV_UART0_CLK_LD = 0x0;
    if (div_n > 511){
        return CSK_DRIVER_ERROR_PARAMETER;
    }
    IP_CMN_SYSCFG->REG_PERI_CLK_CFG1.bit.DIV_UART0_CLK_N = div_n;
    
    if (div_m > 1023){
        return CSK_DRIVER_ERROR_PARAMETER;
    }
    IP_CMN_SYSCFG->REG_PERI_CLK_CFG1.bit.DIV_UART0_CLK_M = div_m-1;
    

    IP_CMN_SYSCFG->REG_PERI_CLK_CFG1.bit.DIV_UART0_CLK_LD = 0x1;
    return CSK_DRIVER_OK;
}
    

/**
  * @brief  Function to configure the UART0 clock source
  * @param  src specifies which source that UART0 can choose
  *         This parameter can be one of the following values:
  *         @arg 0: UART0 using CORE24M to source clock
  *         @arg 1: UART0 using SyspllPeri to source clock
  * @return CSK HAL status
  */
int32_t HAL_CRM_SetUart0ClkSrc(clock_src_name_t src){
    if (src == CRM_IpSrcCORE24M){
        IP_CMN_SYSCFG->REG_PERI_CLK_CFG1.bit.SEL_UART0_CLK = 0;
    }if (src == CRM_IpSrcSyspllPeri){
        IP_CMN_SYSCFG->REG_PERI_CLK_CFG1.bit.SEL_UART0_CLK = 1;
    }
    return CSK_DRIVER_OK;
}

// Retrieves the UART0 clock frequency.
// Returns 0 if UART0 is disabled, otherwise calculates frequency based on source and dividers.
uint32_t CRM_GetUart0Freq(void){
    uint32_t freq = 0;
    uint32_t div_n = 1;
    uint32_t div_m = 1;
    if (!HAL_CRM_Uart0ClkIsEnabled()){
        return 0;
    }
    clock_src_name_t src = 0;
    HAL_CRM_GetUart0ClkConfig(&src,&div_n, &div_m);    
    freq = CRM_GetSrcFreq(src);

    return freq*div_n/div_m;
}

/**
  * @brief  Function to configure the SPI1 clock divider
  * @param div_n must less than or equal to 7
  * @param div_m must less than or equal to 15
  */
int32_t HAL_CRM_SetSpi1ClkDiv(
    uint32_t div_n, uint32_t div_m){
    IP_CMN_SYSCFG->REG_PERI_CLK_CFG2.bit.DIV_SPI1_CLK_LD = 0x0;
    if (div_n > 7){
        return CSK_DRIVER_ERROR_PARAMETER;
    }
    IP_CMN_SYSCFG->REG_PERI_CLK_CFG2.bit.DIV_SPI1_CLK_N = div_n;
    
    if (div_m > 15){
        return CSK_DRIVER_ERROR_PARAMETER;
    }
    IP_CMN_SYSCFG->REG_PERI_CLK_CFG2.bit.DIV_SPI1_CLK_M = div_m-1;
    

    IP_CMN_SYSCFG->REG_PERI_CLK_CFG2.bit.DIV_SPI1_CLK_LD = 0x1;
    return CSK_DRIVER_OK;
}
    

/**
  * @brief  Function to configure the SPI1 clock source
  * @param  src specifies which source that SPI1 can choose
  *         This parameter can be one of the following values:
  *         @arg 0: SPI1 using CORE24M to source clock
  *         @arg 1: SPI1 using SyspllPeri to source clock
  * @return CSK HAL status
  */
int32_t HAL_CRM_SetSpi1ClkSrc(clock_src_name_t src){
    if (src == CRM_IpSrcCORE24M){
        IP_CMN_SYSCFG->REG_PERI_CLK_CFG2.bit.SEL_SPI1_CLK = 0;
    }if (src == CRM_IpSrcSyspllPeri){
        IP_CMN_SYSCFG->REG_PERI_CLK_CFG2.bit.SEL_SPI1_CLK = 1;
    }
    return CSK_DRIVER_OK;
}

// Retrieves the SPI1 clock frequency.
// Returns 0 if SPI1 is disabled, otherwise calculates frequency based on source and dividers.
uint32_t CRM_GetSpi1Freq(void){
    uint32_t freq = 0;
    uint32_t div_n = 1;
    uint32_t div_m = 1;
    if (!HAL_CRM_Spi1ClkIsEnabled()){
        return 0;
    }
    clock_src_name_t src = 0;
    HAL_CRM_GetSpi1ClkConfig(&src,&div_n, &div_m);    
    freq = CRM_GetSrcFreq(src);

    return freq*div_n/div_m;
}

/**
  * @brief  Function to configure the UART1 clock divider
  * @param div_n must less than or equal to 511
  * @param div_m must less than or equal to 1023
  */
int32_t HAL_CRM_SetUart1ClkDiv(
    uint32_t div_n, uint32_t div_m){
    IP_CMN_SYSCFG->REG_PERI_CLK_CFG2.bit.DIV_UART1_CLK_LD = 0x0;
    if (div_n > 511){
        return CSK_DRIVER_ERROR_PARAMETER;
    }
    IP_CMN_SYSCFG->REG_PERI_CLK_CFG2.bit.DIV_UART1_CLK_N = div_n;
    
    if (div_m > 1023){
        return CSK_DRIVER_ERROR_PARAMETER;
    }
    IP_CMN_SYSCFG->REG_PERI_CLK_CFG2.bit.DIV_UART1_CLK_M = div_m-1;
    

    IP_CMN_SYSCFG->REG_PERI_CLK_CFG2.bit.DIV_UART1_CLK_LD = 0x1;
    return CSK_DRIVER_OK;
}
    

/**
  * @brief  Function to configure the UART1 clock source
  * @param  src specifies which source that UART1 can choose
  *         This parameter can be one of the following values:
  *         @arg 0: UART1 using CORE24M to source clock
  *         @arg 1: UART1 using SyspllPeri to source clock
  * @return CSK HAL status
  */
int32_t HAL_CRM_SetUart1ClkSrc(clock_src_name_t src){
    if (src == CRM_IpSrcCORE24M){
        IP_CMN_SYSCFG->REG_PERI_CLK_CFG2.bit.SEL_UART1_CLK = 0;
    }if (src == CRM_IpSrcSyspllPeri){
        IP_CMN_SYSCFG->REG_PERI_CLK_CFG2.bit.SEL_UART1_CLK = 1;
    }
    return CSK_DRIVER_OK;
}

// Retrieves the UART1 clock frequency.
// Returns 0 if UART1 is disabled, otherwise calculates frequency based on source and dividers.
uint32_t CRM_GetUart1Freq(void){
    uint32_t freq = 0;
    uint32_t div_n = 1;
    uint32_t div_m = 1;
    if (!HAL_CRM_Uart1ClkIsEnabled()){
        return 0;
    }
    clock_src_name_t src = 0;
    HAL_CRM_GetUart1ClkConfig(&src,&div_n, &div_m);    
    freq = CRM_GetSrcFreq(src);

    return freq*div_n/div_m;
}

/**
  * @brief  Function to configure the SDIOH clock divider
  * @param div_n must less than or equal to 7
  * @param div_m must less than or equal to 15
  */
int32_t HAL_CRM_SetSdiohClkDiv(
    uint32_t div_n, uint32_t div_m){
    IP_CMN_SYSCFG->REG_PERI_CLK_CFG3.bit.DIV_SDIOH_CLK2X_LD = 0x0;
    if (div_n > 7){
        return CSK_DRIVER_ERROR_PARAMETER;
    }
    IP_CMN_SYSCFG->REG_PERI_CLK_CFG3.bit.DIV_SDIOH_CLK2X_N = div_n;
    
    if (div_m > 15){
        return CSK_DRIVER_ERROR_PARAMETER;
    }
    IP_CMN_SYSCFG->REG_PERI_CLK_CFG3.bit.DIV_SDIOH_CLK2X_M = div_m-1;
    

    IP_CMN_SYSCFG->REG_PERI_CLK_CFG3.bit.DIV_SDIOH_CLK2X_LD = 0x1;
    return CSK_DRIVER_OK;
}
    

/**
  * @brief  Function to configure the SDIOH clock source
  * @param  src specifies which source that SDIOH can choose
  *         This parameter can be one of the following values:
  *         @arg 0: SDIOH using CORE24M to source clock
  *         @arg 1: SDIOH using SyspllSdio to source clock
  * @return CSK HAL status
  */
int32_t HAL_CRM_SetSdiohClkSrc(clock_src_name_t src){
    if (src == CRM_IpSrcCORE24M){
        IP_CMN_SYSCFG->REG_PERI_CLK_CFG3.bit.SEL_SDIOH_CLK2X = 0;
    }if (src == CRM_IpSrcSyspllSdio){
        IP_CMN_SYSCFG->REG_PERI_CLK_CFG3.bit.SEL_SDIOH_CLK2X = 1;
    }
    return CSK_DRIVER_OK;
}

// Retrieves the SDIOH clock frequency.
// Returns 0 if SDIOH is disabled, otherwise calculates frequency based on source and dividers.
uint32_t CRM_GetSdiohFreq(void){
    uint32_t freq = 0;
    uint32_t div_n = 1;
    uint32_t div_m = 1;
    if (!HAL_CRM_SdiohClkIsEnabled()){
        return 0;
    }
    clock_src_name_t src = 0;
    HAL_CRM_GetSdiohClkConfig(&src,&div_n, &div_m);    
    freq = CRM_GetSrcFreq(src);

    return freq*div_n/div_m;
}

/**
  * @brief  Function to configure the UART2 clock divider
  * @param div_n must less than or equal to 511
  * @param div_m must less than or equal to 1023
  */
int32_t HAL_CRM_SetUart2ClkDiv(
    uint32_t div_n, uint32_t div_m){
    IP_CMN_SYSCFG->REG_PERI_CLK_CFG3.bit.DIV_UART2_CLK_LD = 0x0;
    if (div_n > 511){
        return CSK_DRIVER_ERROR_PARAMETER;
    }
    IP_CMN_SYSCFG->REG_PERI_CLK_CFG3.bit.DIV_UART2_CLK_N = div_n;
    
    if (div_m > 1023){
        return CSK_DRIVER_ERROR_PARAMETER;
    }
    IP_CMN_SYSCFG->REG_PERI_CLK_CFG3.bit.DIV_UART2_CLK_M = div_m-1;
    

    IP_CMN_SYSCFG->REG_PERI_CLK_CFG3.bit.DIV_UART2_CLK_LD = 0x1;
    return CSK_DRIVER_OK;
}
    

/**
  * @brief  Function to configure the UART2 clock source
  * @param  src specifies which source that UART2 can choose
  *         This parameter can be one of the following values:
  *         @arg 0: UART2 using CORE24M to source clock
  *         @arg 1: UART2 using SyspllPeri to source clock
  * @return CSK HAL status
  */
int32_t HAL_CRM_SetUart2ClkSrc(clock_src_name_t src){
    if (src == CRM_IpSrcCORE24M){
        IP_CMN_SYSCFG->REG_PERI_CLK_CFG3.bit.SEL_UART2_CLK = 0;
    }if (src == CRM_IpSrcSyspllPeri){
        IP_CMN_SYSCFG->REG_PERI_CLK_CFG3.bit.SEL_UART2_CLK = 1;
    }
    return CSK_DRIVER_OK;
}

// Retrieves the UART2 clock frequency.
// Returns 0 if UART2 is disabled, otherwise calculates frequency based on source and dividers.
uint32_t CRM_GetUart2Freq(void){
    uint32_t freq = 0;
    uint32_t div_n = 1;
    uint32_t div_m = 1;
    if (!HAL_CRM_Uart2ClkIsEnabled()){
        return 0;
    }
    clock_src_name_t src = 0;
    HAL_CRM_GetUart2ClkConfig(&src,&div_n, &div_m);    
    freq = CRM_GetSrcFreq(src);

    return freq*div_n/div_m;
}

/**
  * @brief  Function to configure the VIC_OUT clock divider
  * @param div_m must less than or equal to 511
  */
int32_t HAL_CRM_SetVic_outClkDiv(
    uint32_t div_m){
    IP_CMN_SYSCFG->REG_PERI_CLK_CFG4.bit.DIV_VIC_OUT_CLK_LD = 0x0;
    if (div_m > 511){
        return CSK_DRIVER_ERROR_PARAMETER;
    }
    IP_CMN_SYSCFG->REG_PERI_CLK_CFG4.bit.DIV_VIC_OUT_CLK_M = div_m-1;
    

    IP_CMN_SYSCFG->REG_PERI_CLK_CFG4.bit.DIV_VIC_OUT_CLK_LD = 0x1;
    return CSK_DRIVER_OK;
}
    

/**
  * @brief  Function to configure the VIC_OUT clock source
  * @param  src specifies which source that VIC_OUT can choose
  *         This parameter can be one of the following values:
  *         @arg 0: VIC_OUT using CORE24M to source clock
  *         @arg 1: VIC_OUT using SyspllPeri to source clock
  * @return CSK HAL status
  */
int32_t HAL_CRM_SetVic_outClkSrc(clock_src_name_t src){
    if (src == CRM_IpSrcCORE24M){
        IP_CMN_SYSCFG->REG_PERI_CLK_CFG4.bit.SEL_VIC_OUT_CLK = 0;
    }if (src == CRM_IpSrcSyspllPeri){
        IP_CMN_SYSCFG->REG_PERI_CLK_CFG4.bit.SEL_VIC_OUT_CLK = 1;
    }
    return CSK_DRIVER_OK;
}

// Retrieves the VIC_OUT clock frequency.
// Returns 0 if VIC_OUT is disabled, otherwise calculates frequency based on source and dividers.
uint32_t CRM_GetVic_outFreq(void){
    uint32_t freq = 0;
    uint32_t div_m = 1;
    if (!HAL_CRM_Vic_outClkIsEnabled()){
        return 0;
    }
    clock_src_name_t src = 0;
    HAL_CRM_GetVic_outClkConfig(&src, &div_m);    
    freq = CRM_GetSrcFreq(src);

    return freq/div_m;
}

/**
  * @brief  Function to configure the GPT clock divider
  * @param div_m must less than or equal to 31
  */
int32_t HAL_CRM_SetGptClkDiv(
    uint32_t div_m){
    IP_CMN_SYSCFG->REG_PERI_CLK_CFG4.bit.DIV_GPT_T0_CLK_LD = 0x0;
    if (div_m > 31){
        return CSK_DRIVER_ERROR_PARAMETER;
    }
    IP_CMN_SYSCFG->REG_PERI_CLK_CFG4.bit.DIV_GPT_T0_CLK_M = div_m-1;
    

    IP_CMN_SYSCFG->REG_PERI_CLK_CFG4.bit.DIV_GPT_T0_CLK_LD = 0x1;
    return CSK_DRIVER_OK;
}
    

// Retrieves the GPT clock frequency.
// Returns 0 if GPT is disabled, otherwise calculates frequency based on source and dividers.
uint32_t CRM_GetGptFreq(void){
    uint32_t freq = 0;
    uint32_t div_m = 1;
    if (!HAL_CRM_GptClkIsEnabled()){
        return 0;
    }
    
    HAL_CRM_GetGptClkConfig( &div_m);    
    
    freq = CRM_GetCORE24MFreq();

    return freq/div_m;
}

/**
  * @brief  Function to configure the I8080 clock divider
  * @param div_m must less than or equal to 15
  */
int32_t HAL_CRM_SetI8080ClkDiv(
    uint32_t div_m){
    IP_CMN_SYSCFG->REG_PERI_CLK_CFG5.bit.DIV_I8080_CLK_LD = 0x0;
    if (div_m > 15){
        return CSK_DRIVER_ERROR_PARAMETER;
    }
    IP_CMN_SYSCFG->REG_PERI_CLK_CFG5.bit.DIV_I8080_CLK_M = div_m-1;
    

    IP_CMN_SYSCFG->REG_PERI_CLK_CFG5.bit.DIV_I8080_CLK_LD = 0x1;
    return CSK_DRIVER_OK;
}
    

/**
  * @brief  Function to configure the I8080 clock source
  * @param  src specifies which source that I8080 can choose
  *         This parameter can be one of the following values:
  *         @arg 0: I8080 using CORE24M to source clock
  *         @arg 1: I8080 using SyspllPeri to source clock
  * @return CSK HAL status
  */
int32_t HAL_CRM_SetI8080ClkSrc(clock_src_name_t src){
    if (src == CRM_IpSrcCORE24M){
        IP_CMN_SYSCFG->REG_PERI_CLK_CFG5.bit.SEL_I8080_CLK = 0;
    }if (src == CRM_IpSrcSyspllPeri){
        IP_CMN_SYSCFG->REG_PERI_CLK_CFG5.bit.SEL_I8080_CLK = 1;
    }
    return CSK_DRIVER_OK;
}

// Retrieves the I8080 clock frequency.
// Returns 0 if I8080 is disabled, otherwise calculates frequency based on source and dividers.
uint32_t CRM_GetI8080Freq(void){
    uint32_t freq = 0;
    uint32_t div_m = 1;
    if (!HAL_CRM_I8080ClkIsEnabled()){
        return 0;
    }
    clock_src_name_t src = 0;
    HAL_CRM_GetI8080ClkConfig(&src, &div_m);    
    freq = CRM_GetSrcFreq(src);

    return freq/div_m;
}

/**
  * @brief  Function to configure the IR clock divider
  * @param div_m must less than or equal to 63
  */
int32_t HAL_CRM_SetIrClkDiv(
    uint32_t div_m){
    IP_CMN_SYSCFG->REG_PERI_CLK_CFG5.bit.DIV_IR_CLK_TX_LD = 0x0;
    if (div_m > 63){
        return CSK_DRIVER_ERROR_PARAMETER;
    }
    IP_CMN_SYSCFG->REG_PERI_CLK_CFG5.bit.DIV_IR_CLK_TX_M = div_m-1;
    

    IP_CMN_SYSCFG->REG_PERI_CLK_CFG5.bit.DIV_IR_CLK_TX_LD = 0x1;
    return CSK_DRIVER_OK;
}
    

// Retrieves the IR clock frequency.
// Returns 0 if IR is disabled, otherwise calculates frequency based on source and dividers.
uint32_t CRM_GetIrFreq(void){
    uint32_t freq = 0;
    uint32_t div_m = 1;
    if (!HAL_CRM_IrClkIsEnabled()){
        return 0;
    }
    
    HAL_CRM_GetIrClkConfig( &div_m);    
    
    freq = CRM_GetCORE24MFreq();

    return freq/div_m;
}

/**
  * @brief  Function to configure the GPADC clock divider
  * @param div_m must less than or equal to 1023
  */
int32_t HAL_CRM_SetGpadcClkDiv(
    uint32_t div_m){
    IP_CMN_SYSCFG->REG_PERI_CLK_CFG5.bit.DIV_GPADC_CLK_LD = 0x0;
    if (div_m > 1023){
        return CSK_DRIVER_ERROR_PARAMETER;
    }
    IP_CMN_SYSCFG->REG_PERI_CLK_CFG5.bit.DIV_GPADC_CLK_M = div_m-1;
    

    IP_CMN_SYSCFG->REG_PERI_CLK_CFG5.bit.DIV_GPADC_CLK_LD = 0x1;
    return CSK_DRIVER_OK;
}
    

// Retrieves the GPADC clock frequency.
// Returns 0 if GPADC is disabled, otherwise calculates frequency based on source and dividers.
uint32_t CRM_GetGpadcFreq(void){
    uint32_t freq = 0;
    uint32_t div_m = 1;
    if (!HAL_CRM_GpadcClkIsEnabled()){
        return 0;
    }
    
    HAL_CRM_GetGpadcClkConfig( &div_m);    
    
    freq = CRM_GetCORE24MFreq();

    return freq/div_m;
}

/**
  * @brief  Function to configure the I2S0 clock source
  * @param  src specifies which source that I2S0 can choose
  *         This parameter can be one of the following values:
  *         @arg 0: I2S0 using CORE24M to source clock
  *         @arg 1: I2S0 using SyspllAud to source clock
  * @return CSK HAL status
  */
int32_t HAL_CRM_SetI2s0ClkSrc(clock_src_name_t src){
    if (src == CRM_IpSrcCORE24M){
        IP_CMN_SYSCFG->REG_PERI_CLK_CFG6.bit.SEL_I2S0_MCLK = 0;
    }if (src == CRM_IpSrcSyspllAud){
        IP_CMN_SYSCFG->REG_PERI_CLK_CFG6.bit.SEL_I2S0_MCLK = 1;
    }
    return CSK_DRIVER_OK;
}

// Retrieves the I2S0 clock frequency.
// Returns 0 if I2S0 is disabled, otherwise calculates frequency based on source and dividers.
uint32_t CRM_GetI2s0Freq(void){
    uint32_t freq = 0;
    if (!HAL_CRM_I2s0ClkIsEnabled()){
        return 0;
    }
    clock_src_name_t src = 0;
    HAL_CRM_GetI2s0ClkConfig(&src );    
    freq = CRM_GetSrcFreq(src);

    return freq;
}

/**
  * @brief  Function to configure the I2S1 clock source
  * @param  src specifies which source that I2S1 can choose
  *         This parameter can be one of the following values:
  *         @arg 0: I2S1 using CORE24M to source clock
  *         @arg 1: I2S1 using SyspllAud to source clock
  * @return CSK HAL status
  */
int32_t HAL_CRM_SetI2s1ClkSrc(clock_src_name_t src){
    if (src == CRM_IpSrcCORE24M){
        IP_CMN_SYSCFG->REG_PERI_CLK_CFG6.bit.SEL_I2S1_MCLK = 0;
    }if (src == CRM_IpSrcSyspllAud){
        IP_CMN_SYSCFG->REG_PERI_CLK_CFG6.bit.SEL_I2S1_MCLK = 1;
    }
    return CSK_DRIVER_OK;
}

// Retrieves the I2S1 clock frequency.
// Returns 0 if I2S1 is disabled, otherwise calculates frequency based on source and dividers.
uint32_t CRM_GetI2s1Freq(void){
    uint32_t freq = 0;
    if (!HAL_CRM_I2s1ClkIsEnabled()){
        return 0;
    }
    clock_src_name_t src = 0;
    HAL_CRM_GetI2s1ClkConfig(&src );    
    freq = CRM_GetSrcFreq(src);

    return freq;
}

/**
  * @brief  Function to configure the RGB clock divider
  * @param div_m must less than or equal to 15
  */
int32_t HAL_CRM_SetRgbClkDiv(
    uint32_t div_m){
    IP_CMN_SYSCFG->REG_PERI_CLK_CFG6.bit.DIV_RGB_CLK_LD = 0x0;
    if (div_m > 15){
        return CSK_DRIVER_ERROR_PARAMETER;
    }
    IP_CMN_SYSCFG->REG_PERI_CLK_CFG6.bit.DIV_RGB_CLK_M = div_m-1;
    

    IP_CMN_SYSCFG->REG_PERI_CLK_CFG6.bit.DIV_RGB_CLK_LD = 0x1;
    return CSK_DRIVER_OK;
}
    

/**
  * @brief  Function to configure the RGB clock source
  * @param  src specifies which source that RGB can choose
  *         This parameter can be one of the following values:
  *         @arg 0: RGB using CORE24M to source clock
  *         @arg 1: RGB using SyspllPeri to source clock
  * @return CSK HAL status
  */
int32_t HAL_CRM_SetRgbClkSrc(clock_src_name_t src){
    if (src == CRM_IpSrcCORE24M){
        IP_CMN_SYSCFG->REG_PERI_CLK_CFG6.bit.SEL_RGB_CLK = 0;
    }if (src == CRM_IpSrcSyspllPeri){
        IP_CMN_SYSCFG->REG_PERI_CLK_CFG6.bit.SEL_RGB_CLK = 1;
    }
    return CSK_DRIVER_OK;
}

// Retrieves the RGB clock frequency.
// Returns 0 if RGB is disabled, otherwise calculates frequency based on source and dividers.
uint32_t CRM_GetRgbFreq(void){
    uint32_t freq = 0;
    uint32_t div_m = 1;
    if (!HAL_CRM_RgbClkIsEnabled()){
        return 0;
    }
    clock_src_name_t src = 0;
    HAL_CRM_GetRgbClkConfig(&src, &div_m);    
    freq = CRM_GetSrcFreq(src);

    return freq/div_m;
}

/**
  * @brief  Function to configure the VIC clock source
  * @param  src specifies which source that VIC can choose
  *         This parameter can be one of the following values:
  *         @arg 0: VIC using CORE24M to source clock
  *         @arg 1: VIC using SyspllAud to source clock
  * @return CSK HAL status
  */
int32_t HAL_CRM_SetVicClkSrc(clock_src_name_t src){
    if (src == CRM_IpSrcCORE24M){
        IP_CMN_SYSCFG->REG_PERI_CLK_CFG6.bit.SEL_VIC_CLK = 0;
    }if (src == CRM_IpSrcSyspllAud){
        IP_CMN_SYSCFG->REG_PERI_CLK_CFG6.bit.SEL_VIC_CLK = 1;
    }
    return CSK_DRIVER_OK;
}

// Retrieves the VIC clock frequency.
// Returns 0 if VIC is disabled, otherwise calculates frequency based on source and dividers.
uint32_t CRM_GetVicFreq(void){
    uint32_t freq = 0;
    if (!HAL_CRM_VicClkIsEnabled()){
        return 0;
    }
    clock_src_name_t src = 0;
    HAL_CRM_GetVicClkConfig(&src );    
    freq = CRM_GetSrcFreq(src);

    return freq;
}

/**
  * @brief  Function to configure the QSPI1 clock divider
  * @param div_n must less than or equal to 7
  * @param div_m must less than or equal to 15
  */
int32_t HAL_CRM_SetQspi1ClkDiv(
    uint32_t div_n, uint32_t div_m){
    IP_CMN_SYSCFG->REG_PERI_CLK_CFG6.bit.DIV_QSPI1_CLK_LD = 0x0;
    if (div_n > 7){
        return CSK_DRIVER_ERROR_PARAMETER;
    }
    IP_CMN_SYSCFG->REG_PERI_CLK_CFG6.bit.DIV_QSPI1_CLK_N = div_n;
    
    if (div_m > 15){
        return CSK_DRIVER_ERROR_PARAMETER;
    }
    IP_CMN_SYSCFG->REG_PERI_CLK_CFG6.bit.DIV_QSPI1_CLK_M = div_m-1;
    

    IP_CMN_SYSCFG->REG_PERI_CLK_CFG6.bit.DIV_QSPI1_CLK_LD = 0x1;
    return CSK_DRIVER_OK;
}
    

/**
  * @brief  Function to configure the QSPI1 clock source
  * @param  src specifies which source that QSPI1 can choose
  *         This parameter can be one of the following values:
  *         @arg 0: QSPI1 using CORE24M to source clock
  *         @arg 1: QSPI1 using SyspllPeri to source clock
  * @return CSK HAL status
  */
int32_t HAL_CRM_SetQspi1ClkSrc(clock_src_name_t src){
    if (src == CRM_IpSrcCORE24M){
        IP_CMN_SYSCFG->REG_PERI_CLK_CFG6.bit.SEL_QSPI1_CLK = 0;
    }if (src == CRM_IpSrcSyspllPeri){
        IP_CMN_SYSCFG->REG_PERI_CLK_CFG6.bit.SEL_QSPI1_CLK = 1;
    }
    return CSK_DRIVER_OK;
}

// Retrieves the QSPI1 clock frequency.
// Returns 0 if QSPI1 is disabled, otherwise calculates frequency based on source and dividers.
uint32_t CRM_GetQspi1Freq(void){
    uint32_t freq = 0;
    uint32_t div_n = 1;
    uint32_t div_m = 1;
    if (!HAL_CRM_Qspi1ClkIsEnabled()){
        return 0;
    }
    clock_src_name_t src = 0;
    HAL_CRM_GetQspi1ClkConfig(&src,&div_n, &div_m);    
    freq = CRM_GetSrcFreq(src);

    return freq*div_n/div_m;
}

/**
  * @brief  Function to configure the QSPI0 clock divider
  * @param div_n must less than or equal to 7
  * @param div_m must less than or equal to 15
  */
int32_t HAL_CRM_SetQspi0ClkDiv(
    uint32_t div_n, uint32_t div_m){
    IP_CMN_SYSCFG->REG_PERI_CLK_CFG6.bit.DIV_QSPI0_CLK_LD = 0x0;
    if (div_n > 7){
        return CSK_DRIVER_ERROR_PARAMETER;
    }
    IP_CMN_SYSCFG->REG_PERI_CLK_CFG6.bit.DIV_QSPI0_CLK_N = div_n;
    
    if (div_m > 15){
        return CSK_DRIVER_ERROR_PARAMETER;
    }
    IP_CMN_SYSCFG->REG_PERI_CLK_CFG6.bit.DIV_QSPI0_CLK_M = div_m-1;
    

    IP_CMN_SYSCFG->REG_PERI_CLK_CFG6.bit.DIV_QSPI0_CLK_LD = 0x1;
    return CSK_DRIVER_OK;
}
    

/**
  * @brief  Function to configure the QSPI0 clock source
  * @param  src specifies which source that QSPI0 can choose
  *         This parameter can be one of the following values:
  *         @arg 0: QSPI0 using CORE24M to source clock
  *         @arg 1: QSPI0 using SyspllPeri to source clock
  * @return CSK HAL status
  */
int32_t HAL_CRM_SetQspi0ClkSrc(clock_src_name_t src){
    if (src == CRM_IpSrcCORE24M){
        IP_CMN_SYSCFG->REG_PERI_CLK_CFG6.bit.SEL_QSPI0_CLK = 0;
    }if (src == CRM_IpSrcSyspllPeri){
        IP_CMN_SYSCFG->REG_PERI_CLK_CFG6.bit.SEL_QSPI0_CLK = 1;
    }
    return CSK_DRIVER_OK;
}

// Retrieves the QSPI0 clock frequency.
// Returns 0 if QSPI0 is disabled, otherwise calculates frequency based on source and dividers.
uint32_t CRM_GetQspi0Freq(void){
    uint32_t freq = 0;
    uint32_t div_n = 1;
    uint32_t div_m = 1;
    if (!HAL_CRM_Qspi0ClkIsEnabled()){
        return 0;
    }
    clock_src_name_t src = 0;
    HAL_CRM_GetQspi0ClkConfig(&src,&div_n, &div_m);    
    freq = CRM_GetSrcFreq(src);

    return freq*div_n/div_m;
}

/**
  * @brief  Function to configure the DAC clock source
  * @param  src specifies which source that DAC can choose
  *         This parameter can be one of the following values:
  *         @arg 0: DAC using CORE24M to source clock
  *         @arg 1: DAC using SyspllAud to source clock
  * @return CSK HAL status
  */
int32_t HAL_CRM_SetDacClkSrc(clock_src_name_t src){
    if (src == CRM_IpSrcCORE24M){
        IP_CMN_SYSCFG->REG_PERI_CLK_CFG7.bit.AUD_DAC_MCLK_SRC = 0;
    }if (src == CRM_IpSrcSyspllAud){
        IP_CMN_SYSCFG->REG_PERI_CLK_CFG7.bit.AUD_DAC_MCLK_SRC = 1;
    }
    return CSK_DRIVER_OK;
}

// Retrieves the DAC clock frequency.
// Returns 0 if DAC is disabled, otherwise calculates frequency based on source and dividers.
uint32_t CRM_GetDacFreq(void){
    uint32_t freq = 0;
    
    clock_src_name_t src = 0;
    HAL_CRM_GetDacClkConfig(&src );    
    freq = CRM_GetSrcFreq(src);

    return freq;
}

/**
  * @brief  Function to configure the ADC clock source
  * @param  src specifies which source that ADC can choose
  *         This parameter can be one of the following values:
  *         @arg 0: ADC using CORE24M to source clock
  *         @arg 1: ADC using SyspllAud to source clock
  * @return CSK HAL status
  */
int32_t HAL_CRM_SetAdcClkSrc(clock_src_name_t src){
    if (src == CRM_IpSrcCORE24M){
        IP_CMN_SYSCFG->REG_PERI_CLK_CFG7.bit.AUD_ADC_CLK_SRC = 0;
    }if (src == CRM_IpSrcSyspllAud){
        IP_CMN_SYSCFG->REG_PERI_CLK_CFG7.bit.AUD_ADC_CLK_SRC = 1;
    }
    return CSK_DRIVER_OK;
}

// Retrieves the ADC clock frequency.
// Returns 0 if ADC is disabled, otherwise calculates frequency based on source and dividers.
uint32_t CRM_GetAdcFreq(void){
    uint32_t freq = 0;
    
    clock_src_name_t src = 0;
    HAL_CRM_GetAdcClkConfig(&src );    
    freq = CRM_GetSrcFreq(src);

    return freq;
}

// Retrieves the APC clock frequency.
// Returns 0 if APC is disabled, otherwise calculates frequency based on source and dividers.
uint32_t CRM_GetApcFreq(void){
    uint32_t freq = 0;
    if (!HAL_CRM_ApcClkIsEnabled()){
        return 0;
    }
    
        
    freq = CRM_GetCmn_pclkFreq();

    return freq;
}

// Retrieves the CODEC clock frequency.
// Returns 0 if CODEC is disabled, otherwise calculates frequency based on source and dividers.
uint32_t CRM_GetCodecFreq(void){
    uint32_t freq = 0;
    if (!HAL_CRM_CodecClkIsEnabled()){
        return 0;
    }
    
        
    freq = CRM_GetCmn_pclkFreq();

    return freq;
}

// Retrieves the JPEG clock frequency.
// Returns 0 if JPEG is disabled, otherwise calculates frequency based on source and dividers.
uint32_t CRM_GetJpegFreq(void){
    uint32_t freq = 0;
    if (!HAL_CRM_JpegClkIsEnabled()){
        return 0;
    }
    
        
    freq = CRM_GetHclkFreq();

    return freq;
}

// Retrieves the LUNA clock frequency.
// Returns 0 if LUNA is disabled, otherwise calculates frequency based on source and dividers.
uint32_t CRM_GetLunaFreq(void){
    uint32_t freq = 0;
    if (!HAL_CRM_LunaClkIsEnabled()){
        return 0;
    }
    
        
    freq = CRM_GetHclkFreq();

    return freq;
}

// Retrieves the GPIO1 clock frequency.
// Returns 0 if GPIO1 is disabled, otherwise calculates frequency based on source and dividers.
uint32_t CRM_GetGpio1Freq(void){
    uint32_t freq = 0;
    if (!HAL_CRM_Gpio1ClkIsEnabled()){
        return 0;
    }
    
        
    freq = CRM_GetCmn_pclkFreq();

    return freq;
}

// Retrieves the GPIO0 clock frequency.
// Returns 0 if GPIO0 is disabled, otherwise calculates frequency based on source and dividers.
uint32_t CRM_GetGpio0Freq(void){
    uint32_t freq = 0;
    if (!HAL_CRM_Gpio0ClkIsEnabled()){
        return 0;
    }
    
        
    freq = CRM_GetCmn_pclkFreq();

    return freq;
}

// Retrieves the GPDMA clock frequency.
// Returns 0 if GPDMA is disabled, otherwise calculates frequency based on source and dividers.
uint32_t CRM_GetGpdmaFreq(void){
    uint32_t freq = 0;
    if (!HAL_CRM_GpdmaClkIsEnabled()){
        return 0;
    }
    
        
    freq = CRM_GetHclkFreq();

    return freq;
}

// Retrieves the CMNDMA clock frequency.
// Returns 0 if CMNDMA is disabled, otherwise calculates frequency based on source and dividers.
uint32_t CRM_GetCmndmaFreq(void){
    uint32_t freq = 0;
    if (!HAL_CRM_CmndmaClkIsEnabled()){
        return 0;
    }
    
        
    freq = CRM_GetHclkFreq();

    return freq;
}

// Retrieves the USB clock frequency.
// Returns 0 if USB is disabled, otherwise calculates frequency based on source and dividers.
uint32_t CRM_GetUsbFreq(void){
    uint32_t freq = 0;
    if (!HAL_CRM_UsbClkIsEnabled()){
        return 0;
    }
    
        
    freq = CRM_GetHclkFreq();

    return freq;
}

// Retrieves the I2C1 clock frequency.
// Returns 0 if I2C1 is disabled, otherwise calculates frequency based on source and dividers.
uint32_t CRM_GetI2c1Freq(void){
    uint32_t freq = 0;
    if (!HAL_CRM_I2c1ClkIsEnabled()){
        return 0;
    }
    
        
    freq = CRM_GetCmn_pclkFreq();

    return freq;
}

// Retrieves the I2C0 clock frequency.
// Returns 0 if I2C0 is disabled, otherwise calculates frequency based on source and dividers.
uint32_t CRM_GetI2c0Freq(void){
    uint32_t freq = 0;
    if (!HAL_CRM_I2c0ClkIsEnabled()){
        return 0;
    }
    
        
    freq = CRM_GetCmn_pclkFreq();

    return freq;
}

#endif /* INCLUDE_CLOCKMANAGER_C_ */