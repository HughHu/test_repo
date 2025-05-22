#ifndef INCLUDE_CLOCK_CONFIG_H_
#define INCLUDE_CLOCK_CONFIG_H_

#define BOARD_BOOTCLOCKRUN_RC032K_CLK                       32768UL
#define BOARD_BOOTCLOCKRUN_XTAL32K_CLK                      32768UL
#define BOARD_BOOTCLOCKRUN_RC024M_CLK                       24000000UL
#define BOARD_BOOTCLOCKRUN_XTAL24M_CLK                      24000000UL
#define BOARD_BOOTCLOCKRUN_SYSPLL_CLK                       1200000000UL

#define BOARD_BOOTCLOCKRUN_SRC_CORE_CLK_DEF                 1
#define BOARD_BOOTCLOCKRUN_SRC_CORE_CFG_PARA                CRM_IpSyspllCore_Div35

#define BOARD_BOOTCLOCKRUN_SRC_PERI_CLK_DEF                 1
#define BOARD_BOOTCLOCKRUN_SRC_PERI_CFG_PARA                CRM_IpSyspllPeri_Div6

#define BOARD_BOOTCLOCKRUN_SRC_FLASH_CLK_DEF                1
#define BOARD_BOOTCLOCKRUN_SRC_FLASH_CFG_PARA               CRM_IpSyspllFlash_Div12

#define BOARD_BOOTCLOCKRUN_SRC_PSRAM_CLK_DEF                1
#define BOARD_BOOTCLOCKRUN_SRC_PSRAM_CFG_PARA               CRM_IpSyspllPsram_Div6

#define BOARD_BOOTCLOCKRUN_SRC_SDIO_CLK_DEF                 1
#define BOARD_BOOTCLOCKRUN_SRC_SDIO_CFG_PARA                CRM_IpSyspllSdio_Div6

#if BOARD_BOOTCLOCKRUN_SYSCLK_CLK_DEF
    // SYSCLK

    HAL_CRM_SetSysclkClkSrc(BOARD_BOOTCLOCKRUN_SYSCLK_CLK_SRC);

#endif

#if BOARD_BOOTCLOCKRUN_HCLK_CLK_DEF
    // HCLK

    HAL_CRM_SetHclkClkDiv(BOARD_BOOTCLOCKRUN_HCLK_CLK_N, BOARD_BOOTCLOCKRUN_HCLK_CLK_M);

#endif

#if BOARD_BOOTCLOCKRUN_CMN_PCLK_CLK_DEF
    // CMN_PCLK

    HAL_CRM_SetCmn_pclkClkDiv(BOARD_BOOTCLOCKRUN_CMN_PCLK_CLK_N, BOARD_BOOTCLOCKRUN_CMN_PCLK_CLK_M);

#endif

#if BOARD_BOOTCLOCKRUN_AON_CFG_PCLK_CLK_DEF
    // AON_CFG_PCLK

    HAL_CRM_SetAon_cfg_pclkClkDiv(BOARD_BOOTCLOCKRUN_AON_CFG_PCLK_CLK_N, BOARD_BOOTCLOCKRUN_AON_CFG_PCLK_CLK_M);

#endif

// PSRAM default configure
#define BOARD_BOOTCLOCKRUN_PSRAM_CLK_DEF                    1

#define BOARD_BOOTCLOCKRUN_PSRAM_CLK_SRC                    CRM_IpSrcCORE24M // Clock source configure, you can choice ->  CRM_IpSrcCORE24M,  CRM_IpSrcSyspllPsram, 

#define BOARD_BOOTCLOCKRUN_PSRAM_CLK_M_MAX                  1023 // Clock divider_m
#define BOARD_BOOTCLOCKRUN_PSRAM_CLK_M                      1
#if (BOARD_BOOTCLOCKRUN_PSRAM_CLK_M > BOARD_BOOTCLOCKRUN_PSRAM_CLK_M_MAX) || (BOARD_BOOTCLOCKRUN_PSRAM_CLK_M == 0)
#error "PSRAM divider m configure error"
#endif

// MTIME default configure
#define BOARD_BOOTCLOCKRUN_MTIME_CLK_DEF                    1

// Cloce is only generate from -----------> [CORE24M]

#define BOARD_BOOTCLOCKRUN_MTIME_CLK_M_MAX                  63 // Clock divider_m
#define BOARD_BOOTCLOCKRUN_MTIME_CLK_M                      48
#if (BOARD_BOOTCLOCKRUN_MTIME_CLK_M > BOARD_BOOTCLOCKRUN_MTIME_CLK_M_MAX) || (BOARD_BOOTCLOCKRUN_MTIME_CLK_M == 0)
#error "MTIME divider m configure error"
#endif

// FLASH default configure
#define BOARD_BOOTCLOCKRUN_FLASH_CLK_DEF                    1

#define BOARD_BOOTCLOCKRUN_FLASH_CLK_SRC                    CRM_IpSrcCORE24M // Clock source configure, you can choice ->  CRM_IpSrcCORE24M,  CRM_IpSrcSyspllFlash, 

#define BOARD_BOOTCLOCKRUN_FLASH_CLK_M_MAX                  31 // Clock divider_m
#define BOARD_BOOTCLOCKRUN_FLASH_CLK_M                      1
#if (BOARD_BOOTCLOCKRUN_FLASH_CLK_M > BOARD_BOOTCLOCKRUN_FLASH_CLK_M_MAX) || (BOARD_BOOTCLOCKRUN_FLASH_CLK_M == 0)
#error "FLASH divider m configure error"
#endif

// SPI0 default configure
#define BOARD_BOOTCLOCKRUN_SPI0_CLK_DEF                     1

#define BOARD_BOOTCLOCKRUN_SPI0_CLK_SRC                     CRM_IpSrcCORE24M // Clock source configure, you can choice ->  CRM_IpSrcCORE24M,  CRM_IpSrcSyspllPeri, 

#define BOARD_BOOTCLOCKRUN_SPI0_CLK_N_MAX                   7 // Clock divider_n
#define BOARD_BOOTCLOCKRUN_SPI0_CLK_N                       1
#if (BOARD_BOOTCLOCKRUN_SPI0_CLK_N > BOARD_BOOTCLOCKRUN_SPI0_CLK_N_MAX) || (BOARD_BOOTCLOCKRUN_SPI0_CLK_N == 0)
#error "SPI0 divider n configure error"
#endif

#define BOARD_BOOTCLOCKRUN_SPI0_CLK_M_MAX                   15 // Clock divider_m
#define BOARD_BOOTCLOCKRUN_SPI0_CLK_M                       1
#if (BOARD_BOOTCLOCKRUN_SPI0_CLK_M > BOARD_BOOTCLOCKRUN_SPI0_CLK_M_MAX) || (BOARD_BOOTCLOCKRUN_SPI0_CLK_M == 0)
#error "SPI0 divider m configure error"
#endif

// UART0 default configure
#define BOARD_BOOTCLOCKRUN_UART0_CLK_DEF                    1

#define BOARD_BOOTCLOCKRUN_UART0_CLK_SRC                    CRM_IpSrcCORE24M // Clock source configure, you can choice ->  CRM_IpSrcCORE24M,  CRM_IpSrcSyspllPeri, 

#define BOARD_BOOTCLOCKRUN_UART0_CLK_N_MAX                  511 // Clock divider_n
#define BOARD_BOOTCLOCKRUN_UART0_CLK_N                      1
#if (BOARD_BOOTCLOCKRUN_UART0_CLK_N > BOARD_BOOTCLOCKRUN_UART0_CLK_N_MAX) || (BOARD_BOOTCLOCKRUN_UART0_CLK_N == 0)
#error "UART0 divider n configure error"
#endif

#define BOARD_BOOTCLOCKRUN_UART0_CLK_M_MAX                  1023 // Clock divider_m
#define BOARD_BOOTCLOCKRUN_UART0_CLK_M                      1
#if (BOARD_BOOTCLOCKRUN_UART0_CLK_M > BOARD_BOOTCLOCKRUN_UART0_CLK_M_MAX) || (BOARD_BOOTCLOCKRUN_UART0_CLK_M == 0)
#error "UART0 divider m configure error"
#endif

// SPI1 default configure
#define BOARD_BOOTCLOCKRUN_SPI1_CLK_DEF                     1

#define BOARD_BOOTCLOCKRUN_SPI1_CLK_SRC                     CRM_IpSrcCORE24M // Clock source configure, you can choice ->  CRM_IpSrcCORE24M,  CRM_IpSrcSyspllPeri, 

#define BOARD_BOOTCLOCKRUN_SPI1_CLK_N_MAX                   7 // Clock divider_n
#define BOARD_BOOTCLOCKRUN_SPI1_CLK_N                       1
#if (BOARD_BOOTCLOCKRUN_SPI1_CLK_N > BOARD_BOOTCLOCKRUN_SPI1_CLK_N_MAX) || (BOARD_BOOTCLOCKRUN_SPI1_CLK_N == 0)
#error "SPI1 divider n configure error"
#endif

#define BOARD_BOOTCLOCKRUN_SPI1_CLK_M_MAX                   15 // Clock divider_m
#define BOARD_BOOTCLOCKRUN_SPI1_CLK_M                       1
#if (BOARD_BOOTCLOCKRUN_SPI1_CLK_M > BOARD_BOOTCLOCKRUN_SPI1_CLK_M_MAX) || (BOARD_BOOTCLOCKRUN_SPI1_CLK_M == 0)
#error "SPI1 divider m configure error"
#endif

// UART1 default configure
#define BOARD_BOOTCLOCKRUN_UART1_CLK_DEF                    1

#define BOARD_BOOTCLOCKRUN_UART1_CLK_SRC                    CRM_IpSrcCORE24M // Clock source configure, you can choice ->  CRM_IpSrcCORE24M,  CRM_IpSrcSyspllPeri, 

#define BOARD_BOOTCLOCKRUN_UART1_CLK_N_MAX                  511 // Clock divider_n
#define BOARD_BOOTCLOCKRUN_UART1_CLK_N                      1
#if (BOARD_BOOTCLOCKRUN_UART1_CLK_N > BOARD_BOOTCLOCKRUN_UART1_CLK_N_MAX) || (BOARD_BOOTCLOCKRUN_UART1_CLK_N == 0)
#error "UART1 divider n configure error"
#endif

#define BOARD_BOOTCLOCKRUN_UART1_CLK_M_MAX                  1023 // Clock divider_m
#define BOARD_BOOTCLOCKRUN_UART1_CLK_M                      1
#if (BOARD_BOOTCLOCKRUN_UART1_CLK_M > BOARD_BOOTCLOCKRUN_UART1_CLK_M_MAX) || (BOARD_BOOTCLOCKRUN_UART1_CLK_M == 0)
#error "UART1 divider m configure error"
#endif

// SDIOH default configure
#define BOARD_BOOTCLOCKRUN_SDIOH_CLK_DEF                    1

#define BOARD_BOOTCLOCKRUN_SDIOH_CLK_SRC                    CRM_IpSrcCORE24M // Clock source configure, you can choice ->  CRM_IpSrcCORE24M,  CRM_IpSrcSyspllSdio, 

#define BOARD_BOOTCLOCKRUN_SDIOH_CLK_N_MAX                  7 // Clock divider_n
#define BOARD_BOOTCLOCKRUN_SDIOH_CLK_N                      1
#if (BOARD_BOOTCLOCKRUN_SDIOH_CLK_N > BOARD_BOOTCLOCKRUN_SDIOH_CLK_N_MAX) || (BOARD_BOOTCLOCKRUN_SDIOH_CLK_N == 0)
#error "SDIOH divider n configure error"
#endif

#define BOARD_BOOTCLOCKRUN_SDIOH_CLK_M_MAX                  15 // Clock divider_m
#define BOARD_BOOTCLOCKRUN_SDIOH_CLK_M                      1
#if (BOARD_BOOTCLOCKRUN_SDIOH_CLK_M > BOARD_BOOTCLOCKRUN_SDIOH_CLK_M_MAX) || (BOARD_BOOTCLOCKRUN_SDIOH_CLK_M == 0)
#error "SDIOH divider m configure error"
#endif

// UART2 default configure
#define BOARD_BOOTCLOCKRUN_UART2_CLK_DEF                    1

#define BOARD_BOOTCLOCKRUN_UART2_CLK_SRC                    CRM_IpSrcCORE24M // Clock source configure, you can choice ->  CRM_IpSrcCORE24M,  CRM_IpSrcSyspllPeri, 

#define BOARD_BOOTCLOCKRUN_UART2_CLK_N_MAX                  511 // Clock divider_n
#define BOARD_BOOTCLOCKRUN_UART2_CLK_N                      1
#if (BOARD_BOOTCLOCKRUN_UART2_CLK_N > BOARD_BOOTCLOCKRUN_UART2_CLK_N_MAX) || (BOARD_BOOTCLOCKRUN_UART2_CLK_N == 0)
#error "UART2 divider n configure error"
#endif

#define BOARD_BOOTCLOCKRUN_UART2_CLK_M_MAX                  1023 // Clock divider_m
#define BOARD_BOOTCLOCKRUN_UART2_CLK_M                      1
#if (BOARD_BOOTCLOCKRUN_UART2_CLK_M > BOARD_BOOTCLOCKRUN_UART2_CLK_M_MAX) || (BOARD_BOOTCLOCKRUN_UART2_CLK_M == 0)
#error "UART2 divider m configure error"
#endif

// VIC_OUT default configure
#define BOARD_BOOTCLOCKRUN_VIC_OUT_CLK_DEF                  1

#define BOARD_BOOTCLOCKRUN_VIC_OUT_CLK_SRC                  CRM_IpSrcCORE24M // Clock source configure, you can choice ->  CRM_IpSrcCORE24M,  CRM_IpSrcSyspllPeri, 

#define BOARD_BOOTCLOCKRUN_VIC_OUT_CLK_M_MAX                511 // Clock divider_m
#define BOARD_BOOTCLOCKRUN_VIC_OUT_CLK_M                    1
#if (BOARD_BOOTCLOCKRUN_VIC_OUT_CLK_M > BOARD_BOOTCLOCKRUN_VIC_OUT_CLK_M_MAX) || (BOARD_BOOTCLOCKRUN_VIC_OUT_CLK_M == 0)
#error "VIC_OUT divider m configure error"
#endif

// GPT default configure
#define BOARD_BOOTCLOCKRUN_GPT_CLK_DEF                      1

// Cloce is only generate from -----------> [CORE24M]

#define BOARD_BOOTCLOCKRUN_GPT_CLK_M_MAX                    31 // Clock divider_m
#define BOARD_BOOTCLOCKRUN_GPT_CLK_M                        1
#if (BOARD_BOOTCLOCKRUN_GPT_CLK_M > BOARD_BOOTCLOCKRUN_GPT_CLK_M_MAX) || (BOARD_BOOTCLOCKRUN_GPT_CLK_M == 0)
#error "GPT divider m configure error"
#endif

// I8080 default configure
#define BOARD_BOOTCLOCKRUN_I8080_CLK_DEF                    1

#define BOARD_BOOTCLOCKRUN_I8080_CLK_SRC                    CRM_IpSrcCORE24M // Clock source configure, you can choice ->  CRM_IpSrcCORE24M,  CRM_IpSrcSyspllPeri, 

#define BOARD_BOOTCLOCKRUN_I8080_CLK_M_MAX                  15 // Clock divider_m
#define BOARD_BOOTCLOCKRUN_I8080_CLK_M                      1
#if (BOARD_BOOTCLOCKRUN_I8080_CLK_M > BOARD_BOOTCLOCKRUN_I8080_CLK_M_MAX) || (BOARD_BOOTCLOCKRUN_I8080_CLK_M == 0)
#error "I8080 divider m configure error"
#endif

// IR default configure
#define BOARD_BOOTCLOCKRUN_IR_CLK_DEF                       1

// Cloce is only generate from -----------> [CORE24M]

#define BOARD_BOOTCLOCKRUN_IR_CLK_M_MAX                     63 // Clock divider_m
#define BOARD_BOOTCLOCKRUN_IR_CLK_M                         1
#if (BOARD_BOOTCLOCKRUN_IR_CLK_M > BOARD_BOOTCLOCKRUN_IR_CLK_M_MAX) || (BOARD_BOOTCLOCKRUN_IR_CLK_M == 0)
#error "IR divider m configure error"
#endif

// GPADC default configure
#define BOARD_BOOTCLOCKRUN_GPADC_CLK_DEF                    1

// Cloce is only generate from -----------> [CORE24M]

#define BOARD_BOOTCLOCKRUN_GPADC_CLK_M_MAX                  1023 // Clock divider_m
#define BOARD_BOOTCLOCKRUN_GPADC_CLK_M                      1
#if (BOARD_BOOTCLOCKRUN_GPADC_CLK_M > BOARD_BOOTCLOCKRUN_GPADC_CLK_M_MAX) || (BOARD_BOOTCLOCKRUN_GPADC_CLK_M == 0)
#error "GPADC divider m configure error"
#endif

// I2S0 default configure
#define BOARD_BOOTCLOCKRUN_I2S0_CLK_DEF                     1

#define BOARD_BOOTCLOCKRUN_I2S0_CLK_SRC                     CRM_IpSrcCORE24M // Clock source configure, you can choice ->  CRM_IpSrcCORE24M,  CRM_IpSrcSyspllAud, 

// I2S1 default configure
#define BOARD_BOOTCLOCKRUN_I2S1_CLK_DEF                     1

#define BOARD_BOOTCLOCKRUN_I2S1_CLK_SRC                     CRM_IpSrcCORE24M // Clock source configure, you can choice ->  CRM_IpSrcCORE24M,  CRM_IpSrcSyspllAud, 

// RGB default configure
#define BOARD_BOOTCLOCKRUN_RGB_CLK_DEF                      1

#define BOARD_BOOTCLOCKRUN_RGB_CLK_SRC                      CRM_IpSrcCORE24M // Clock source configure, you can choice ->  CRM_IpSrcCORE24M,  CRM_IpSrcSyspllPeri, 

#define BOARD_BOOTCLOCKRUN_RGB_CLK_M_MAX                    15 // Clock divider_m
#define BOARD_BOOTCLOCKRUN_RGB_CLK_M                        1
#if (BOARD_BOOTCLOCKRUN_RGB_CLK_M > BOARD_BOOTCLOCKRUN_RGB_CLK_M_MAX) || (BOARD_BOOTCLOCKRUN_RGB_CLK_M == 0)
#error "RGB divider m configure error"
#endif

// VIC default configure
#define BOARD_BOOTCLOCKRUN_VIC_CLK_DEF                      1

#define BOARD_BOOTCLOCKRUN_VIC_CLK_SRC                      CRM_IpSrcCORE24M // Clock source configure, you can choice ->  CRM_IpSrcCORE24M,  CRM_IpSrcSyspllAud, 

// QSPI1 default configure
#define BOARD_BOOTCLOCKRUN_QSPI1_CLK_DEF                    1

#define BOARD_BOOTCLOCKRUN_QSPI1_CLK_SRC                    CRM_IpSrcCORE24M // Clock source configure, you can choice ->  CRM_IpSrcCORE24M,  CRM_IpSrcSyspllPeri, 

#define BOARD_BOOTCLOCKRUN_QSPI1_CLK_N_MAX                  7 // Clock divider_n
#define BOARD_BOOTCLOCKRUN_QSPI1_CLK_N                      1
#if (BOARD_BOOTCLOCKRUN_QSPI1_CLK_N > BOARD_BOOTCLOCKRUN_QSPI1_CLK_N_MAX) || (BOARD_BOOTCLOCKRUN_QSPI1_CLK_N == 0)
#error "QSPI1 divider n configure error"
#endif

#define BOARD_BOOTCLOCKRUN_QSPI1_CLK_M_MAX                  15 // Clock divider_m
#define BOARD_BOOTCLOCKRUN_QSPI1_CLK_M                      1
#if (BOARD_BOOTCLOCKRUN_QSPI1_CLK_M > BOARD_BOOTCLOCKRUN_QSPI1_CLK_M_MAX) || (BOARD_BOOTCLOCKRUN_QSPI1_CLK_M == 0)
#error "QSPI1 divider m configure error"
#endif

// QSPI0 default configure
#define BOARD_BOOTCLOCKRUN_QSPI0_CLK_DEF                    1

#define BOARD_BOOTCLOCKRUN_QSPI0_CLK_SRC                    CRM_IpSrcCORE24M // Clock source configure, you can choice ->  CRM_IpSrcCORE24M,  CRM_IpSrcSyspllPeri, 

#define BOARD_BOOTCLOCKRUN_QSPI0_CLK_N_MAX                  7 // Clock divider_n
#define BOARD_BOOTCLOCKRUN_QSPI0_CLK_N                      1
#if (BOARD_BOOTCLOCKRUN_QSPI0_CLK_N > BOARD_BOOTCLOCKRUN_QSPI0_CLK_N_MAX) || (BOARD_BOOTCLOCKRUN_QSPI0_CLK_N == 0)
#error "QSPI0 divider n configure error"
#endif

#define BOARD_BOOTCLOCKRUN_QSPI0_CLK_M_MAX                  15 // Clock divider_m
#define BOARD_BOOTCLOCKRUN_QSPI0_CLK_M                      1
#if (BOARD_BOOTCLOCKRUN_QSPI0_CLK_M > BOARD_BOOTCLOCKRUN_QSPI0_CLK_M_MAX) || (BOARD_BOOTCLOCKRUN_QSPI0_CLK_M == 0)
#error "QSPI0 divider m configure error"
#endif

// DAC default configure
#define BOARD_BOOTCLOCKRUN_DAC_CLK_DEF                      1

#define BOARD_BOOTCLOCKRUN_DAC_CLK_SRC                      CRM_IpSrcCORE24M // Clock source configure, you can choice ->  CRM_IpSrcCORE24M,  CRM_IpSrcSyspllAud, 

// ADC default configure
#define BOARD_BOOTCLOCKRUN_ADC_CLK_DEF                      1

#define BOARD_BOOTCLOCKRUN_ADC_CLK_SRC                      CRM_IpSrcCORE24M // Clock source configure, you can choice ->  CRM_IpSrcCORE24M,  CRM_IpSrcSyspllAud, 

#endif /* INCLUDE_CLOCK_CONFIG_H_ */