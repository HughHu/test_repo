//  ----------------------------------------------------------------------
//  File Name   : GenCFold/gpadc_reg_venusa.h
//  Description : C header file generated by Python script.
//  Author      : dlchang
//  Script Ver  : LS.AUTO_REG.2024.12.12
//  SVN Revision: Can't find <<VenusA_SoC_Memory_Mapping.xlsx>> SVN detail info,pls chk����
//  Create Time : 2025-04-18 09:52:08
//  Comments    : 
//  ----------------------------------------------------------------------

#ifndef __GPADC_REGFILE_H__
#define __GPADC_REGFILE_H__

#include <stdint.h>

#define GPADC_ADC_CTRL0_OFFSET                      0x000
#define GPADC_ADC_CTRL0_ADC_TRIG_NUM_Pos            24
#define GPADC_ADC_CTRL0_ADC_TRIG_NUM_Msk            0xff000000
#define GPADC_ADC_CTRL0_ADC_SETUP_WAIT_Pos          16
#define GPADC_ADC_CTRL0_ADC_SETUP_WAIT_Msk          0xff0000
#define GPADC_ADC_CTRL0_SAMPLE_TIME_Pos             8
#define GPADC_ADC_CTRL0_SAMPLE_TIME_Msk             0x7f00
#define GPADC_ADC_CTRL0_TSENSOR_COEF_Pos            3
#define GPADC_ADC_CTRL0_TSENSOR_COEF_Msk            0xf8
#define GPADC_ADC_CTRL0_KS_TRIG_EN_Pos              2
#define GPADC_ADC_CTRL0_KS_TRIG_EN_Msk              0x4
#define GPADC_ADC_CTRL0_GPT_TRIG_EN_Pos             1
#define GPADC_ADC_CTRL0_GPT_TRIG_EN_Msk             0x2
#define GPADC_ADC_CTRL0_SOFT_TRIG_Pos               0
#define GPADC_ADC_CTRL0_SOFT_TRIG_Msk               0x1

#define GPADC_ADC_CTRL1_OFFSET                      0x004
#define GPADC_ADC_CTRL1_FORCE_VIN_SEL_Pos           16
#define GPADC_ADC_CTRL1_FORCE_VIN_SEL_Msk           0x3f0000
#define GPADC_ADC_CTRL1_ADC_BUSY_Pos                7
#define GPADC_ADC_CTRL1_ADC_BUSY_Msk                0x80
#define GPADC_ADC_CTRL1_DEBUG_SEL_Pos               5
#define GPADC_ADC_CTRL1_DEBUG_SEL_Msk               0x60
#define GPADC_ADC_CTRL1_CALIBR_OPT_Pos              4
#define GPADC_ADC_CTRL1_CALIBR_OPT_Msk              0x10
#define GPADC_ADC_CTRL1_DMA_CH_NUM_V_Pos            3
#define GPADC_ADC_CTRL1_DMA_CH_NUM_V_Msk            0x8
#define GPADC_ADC_CTRL1_DATA_CNT_SEL_Pos            2
#define GPADC_ADC_CTRL1_DATA_CNT_SEL_Msk            0x4
#define GPADC_ADC_CTRL1_HARD_TRIG_Pos               1
#define GPADC_ADC_CTRL1_HARD_TRIG_Msk               0x2
#define GPADC_ADC_CTRL1_ADC_EN_Pos                  0
#define GPADC_ADC_CTRL1_ADC_EN_Msk                  0x1

#define GPADC_ADC_CONFIG_OFFSET                     0x008
#define GPADC_ADC_CONFIG_EXT_TRIG_EN_Pos            14
#define GPADC_ADC_CONFIG_EXT_TRIG_EN_Msk            0x4000
#define GPADC_ADC_CONFIG_LDO_EN_Pos                 13
#define GPADC_ADC_CONFIG_LDO_EN_Msk                 0x2000
#define GPADC_ADC_CONFIG_LDO_TUNE_Pos               10
#define GPADC_ADC_CONFIG_LDO_TUNE_Msk               0x1c00
#define GPADC_ADC_CONFIG_ANA_TEST_EN_Pos            8
#define GPADC_ADC_CONFIG_ANA_TEST_EN_Msk            0x300
#define GPADC_ADC_CONFIG_DIG_TEST_EN_Pos            7
#define GPADC_ADC_CONFIG_DIG_TEST_EN_Msk            0x80
#define GPADC_ADC_CONFIG_CLK_MODE_Pos               6
#define GPADC_ADC_CONFIG_CLK_MODE_Msk               0x40
#define GPADC_ADC_CONFIG_VREF_SEL_Pos               4
#define GPADC_ADC_CONFIG_VREF_SEL_Msk               0x30
#define GPADC_ADC_CONFIG_VIN_BUF_EN_FORCE_Pos       3
#define GPADC_ADC_CONFIG_VIN_BUF_EN_FORCE_Msk       0x8
#define GPADC_ADC_CONFIG_VIN_BUF_EN_Pos             2
#define GPADC_ADC_CONFIG_VIN_BUF_EN_Msk             0x4
#define GPADC_ADC_CONFIG_COMP_SAMP_MODE_Pos         0
#define GPADC_ADC_CONFIG_COMP_SAMP_MODE_Msk         0x3

#define GPADC_ADC_CH_SEL_OFFSET                     0x00C
#define GPADC_ADC_CH_SEL_DMA_CH_EN_Pos              16
#define GPADC_ADC_CH_SEL_DMA_CH_EN_Msk              0x3f0000
#define GPADC_ADC_CH_SEL_ADC_CH_SEL_Pos             0
#define GPADC_ADC_CH_SEL_ADC_CH_SEL_Msk             0x3f

#define GPADC_ADC_DEBUG_BUS_OFFSET                  0x028
#define GPADC_ADC_DEBUG_BUS_DEBUG_BUS_Pos           0
#define GPADC_ADC_DEBUG_BUS_DEBUG_BUS_Msk           0xffff

#define GPADC_ADC_GAIN_A_B_OFFSET                   0x040
#define GPADC_ADC_GAIN_A_B_CAL_GAIN_B_Pos           16
#define GPADC_ADC_GAIN_A_B_CAL_GAIN_B_Msk           0x1ff0000
#define GPADC_ADC_GAIN_A_B_CAL_GAIN_A_Pos           0
#define GPADC_ADC_GAIN_A_B_CAL_GAIN_A_Msk           0x1ff

#define GPADC_ADC_GAIN_C_D_OFFSET                   0x044
#define GPADC_ADC_GAIN_C_D_CAL_GAIN_D_Pos           16
#define GPADC_ADC_GAIN_C_D_CAL_GAIN_D_Msk           0x1ff0000
#define GPADC_ADC_GAIN_C_D_CAL_GAIN_C_Pos           0
#define GPADC_ADC_GAIN_C_D_CAL_GAIN_C_Msk           0x1ff

#define GPADC_ADC_OFFSET_A_OFFSET                   0x048
#define GPADC_ADC_OFFSET_A_CAL_OFFSET_A1_Pos        16
#define GPADC_ADC_OFFSET_A_CAL_OFFSET_A1_Msk        0xff0000
#define GPADC_ADC_OFFSET_A_CAL_OFFSET_A0_Pos        0
#define GPADC_ADC_OFFSET_A_CAL_OFFSET_A0_Msk        0xff

#define GPADC_ADC_OFFSET_B_OFFSET                   0x04C
#define GPADC_ADC_OFFSET_B_CAL_OFFSET_B1_Pos        16
#define GPADC_ADC_OFFSET_B_CAL_OFFSET_B1_Msk        0xff0000
#define GPADC_ADC_OFFSET_B_CAL_OFFSET_B0_Pos        0
#define GPADC_ADC_OFFSET_B_CAL_OFFSET_B0_Msk        0xff

#define GPADC_ADC_OFFSET_C_OFFSET                   0x050
#define GPADC_ADC_OFFSET_C_CAL_OFFSET_C1_Pos        16
#define GPADC_ADC_OFFSET_C_CAL_OFFSET_C1_Msk        0xff0000
#define GPADC_ADC_OFFSET_C_CAL_OFFSET_C0_Pos        0
#define GPADC_ADC_OFFSET_C_CAL_OFFSET_C0_Msk        0xff

#define GPADC_ADC_OFFSET_D_OFFSET                   0x054
#define GPADC_ADC_OFFSET_D_CAL_OFFSET_D_Pos         0
#define GPADC_ADC_OFFSET_D_CAL_OFFSET_D_Msk         0xff

#define GPADC_ADC_IMR0_OFFSET                       0x060
#define GPADC_ADC_IMR0_FIFO_EMPTY_IMR_CH_Pos        16
#define GPADC_ADC_IMR0_FIFO_EMPTY_IMR_CH_Msk        0x3f0000
#define GPADC_ADC_IMR0_ADC_COMPLETE_IMR_Pos         3
#define GPADC_ADC_IMR0_ADC_COMPLETE_IMR_Msk         0x8
#define GPADC_ADC_IMR0_EOC_ERR_IMR_Pos              2
#define GPADC_ADC_IMR0_EOC_ERR_IMR_Msk              0x4
#define GPADC_ADC_IMR0_ADC_READY_IMR_Pos            1
#define GPADC_ADC_IMR0_ADC_READY_IMR_Msk            0x2
#define GPADC_ADC_IMR0_CHANNEL_ERR_IMR_Pos          0
#define GPADC_ADC_IMR0_CHANNEL_ERR_IMR_Msk          0x1

#define GPADC_ADC_IMR1_OFFSET                       0x064
#define GPADC_ADC_IMR1_FIFO_THD_IMR_CH_Pos          16
#define GPADC_ADC_IMR1_FIFO_THD_IMR_CH_Msk          0x3f0000
#define GPADC_ADC_IMR1_FIFO_FULL_IMR_CH_Pos         0
#define GPADC_ADC_IMR1_FIFO_FULL_IMR_CH_Msk         0x3f

#define GPADC_ADC_IMR2_OFFSET                       0x068
#define GPADC_ADC_IMR2_FIFO_UNDERFLOW_IMR_CH_Pos    16
#define GPADC_ADC_IMR2_FIFO_UNDERFLOW_IMR_CH_Msk    0x3f0000
#define GPADC_ADC_IMR2_FIFO_OVERFLOW_IMR_CH_Pos     0
#define GPADC_ADC_IMR2_FIFO_OVERFLOW_IMR_CH_Msk     0x3f

#define GPADC_ADC_IRSR0_OFFSET                      0x078
#define GPADC_ADC_IRSR0_FIFO_EMPTY_IRSR_CH_Pos      16
#define GPADC_ADC_IRSR0_FIFO_EMPTY_IRSR_CH_Msk      0x3f0000
#define GPADC_ADC_IRSR0_ADC_COMPLETE_IRSR_Pos       3
#define GPADC_ADC_IRSR0_ADC_COMPLETE_IRSR_Msk       0x8
#define GPADC_ADC_IRSR0_EOC_ERR_IRSR_Pos            2
#define GPADC_ADC_IRSR0_EOC_ERR_IRSR_Msk            0x4
#define GPADC_ADC_IRSR0_ADC_READY_IRSR_Pos          1
#define GPADC_ADC_IRSR0_ADC_READY_IRSR_Msk          0x2
#define GPADC_ADC_IRSR0_CHANNEL_ERR_IRSR_Pos        0
#define GPADC_ADC_IRSR0_CHANNEL_ERR_IRSR_Msk        0x1

#define GPADC_ADC_IRSR1_OFFSET                      0x07C
#define GPADC_ADC_IRSR1_FIFO_THD_IRSR_CH_Pos        16
#define GPADC_ADC_IRSR1_FIFO_THD_IRSR_CH_Msk        0x3f0000
#define GPADC_ADC_IRSR1_FIFO_FULL_IRSR_CH_Pos       0
#define GPADC_ADC_IRSR1_FIFO_FULL_IRSR_CH_Msk       0x3f

#define GPADC_ADC_IRSR2_OFFSET                      0x080
#define GPADC_ADC_IRSR2_FIFO_UNDERFLOW_IRSR_CH_Pos    16
#define GPADC_ADC_IRSR2_FIFO_UNDERFLOW_IRSR_CH_Msk    0x3f0000
#define GPADC_ADC_IRSR2_FIFO_OVERFLOW_IRSR_CH_Pos    0
#define GPADC_ADC_IRSR2_FIFO_OVERFLOW_IRSR_CH_Msk    0x3f

#define GPADC_ADC_ISR0_OFFSET                       0x084
#define GPADC_ADC_ISR0_FIFO_EMPTY_ISR_CH_Pos        16
#define GPADC_ADC_ISR0_FIFO_EMPTY_ISR_CH_Msk        0x3f0000
#define GPADC_ADC_ISR0_ADC_COMPLETE_ISR_Pos         3
#define GPADC_ADC_ISR0_ADC_COMPLETE_ISR_Msk         0x8
#define GPADC_ADC_ISR0_EOC_ERR_ISR_Pos              2
#define GPADC_ADC_ISR0_EOC_ERR_ISR_Msk              0x4
#define GPADC_ADC_ISR0_ADC_READY_ISR_Pos            1
#define GPADC_ADC_ISR0_ADC_READY_ISR_Msk            0x2
#define GPADC_ADC_ISR0_CHANNEL_ERR_ISR_Pos          0
#define GPADC_ADC_ISR0_CHANNEL_ERR_ISR_Msk          0x1

#define GPADC_ADC_ISR1_OFFSET                       0x088
#define GPADC_ADC_ISR1_FIFO_THD_ISR_CH_Pos          16
#define GPADC_ADC_ISR1_FIFO_THD_ISR_CH_Msk          0x3f0000
#define GPADC_ADC_ISR1_FIFO_FULL_ISR_CH_Pos         0
#define GPADC_ADC_ISR1_FIFO_FULL_ISR_CH_Msk         0x3f

#define GPADC_ADC_ISR2_OFFSET                       0x08C
#define GPADC_ADC_ISR2_FIFO_UNDERFLOW_ISR_CH_Pos    16
#define GPADC_ADC_ISR2_FIFO_UNDERFLOW_ISR_CH_Msk    0x3f0000
#define GPADC_ADC_ISR2_FIFO_OVERFLOW_ISR_CH_Pos     0
#define GPADC_ADC_ISR2_FIFO_OVERFLOW_ISR_CH_Msk     0x3f

#define GPADC_DMA_RDR_OFFSET                        0x090
#define GPADC_DMA_RDR_RDATA_DMA_Pos                 0
#define GPADC_DMA_RDR_RDATA_DMA_Msk                 0xffffffff

#define GPADC_ADC_FIFO_DATA_00_OFFSET               0x094
#define GPADC_ADC_FIFO_DATA_00_READ_DATA_CH00_Pos    0
#define GPADC_ADC_FIFO_DATA_00_READ_DATA_CH00_Msk    0x3ff

#define GPADC_ADC_FIFO_DATA_01_OFFSET               0x098
#define GPADC_ADC_FIFO_DATA_01_READ_DATA_CH01_Pos    0
#define GPADC_ADC_FIFO_DATA_01_READ_DATA_CH01_Msk    0x3ff

#define GPADC_ADC_FIFO_DATA_02_OFFSET               0x09C
#define GPADC_ADC_FIFO_DATA_02_READ_DATA_CH02_Pos    0
#define GPADC_ADC_FIFO_DATA_02_READ_DATA_CH02_Msk    0x3ff

#define GPADC_ADC_FIFO_DATA_03_OFFSET               0x0A0
#define GPADC_ADC_FIFO_DATA_03_READ_DATA_CH03_Pos    0
#define GPADC_ADC_FIFO_DATA_03_READ_DATA_CH03_Msk    0x3ff

#define GPADC_ADC_FIFO_DATA_04_OFFSET               0x0A4
#define GPADC_ADC_FIFO_DATA_04_READ_DATA_CH04_Pos    0
#define GPADC_ADC_FIFO_DATA_04_READ_DATA_CH04_Msk    0x3ff

#define GPADC_ADC_FIFO_DATA_05_OFFSET               0x0A8
#define GPADC_ADC_FIFO_DATA_05_READ_DATA_CH05_Pos    0
#define GPADC_ADC_FIFO_DATA_05_READ_DATA_CH05_Msk    0x3ff

#define GPADC_ADC_FIFO_CNTL_00_OFFSET               0x0D4
#define GPADC_ADC_FIFO_CNTL_00_RDATA_CNT_CH00_Pos    8
#define GPADC_ADC_FIFO_CNTL_00_RDATA_CNT_CH00_Msk    0x1f00
#define GPADC_ADC_FIFO_CNTL_00_FIFO_CH00_Pos        4
#define GPADC_ADC_FIFO_CNTL_00_FIFO_CH00_Msk        0x10
#define GPADC_ADC_FIFO_CNTL_00_FIFO_THD_CH00_Pos    0
#define GPADC_ADC_FIFO_CNTL_00_FIFO_THD_CH00_Msk    0xf

#define GPADC_ADC_FIFO_CNTL_01_OFFSET               0x0D8
#define GPADC_ADC_FIFO_CNTL_01_RDATA_CNT_CH01_Pos    8
#define GPADC_ADC_FIFO_CNTL_01_RDATA_CNT_CH01_Msk    0x1f00
#define GPADC_ADC_FIFO_CNTL_01_FIFO_CH01_Pos        4
#define GPADC_ADC_FIFO_CNTL_01_FIFO_CH01_Msk        0x10
#define GPADC_ADC_FIFO_CNTL_01_FIFO_THD_CH01_Pos    0
#define GPADC_ADC_FIFO_CNTL_01_FIFO_THD_CH01_Msk    0xf

#define GPADC_ADC_FIFO_CNTL_02_OFFSET               0x0DC
#define GPADC_ADC_FIFO_CNTL_02_RDATA_CNT_CH02_Pos    8
#define GPADC_ADC_FIFO_CNTL_02_RDATA_CNT_CH02_Msk    0x1f00
#define GPADC_ADC_FIFO_CNTL_02_FIFO_CH02_Pos        4
#define GPADC_ADC_FIFO_CNTL_02_FIFO_CH02_Msk        0x10
#define GPADC_ADC_FIFO_CNTL_02_FIFO_THD_CH02_Pos    0
#define GPADC_ADC_FIFO_CNTL_02_FIFO_THD_CH02_Msk    0xf

#define GPADC_ADC_FIFO_CNTL_03_OFFSET               0x0E0
#define GPADC_ADC_FIFO_CNTL_03_RDATA_CNT_CH03_Pos    8
#define GPADC_ADC_FIFO_CNTL_03_RDATA_CNT_CH03_Msk    0x1f00
#define GPADC_ADC_FIFO_CNTL_03_FIFO_CH03_Pos        4
#define GPADC_ADC_FIFO_CNTL_03_FIFO_CH03_Msk        0x10
#define GPADC_ADC_FIFO_CNTL_03_FIFO_THD_CH03_Pos    0
#define GPADC_ADC_FIFO_CNTL_03_FIFO_THD_CH03_Msk    0xf

#define GPADC_ADC_FIFO_CNTL_04_OFFSET               0x0E4
#define GPADC_ADC_FIFO_CNTL_04_RDATA_CNT_CH04_Pos    8
#define GPADC_ADC_FIFO_CNTL_04_RDATA_CNT_CH04_Msk    0x1f00
#define GPADC_ADC_FIFO_CNTL_04_FIFO_CH04_Pos        4
#define GPADC_ADC_FIFO_CNTL_04_FIFO_CH04_Msk        0x10
#define GPADC_ADC_FIFO_CNTL_04_FIFO_THD_CH04_Pos    0
#define GPADC_ADC_FIFO_CNTL_04_FIFO_THD_CH04_Msk    0xf

#define GPADC_ADC_FIFO_CNTL_05_OFFSET               0x0E8
#define GPADC_ADC_FIFO_CNTL_05_RDATA_CNT_CH05_Pos    8
#define GPADC_ADC_FIFO_CNTL_05_RDATA_CNT_CH05_Msk    0x1f00
#define GPADC_ADC_FIFO_CNTL_05_FIFO_CH05_Pos        4
#define GPADC_ADC_FIFO_CNTL_05_FIFO_CH05_Msk        0x10
#define GPADC_ADC_FIFO_CNTL_05_FIFO_THD_CH05_Pos    0
#define GPADC_ADC_FIFO_CNTL_05_FIFO_THD_CH05_Msk    0xf

struct GPADC_REG_ADC_CTRL0_BITS
{
    volatile uint32_t SOFT_TRIG                     : 1; // bit 0~0
    volatile uint32_t GPT_TRIG_EN                   : 1; // bit 1~1
    volatile uint32_t KS_TRIG_EN                    : 1; // bit 2~2
    volatile uint32_t TSENSOR_COEF                  : 5; // bit 3~7
    volatile uint32_t SAMPLE_TIME                   : 7; // bit 8~14
    volatile uint32_t RESV_15_15                    : 1; // bit 15~15
    volatile uint32_t ADC_SETUP_WAIT                : 8; // bit 16~23
    volatile uint32_t ADC_TRIG_NUM                  : 8; // bit 24~31
};

union GPADC_REG_ADC_CTRL0 {
    volatile uint32_t                               all;
    struct GPADC_REG_ADC_CTRL0_BITS                 bit;
};

struct GPADC_REG_ADC_CTRL1_BITS
{
    volatile uint32_t ADC_EN                        : 1; // bit 0~0
    volatile uint32_t HARD_TRIG                     : 1; // bit 1~1
    volatile uint32_t DATA_CNT_SEL                  : 1; // bit 2~2
    volatile uint32_t DMA_CH_NUM_V                  : 1; // bit 3~3
    volatile uint32_t CALIBR_OPT                    : 1; // bit 4~4
    volatile uint32_t DEBUG_SEL                     : 2; // bit 5~6
    volatile uint32_t ADC_BUSY                      : 1; // bit 7~7
    volatile uint32_t RESV_8_15                     : 8; // bit 8~15
    volatile uint32_t FORCE_VIN_SEL                 : 6; // bit 16~21
    volatile uint32_t RESV_22_31                    : 10; // bit 22~31
};

union GPADC_REG_ADC_CTRL1 {
    volatile uint32_t                               all;
    struct GPADC_REG_ADC_CTRL1_BITS                 bit;
};

struct GPADC_REG_ADC_CONFIG_BITS
{
    volatile uint32_t COMP_SAMP_MODE                : 2; // bit 0~1
    volatile uint32_t VIN_BUF_EN                    : 1; // bit 2~2
    volatile uint32_t VIN_BUF_EN_FORCE              : 1; // bit 3~3
    volatile uint32_t VREF_SEL                      : 2; // bit 4~5
    volatile uint32_t CLK_MODE                      : 1; // bit 6~6
    volatile uint32_t DIG_TEST_EN                   : 1; // bit 7~7
    volatile uint32_t ANA_TEST_EN                   : 2; // bit 8~9
    volatile uint32_t LDO_TUNE                      : 3; // bit 10~12
    volatile uint32_t LDO_EN                        : 1; // bit 13~13
    volatile uint32_t EXT_TRIG_EN                   : 1; // bit 14~14
    volatile uint32_t RESV_15_31                    : 17; // bit 15~31
};

union GPADC_REG_ADC_CONFIG {
    volatile uint32_t                               all;
    struct GPADC_REG_ADC_CONFIG_BITS                bit;
};

struct GPADC_REG_ADC_CH_SEL_BITS
{
    volatile uint32_t ADC_CH_SEL                    : 6; // bit 0~5
    volatile uint32_t RESV_6_15                     : 10; // bit 6~15
    volatile uint32_t DMA_CH_EN                     : 6; // bit 16~21
    volatile uint32_t RESV_22_31                    : 10; // bit 22~31
};

union GPADC_REG_ADC_CH_SEL {
    volatile uint32_t                               all;
    struct GPADC_REG_ADC_CH_SEL_BITS                bit;
};

struct GPADC_REG_ADC_DEBUG_BUS_BITS
{
    volatile uint32_t DEBUG_BUS                     : 16; // bit 0~15
    volatile uint32_t RESV_16_31                    : 16; // bit 16~31
};

union GPADC_REG_ADC_DEBUG_BUS {
    volatile uint32_t                               all;
    struct GPADC_REG_ADC_DEBUG_BUS_BITS             bit;
};

struct GPADC_REG_ADC_GAIN_A_B_BITS
{
    volatile uint32_t CAL_GAIN_A                    : 9; // bit 0~8
    volatile uint32_t RESV_9_15                     : 7; // bit 9~15
    volatile uint32_t CAL_GAIN_B                    : 9; // bit 16~24
    volatile uint32_t RESV_25_31                    : 7; // bit 25~31
};

union GPADC_REG_ADC_GAIN_A_B {
    volatile uint32_t                               all;
    struct GPADC_REG_ADC_GAIN_A_B_BITS              bit;
};

struct GPADC_REG_ADC_GAIN_C_D_BITS
{
    volatile uint32_t CAL_GAIN_C                    : 9; // bit 0~8
    volatile uint32_t RESV_9_15                     : 7; // bit 9~15
    volatile uint32_t CAL_GAIN_D                    : 9; // bit 16~24
    volatile uint32_t RESV_25_31                    : 7; // bit 25~31
};

union GPADC_REG_ADC_GAIN_C_D {
    volatile uint32_t                               all;
    struct GPADC_REG_ADC_GAIN_C_D_BITS              bit;
};

struct GPADC_REG_ADC_OFFSET_A_BITS
{
    volatile uint32_t CAL_OFFSET_A0                 : 8; // bit 0~7
    volatile uint32_t RESV_8_15                     : 8; // bit 8~15
    volatile uint32_t CAL_OFFSET_A1                 : 8; // bit 16~23
    volatile uint32_t RESV_24_31                    : 8; // bit 24~31
};

union GPADC_REG_ADC_OFFSET_A {
    volatile uint32_t                               all;
    struct GPADC_REG_ADC_OFFSET_A_BITS              bit;
};

struct GPADC_REG_ADC_OFFSET_B_BITS
{
    volatile uint32_t CAL_OFFSET_B0                 : 8; // bit 0~7
    volatile uint32_t RESV_8_15                     : 8; // bit 8~15
    volatile uint32_t CAL_OFFSET_B1                 : 8; // bit 16~23
    volatile uint32_t RESV_24_31                    : 8; // bit 24~31
};

union GPADC_REG_ADC_OFFSET_B {
    volatile uint32_t                               all;
    struct GPADC_REG_ADC_OFFSET_B_BITS              bit;
};

struct GPADC_REG_ADC_OFFSET_C_BITS
{
    volatile uint32_t CAL_OFFSET_C0                 : 8; // bit 0~7
    volatile uint32_t RESV_8_15                     : 8; // bit 8~15
    volatile uint32_t CAL_OFFSET_C1                 : 8; // bit 16~23
    volatile uint32_t RESV_24_31                    : 8; // bit 24~31
};

union GPADC_REG_ADC_OFFSET_C {
    volatile uint32_t                               all;
    struct GPADC_REG_ADC_OFFSET_C_BITS              bit;
};

struct GPADC_REG_ADC_OFFSET_D_BITS
{
    volatile uint32_t CAL_OFFSET_D                  : 8; // bit 0~7
    volatile uint32_t RESV_8_31                     : 24; // bit 8~31
};

union GPADC_REG_ADC_OFFSET_D {
    volatile uint32_t                               all;
    struct GPADC_REG_ADC_OFFSET_D_BITS              bit;
};

struct GPADC_REG_ADC_IMR0_BITS
{
    volatile uint32_t CHANNEL_ERR_IMR               : 1; // bit 0~0
    volatile uint32_t ADC_READY_IMR                 : 1; // bit 1~1
    volatile uint32_t EOC_ERR_IMR                   : 1; // bit 2~2
    volatile uint32_t ADC_COMPLETE_IMR              : 1; // bit 3~3
    volatile uint32_t RESV_4_15                     : 12; // bit 4~15
    volatile uint32_t FIFO_EMPTY_IMR_CH             : 6; // bit 16~21
    volatile uint32_t RESV_22_31                    : 10; // bit 22~31
};

union GPADC_REG_ADC_IMR0 {
    volatile uint32_t                               all;
    struct GPADC_REG_ADC_IMR0_BITS                  bit;
};

struct GPADC_REG_ADC_IMR1_BITS
{
    volatile uint32_t FIFO_FULL_IMR_CH              : 6; // bit 0~5
    volatile uint32_t RESV_6_15                     : 10; // bit 6~15
    volatile uint32_t FIFO_THD_IMR_CH               : 6; // bit 16~21
    volatile uint32_t RESV_22_31                    : 10; // bit 22~31
};

union GPADC_REG_ADC_IMR1 {
    volatile uint32_t                               all;
    struct GPADC_REG_ADC_IMR1_BITS                  bit;
};

struct GPADC_REG_ADC_IMR2_BITS
{
    volatile uint32_t FIFO_OVERFLOW_IMR_CH          : 6; // bit 0~5
    volatile uint32_t RESV_6_15                     : 10; // bit 6~15
    volatile uint32_t FIFO_UNDERFLOW_IMR_CH         : 6; // bit 16~21
    volatile uint32_t RESV_22_31                    : 10; // bit 22~31
};

union GPADC_REG_ADC_IMR2 {
    volatile uint32_t                               all;
    struct GPADC_REG_ADC_IMR2_BITS                  bit;
};

struct GPADC_REG_ADC_IRSR0_BITS
{
    volatile uint32_t CHANNEL_ERR_IRSR              : 1; // bit 0~0
    volatile uint32_t ADC_READY_IRSR                : 1; // bit 1~1
    volatile uint32_t EOC_ERR_IRSR                  : 1; // bit 2~2
    volatile uint32_t ADC_COMPLETE_IRSR             : 1; // bit 3~3
    volatile uint32_t RESV_4_15                     : 12; // bit 4~15
    volatile uint32_t FIFO_EMPTY_IRSR_CH            : 6; // bit 16~21
    volatile uint32_t RESV_22_31                    : 10; // bit 22~31
};

union GPADC_REG_ADC_IRSR0 {
    volatile uint32_t                               all;
    struct GPADC_REG_ADC_IRSR0_BITS                 bit;
};

struct GPADC_REG_ADC_IRSR1_BITS
{
    volatile uint32_t FIFO_FULL_IRSR_CH             : 6; // bit 0~5
    volatile uint32_t RESV_6_15                     : 10; // bit 6~15
    volatile uint32_t FIFO_THD_IRSR_CH              : 6; // bit 16~21
    volatile uint32_t RESV_22_31                    : 10; // bit 22~31
};

union GPADC_REG_ADC_IRSR1 {
    volatile uint32_t                               all;
    struct GPADC_REG_ADC_IRSR1_BITS                 bit;
};

struct GPADC_REG_ADC_IRSR2_BITS
{
    volatile uint32_t FIFO_OVERFLOW_IRSR_CH         : 6; // bit 0~5
    volatile uint32_t RESV_6_15                     : 10; // bit 6~15
    volatile uint32_t FIFO_UNDERFLOW_IRSR_CH        : 6; // bit 16~21
    volatile uint32_t RESV_22_31                    : 10; // bit 22~31
};

union GPADC_REG_ADC_IRSR2 {
    volatile uint32_t                               all;
    struct GPADC_REG_ADC_IRSR2_BITS                 bit;
};

struct GPADC_REG_ADC_ISR0_BITS
{
    volatile uint32_t CHANNEL_ERR_ISR               : 1; // bit 0~0
    volatile uint32_t ADC_READY_ISR                 : 1; // bit 1~1
    volatile uint32_t EOC_ERR_ISR                   : 1; // bit 2~2
    volatile uint32_t ADC_COMPLETE_ISR              : 1; // bit 3~3
    volatile uint32_t RESV_4_15                     : 12; // bit 4~15
    volatile uint32_t FIFO_EMPTY_ISR_CH             : 6; // bit 16~21
    volatile uint32_t RESV_22_31                    : 10; // bit 22~31
};

union GPADC_REG_ADC_ISR0 {
    volatile uint32_t                               all;
    struct GPADC_REG_ADC_ISR0_BITS                  bit;
};

struct GPADC_REG_ADC_ISR1_BITS
{
    volatile uint32_t FIFO_FULL_ISR_CH              : 6; // bit 0~5
    volatile uint32_t RESV_6_15                     : 10; // bit 6~15
    volatile uint32_t FIFO_THD_ISR_CH               : 6; // bit 16~21
    volatile uint32_t RESV_22_31                    : 10; // bit 22~31
};

union GPADC_REG_ADC_ISR1 {
    volatile uint32_t                               all;
    struct GPADC_REG_ADC_ISR1_BITS                  bit;
};

struct GPADC_REG_ADC_ISR2_BITS
{
    volatile uint32_t FIFO_OVERFLOW_ISR_CH          : 6; // bit 0~5
    volatile uint32_t RESV_6_15                     : 10; // bit 6~15
    volatile uint32_t FIFO_UNDERFLOW_ISR_CH         : 6; // bit 16~21
    volatile uint32_t RESV_22_31                    : 10; // bit 22~31
};

union GPADC_REG_ADC_ISR2 {
    volatile uint32_t                               all;
    struct GPADC_REG_ADC_ISR2_BITS                  bit;
};

struct GPADC_REG_DMA_RDR_BITS
{
    volatile uint32_t RDATA_DMA                     : 32; // bit 0~31
};

union GPADC_REG_DMA_RDR {
    volatile uint32_t                               all;
    struct GPADC_REG_DMA_RDR_BITS                   bit;
};

struct GPADC_REG_ADC_FIFO_DATA_00_BITS
{
    volatile uint32_t READ_DATA_CH00                : 10; // bit 0~9
    volatile uint32_t RESV_10_31                    : 22; // bit 10~31
};

union GPADC_REG_ADC_FIFO_DATA_00 {
    volatile uint32_t                               all;
    struct GPADC_REG_ADC_FIFO_DATA_00_BITS          bit;
};

struct GPADC_REG_ADC_FIFO_DATA_01_BITS
{
    volatile uint32_t READ_DATA_CH01                : 10; // bit 0~9
    volatile uint32_t RESV_10_31                    : 22; // bit 10~31
};

union GPADC_REG_ADC_FIFO_DATA_01 {
    volatile uint32_t                               all;
    struct GPADC_REG_ADC_FIFO_DATA_01_BITS          bit;
};

struct GPADC_REG_ADC_FIFO_DATA_02_BITS
{
    volatile uint32_t READ_DATA_CH02                : 10; // bit 0~9
    volatile uint32_t RESV_10_31                    : 22; // bit 10~31
};

union GPADC_REG_ADC_FIFO_DATA_02 {
    volatile uint32_t                               all;
    struct GPADC_REG_ADC_FIFO_DATA_02_BITS          bit;
};

struct GPADC_REG_ADC_FIFO_DATA_03_BITS
{
    volatile uint32_t READ_DATA_CH03                : 10; // bit 0~9
    volatile uint32_t RESV_10_31                    : 22; // bit 10~31
};

union GPADC_REG_ADC_FIFO_DATA_03 {
    volatile uint32_t                               all;
    struct GPADC_REG_ADC_FIFO_DATA_03_BITS          bit;
};

struct GPADC_REG_ADC_FIFO_DATA_04_BITS
{
    volatile uint32_t READ_DATA_CH04                : 10; // bit 0~9
    volatile uint32_t RESV_10_31                    : 22; // bit 10~31
};

union GPADC_REG_ADC_FIFO_DATA_04 {
    volatile uint32_t                               all;
    struct GPADC_REG_ADC_FIFO_DATA_04_BITS          bit;
};

struct GPADC_REG_ADC_FIFO_DATA_05_BITS
{
    volatile uint32_t READ_DATA_CH05                : 10; // bit 0~9
    volatile uint32_t RESV_10_31                    : 22; // bit 10~31
};

union GPADC_REG_ADC_FIFO_DATA_05 {
    volatile uint32_t                               all;
    struct GPADC_REG_ADC_FIFO_DATA_05_BITS          bit;
};

struct GPADC_REG_ADC_FIFO_CNTL_00_BITS
{
    volatile uint32_t FIFO_THD_CH00                 : 4; // bit 0~3
    volatile uint32_t FIFO_CH00                     : 1; // bit 4~4
    volatile uint32_t RESV_5_7                      : 3; // bit 5~7
    volatile uint32_t RDATA_CNT_CH00                : 5; // bit 8~12
    volatile uint32_t RESV_13_31                    : 19; // bit 13~31
};

union GPADC_REG_ADC_FIFO_CNTL_00 {
    volatile uint32_t                               all;
    struct GPADC_REG_ADC_FIFO_CNTL_00_BITS          bit;
};

struct GPADC_REG_ADC_FIFO_CNTL_01_BITS
{
    volatile uint32_t FIFO_THD_CH01                 : 4; // bit 0~3
    volatile uint32_t FIFO_CH01                     : 1; // bit 4~4
    volatile uint32_t RESV_5_7                      : 3; // bit 5~7
    volatile uint32_t RDATA_CNT_CH01                : 5; // bit 8~12
    volatile uint32_t RESV_13_31                    : 19; // bit 13~31
};

union GPADC_REG_ADC_FIFO_CNTL_01 {
    volatile uint32_t                               all;
    struct GPADC_REG_ADC_FIFO_CNTL_01_BITS          bit;
};

struct GPADC_REG_ADC_FIFO_CNTL_02_BITS
{
    volatile uint32_t FIFO_THD_CH02                 : 4; // bit 0~3
    volatile uint32_t FIFO_CH02                     : 1; // bit 4~4
    volatile uint32_t RESV_5_7                      : 3; // bit 5~7
    volatile uint32_t RDATA_CNT_CH02                : 5; // bit 8~12
    volatile uint32_t RESV_13_31                    : 19; // bit 13~31
};

union GPADC_REG_ADC_FIFO_CNTL_02 {
    volatile uint32_t                               all;
    struct GPADC_REG_ADC_FIFO_CNTL_02_BITS          bit;
};

struct GPADC_REG_ADC_FIFO_CNTL_03_BITS
{
    volatile uint32_t FIFO_THD_CH03                 : 4; // bit 0~3
    volatile uint32_t FIFO_CH03                     : 1; // bit 4~4
    volatile uint32_t RESV_5_7                      : 3; // bit 5~7
    volatile uint32_t RDATA_CNT_CH03                : 5; // bit 8~12
    volatile uint32_t RESV_13_31                    : 19; // bit 13~31
};

union GPADC_REG_ADC_FIFO_CNTL_03 {
    volatile uint32_t                               all;
    struct GPADC_REG_ADC_FIFO_CNTL_03_BITS          bit;
};

struct GPADC_REG_ADC_FIFO_CNTL_04_BITS
{
    volatile uint32_t FIFO_THD_CH04                 : 4; // bit 0~3
    volatile uint32_t FIFO_CH04                     : 1; // bit 4~4
    volatile uint32_t RESV_5_7                      : 3; // bit 5~7
    volatile uint32_t RDATA_CNT_CH04                : 5; // bit 8~12
    volatile uint32_t RESV_13_31                    : 19; // bit 13~31
};

union GPADC_REG_ADC_FIFO_CNTL_04 {
    volatile uint32_t                               all;
    struct GPADC_REG_ADC_FIFO_CNTL_04_BITS          bit;
};

struct GPADC_REG_ADC_FIFO_CNTL_05_BITS
{
    volatile uint32_t FIFO_THD_CH05                 : 4; // bit 0~3
    volatile uint32_t FIFO_CH05                     : 1; // bit 4~4
    volatile uint32_t RESV_5_7                      : 3; // bit 5~7
    volatile uint32_t RDATA_CNT_CH05                : 5; // bit 8~12
    volatile uint32_t RESV_13_31                    : 19; // bit 13~31
};

union GPADC_REG_ADC_FIFO_CNTL_05 {
    volatile uint32_t                               all;
    struct GPADC_REG_ADC_FIFO_CNTL_05_BITS          bit;
};

typedef struct
{
    union GPADC_REG_ADC_CTRL0                       REG_ADC_CTRL0; // 0x000
    union GPADC_REG_ADC_CTRL1                       REG_ADC_CTRL1; // 0x004
    union GPADC_REG_ADC_CONFIG                      REG_ADC_CONFIG; // 0x008
    union GPADC_REG_ADC_CH_SEL                      REG_ADC_CH_SEL; // 0x00C
    volatile uint32_t                               REG_RESV_0X10_0X24[6];
    union GPADC_REG_ADC_DEBUG_BUS                   REG_ADC_DEBUG_BUS; // 0x028
    volatile uint32_t                               REG_RESV_0X2C_0X3C[5];
    union GPADC_REG_ADC_GAIN_A_B                    REG_ADC_GAIN_A_B; // 0x040
    union GPADC_REG_ADC_GAIN_C_D                    REG_ADC_GAIN_C_D; // 0x044
    union GPADC_REG_ADC_OFFSET_A                    REG_ADC_OFFSET_A; // 0x048
    union GPADC_REG_ADC_OFFSET_B                    REG_ADC_OFFSET_B; // 0x04C
    union GPADC_REG_ADC_OFFSET_C                    REG_ADC_OFFSET_C; // 0x050
    union GPADC_REG_ADC_OFFSET_D                    REG_ADC_OFFSET_D; // 0x054
    volatile uint32_t                               REG_RESV_0X58_0X5C[2];
    union GPADC_REG_ADC_IMR0                        REG_ADC_IMR0; // 0x060
    union GPADC_REG_ADC_IMR1                        REG_ADC_IMR1; // 0x064
    union GPADC_REG_ADC_IMR2                        REG_ADC_IMR2; // 0x068
    volatile uint32_t                               REG_RESV_0X6C_0X74[3];
    union GPADC_REG_ADC_IRSR0                       REG_ADC_IRSR0; // 0x078
    union GPADC_REG_ADC_IRSR1                       REG_ADC_IRSR1; // 0x07C
    union GPADC_REG_ADC_IRSR2                       REG_ADC_IRSR2; // 0x080
    union GPADC_REG_ADC_ISR0                        REG_ADC_ISR0; // 0x084
    union GPADC_REG_ADC_ISR1                        REG_ADC_ISR1; // 0x088
    union GPADC_REG_ADC_ISR2                        REG_ADC_ISR2; // 0x08C
    union GPADC_REG_DMA_RDR                         REG_DMA_RDR; // 0x090
    union GPADC_REG_ADC_FIFO_DATA_00                REG_ADC_FIFO_DATA_00; // 0x094
    union GPADC_REG_ADC_FIFO_DATA_01                REG_ADC_FIFO_DATA_01; // 0x098
    union GPADC_REG_ADC_FIFO_DATA_02                REG_ADC_FIFO_DATA_02; // 0x09C
    union GPADC_REG_ADC_FIFO_DATA_03                REG_ADC_FIFO_DATA_03; // 0x0A0
    union GPADC_REG_ADC_FIFO_DATA_04                REG_ADC_FIFO_DATA_04; // 0x0A4
    union GPADC_REG_ADC_FIFO_DATA_05                REG_ADC_FIFO_DATA_05; // 0x0A8
    volatile uint32_t                               REG_RESV_0XAC_0XD0[10];
    union GPADC_REG_ADC_FIFO_CNTL_00                REG_ADC_FIFO_CNTL_00; // 0x0D4
    union GPADC_REG_ADC_FIFO_CNTL_01                REG_ADC_FIFO_CNTL_01; // 0x0D8
    union GPADC_REG_ADC_FIFO_CNTL_02                REG_ADC_FIFO_CNTL_02; // 0x0DC
    union GPADC_REG_ADC_FIFO_CNTL_03                REG_ADC_FIFO_CNTL_03; // 0x0E0
    union GPADC_REG_ADC_FIFO_CNTL_04                REG_ADC_FIFO_CNTL_04; // 0x0E4
    union GPADC_REG_ADC_FIFO_CNTL_05                REG_ADC_FIFO_CNTL_05; // 0x0E8
} GPADC_RegDef;


#endif // __GPADC_REGFILE_H__

