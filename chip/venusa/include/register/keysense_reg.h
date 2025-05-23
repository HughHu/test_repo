//  ----------------------------------------------------------------------
//  File Name   : GenCFold/keysense_reg_venusa.h
//  Description : C header file generated by Python script.
//  Author      : dlchang
//  Script Ver  : LS.AUTO_REG.2024.12.12
//  SVN Revision: Can't find <<VenusA_SoC_Memory_Mapping.xlsx>> SVN detail info,pls chk����
//  Create Time : 2025-04-18 09:52:08
//  Comments    : 
//  ----------------------------------------------------------------------

#ifndef __KEYSENSE_REGFILE_H__
#define __KEYSENSE_REGFILE_H__

#include <stdint.h>

#define KEYSENSE_KS_CFG_OFFSET                      0x000
#define KEYSENSE_KS_CFG_KS_DBG_EN_Pos               2
#define KEYSENSE_KS_CFG_KS_DBG_EN_Msk               0x4
#define KEYSENSE_KS_CFG_KS_DBG_SEL_Pos              1
#define KEYSENSE_KS_CFG_KS_DBG_SEL_Msk              0x2
#define KEYSENSE_KS_CFG_KS_EN_Pos                   0
#define KEYSENSE_KS_CFG_KS_EN_Msk                   0x1

#define KEYSENSE_KS_STAT_OFFSET                     0x004
#define KEYSENSE_KS_STAT_KS_CNT_Pos                 16
#define KEYSENSE_KS_STAT_KS_CNT_Msk                 0xffff0000
#define KEYSENSE_KS_STAT_KS_ADC_TRIG_INTR_TRG_Pos    5
#define KEYSENSE_KS_STAT_KS_ADC_TRIG_INTR_TRG_Msk    0x20
#define KEYSENSE_KS_STAT_KS_ADC_TRIG_Pos            4
#define KEYSENSE_KS_STAT_KS_ADC_TRIG_Msk            0x10
#define KEYSENSE_KS_STAT_KS_WAKEUP_INTR_TRG_Pos     3
#define KEYSENSE_KS_STAT_KS_WAKEUP_INTR_TRG_Msk     0x8
#define KEYSENSE_KS_STAT_KS_WAKEUP_Pos              2
#define KEYSENSE_KS_STAT_KS_WAKEUP_Msk              0x4
#define KEYSENSE_KS_STAT_KEYIN_DETECT_SYNC_Pos      1
#define KEYSENSE_KS_STAT_KEYIN_DETECT_SYNC_Msk      0x2
#define KEYSENSE_KS_STAT_KEYIN_DETECT_Pos           0
#define KEYSENSE_KS_STAT_KEYIN_DETECT_Msk           0x1

#define KEYSENSE_KS_THD_OFFSET                      0x008
#define KEYSENSE_KS_THD_KS_THD_ADC_TRIG_Pos         16
#define KEYSENSE_KS_THD_KS_THD_ADC_TRIG_Msk         0xffff0000
#define KEYSENSE_KS_THD_KS_THD_WAKEUP_Pos           0
#define KEYSENSE_KS_THD_KS_THD_WAKEUP_Msk           0xffff

#define KEYSENSE_KS_IMR_OFFSET                      0x00C
#define KEYSENSE_KS_IMR_KS_PRESS_IMR_Pos            3
#define KEYSENSE_KS_IMR_KS_PRESS_IMR_Msk            0x8
#define KEYSENSE_KS_IMR_KS_RELEASE_IMR_Pos          2
#define KEYSENSE_KS_IMR_KS_RELEASE_IMR_Msk          0x4
#define KEYSENSE_KS_IMR_KS_ADC_TRIG_IMR_Pos         1
#define KEYSENSE_KS_IMR_KS_ADC_TRIG_IMR_Msk         0x2
#define KEYSENSE_KS_IMR_KS_WAKEUP_IMR_Pos           0
#define KEYSENSE_KS_IMR_KS_WAKEUP_IMR_Msk           0x1

#define KEYSENSE_KS_ICR_OFFSET                      0x010
#define KEYSENSE_KS_ICR_KS_PRESS_ICR_Pos            3
#define KEYSENSE_KS_ICR_KS_PRESS_ICR_Msk            0x8
#define KEYSENSE_KS_ICR_KS_RELEASE_ICR_Pos          2
#define KEYSENSE_KS_ICR_KS_RELEASE_ICR_Msk          0x4
#define KEYSENSE_KS_ICR_KS_ADC_TRIG_ICR_Pos         1
#define KEYSENSE_KS_ICR_KS_ADC_TRIG_ICR_Msk         0x2
#define KEYSENSE_KS_ICR_KS_WAKEUP_ICR_Pos           0
#define KEYSENSE_KS_ICR_KS_WAKEUP_ICR_Msk           0x1

#define KEYSENSE_KS_IRSR_OFFSET                     0x014
#define KEYSENSE_KS_IRSR_KS_PRESS_IRSR_Pos          3
#define KEYSENSE_KS_IRSR_KS_PRESS_IRSR_Msk          0x8
#define KEYSENSE_KS_IRSR_KS_RELEASE_IRSR_Pos        2
#define KEYSENSE_KS_IRSR_KS_RELEASE_IRSR_Msk        0x4
#define KEYSENSE_KS_IRSR_KS_ADC_TRIG_IRSR_Pos       1
#define KEYSENSE_KS_IRSR_KS_ADC_TRIG_IRSR_Msk       0x2
#define KEYSENSE_KS_IRSR_KS_WAKEUP_IRSR_Pos         0
#define KEYSENSE_KS_IRSR_KS_WAKEUP_IRSR_Msk         0x1

#define KEYSENSE_KS_ISR_OFFSET                      0x018
#define KEYSENSE_KS_ISR_KS_PRESS_ISR_Pos            3
#define KEYSENSE_KS_ISR_KS_PRESS_ISR_Msk            0x8
#define KEYSENSE_KS_ISR_KS_RELEASE_ISR_Pos          2
#define KEYSENSE_KS_ISR_KS_RELEASE_ISR_Msk          0x4
#define KEYSENSE_KS_ISR_KS_ADC_TRIG_ISR_Pos         1
#define KEYSENSE_KS_ISR_KS_ADC_TRIG_ISR_Msk         0x2
#define KEYSENSE_KS_ISR_KS_WAKEUP_ISR_Pos           0
#define KEYSENSE_KS_ISR_KS_WAKEUP_ISR_Msk           0x1

struct KEYSENSE_REG_KS_CFG_BITS
{
    volatile uint32_t KS_EN                         : 1; // bit 0~0
    volatile uint32_t KS_DBG_SEL                    : 1; // bit 1~1
    volatile uint32_t KS_DBG_EN                     : 1; // bit 2~2
    volatile uint32_t RESV_3_31                     : 29; // bit 3~31
};

union KEYSENSE_REG_KS_CFG {
    volatile uint32_t                               all;
    struct KEYSENSE_REG_KS_CFG_BITS                 bit;
};

struct KEYSENSE_REG_KS_STAT_BITS
{
    volatile uint32_t KEYIN_DETECT                  : 1; // bit 0~0
    volatile uint32_t KEYIN_DETECT_SYNC             : 1; // bit 1~1
    volatile uint32_t KS_WAKEUP                     : 1; // bit 2~2
    volatile uint32_t KS_WAKEUP_INTR_TRG            : 1; // bit 3~3
    volatile uint32_t KS_ADC_TRIG                   : 1; // bit 4~4
    volatile uint32_t KS_ADC_TRIG_INTR_TRG          : 1; // bit 5~5
    volatile uint32_t RESV_6_15                     : 10; // bit 6~15
    volatile uint32_t KS_CNT                        : 16; // bit 16~31
};

union KEYSENSE_REG_KS_STAT {
    volatile uint32_t                               all;
    struct KEYSENSE_REG_KS_STAT_BITS                bit;
};

struct KEYSENSE_REG_KS_THD_BITS
{
    volatile uint32_t KS_THD_WAKEUP                 : 16; // bit 0~15
    volatile uint32_t KS_THD_ADC_TRIG               : 16; // bit 16~31
};

union KEYSENSE_REG_KS_THD {
    volatile uint32_t                               all;
    struct KEYSENSE_REG_KS_THD_BITS                 bit;
};

struct KEYSENSE_REG_KS_IMR_BITS
{
    volatile uint32_t KS_WAKEUP_IMR                 : 1; // bit 0~0
    volatile uint32_t KS_ADC_TRIG_IMR               : 1; // bit 1~1
    volatile uint32_t KS_RELEASE_IMR                : 1; // bit 2~2
    volatile uint32_t KS_PRESS_IMR                  : 1; // bit 3~3
    volatile uint32_t RESV_4_31                     : 28; // bit 4~31
};

union KEYSENSE_REG_KS_IMR {
    volatile uint32_t                               all;
    struct KEYSENSE_REG_KS_IMR_BITS                 bit;
};

struct KEYSENSE_REG_KS_ICR_BITS
{
    volatile uint32_t KS_WAKEUP_ICR                 : 1; // bit 0~0
    volatile uint32_t KS_ADC_TRIG_ICR               : 1; // bit 1~1
    volatile uint32_t KS_RELEASE_ICR                : 1; // bit 2~2
    volatile uint32_t KS_PRESS_ICR                  : 1; // bit 3~3
    volatile uint32_t RESV_4_31                     : 28; // bit 4~31
};

union KEYSENSE_REG_KS_ICR {
    volatile uint32_t                               all;
    struct KEYSENSE_REG_KS_ICR_BITS                 bit;
};

struct KEYSENSE_REG_KS_IRSR_BITS
{
    volatile uint32_t KS_WAKEUP_IRSR                : 1; // bit 0~0
    volatile uint32_t KS_ADC_TRIG_IRSR              : 1; // bit 1~1
    volatile uint32_t KS_RELEASE_IRSR               : 1; // bit 2~2
    volatile uint32_t KS_PRESS_IRSR                 : 1; // bit 3~3
    volatile uint32_t RESV_4_31                     : 28; // bit 4~31
};

union KEYSENSE_REG_KS_IRSR {
    volatile uint32_t                               all;
    struct KEYSENSE_REG_KS_IRSR_BITS                bit;
};

struct KEYSENSE_REG_KS_ISR_BITS
{
    volatile uint32_t KS_WAKEUP_ISR                 : 1; // bit 0~0
    volatile uint32_t KS_ADC_TRIG_ISR               : 1; // bit 1~1
    volatile uint32_t KS_RELEASE_ISR                : 1; // bit 2~2
    volatile uint32_t KS_PRESS_ISR                  : 1; // bit 3~3
    volatile uint32_t RESV_4_31                     : 28; // bit 4~31
};

union KEYSENSE_REG_KS_ISR {
    volatile uint32_t                               all;
    struct KEYSENSE_REG_KS_ISR_BITS                 bit;
};

typedef struct
{
    union KEYSENSE_REG_KS_CFG                       REG_KS_CFG;  // 0x000
    union KEYSENSE_REG_KS_STAT                      REG_KS_STAT; // 0x004
    union KEYSENSE_REG_KS_THD                       REG_KS_THD;  // 0x008
    union KEYSENSE_REG_KS_IMR                       REG_KS_IMR;  // 0x00C
    union KEYSENSE_REG_KS_ICR                       REG_KS_ICR;  // 0x010
    union KEYSENSE_REG_KS_IRSR                      REG_KS_IRSR; // 0x014
    union KEYSENSE_REG_KS_ISR                       REG_KS_ISR;  // 0x018
} KEYSENSE_RegDef;


#endif // __KEYSENSE_REGFILE_H__

