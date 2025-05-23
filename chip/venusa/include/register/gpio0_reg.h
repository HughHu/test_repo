//  ----------------------------------------------------------------------
//  File Name   : GenCFold/gpio0_reg_venusa.h
//  Description : C header file generated by Python script.
//  Author      : dlchang
//  Script Ver  : LS.AUTO_REG.2024.12.12
//  SVN Revision: Can't find <<VenusA_SoC_Memory_Mapping.xlsx>> SVN detail info,pls chk����
//  Create Time : 2025-04-18 09:52:08
//  Comments    : 
//  ----------------------------------------------------------------------

#ifndef __GPIO0_REGFILE_H__
#define __GPIO0_REGFILE_H__

#include <stdint.h>

#define GPIO0_IDREV_OFFSET                          0x000
#define GPIO0_IDREV_ID_Pos                          12
#define GPIO0_IDREV_ID_Msk                          0xfffff000
#define GPIO0_IDREV_REVMAJOR_Pos                    4
#define GPIO0_IDREV_REVMAJOR_Msk                    0xff0
#define GPIO0_IDREV_REVMINOR_Pos                    0
#define GPIO0_IDREV_REVMINOR_Msk                    0xf

#define GPIO0_CFG_OFFSET                            0x010
#define GPIO0_CFG_PULL_Pos                          31
#define GPIO0_CFG_PULL_Msk                          0x80000000
#define GPIO0_CFG_INTR_Pos                          30
#define GPIO0_CFG_INTR_Msk                          0x40000000
#define GPIO0_CFG_DEBOUNCE_Pos                      29
#define GPIO0_CFG_DEBOUNCE_Msk                      0x20000000
#define GPIO0_CFG_CHANNELNUM_Pos                    0
#define GPIO0_CFG_CHANNELNUM_Msk                    0x3f

#define GPIO0_DATAIN_OFFSET                         0x020
#define GPIO0_DATAIN_DATAIN_Pos                     0
#define GPIO0_DATAIN_DATAIN_Msk                     0xffffffff

#define GPIO0_DATAOUT_OFFSET                        0x024
#define GPIO0_DATAOUT_DATAOUT_Pos                   0
#define GPIO0_DATAOUT_DATAOUT_Msk                   0xffffffff

#define GPIO0_CHANNELDIR_OFFSET                     0x028
#define GPIO0_CHANNELDIR_CHANNELDIR_Pos             0
#define GPIO0_CHANNELDIR_CHANNELDIR_Msk             0xffffffff

#define GPIO0_DOUTCLEAR_OFFSET                      0x02C
#define GPIO0_DOUTCLEAR_DOUTCLEAR_Pos               0
#define GPIO0_DOUTCLEAR_DOUTCLEAR_Msk               0xffffffff

#define GPIO0_DOUTSET_OFFSET                        0x030
#define GPIO0_DOUTSET_DOUTSET_Pos                   0
#define GPIO0_DOUTSET_DOUTSET_Msk                   0xffffffff

#define GPIO0_PULLEN_OFFSET                         0x040
#define GPIO0_PULLEN_PULLEN_Pos                     0
#define GPIO0_PULLEN_PULLEN_Msk                     0xffffffff

#define GPIO0_PULLTYPE_OFFSET                       0x044
#define GPIO0_PULLTYPE_PULLTYPE_Pos                 0
#define GPIO0_PULLTYPE_PULLTYPE_Msk                 0xffffffff

#define GPIO0_INTREN_OFFSET                         0x050
#define GPIO0_INTREN_INTEN_Pos                      0
#define GPIO0_INTREN_INTEN_Msk                      0xffffffff

#define GPIO0_INTRMODE0_OFFSET                      0x054
#define GPIO0_INTRMODE0_CH7INTRM_Pos                28
#define GPIO0_INTRMODE0_CH7INTRM_Msk                0x70000000
#define GPIO0_INTRMODE0_CH6INTRM_Pos                24
#define GPIO0_INTRMODE0_CH6INTRM_Msk                0x7000000
#define GPIO0_INTRMODE0_CH5INTRM_Pos                20
#define GPIO0_INTRMODE0_CH5INTRM_Msk                0x700000
#define GPIO0_INTRMODE0_CH4INTRM_Pos                16
#define GPIO0_INTRMODE0_CH4INTRM_Msk                0x70000
#define GPIO0_INTRMODE0_CH3INTRM_Pos                12
#define GPIO0_INTRMODE0_CH3INTRM_Msk                0x7000
#define GPIO0_INTRMODE0_CH2INTRM_Pos                8
#define GPIO0_INTRMODE0_CH2INTRM_Msk                0x700
#define GPIO0_INTRMODE0_CH1INTRM_Pos                4
#define GPIO0_INTRMODE0_CH1INTRM_Msk                0x70
#define GPIO0_INTRMODE0_CH0INTRM_Pos                0
#define GPIO0_INTRMODE0_CH0INTRM_Msk                0x7

#define GPIO0_INTRMODE1_OFFSET                      0x058
#define GPIO0_INTRMODE1_CH15INTRM_Pos               28
#define GPIO0_INTRMODE1_CH15INTRM_Msk               0x70000000
#define GPIO0_INTRMODE1_CH14INTRM_Pos               24
#define GPIO0_INTRMODE1_CH14INTRM_Msk               0x7000000
#define GPIO0_INTRMODE1_CH13INTRM_Pos               20
#define GPIO0_INTRMODE1_CH13INTRM_Msk               0x700000
#define GPIO0_INTRMODE1_CH12INTRM_Pos               16
#define GPIO0_INTRMODE1_CH12INTRM_Msk               0x70000
#define GPIO0_INTRMODE1_CH11INTRM_Pos               12
#define GPIO0_INTRMODE1_CH11INTRM_Msk               0x7000
#define GPIO0_INTRMODE1_CH10INTRM_Pos               8
#define GPIO0_INTRMODE1_CH10INTRM_Msk               0x700
#define GPIO0_INTRMODE1_CH9INTRM_Pos                4
#define GPIO0_INTRMODE1_CH9INTRM_Msk                0x70
#define GPIO0_INTRMODE1_CH8INTRM_Pos                0
#define GPIO0_INTRMODE1_CH8INTRM_Msk                0x7

#define GPIO0_INTRMODE2_OFFSET                      0x05C
#define GPIO0_INTRMODE2_CH23INTRM_Pos               28
#define GPIO0_INTRMODE2_CH23INTRM_Msk               0x70000000
#define GPIO0_INTRMODE2_CH22INTRM_Pos               24
#define GPIO0_INTRMODE2_CH22INTRM_Msk               0x7000000
#define GPIO0_INTRMODE2_CH21INTRM_Pos               20
#define GPIO0_INTRMODE2_CH21INTRM_Msk               0x700000
#define GPIO0_INTRMODE2_CH20INTRM_Pos               16
#define GPIO0_INTRMODE2_CH20INTRM_Msk               0x70000
#define GPIO0_INTRMODE2_CH19INTRM_Pos               12
#define GPIO0_INTRMODE2_CH19INTRM_Msk               0x7000
#define GPIO0_INTRMODE2_CH18INTRM_Pos               8
#define GPIO0_INTRMODE2_CH18INTRM_Msk               0x700
#define GPIO0_INTRMODE2_CH17INTRM_Pos               4
#define GPIO0_INTRMODE2_CH17INTRM_Msk               0x70
#define GPIO0_INTRMODE2_CH16INTRM_Pos               0
#define GPIO0_INTRMODE2_CH16INTRM_Msk               0x7

#define GPIO0_INTRMODE3_OFFSET                      0x060
#define GPIO0_INTRMODE3_CH31INTRM_Pos               28
#define GPIO0_INTRMODE3_CH31INTRM_Msk               0x70000000
#define GPIO0_INTRMODE3_CH30INTRM_Pos               24
#define GPIO0_INTRMODE3_CH30INTRM_Msk               0x7000000
#define GPIO0_INTRMODE3_CH29INTRM_Pos               20
#define GPIO0_INTRMODE3_CH29INTRM_Msk               0x700000
#define GPIO0_INTRMODE3_CH28INTRM_Pos               16
#define GPIO0_INTRMODE3_CH28INTRM_Msk               0x70000
#define GPIO0_INTRMODE3_CH27INTRM_Pos               12
#define GPIO0_INTRMODE3_CH27INTRM_Msk               0x7000
#define GPIO0_INTRMODE3_CH26INTRM_Pos               8
#define GPIO0_INTRMODE3_CH26INTRM_Msk               0x700
#define GPIO0_INTRMODE3_CH25INTRM_Pos               4
#define GPIO0_INTRMODE3_CH25INTRM_Msk               0x70
#define GPIO0_INTRMODE3_CH24INTRM_Pos               0
#define GPIO0_INTRMODE3_CH24INTRM_Msk               0x7

#define GPIO0_INTRSTATUS_OFFSET                     0x064
#define GPIO0_INTRSTATUS_INTRSTATUS_Pos             0
#define GPIO0_INTRSTATUS_INTRSTATUS_Msk             0xffffffff

#define GPIO0_DEBOUNCEEN_OFFSET                     0x070
#define GPIO0_DEBOUNCEEN_DEBOUNCEEN_Pos             0
#define GPIO0_DEBOUNCEEN_DEBOUNCEEN_Msk             0xffffffff

#define GPIO0_DEBOUNCECTRL_OFFSET                   0x074
#define GPIO0_DEBOUNCECTRL_DBCLKSEL_Pos             31
#define GPIO0_DEBOUNCECTRL_DBCLKSEL_Msk             0x80000000
#define GPIO0_DEBOUNCECTRL_DBPRESCALE_Pos           0
#define GPIO0_DEBOUNCECTRL_DBPRESCALE_Msk           0xff

struct GPIO0_REG_IDREV_BITS
{
    volatile uint32_t REVMINOR                      : 4; // bit 0~3
    volatile uint32_t REVMAJOR                      : 8; // bit 4~11
    volatile uint32_t ID                            : 20; // bit 12~31
};

union GPIO0_REG_IDREV {
    volatile uint32_t                               all;
    struct GPIO0_REG_IDREV_BITS                     bit;
};

struct GPIO0_REG_CFG_BITS
{
    volatile uint32_t CHANNELNUM                    : 6; // bit 0~5
    volatile uint32_t RESV_6_28                     : 23; // bit 6~28
    volatile uint32_t DEBOUNCE                      : 1; // bit 29~29
    volatile uint32_t INTR                          : 1; // bit 30~30
    volatile uint32_t PULL                          : 1; // bit 31~31
};

union GPIO0_REG_CFG {
    volatile uint32_t                               all;
    struct GPIO0_REG_CFG_BITS                       bit;
};

struct GPIO0_REG_DATAIN_BITS
{
    volatile uint32_t DATAIN                        : 32; // bit 0~31
};

union GPIO0_REG_DATAIN {
    volatile uint32_t                               all;
    struct GPIO0_REG_DATAIN_BITS                    bit;
};

struct GPIO0_REG_DATAOUT_BITS
{
    volatile uint32_t DATAOUT                       : 32; // bit 0~31
};

union GPIO0_REG_DATAOUT {
    volatile uint32_t                               all;
    struct GPIO0_REG_DATAOUT_BITS                   bit;
};

struct GPIO0_REG_CHANNELDIR_BITS
{
    volatile uint32_t CHANNELDIR                    : 32; // bit 0~31
};

union GPIO0_REG_CHANNELDIR {
    volatile uint32_t                               all;
    struct GPIO0_REG_CHANNELDIR_BITS                bit;
};

struct GPIO0_REG_DOUTCLEAR_BITS
{
    volatile uint32_t DOUTCLEAR                     : 32; // bit 0~31
};

union GPIO0_REG_DOUTCLEAR {
    volatile uint32_t                               all;
    struct GPIO0_REG_DOUTCLEAR_BITS                 bit;
};

struct GPIO0_REG_DOUTSET_BITS
{
    volatile uint32_t DOUTSET                       : 32; // bit 0~31
};

union GPIO0_REG_DOUTSET {
    volatile uint32_t                               all;
    struct GPIO0_REG_DOUTSET_BITS                   bit;
};

struct GPIO0_REG_PULLEN_BITS
{
    volatile uint32_t PULLEN                        : 32; // bit 0~31
};

union GPIO0_REG_PULLEN {
    volatile uint32_t                               all;
    struct GPIO0_REG_PULLEN_BITS                    bit;
};

struct GPIO0_REG_PULLTYPE_BITS
{
    volatile uint32_t PULLTYPE                      : 32; // bit 0~31
};

union GPIO0_REG_PULLTYPE {
    volatile uint32_t                               all;
    struct GPIO0_REG_PULLTYPE_BITS                  bit;
};

struct GPIO0_REG_INTREN_BITS
{
    volatile uint32_t INTEN                         : 32; // bit 0~31
};

union GPIO0_REG_INTREN {
    volatile uint32_t                               all;
    struct GPIO0_REG_INTREN_BITS                    bit;
};

struct GPIO0_REG_INTRMODE0_BITS
{
    volatile uint32_t CH0INTRM                      : 3; // bit 0~2
    volatile uint32_t RESV_3_3                      : 1; // bit 3~3
    volatile uint32_t CH1INTRM                      : 3; // bit 4~6
    volatile uint32_t RESV_7_7                      : 1; // bit 7~7
    volatile uint32_t CH2INTRM                      : 3; // bit 8~10
    volatile uint32_t RESV_11_11                    : 1; // bit 11~11
    volatile uint32_t CH3INTRM                      : 3; // bit 12~14
    volatile uint32_t RESV_15_15                    : 1; // bit 15~15
    volatile uint32_t CH4INTRM                      : 3; // bit 16~18
    volatile uint32_t RESV_19_19                    : 1; // bit 19~19
    volatile uint32_t CH5INTRM                      : 3; // bit 20~22
    volatile uint32_t RESV_23_23                    : 1; // bit 23~23
    volatile uint32_t CH6INTRM                      : 3; // bit 24~26
    volatile uint32_t RESV_27_27                    : 1; // bit 27~27
    volatile uint32_t CH7INTRM                      : 3; // bit 28~30
    volatile uint32_t RESV_31_31                    : 1; // bit 31~31
};

union GPIO0_REG_INTRMODE0 {
    volatile uint32_t                               all;
    struct GPIO0_REG_INTRMODE0_BITS                 bit;
};

struct GPIO0_REG_INTRMODE1_BITS
{
    volatile uint32_t CH8INTRM                      : 3; // bit 0~2
    volatile uint32_t RESV_3_3                      : 1; // bit 3~3
    volatile uint32_t CH9INTRM                      : 3; // bit 4~6
    volatile uint32_t RESV_7_7                      : 1; // bit 7~7
    volatile uint32_t CH10INTRM                     : 3; // bit 8~10
    volatile uint32_t RESV_11_11                    : 1; // bit 11~11
    volatile uint32_t CH11INTRM                     : 3; // bit 12~14
    volatile uint32_t RESV_15_15                    : 1; // bit 15~15
    volatile uint32_t CH12INTRM                     : 3; // bit 16~18
    volatile uint32_t RESV_19_19                    : 1; // bit 19~19
    volatile uint32_t CH13INTRM                     : 3; // bit 20~22
    volatile uint32_t RESV_23_23                    : 1; // bit 23~23
    volatile uint32_t CH14INTRM                     : 3; // bit 24~26
    volatile uint32_t RESV_27_27                    : 1; // bit 27~27
    volatile uint32_t CH15INTRM                     : 3; // bit 28~30
    volatile uint32_t RESV_31_31                    : 1; // bit 31~31
};

union GPIO0_REG_INTRMODE1 {
    volatile uint32_t                               all;
    struct GPIO0_REG_INTRMODE1_BITS                 bit;
};

struct GPIO0_REG_INTRMODE2_BITS
{
    volatile uint32_t CH16INTRM                     : 3; // bit 0~2
    volatile uint32_t RESV_3_3                      : 1; // bit 3~3
    volatile uint32_t CH17INTRM                     : 3; // bit 4~6
    volatile uint32_t RESV_7_7                      : 1; // bit 7~7
    volatile uint32_t CH18INTRM                     : 3; // bit 8~10
    volatile uint32_t RESV_11_11                    : 1; // bit 11~11
    volatile uint32_t CH19INTRM                     : 3; // bit 12~14
    volatile uint32_t RESV_15_15                    : 1; // bit 15~15
    volatile uint32_t CH20INTRM                     : 3; // bit 16~18
    volatile uint32_t RESV_19_19                    : 1; // bit 19~19
    volatile uint32_t CH21INTRM                     : 3; // bit 20~22
    volatile uint32_t RESV_23_23                    : 1; // bit 23~23
    volatile uint32_t CH22INTRM                     : 3; // bit 24~26
    volatile uint32_t RESV_27_27                    : 1; // bit 27~27
    volatile uint32_t CH23INTRM                     : 3; // bit 28~30
    volatile uint32_t RESV_31_31                    : 1; // bit 31~31
};

union GPIO0_REG_INTRMODE2 {
    volatile uint32_t                               all;
    struct GPIO0_REG_INTRMODE2_BITS                 bit;
};

struct GPIO0_REG_INTRMODE3_BITS
{
    volatile uint32_t CH24INTRM                     : 3; // bit 0~2
    volatile uint32_t RESV_3_3                      : 1; // bit 3~3
    volatile uint32_t CH25INTRM                     : 3; // bit 4~6
    volatile uint32_t RESV_7_7                      : 1; // bit 7~7
    volatile uint32_t CH26INTRM                     : 3; // bit 8~10
    volatile uint32_t RESV_11_11                    : 1; // bit 11~11
    volatile uint32_t CH27INTRM                     : 3; // bit 12~14
    volatile uint32_t RESV_15_15                    : 1; // bit 15~15
    volatile uint32_t CH28INTRM                     : 3; // bit 16~18
    volatile uint32_t RESV_19_19                    : 1; // bit 19~19
    volatile uint32_t CH29INTRM                     : 3; // bit 20~22
    volatile uint32_t RESV_23_23                    : 1; // bit 23~23
    volatile uint32_t CH30INTRM                     : 3; // bit 24~26
    volatile uint32_t RESV_27_27                    : 1; // bit 27~27
    volatile uint32_t CH31INTRM                     : 3; // bit 28~30
    volatile uint32_t RESV_31_31                    : 1; // bit 31~31
};

union GPIO0_REG_INTRMODE3 {
    volatile uint32_t                               all;
    struct GPIO0_REG_INTRMODE3_BITS                 bit;
};

struct GPIO0_REG_INTRSTATUS_BITS
{
    volatile uint32_t INTRSTATUS                    : 32; // bit 0~31
};

union GPIO0_REG_INTRSTATUS {
    volatile uint32_t                               all;
    struct GPIO0_REG_INTRSTATUS_BITS                bit;
};

struct GPIO0_REG_DEBOUNCEEN_BITS
{
    volatile uint32_t DEBOUNCEEN                    : 32; // bit 0~31
};

union GPIO0_REG_DEBOUNCEEN {
    volatile uint32_t                               all;
    struct GPIO0_REG_DEBOUNCEEN_BITS                bit;
};

struct GPIO0_REG_DEBOUNCECTRL_BITS
{
    volatile uint32_t DBPRESCALE                    : 8; // bit 0~7
    volatile uint32_t RESV_8_30                     : 23; // bit 8~30
    volatile uint32_t DBCLKSEL                      : 1; // bit 31~31
};

union GPIO0_REG_DEBOUNCECTRL {
    volatile uint32_t                               all;
    struct GPIO0_REG_DEBOUNCECTRL_BITS              bit;
};

typedef struct
{
    union GPIO0_REG_IDREV                           REG_IDREV;   // 0x000
    volatile uint32_t                               REG_RESV_0X4_0XC[3];
    union GPIO0_REG_CFG                             REG_CFG;     // 0x010
    volatile uint32_t                               REG_RESV_0X14_0X1C[3];
    union GPIO0_REG_DATAIN                          REG_DATAIN;  // 0x020
    union GPIO0_REG_DATAOUT                         REG_DATAOUT; // 0x024
    union GPIO0_REG_CHANNELDIR                      REG_CHANNELDIR; // 0x028
    union GPIO0_REG_DOUTCLEAR                       REG_DOUTCLEAR; // 0x02C
    union GPIO0_REG_DOUTSET                         REG_DOUTSET; // 0x030
    volatile uint32_t                               REG_RESV_0X34_0X3C[3];
    union GPIO0_REG_PULLEN                          REG_PULLEN;  // 0x040
    union GPIO0_REG_PULLTYPE                        REG_PULLTYPE; // 0x044
    volatile uint32_t                               REG_RESV_0X48_0X4C[2];
    union GPIO0_REG_INTREN                          REG_INTREN;  // 0x050
    union GPIO0_REG_INTRMODE0                       REG_INTRMODE0; // 0x054
    union GPIO0_REG_INTRMODE1                       REG_INTRMODE1; // 0x058
    union GPIO0_REG_INTRMODE2                       REG_INTRMODE2; // 0x05C
    union GPIO0_REG_INTRMODE3                       REG_INTRMODE3; // 0x060
    union GPIO0_REG_INTRSTATUS                      REG_INTRSTATUS; // 0x064
    volatile uint32_t                               REG_RESV_0X68_0X6C[2];
    union GPIO0_REG_DEBOUNCEEN                      REG_DEBOUNCEEN; // 0x070
    union GPIO0_REG_DEBOUNCECTRL                    REG_DEBOUNCECTRL; // 0x074
} GPIO0_RegDef;


#endif // __GPIO0_REGFILE_H__

