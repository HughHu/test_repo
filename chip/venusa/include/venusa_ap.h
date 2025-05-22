#ifndef INCLUDE_VENUSA_AP_H_
#define INCLUDE_VENUSA_AP_H_

#include <stddef.h>
#include <stdint.h>
#include "venusa_ap_base.h"
#include "mmio.h"


#include "cmn_buscfg_reg.h"
#include "cmn_syscfg_reg.h"
#include "dual_timer_reg.h"
#include "calendar_reg.h"
#include "gpadc_reg.h"
#include "gpio_reg.h"
#include "uart_reg.h"
#include "i2c_reg.h"
#include "ir_reg.h"
#include "spi_reg.h"
#include "gpt_reg.h"
#include "core_iomux_reg.h"
#include "flashc_reg.h"
#include "flash_dl_reg.h"
#include "wdt_reg.h"
#include "psram_mc_reg.h"
#include "apc_reg.h"
#include "audio_codec_reg.h"
#include "gpdma2d_reg.h"
#include "sdioh_reg.h"
#include "aon_ctrl_reg.h"
#include "aon_iomux_reg.h"
#include "keysense_reg.h"
#include "aon_timer_reg.h"
#include "aon_wdt_reg.h"
#include "efuse_ctrl_reg.h"
#include "image_vic_reg.h"
#include "jpeg_reg.h"
#include "qspi_sensor_in_reg.h"
#include "qspi_lcd_reg.h"
#include "rgb_interface_reg.h"
#include "i8080_out_reg.h"
#include "usb_reg.h"


#define IP_CMN_BUSCFG                                          ((CMN_BUSCFG_RegDef *) CMN_BUSCFG_BASE)
#define IP_CMN_SYSCFG                                          ((CMN_SYSCFG_RegDef *) CMN_SYSCFG_BASE)
#define IP_DUALTIMERS0                                         ((DUAL_TIMER_RegDef *) DUALTIMERS0_BASE)
#define IP_DUALTIMERS1                                         ((DUAL_TIMER_RegDef *) DUALTIMERS1_BASE)
#define IP_CALENDAR_TOP                                        ((CALENDAR_RegDef *) CALENDAR_TOP_BASE)
#define IP_GPADC                                               ((GPADC_RegDef *) GPADC_BASE)
#define IP_GPIO0                                               ((GPIO_RegDef *) GPIO0_BASE)
#define IP_GPIO1                                               ((GPIO_RegDef *) GPIO1_BASE)
#define IP_UART0                                               ((UART_RegDef *) UART0_BASE)
#define IP_UART1                                               ((UART_RegDef *) UART1_BASE)
#define IP_UART2                                               ((UART_RegDef *) UART2_BASE)
#define IP_I2C0                                                ((I2C_RegDef *) I2C0_BASE)
#define IP_I2C1                                                ((I2C_RegDef *) I2C1_BASE)
#define IP_IR                                                  ((IR_RegDef *) IR_BASE)
#define IP_SPI0                                                ((SPI_RegDef *) SPI0_BASE)
#define IP_SPI1                                                ((SPI_RegDef *) SPI1_BASE)
#define IP_GPT                                                 ((GPT_RegDef *) GPT_BASE)
#define IP_CORE_IOMUX                                          ((CORE_IOMUX_RegDef *) CORE_IOMUX_BASE)
#define IP_FLASH_CTRL                                          ((FLASHC_RegDef *) FLASH_CTRL_BASE)
#define IP_FLASH_DL                                            ((FLASH_DL_RegDef *) FLASH_DL_BASE)
#define IP_CORE0_WDT                                           ((WDT_RegDef *) CORE0_WDT_BASE)
#define IP_CORE1_WDT                                           ((WDT_RegDef *) CORE1_WDT_BASE)
#define IP_PSRAM_CTRL                                          ((PSRAM_MC_RegDef *) PSRAM_CTRL_BASE)
#define IP_APC                                                 ((APC_RegDef *) APC_BASE)
#define IP_CODEC                                               ((AUDIO_CODEC_RegDef *) CODEC_BASE)
#define IP_GPDMA2D                                             ((GPDMA2D_RegDef *) GPDMA2D_BASE)
#define IP_SDIOH                                               ((SDIOH_RegDef *) SDIOH_BASE)
#define IP_AON_CTRL                                            ((AON_CTRL_RegDef *) AON_CTRL_BASE)
#define IP_AON_IOMUX                                           ((AON_IOMUX_RegDef *) AON_IOMUX_BASE)
#define IP_KEYSENSE0                                           ((KEYSENSE_RegDef *) KEYSENSE0_BASE)
#define IP_AON_TIMER                                           ((AON_TIMER_RegDef *) AON_TIMER_BASE)
#define IP_AON_WDT                                             ((AON_WDT_RegDef *) AON_WDT_BASE)
#define IP_EFUSE_CTRL                                          ((EFUSE_CTRL_RegDef *) EFUSE_CTRL_BASE)
#define IP_DVP_IN                                              ((IMAGE_VIC_RegDef *) DVP_IN_BASE)
#define IP_JPEG_TOP                                            ((JPEG_RegDef *) JPEG_TOP_BASE)
#define IP_QSPI_IN                                             ((QSPI_SENSOR_IN_RegDef *) QSPI_IN_BASE)
#define IP_QSPI_OUT                                            ((QSPI_LCD_RegDef *) QSPI_OUT_BASE)
#define IP_RGB_OUT                                             ((RGB_INTERFACE_RegDef *) RGB_OUT_BASE)
#define IP_I8080_OUT                                           ((I8080_OUT_RegDef *) I8080_OUT_BASE)
#define IP_SYSCTRL                                             IP_CMN_SYSCFG
#define IP_SYSNODEF                                            IP_CMN_BUSCFG
#define IP_USBC                                                ((CSK_USB_RegDef *) USBC_BASE)
#define IP_CMN_SYS                                             IP_SYSCTRL

typedef struct IRegion_Info {
    unsigned long iregion_base;         /*!< Internal region base address */
    unsigned long eclic_base;           /*!< eclic base address */
    unsigned long systimer_base;        /*!< system timer base address */
    unsigned long smp_base;             /*!< smp base address */
    unsigned long idu_base;             /*!< idu base address */
} IRegion_Info_Type;

typedef enum EXCn {
    /* =======================================  Nuclei N/NX Specific Exception Code  ======================================== */
    InsUnalign_EXCn          =   0,              /*!<  Instruction address misaligned */
    InsAccFault_EXCn         =   1,              /*!<  Instruction access fault */
    IlleIns_EXCn             =   2,              /*!<  Illegal instruction */
    Break_EXCn               =   3,              /*!<  Beakpoint */
    LdAddrUnalign_EXCn       =   4,              /*!<  Load address misaligned */
    LdFault_EXCn             =   5,              /*!<  Load access fault */
    StAddrUnalign_EXCn       =   6,              /*!<  Store or AMO address misaligned */
    StAccessFault_EXCn       =   7,              /*!<  Store or AMO access fault */
    UmodeEcall_EXCn          =   8,              /*!<  Environment call from User mode */
    SmodeEcall_EXCn          =   9,              /*!<  Environment call from S-mode */
    MmodeEcall_EXCn          =  11,              /*!<  Environment call from Machine mode */
    InsPageFault_EXCn        =  12,              /*!<  Instruction page fault */
    LdPageFault_EXCn         =  13,              /*!<  Load page fault */
    StPageFault_EXCn         =  15,              /*!<  Store or AMO page fault */
    NMI_EXCn                 =  0xfff,           /*!<  NMI interrupt */
} EXCn_Type;

#if __riscv_xlen == 32

#ifndef __NUCLEI_CORE_REV
#define __NUCLEI_N_REV            0x0104    /*!< Core Revision r1p4 */
#else
#define __NUCLEI_N_REV            __NUCLEI_CORE_REV
#endif

#elif __riscv_xlen == 64

#ifndef __NUCLEI_CORE_REV
#define __NUCLEI_NX_REV           0x0100    /*!< Core Revision r1p0 */
#else
#define __NUCLEI_NX_REV           __NUCLEI_CORE_REV
#endif

#endif /* __riscv_xlen == 64 */



/* Define the correct core features */
#define __ECLIC_PRESENT           1                     /*!< Set to 1 if ECLIC is present */
#define __ECLIC_BASEADDR          0xE0020000            /*!< Set to ECLIC baseaddr of your device */

#define __ECLIC_INTCTLBITS        3                     /*!< Set to 1 - 8, the number of hardware bits are actually implemented in the clicintctl registers. */
#define __ECLIC_INTNUM            67                    /*!< Set to 1 - 1024, total interrupt number of ECLIC Unit */

#define __SYSTIMER_PRESENT        1                     /*!< Set to 1 if System Timer is present */
#define __SYSTIMER_BASEADDR       0xE0030000            /*!< Set to SysTimer baseaddr of your device */

#define __CIDU_PRESENT            1                     /*!< Set to 1 if CIDU is present */
#define __CIDU_BASEADDR           0x47000000            /*!< Set to cidu baseaddr of your device */


/*!< Set to 0, 1, or 2, 0 not present, 1 single floating point unit present, 2 double floating point unit present */
#if !defined(__riscv_flen)
#define __FPU_PRESENT             0
#elif __riscv_flen == 32
#define __FPU_PRESENT             1
#else
#define __FPU_PRESENT             2
#endif


/* __riscv_bitmanip/__riscv_dsp/__riscv_vector is introduced
 * in nuclei gcc 10.2 when b/p/v extension compiler option is selected.
 * For example:
 * -march=rv32imacb -mabi=ilp32 : __riscv_bitmanip macro will be defined
 * -march=rv32imacp -mabi=ilp32 : __riscv_dsp macro will be defined
 * -march=rv64imacv -mabi=lp64 : __riscv_vector macro will be defined
 */
#if defined(__riscv_bitmanip)
#define __BITMANIP_PRESENT        1                     /*!< Set to 1 if Bitmainpulation extension is present */
#else
#define __BITMANIP_PRESENT        0                     /*!< Set to 1 if Bitmainpulation extension is present */
#endif
#if defined(__riscv_dsp)
#define __DSP_PRESENT             1                     /*!< Set to 1 if Partial SIMD(DSP) extension is present */
#else
#define __DSP_PRESENT             0                     /*!< Set to 1 if Partial SIMD(DSP) extension is present */
#endif
#if defined(__riscv_vector)
#define __VECTOR_PRESENT          1                     /*!< Set to 1 if Vector extension is present */
#else
#define __VECTOR_PRESENT          0                     /*!< Set to 1 if Vector extension is present */
#endif

#define __PMP_PRESENT             1                     /*!< Set to 1 if PMP is present */
#define __PMP_ENTRY_NUM           8                    /*!< Set to 8 or 16, the number of PMP entries */

#define __SPMP_PRESENT            0                     /*!< Set to 1 if SPMP is present */
#define __SPMP_ENTRY_NUM          16                    /*!< Set to 8 or 16, the number of SPMP entries */

#ifndef __TEE_PRESENT
#define __TEE_PRESENT             0                     /*!< Set to 1 if TEE is present */
#endif

#define __ICACHE_PRESENT          1                     /*!< Set to 1 if I-Cache is present */
#define __DCACHE_PRESENT          1                     /*!< Set to 1 if D-Cache is present */
#define __CCM_PRESENT             1                     /*!< Set to 1 if Cache Control and Mantainence Unit is present */

/* TEE feature depends on PMP */
#if defined(__TEE_PRESENT) && (__TEE_PRESENT == 1)
#if !defined(__PMP_PRESENT) || (__PMP_PRESENT != 1)
#error "__PMP_PRESENT must be defined as 1!"
#endif /* !defined(__PMP_PRESENT) || (__PMP_PRESENT != 1) */
#if !defined(__SPMP_PRESENT) || (__SPMP_PRESENT != 1)
#error "__SPMP_PRESENT must be defined as 1!"
#endif /* !defined(__SPMP_PRESENT) || (__SPMP_PRESENT != 1) */
#endif /* defined(__TEE_PRESENT) && (__TEE_PRESENT == 1) */

#ifndef __INC_INTRINSIC_API
#define __INC_INTRINSIC_API       0                     /*!< Set to 1 if intrinsic api header files need to be included */
#endif

#define __Vendor_SysTickConfig    0                     /*!< Set to 1 if different SysTick Config is used */
#define __Vendor_EXCEPTION        0                     /*!< Set to 1 if vendor exception hander is present */

/** @} */ /* End of group Configuration_of_NMSIS */


/* Define boot hart id */
#ifndef BOOT_HARTID
#define BOOT_HARTID               0                     /*!< Choosen boot hart id in current cluster when in soc system, need to align with the value defined in startup_<Device>.S, should start from 0, taken the mhartid bit 0-7 value */
#endif

#include <nmsis_core.h>                         /*!< Nuclei N/NX class processor and core peripherals */
/* ToDo: include your system_mars.h file
         replace 'Device' with your device name */
#include <nmsis_core.h>                         /*!< Nuclei N/NX class processor and core peripherals */
/* ToDo: include your system_mars.h file
         replace 'Device' with your device name */
#include "system_RISCVN300.h"                    /*!< riscv N300 System */

#ifndef   __COMPILER_BARRIER
  #define __COMPILER_BARRIER()                   __ASM volatile("":::"memory")
#endif

#define SOC_TIMER_FREQ              1000000
/*****************************************************************************
 * Macros for Register Access
 ****************************************************************************/
#define inw(reg)               (*((volatile unsigned int *) (reg)))
#define outw(reg, data)        ((*((volatile unsigned int *)(reg)))=(unsigned int)(data))
#define inb(reg)               (*((volatile unsigned char *) (reg)))
#define outb(reg, data)        ((*((volatile unsigned char *)(reg)))=(unsigned char)(data))

#define __I                     volatile const  /* 'read only' permissions      */
#define __O                     volatile        /* 'write only' permissions     */
#define __IO                    volatile        /* 'read / write' permissions   */
/************************************************************************************
 * Linker definitions
 ************************************************************************************/
#define _CORE0_ILM_TEXT_SEC              ".text.core0_ilm"

#define _CORE0_DLM_DATA_SEC              ".data.core0_dlm"
#define _CORE0_DLM_BSS_SEC               ".bss.core0_dlm"

#define _CORE1_ILM_TEXT_SEC              ".text.core1_ilm"

#define _CORE1_DLM_DATA_SEC              ".data.core1_dlm"
#define _CORE1_DLM_BSS_SEC               ".bss.core1_dlm"

#define _CMN_RAM0_TEXT_SEC               ".text.cmn_ram0"

#define _CMN_RAM0_DATA_SEC               ".data.cmn_ram0"
#define _CMN_RAM0_BSS_SEC                ".bss.cmn_ram0"

#define _CMN_RAM1_TEXT_SEC               ".text.cmn_ram1"

#define _CMN_RAM1_DATA_SEC               ".data.cmn_ram1"
#define _CMN_RAM1_BSS_SEC                ".bss.cmn_ram1"

#define _CMN_LUNA_RAM_TEXT_SEC           ".text.cmn_luna_ram"

#define _CMN_LUNA_RAM_DATA_SEC           ".data.cmn_luna_ram"
#define _CMN_LUNA_RAM_BSS_SEC            ".bss.cmn_luna_ram"

#define _CMN_PSRAM_TEXT_SEC              ".text.cmn_psram"

#define _CMN_PSRAM_DATA_SEC              ".data.cmn_psram"
#define _CMN_PSRAM_BSS_SEC               ".bss.cmn_psram"

#define _CORE0_ILM_TEXT                  __attribute__ ((section (_CORE0_ILM_TEXT_SEC)))
#define _CORE0_ILM_TEXT_TAG(tag)         __attribute__ ((section (_CORE0_ILM_TEXT_SEC"."#tag)))

#define _CORE0_DLM_DATA                  __attribute__ ((section (_CORE0_DLM_DATA_SEC)))
#define _CORE0_DLM_DATA_TAG(tag)         __attribute__ ((section (_CORE0_DLM_DATA_SEC"."#tag)))
#define _CORE0_DLM_BSS                   __attribute__ ((section (_CORE0_DLM_BSS_SEC)))
#define _CORE0_DLM_BSS_TAG(tag)          __attribute__ ((section (_CORE0_DLM_BSS_SEC"."#tag)))

#define _CORE1_ILM_TEXT                  __attribute__ ((section (_CORE1_ILM_TEXT_SEC)))
#define _CORE1_ILM_TEXT_TAG(tag)         __attribute__ ((section (_CORE1_ILM_TEXT_SEC"."#tag)))

#define _CORE1_DLM_DATA                  __attribute__ ((section (_CORE1_DLM_DATA_SEC)))
#define _CORE1_DLM_DATA_TAG(tag)         __attribute__ ((section (_CORE1_DLM_DATA_SEC"."#tag)))
#define _CORE1_DLM_BSS                   __attribute__ ((section (_CORE1_DLM_BSS_SEC)))
#define _CORE1_DLM_BSS_TAG(tag)          __attribute__ ((section (_CORE1_DLM_BSS_SEC"."#tag)))

#define _CMN_RAM0_TEXT                   __attribute__ ((section (_CMN_RAM0_TEXT_SEC)))
#define _CMN_RAM0_TEXT_TAG(tag)          __attribute__ ((section (_CMN_RAM0_TEXT_SEC"."#tag)))

#define _CMN_RAM0_DATA                   __attribute__ ((section (_CMN_RAM0_DATA_SEC)))
#define _CMN_RAM0_DATA_TAG(tag)          __attribute__ ((section (_CMN_RAM0_DATA_SEC"."#tag)))
#define _CMN_RAM0_BSS                    __attribute__ ((section (_CMN_RAM0_BSS_SEC)))
#define _CMN_RAM0_BSS_TAG(tag)           __attribute__ ((section (_CMN_RAM0_BSS_SEC"."#tag)))

#define _CMN_RAM1_TEXT                   __attribute__ ((section (_CMN_RAM1_TEXT_SEC)))
#define _CMN_RAM1_TEXT_TAG(tag)          __attribute__ ((section (_CMN_RAM1_TEXT_SEC"."#tag)))

#define _CMN_RAM1_DATA                   __attribute__ ((section (_CMN_RAM1_DATA_SEC)))
#define _CMN_RAM1_DATA_TAG(tag)          __attribute__ ((section (_CMN_RAM1_DATA_SEC"."#tag)))
#define _CMN_RAM1_BSS                    __attribute__ ((section (_CMN_RAM1_BSS_SEC)))
#define _CMN_RAM1_BSS_TAG(tag)           __attribute__ ((section (_CMN_RAM1_BSS_SEC"."#tag)))

#define _CMN_LUNA_RAM_TEXT               __attribute__ ((section (_CMN_LUNA_RAM_TEXT_SEC)))
#define _CMN_LUNA_RAM_TEXT_TAG(tag)      __attribute__ ((section (_CMN_LUNA_RAM_TEXT_SEC"."#tag)))

#define _CMN_LUNA_RAM_DATA               __attribute__ ((section (_CMN_LUNA_RAM_DATA_SEC)))
#define _CMN_LUNA_RAM_DATA_TAG(tag)      __attribute__ ((section (_CMN_LUNA_RAM_DATA_SEC"."#tag)))
#define _CMN_LUNA_RAM_BSS                __attribute__ ((section (_CMN_LUNA_RAM_BSS_SEC)))
#define _CMN_LUNA_RAM_BSS_TAG(tag)       __attribute__ ((section (_CMN_LUNA_RAM_BSS_SEC"."#tag)))

#define _CMN_PSRAM_TEXT                  __attribute__ ((section (_CMN_PSRAM_TEXT_SEC)))
#define _CMN_PSRAM_TEXT_TAG(tag)         __attribute__ ((section (_CMN_PSRAM_TEXT_SEC"."#tag)))

#define _CMN_PSRAM_DATA                  __attribute__ ((section (_CMN_PSRAM_DATA_SEC)))
#define _CMN_PSRAM_DATA_TAG(tag)         __attribute__ ((section (_CMN_PSRAM_DATA_SEC"."#tag)))
#define _CMN_PSRAM_BSS                   __attribute__ ((section (_CMN_PSRAM_BSS_SEC)))
#define _CMN_PSRAM_BSS_TAG(tag)          __attribute__ ((section (_CMN_PSRAM_BSS_SEC"."#tag)))

#if (BOOT_HARTID == 0) // Core0
#define _FAST_TEXT                       _CORE0_ILM_TEXT
#define _FAST_TEXT_TAG(tag)              _CORE0_ILM_TEXT_TAG(tag)
#define _FAST_DATA                       _CORE0_DLM_DATA
#define _FAST_DATA_TAG(tag)              _CORE0_DLM_DATA_TAG(tag)
#define _FAST_BSS                        _CORE0_DLM_BSS
#define _FAST_BSS_TAG(tag)               _CORE0_DLM_BSS_TAG(tag)
#else // Core1
#define _FAST_TEXT                       _CORE1_ILM_TEXT
#define _FAST_TEXT_TAG(tag)              _CORE1_ILM_TEXT_TAG(tag)
#define _FAST_DATA                       _CORE1_DLM_DATA
#define _FAST_DATA_TAG(tag)              _CORE1_DLM_DATA_TAG(tag)
#define _FAST_BSS                        _CORE1_DLM_BSS
#define _FAST_BSS_TAG(tag)               _CORE1_DLM_BSS_TAG(tag)
#endif

#define _DMA
#define _DMA_PRAM                         __attribute__((aligned(32)))

#define _FAST_FUNC_RO
#define _FAST_DATA_VI
#define _FAST_DATA_ZI

#ifndef __ASM
#define __ASM                   __asm     /*!< asm keyword for GNU Compiler */
#endif

#ifndef __INLINE
#define __INLINE                inline    /*!< inline keyword for GNU Compiler */
#endif

#ifndef __ALWAYS_STATIC_INLINE
#define __ALWAYS_STATIC_INLINE  __attribute__((always_inline)) static inline
#endif

#ifndef __STATIC_INLINE
#define __STATIC_INLINE         static inline
#endif

#define NMI_EXPn                (-2)      /* NMI Exception */

/************************************************************************************
 * GI(Global Interrupt) and IRQ vector inline functions
 ************************************************************************************/
//The greater value of level, the higher priority
#define MAX_INTERRUPT_PRIORITY_RVAL     3
#define MID_INTERRUPT_PRIORITY          2
#define DEF_INTERRUPT_PRIORITY          0
#define DEF_INTERRUPT_LEVEL             0

// Is Global Interrupt enabled? 0 = disabled, 1 = enabled
static inline uint8_t GINT_enabled()
{
    return ((__RV_CSR_READ(CSR_MSTATUS) & MSTATUS_MIE) == MSTATUS_MIE);
}

// Enable GINT
static inline void enable_GINT()
{
	__RV_CSR_SET(CSR_MSTATUS, MSTATUS_MIE);
}

// Disable GINT
static inline void disable_GINT()
{
	__RV_CSR_CLEAR(CSR_MSTATUS, MSTATUS_MIE);
}

// ISR function prototype
typedef void (*ISR)(void);

// Register ISR into Interrupt Vector Table
void register_ISR(uint32_t irq_no, ISR isr, ISR* isr_old);


static inline uint8_t IRQ_enabled(uint32_t irq_no)
{
    return (uint8_t)ECLIC_GetEnableIRQ(irq_no);
}

static inline void enable_IRQ(uint32_t irq_no)
{
	ECLIC_EnableIRQ(irq_no);
}

static inline void disable_IRQ(uint32_t irq_no)
{
	ECLIC_DisableIRQ(irq_no);
}

static inline void clear_IRQ(uint32_t irq_no)
{
	ECLIC_ClearPendingIRQ(irq_no);
}

void non_cacheable_region_enable(uint32_t base_addr, uint32_t len);

void non_cacheable_region_disable(void);

extern void mpu_init( void );


#define HAL_CMN_DMA_SEL0_HSID_0_UART0_RX                                            0
#define HAL_CMN_DMA_SEL0_HSID_1_UART0_TX                                            1
#define HAL_CMN_DMA_SEL0_HSID_2_UART1_RX                                            2
#define HAL_CMN_DMA_SEL0_HSID_3_UART1_TX                                            3
#define HAL_CMN_DMA_SEL0_HSID_4_UART2_RX                                            4
#define HAL_CMN_DMA_SEL0_HSID_5_UART2_TX                                            5
#define HAL_CMN_DMA_SEL0_HSID_6_SPI0_RX                                             6
#define HAL_CMN_DMA_SEL0_HSID_7_SPI0_TX                                             7
#define HAL_CMN_DMA_SEL0_HSID_8_SPI1_RX                                             8
#define HAL_CMN_DMA_SEL0_HSID_9_SPI1_TX                                             9
#define HAL_CMN_DMA_SEL0_HSID_10_IR_RX                                               10
#define HAL_CMN_DMA_SEL0_HSID_11_IR_TX                                               11
#define HAL_CMN_DMA_SEL0_HSID_12_GPADC                                               12
#define HAL_CMN_DMA_SEL0_HSID_13_I2C0                                                13
#define HAL_CMN_DMA_SEL0_HSID_14_I2C1                                                14
#define HAL_CMN_DMA_SEL0_HSID_15_DVP_DMA                                             15


#define HAL_CMN_DMA_SEL1_HSID_0_RGB_DMA                                             0
#define HAL_CMN_DMA_SEL1_HSID_1_QSPI_OUT_DMA                                        1
#define HAL_CMN_DMA_SEL1_HSID_2_QSPI_IN_DMA                                         2
#define HAL_CMN_DMA_SEL1_HSID_3_DVP_DMA                                             3
#define HAL_CMN_DMA_SEL1_HSID_4_JPG_DMA_E                                           4
#define HAL_CMN_DMA_SEL1_HSID_5_JPG_DMA_P                                           5
#define HAL_CMN_DMA_SEL1_HSID_6_UART1_RX                                            6
#define HAL_CMN_DMA_SEL1_HSID_7_UART1_TX                                            7
#define HAL_CMN_DMA_SEL1_HSID_8_APC_DMA_TX_0                                        8
#define HAL_CMN_DMA_SEL1_HSID_9_APC_DMA_TX_1                                        9
#define HAL_CMN_DMA_SEL1_HSID_10_APC_DMA_TX_2                                        10
#define HAL_CMN_DMA_SEL1_HSID_11_I8080_DMA                                           11
#define HAL_CMN_DMA_SEL1_HSID_12_APC_DMA_RX_0                                        12
#define HAL_CMN_DMA_SEL1_HSID_13_APC_DMA_RX_1                                        13
#define HAL_CMN_DMA_SEL1_HSID_14_APC_DMA_RX_2                                        14
#define HAL_CMN_DMA_SEL1_HSID_15_APC_DMA_RX_3                                        15


#define HAL_GP_DMA_SEL0_HSID0_RGB_DMA                                             0
#define HAL_GP_DMA_SEL0_HSID1_QSPI_OUT_DMA                                        1
#define HAL_GP_DMA_SEL0_HSID2_QSPI_IN_DMA                                         2
#define HAL_GP_DMA_SEL0_HSID3_DVP_DMA                                             3
#define HAL_GP_DMA_SEL0_HSID4_JPG_DMA_E                                           4
#define HAL_GP_DMA_SEL0_HSID5_JPG_DMA_P                                           5
#define HAL_GP_DMA_SEL0_HSID6_UART0_RX                                            6
#define HAL_GP_DMA_SEL0_HSID7_UART0_TX                                            7
#define HAL_GP_DMA_SEL0_HSID8_APC_DMA_TX_0                                        8
#define HAL_GP_DMA_SEL0_HSID9_APC_DMA_TX_1                                        9
#define HAL_GP_DMA_SEL0_HSID10_APC_DMA_TX_2                                        10
#define HAL_GP_DMA_SEL0_HSID11_I8080_DMA                                           11
#define HAL_GP_DMA_SEL0_HSID12_APC_DMA_RX_0                                        12
#define HAL_GP_DMA_SEL0_HSID13_APC_DMA_RX_1                                        13
#define HAL_GP_DMA_SEL0_HSID14_APC_DMA_RX_2                                        14
#define HAL_GP_DMA_SEL0_HSID15_APC_DMA_RX_3                                        15


#define HAL_GP_DMA_SEL1_HSID0_UART0_RX                                            0
#define HAL_GP_DMA_SEL1_HSID1_UART0_TX                                            1
#define HAL_GP_DMA_SEL1_HSID2_UART1_RX                                            2
#define HAL_GP_DMA_SEL1_HSID3_UART1_TX                                            3
#define HAL_GP_DMA_SEL1_HSID4_UART2_RX                                            4
#define HAL_GP_DMA_SEL1_HSID5_UART2_TX                                            5
#define HAL_GP_DMA_SEL1_HSID6_SPI0_RX                                             6
#define HAL_GP_DMA_SEL1_HSID7_SPI0_TX                                             7
#define HAL_GP_DMA_SEL1_HSID8_SPI1_RX                                             8
#define HAL_GP_DMA_SEL1_HSID9_SPI1_TX                                             9
#define HAL_GP_DMA_SEL1_HSID10_IR_RX                                               10
#define HAL_GP_DMA_SEL1_HSID11_IR_TX                                               11
#define HAL_GP_DMA_SEL1_HSID12_GPADC                                               12
#define HAL_GP_DMA_SEL1_HSID13_I2C0                                                13
#define HAL_GP_DMA_SEL1_HSID14_I2C1                                                14
#define HAL_GP_DMA_SEL1_HSID15_GPADC                                               15

#endif /* INCLUDE_VENUSA_AP_H_ */
