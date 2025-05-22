#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <FreeRTOS.h>
#include <semphr.h>

#include "chip.h"
#include "log_print.h"
#include "systick.h"
#include "ClockManager.h"
#include "IOMuxManager.h"
#include "Driver_GPIO.h"
#include "Driver_I2C.h"
#include "Driver_DVP.h"
#include "cache.h"
#include "ls_jpeg.h"
// #include "config.h"
#include "jpeg_check.h"

#define VIDEO_LOG
#define DELAY_MS(x) SysTick_Delay_Ms(x)
#define DELAY_US(x) SysTick_Delay_Us(x)



static int32_t dvp_cmndma_init(void);
static int32_t dvp_cmndma_start(void* pbuf, uint32_t size_word);
static uint32_t dvp_cmndma_finish_cnt_get(void);
static void dvp_cmndma_finish_cnt_clear(void);
static void cmndma_reg_dump(uint8_t dma_ch);

static int csk_i2c_init(uint8_t i2c_index);
static int csk_i2c_write_reg8(uint8_t i2c_index, uint16_t addr, uint8_t reg, uint8_t value);
static int csk_i2c_read(uint8_t i2c_index, uint16_t addr, uint8_t reg, uint8_t *data, uint16_t num);
static uint8_t csk_i2c_read_reg8(uint8_t i2c_index, uint16_t addr, uint8_t reg);
static void gc0328_init(uint8_t colorbar_enable);

static void dvp_callback(DVP_emIrqEvent event, uint32_t param);
static uint32_t dvp_eof_cnt_get(void);
static void dvp_eof_cnt_clear(void);
static void dvp_reg_dump(void);


static void test_jpeg_dvp_callback(DVP_emIrqEvent event, uint32_t param)
{
    void *dvp_dev = DVP0();
    uint32_t error;

    switch(event)
    {
        case DVP_IRQ_EVENT_SOF:
            //VIDEO_LOG("[%s:%d] DVP SOF event: %d", __func__, __LINE__, event);
            break;

        case DVP_IRQ_EVENT_EOF:
            //VIDEO_LOG("[%s:%d] DVP EOF event: %d", __func__, __LINE__, event);
            break;

        case DVP_IRQ_EVENT_EOF_CNT_ABNOR:
            VIDEO_LOG("[%s:%d] DVP_IRQ_EVENT_EOF_CNT_ABNOR", __func__, __LINE__);
            break;

        case DVP_IRQ_EVENT_DMA_VIC_SINGLE:
            VIDEO_LOG("[%s:%d] DVP_IRQ_EVENT_DMA_VIC_SINGLE", __func__, __LINE__);
            break;

        case DVP_IRQ_EVENT_DMA_VIC_REQ:
            VIDEO_LOG("[%s:%d] DVP_IRQ_EVENT_DMA_VIC_REQ", __func__, __LINE__);
            break;

        case DVP_IRQ_EVENT_FIFO_UNFLOW:
            VIDEO_LOG("[%s:%d] DVP_IRQ_EVENT_FIFO_UNFLOW", __func__, __LINE__);
            break;

        case DVP_IRQ_EVENT_FIFO_OVFLOW:
            VIDEO_LOG("[%s:%d] DVP_IRQ_EVENT_FIFO_OVFLOW", __func__, __LINE__);
            break;

        case DVP_IRQ_EVENT_FIFO_RD_EMPTY:
            VIDEO_LOG("[%s:%d] DVP_IRQ_EVENT_FIFO_RD_EMPTY", __func__, __LINE__);
            break;

        case DVP_IRQ_EVENT_FIFO_WR_FULL:
            VIDEO_LOG("[%s:%d] DVP_IRQ_EVENT_FIFO_WR_FULL", __func__, __LINE__);
            break;

        default:
            VIDEO_LOG("[%s:%d] DVP error event: %d", __func__, __LINE__, event);
            break;
    }

    error = DVP_GetError(dvp_dev, &error);

    if(error != DVP_ERROR_NONE)
    {
        /* turn off the clock out when the required frames received*/
        DVP_DisableClockout(dvp_dev);
        DVP_Stop(dvp_dev);
        VIDEO_LOG("[%s:%d] DVP Error code: %d", __func__, __LINE__, error);
    }

    return;
}

int32_t test_jpeg_dvp_start(Jpeg_CodecCfg *enc_cfg, uint8_t *dvp_image, uint32_t imgae_size)
{
    int32_t ret = FAILURE;
    DVP_InitTypeDef dvp_cfg = {
        .FrameWidth = enc_cfg->width,
        .FrameHeight = enc_cfg->height,
        .PixelOffset = 0,
        .LineOffset = 0,
        .InputFormat = DVP_INPUT_FORM_YUV422_Y0CBY1CR,
        .PCKPolarity = DVP_POL_RISING,
        .VSPolarity = DVP_POL_FALLING,
        .HSPolarity = DVP_POL_RISING,
        .DataAlign = DVP_DATA_ALIGN_LEFT,
    };

    /* pinmux */
    IOMuxManager_PinConfigure(CSK_IOMUX_PAD_B,  3, CSK_IOMUX_FUNC_ALTER13);  // PIN_DVP_MCLK
    IOMuxManager_PinConfigure(CSK_IOMUX_PAD_A, 23, CSK_IOMUX_FUNC_ALTER13);  // PIN_DVP_PCLK
    IOMuxManager_PinConfigure(CSK_IOMUX_PAD_A, 22, CSK_IOMUX_FUNC_ALTER13);  // PIN_DVP_VS
    IOMuxManager_PinConfigure(CSK_IOMUX_PAD_A, 21, CSK_IOMUX_FUNC_ALTER13);  // PIN_DVP_HS
    IOMuxManager_PinConfigure(CSK_IOMUX_PAD_A, 24, CSK_IOMUX_FUNC_ALTER13);  // PIN_DVP_D0
    IOMuxManager_PinConfigure(CSK_IOMUX_PAD_A, 25, CSK_IOMUX_FUNC_ALTER13);  // PIN_DVP_D1
    IOMuxManager_PinConfigure(CSK_IOMUX_PAD_A, 26, CSK_IOMUX_FUNC_ALTER13);  // PIN_DVP_D2
    IOMuxManager_PinConfigure(CSK_IOMUX_PAD_A, 27, CSK_IOMUX_FUNC_ALTER13);  // PIN_DVP_D3
    IOMuxManager_PinConfigure(CSK_IOMUX_PAD_A, 28, CSK_IOMUX_FUNC_ALTER13);  // PIN_DVP_D4
    IOMuxManager_PinConfigure(CSK_IOMUX_PAD_A, 29, CSK_IOMUX_FUNC_ALTER13);  // PIN_DVP_D5
    IOMuxManager_PinConfigure(CSK_IOMUX_PAD_A, 30, CSK_IOMUX_FUNC_ALTER13);  // PIN_DVP_D6
    IOMuxManager_PinConfigure(CSK_IOMUX_PAD_A, 31, CSK_IOMUX_FUNC_ALTER13);  // PIN_DVP_D7
    IOMuxManager_PinConfigure(CSK_IOMUX_PAD_B,  2, CSK_IOMUX_FUNC_DEFAULT);  // PIN_DVP_RST
    IOMuxManager_PinConfigure(CSK_IOMUX_PAD_C,  3, CSK_IOMUX_FUNC_ALTER1);   // PIN_DVP_PWDN
    IOMuxManager_PinConfigure(CSK_IOMUX_PAD_C,  1, CSK_IOMUX_FUNC_ALTER1);   // PIN_DVP_SCL
    IOMuxManager_PinConfigure(CSK_IOMUX_PAD_C,  0, CSK_IOMUX_FUNC_ALTER1);   // PIN_DVP_SDA

    /* PIN_DVP_RST */
    GPIO_SetDir(GPIOB(), (1UL << 2), CSK_GPIO_DIR_OUTPUT);
    GPIO_PinWrite(GPIOB(), (1UL << 2), 0);
    DELAY_MS(100);
    GPIO_PinWrite(GPIOB(), (1UL << 2), 1);
    DELAY_MS(100);

    /* PIN_DVP_PWDN */
    GPIO_SetDir(GPIOB(), (1UL << 9), CSK_GPIO_DIR_OUTPUT);
    GPIO_PinWrite(GPIOB(), (1UL << 9), 0);

    /* dvp clk enable and reset */
    IP_SYSCTRL->REG_PERI_CLK_CFG7.bit.ENA_VIC_CLK     = 1;
    IP_SYSCTRL->REG_SW_RESET_CFG2.bit.DVP_RESET        = 1;
    DELAY_MS(10);

    /* dvp init */
    ret = DVP_Initialize(DVP0(), test_jpeg_dvp_callback, &dvp_cfg);
    CHECK_RET_EQ_EXIT(ret, CSK_DRIVER_OK, error0);
    ret = DVP_EnableClockout(DVP0(), DVP_MCLK_OUT);
    CHECK_RET_EQ_EXIT(ret, CSK_DRIVER_OK, error0);
    DELAY_MS(100);

    /* camera init */
    gc0328_init(0);
    DELAY_MS(100);
    VIDEO_LOG("[%s:%d] ", __func__, __LINE__);

    /* CMN DMA init */
    dvp_cmndma_init();
    dvp_cmndma_finish_cnt_clear();
    dvp_eof_cnt_clear();
    ret = dvp_cmndma_start(dvp_image, imgae_size / sizeof(uint32_t));
    CHECK_RET_EQ_EXIT(ret, CSK_DRIVER_OK, error0);


    /* dvp start */
    ret = DVP_Start(DVP0());
    CHECK_RET_EQ_EXIT(ret, CSK_DRIVER_OK, error0);

	VIDEO_LOG("[%s:%d] test SUCCESS", __func__, __LINE__);

    return ret;

error0:
    dvp_reg_dump();
    DVP_Stop(DVP0());
    DVP_Uninitialize(DVP0());
    DVP_DisableClockout(DVP0());

    VIDEO_LOG("[%s:%d] test FAILED", __func__, __LINE__);

    return ret;
}

void test_jpeg_dvp_stop()
{
    DVP_Stop(DVP0());
    // DVP_Uninitialize(DVP0());
    // DVP_DisableClockout(DVP0());
    return;
}

void test_jpeg_dvp_restart(uint8_t *dvp_image, uint32_t imgae_size)
{
    int ret = dvp_cmndma_start(dvp_image, imgae_size / sizeof(uint32_t));
    CHECK_RET_EQ(ret, CSK_DRIVER_OK);

    ret = DVP_Start(DVP0());
    CHECK_RET_EQ(ret, CSK_DRIVER_OK);

    return;
}

/****************************************** CMN_DMA ************************************************/
#include "dma.h"

static volatile uint32_t dvp_cmndma_finish_cnt = 0;

extern SemaphoreHandle_t dvpSemaphore;

static void dvp_cmndma_callback(uint32_t event_info, uint32_t xfer_bytes, uint32_t usr_param)
{
    //VIDEO_LOG("[%s]: event = %d, channel = %d, xfer_bytes = %d", __func__, event_info & 0xFF, (event_info >> 8) & 0xFF, xfer_bytes);
    if(event_info & DMA_EVENT_TRANSFER_COMPLETE){
        dvp_cmndma_finish_cnt++;
        if (dvpSemaphore != NULL) {
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            xSemaphoreGiveFromISR(dvpSemaphore, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
}

static uint32_t dvp_cmndma_finish_cnt_get(void)
{
    return dvp_cmndma_finish_cnt;
}

static void dvp_cmndma_finish_cnt_clear(void)
{
    dvp_cmndma_finish_cnt = 0;
}

static int32_t dvp_cmndma_init(void)
{
    dvp_cmndma_finish_cnt = 0;

    IP_SYSCTRL->REG_PERI_CLK_CFG7.bit.ENA_CMNDMAC_CLK = 1;
    IP_SYSCTRL->REG_SW_RESET_CFG2.bit.CMNDMA_RESET     = 1;

    dma_initialize();

    IP_SYSCTRL->REG_DMA_HS.bit.DMA_HS_SEL_03 = 1;
    IP_SYSCTRL->REG_DMA_HS.bit.DMA_HS_SEL_15 = 1;
    IP_SYSCTRL->REG_DMA_HS.bit.DMA_HS_SEL_19 = 1;

    return CSK_DRIVER_OK;
}


static int32_t dvp_cmndma_start(void* pbuf, uint32_t size_word)
{
    int32_t ret = 0;
    uint8_t dma_ch = 0;
    uint32_t control, config_low, config_high;
    uint32_t src_addr = DVP0_Buf();
    uint32_t dst_addr = (uint32_t)pbuf;

    CHECK_POINT_NOT_NULL(pbuf);

    control = DMA_CH_CTLL_INT_EN | DMA_CH_CTLL_DST_WIDTH(DMA_WIDTH_WORD) | DMA_CH_CTLL_SRC_WIDTH(DMA_WIDTH_WORD) |
            DMA_CH_CTLL_DST_BSIZE(DMA_BSIZE_8) | DMA_CH_CTLL_SRC_BSIZE(DMA_BSIZE_8) |
            DMA_CH_CTLL_TTFC_P2M | DMA_CH_CTLL_DMS(0) | DMA_CH_CTLL_SMS(1);

    control |=  DMA_CH_CTLL_SRC_FIX | DMA_CH_CTLL_DST_INC;

    config_low = DMA_CH_CFGL_CH_PRIOR(0);
    config_high = DMA_CH_CFGH_FIFO_MODE | DMA_CH_CFGH_SRC_PER(3); // DMA_CH_CFGH_SRC_PER(x) | DMA_CH_CFGH_DST_PER

    ret = dma_channel_select(&dma_ch, dvp_cmndma_callback, 0, DMA_CACHE_SYNC_NOP);
    if (ret == DMA_CHANNEL_ANY) {
        VIDEO_LOG("[FAILED] NO free DMA channel!!");
        return ret;
    }
    //VIDEO_LOG("[%s:%d] dma_ch=%d", __func__, __LINE__, dma_ch);

    ret = dma_channel_configure (dma_ch, src_addr, dst_addr, size_word, control, config_low, config_high, 0, 0);
    CHECK_RET_EQ(ret, CSK_DRIVER_OK);

    return ret;
}


//--------------------------------------------------------------------------
// For DWORD (64bit) register, low WORD(32bit) is valid and high WORD(32bit) is not used.
#define DWORD_REG(name)     uint32_t name; uint32_t __pad_##name

// Per-channel hardware register definitions
typedef struct {
    // The first 6 DWORD(64bit) registers are same as DMA_LLI
    __IO DWORD_REG(SAR);    // Source Address Register
    __IO DWORD_REG(DAR);    // Destination Address Register
    __IO DWORD_REG(LLP);    // Linked List Pointer
    __IO uint32_t CTL_LO;   // Control Register Low WORD
    __IO uint32_t CTL_HI;   // Control Register High WORD
    __IO DWORD_REG(SSTAT);  // Source Status Register, unimplemented, set to 0
    __IO DWORD_REG(DSTAT);  // Destination Status Register, unimplemented, set to 0
    // The following registers are NOT in DMA_LLI
    __IO DWORD_REG(SSTATAR); // Source Status Address Register, unused
    __IO DWORD_REG(DSTATAR); // Destination Status Address Register, unused
    __IO uint32_t CFG_LO;   // Configuration Register Low WORD
    __IO uint32_t CFG_HI;   // Configuration Register High WORD
    __IO DWORD_REG(SGR);    // Source Gather Register
    __IO DWORD_REG(DSR);    // Destination Scatter Register
} DMA_CHANNEL_REG;

// Interrupt register definitions
typedef struct {
    __IO DWORD_REG(XFER);
    __IO DWORD_REG(BLOCK);
    __IO DWORD_REG(SRC_TRAN);
    __IO DWORD_REG(DST_TRAN);
    __IO DWORD_REG(ERROR);
} DMA_IRQ_REG;

// Overall register memory map
typedef struct {
    // 0x000 ~ 0x2b8 N Channels' Registers
    DMA_CHANNEL_REG   CHANNEL[DMA_MAX_NR_CHANNELS];

    DMA_IRQ_REG     RAW;    // [RO] 0x2c0 ~ 0x2e0 raw
    DMA_IRQ_REG     STATUS; // [RO] 0x2e8 ~ 0x308 (raw & mask)
    DMA_IRQ_REG     MASK;   // [RW] 0x310 ~ 0x330 (set = irq enabled)
    DMA_IRQ_REG     CLEAR;  // [WO] 0x338 ~ 0x358 (clear raw and status)

    // [RO] 0x360 Combined Interrupt Status Register
    __IO DWORD_REG(STA_INT);

    // 0x368 ~ 0x390 software handshaking
    __IO DWORD_REG(REQ_SRC);
    __IO DWORD_REG(REQ_DST);
    __IO DWORD_REG(SGL_REQ_SRC);
    __IO DWORD_REG(SGL_REQ_DST);
    __IO DWORD_REG(LAST_SRC);
    __IO DWORD_REG(LAST_DST);

    // 0x398 ~ 0x3b0 miscellaneous
    __IO DWORD_REG(CFG);
    __IO DWORD_REG(CH_EN);
    __IO DWORD_REG(ID);
    __IO DWORD_REG(TEST);

    // 0x3b8 ~ 0x3c0 reserved
    __IO DWORD_REG(__RSVD0);
    __IO DWORD_REG(__RSVD1);

    // 0x3c8 ~ 0x3f0 hardware configuration parameters
    __I uint64_t COMP_PARAMS[6];

    // 0x3f8 Component version register
    __I uint64_t COMP_VER;

} DMA_RegMap;

#define IP_DMA              ((DMA_RegMap *) DMAC_BASE)

static void cmndma_reg_dump(uint8_t dma_ch)
{
    VIDEO_LOG("[SYS] CFG7       *0x%08x = 0x%08x", &IP_SYSCTRL->REG_PERI_CLK_CFG7.all, IP_SYSCTRL->REG_PERI_CLK_CFG7.all);
    VIDEO_LOG("[SYS] RESET_CFG2 *0x%08x = 0x%08x", &IP_SYSCTRL->REG_SW_RESET_CFG2.all, IP_SYSCTRL->REG_SW_RESET_CFG2.all);
    VIDEO_LOG("[SYS] DMA_HS     *0x%08x = 0x%08x", &IP_SYSCTRL->REG_DMA_HS.all, IP_SYSCTRL->REG_DMA_HS.all);
    VIDEO_LOG("[DMA] SAR[%d]    *0x%08x = 0x%08x", dma_ch, &IP_DMA->CHANNEL[dma_ch].SAR, IP_DMA->CHANNEL[dma_ch].SAR);
    VIDEO_LOG("[DMA] DAR[%d]    *0x%08x = 0x%08x", dma_ch, &IP_DMA->CHANNEL[dma_ch].DAR, IP_DMA->CHANNEL[dma_ch].DAR);
    VIDEO_LOG("[DMA] CTL_LO[%d] *0x%08x = 0x%08x", dma_ch, &IP_DMA->CHANNEL[dma_ch].CTL_LO, IP_DMA->CHANNEL[dma_ch].CTL_LO);
    VIDEO_LOG("[DMA] CTL_HI[%d] *0x%08x = 0x%08x", dma_ch, &IP_DMA->CHANNEL[dma_ch].CTL_HI, IP_DMA->CHANNEL[dma_ch].CTL_HI);
    VIDEO_LOG("[DMA] CFG_LO[%d] *0x%08x = 0x%08x", dma_ch, &IP_DMA->CHANNEL[dma_ch].CFG_LO, IP_DMA->CHANNEL[dma_ch].CFG_LO);
    VIDEO_LOG("[DMA] CFG_HI[%d] *0x%08x = 0x%08x", dma_ch, &IP_DMA->CHANNEL[dma_ch].CFG_HI, IP_DMA->CHANNEL[dma_ch].CFG_HI);
    VIDEO_LOG("[DMA] CFG        *0x%08x = 0x%08x", &IP_DMA->CFG, IP_DMA->CFG);
    VIDEO_LOG("[DMA] CH_EN      *0x%08x = 0x%08x", &IP_DMA->CH_EN, IP_DMA->CH_EN);
    VIDEO_LOG("[DMA] MASK       *0x%08x = 0x%08x", &IP_DMA->MASK.XFER, IP_DMA->MASK.XFER);
    VIDEO_LOG("[DMA] RAW        *0x%08x = 0x%08x", &IP_DMA->RAW.XFER, IP_DMA->RAW.XFER);
    VIDEO_LOG("[DMA] STATUS     *0x%08x = 0x%08x", &IP_DMA->STATUS.XFER, IP_DMA->STATUS.XFER);
}


/************************** dvp callback ********************************************************/
static volatile uint32_t dvp_eof_cnt = 0;

static void dvp_callback(DVP_emIrqEvent event, uint32_t param)
{
    void *dvp_dev = DVP0();
    uint32_t error;

    switch(event)
    {
        case DVP_IRQ_EVENT_SOF:
            //VIDEO_LOG("[%s:%d] DVP SOF event: %d", __func__, __LINE__, event);
            break;

        case DVP_IRQ_EVENT_EOF:
            //VIDEO_LOG("[%s:%d] DVP EOF event: %d", __func__, __LINE__, event);
            dvp_eof_cnt++;
            break;

#if 0
            if(dvp_frm_cnt >= DVP_TEST_FRAME_CNT_MAX)
            {
                /* turn off the clock out when the required frames received*/
                DVP_DisableClockout(dvp_dev);
                DVP_Stop(dvp_dev);
            }
#endif
            //DVP_DisableClockout(dvp_dev);
            //DVP_Stop(dvp_dev);
            //DVP_Start(dvp_dev, &buffer_dvp);

            break;

        case DVP_IRQ_EVENT_EOF_CNT_ABNOR:
            VIDEO_LOG("[%s:%d] DVP_IRQ_EVENT_EOF_CNT_ABNOR", __func__, __LINE__);
            break;

        case DVP_IRQ_EVENT_DMA_VIC_SINGLE:
            VIDEO_LOG("[%s:%d] DVP_IRQ_EVENT_DMA_VIC_SINGLE", __func__, __LINE__);
            break;

        case DVP_IRQ_EVENT_DMA_VIC_REQ:
            VIDEO_LOG("[%s:%d] DVP_IRQ_EVENT_DMA_VIC_REQ", __func__, __LINE__);
            break;

        case DVP_IRQ_EVENT_FIFO_UNFLOW:
            VIDEO_LOG("[%s:%d] DVP_IRQ_EVENT_FIFO_UNFLOW", __func__, __LINE__);
            break;

        case DVP_IRQ_EVENT_FIFO_OVFLOW:
            VIDEO_LOG("[%s:%d] DVP_IRQ_EVENT_FIFO_OVFLOW", __func__, __LINE__);
            break;

        case DVP_IRQ_EVENT_FIFO_RD_EMPTY:
            VIDEO_LOG("[%s:%d] DVP_IRQ_EVENT_FIFO_RD_EMPTY", __func__, __LINE__);
            break;

        case DVP_IRQ_EVENT_FIFO_WR_FULL:
            VIDEO_LOG("[%s:%d] DVP_IRQ_EVENT_FIFO_WR_FULL", __func__, __LINE__);
            break;

        default:
            //VIDEO_LOG("[%s:%d] DVP error event: %d", __func__, __LINE__, event);
            break;
    }

    error = DVP_GetError(dvp_dev, &error);

    if(error != DVP_ERROR_NONE)
    {
        /* turn off the clock out when the required frames received*/
        DVP_DisableClockout(dvp_dev);
        DVP_Stop(dvp_dev);
        VIDEO_LOG("[%s:%d] DVP Error code: %d", __func__, __LINE__, error);
    }

    return;
}


static uint32_t dvp_eof_cnt_get(void)
{
    return dvp_eof_cnt;
}


static void dvp_eof_cnt_clear(void)
{
    dvp_eof_cnt = 0;
}


static void dvp_reg_dump(void)
{
    VIDEO_LOG("[DVP] F_HOR          *0x%08x = 0x%08x", &IP_DVP_IN->REG_F_HOR.all, IP_DVP_IN->REG_F_HOR.all);
    VIDEO_LOG("[DVP] F_VER          *0x%08x = 0x%08x", &IP_DVP_IN->REG_F_VER.all, IP_DVP_IN->REG_F_VER.all);
    VIDEO_LOG("[DVP] P_OFFSET       *0x%08x = 0x%08x", &IP_DVP_IN->REG_P_OFFSET.all, IP_DVP_IN->REG_P_OFFSET.all);
    VIDEO_LOG("[DVP] L_OFFSET       *0x%08x = 0x%08x", &IP_DVP_IN->REG_L_OFFSET.all, IP_DVP_IN->REG_L_OFFSET.all);
    VIDEO_LOG("[DVP] CTRL           *0x%08x = 0x%08x", &IP_DVP_IN->REG_IMAGE_VIC_CTRL.all, IP_DVP_IN->REG_IMAGE_VIC_CTRL.all);
    //VIDEO_LOG("[DVP] FREQ_OUT       *0x%08x = 0x%08x", &IP_DVP_IN->REG_FREQ_OUT.all, IP_DVP_IN->REG_FREQ_OUT.all);
    VIDEO_LOG("[DVP] INPUT_FORM     *0x%08x = 0x%08x", &IP_DVP_IN->REG_INPUT_FORM.all, IP_DVP_IN->REG_INPUT_FORM.all);
    VIDEO_LOG("[DVP] VIC_EN         *0x%08x = 0x%08x", &IP_DVP_IN->REG_VIC_EN.all, IP_DVP_IN->REG_VIC_EN.all);
    VIDEO_LOG("[DVP] DMA_BURST_THD  *0x%08x = 0x%08x", &IP_DVP_IN->REG_DMA_BURST_THD.all, IP_DVP_IN->REG_DMA_BURST_THD.all);
    VIDEO_LOG("[DVP] INTR_MSK       *0x%08x = 0x%08x", &IP_DVP_IN->REG_IMAGE_VIC_INTR_MASK.all, IP_DVP_IN->REG_IMAGE_VIC_INTR_MASK.all);
    VIDEO_LOG("[DVP] INTR_CLR       *0x%08x = 0x%08x", &IP_DVP_IN->REG_IMAGE_VIC_INTR_CLR.all, IP_DVP_IN->REG_IMAGE_VIC_INTR_CLR.all);
    VIDEO_LOG("[DVP] VIC_IRQ        *0x%08x = 0x%08x", &IP_DVP_IN->REG_IMAGE_VIC_IRQ.all, IP_DVP_IN->REG_IMAGE_VIC_IRQ.all);
    VIDEO_LOG("[DVP] VIC_INT_STATUS *0x%08x = 0x%08x", &IP_DVP_IN->REG_IMAGE_VIC_INT_STATUS.all, IP_DVP_IN->REG_IMAGE_VIC_INT_STATUS.all);
    VIDEO_LOG("[DVP] VIC_INT_RAW    *0x%08x = 0x%08x", &IP_DVP_IN->REG_IMAGE_VIC_INT_RAW_STATUS.all, IP_DVP_IN->REG_IMAGE_VIC_INT_RAW_STATUS.all);
    VIDEO_LOG("[DVP] VIC_DEBUG      *0x%08x = 0x%08x", &IP_DVP_IN->REG_IMAGE_VIC_DEBUG.all, IP_DVP_IN->REG_IMAGE_VIC_DEBUG.all);
    VIDEO_LOG("[DVP] ST_DEBUG       *0x%08x = 0x%08x", &IP_DVP_IN->REG_ST_DEBUG.all, IP_DVP_IN->REG_ST_DEBUG.all);
}


/***************************** I2C *******************************************/

static void* I2C_Handle[2] = {NULL};
static volatile uint32_t I2C_Event[2] = {0};

static inline void CLR_XFER_DONE(uint8_t i2c_index)
{
    I2C_Event[i2c_index] = 0;
}

static inline bool GET_XFER_DONE(uint8_t i2c_index)
{
    return (I2C_Event[i2c_index] & CSK_I2C_EVENT_TRANSFER_DONE);
}


static bool wait_xfer_done_timeout(uint8_t i2c_index, uint32_t max_wait_ms)
{
    bool ret = false;
    uint32_t cnt = 0;

    while(cnt++ < (max_wait_ms * 1000))
    {
        if(GET_XFER_DONE(i2c_index))
        {
            CLR_XFER_DONE(i2c_index);
            ret = true;
            break;
        }
        DELAY_US(1);
    }
    CLR_XFER_DONE(i2c_index);

    return ret;
}


static void I2C0_EventCallback(uint32_t event, void* workspace){
    I2C_Event[0] |= event;
}

static void I2C1_EventCallback(uint32_t event, void* workspace){
    I2C_Event[1] |= event;
}


/**
  * @brief  Initializes Camera low level.
  * @retval None
  */
static int csk_i2c_init(uint8_t i2c_index)
{
    if (i2c_index > 1) {
        VIDEO_LOG("[%s:%d] i2c_index=%d error, must be 0/1", __func__, __LINE__, i2c_index);
        return -1;
    }

    IP_SYSCTRL->REG_PERI_CLK_CFG7.bit.ENA_I2C0_CLK    = 1; // bit 0~0
    IP_SYSCTRL->REG_PERI_CLK_CFG7.bit.ENA_I2C1_CLK    = 1; // bit 1~1

    IOMuxManager_PinConfigure(CSK_IOMUX_PAD_C,  1, CSK_IOMUX_FUNC_ALTER7);   // PIN_DVP_SCL
    IOMuxManager_PinConfigure(CSK_IOMUX_PAD_C,  0, CSK_IOMUX_FUNC_ALTER7);   // PIN_DVP_SDA

    if (i2c_index == 0) {
        I2C_Handle[i2c_index] = I2C0();
        I2C_Initialize(I2C_Handle[i2c_index], I2C0_EventCallback, NULL);
    } else {
        I2C_Handle[i2c_index] = I2C1();
        I2C_Initialize(I2C_Handle[i2c_index], I2C1_EventCallback, NULL);
    }

    I2C_PowerControl(I2C_Handle[i2c_index], CSK_POWER_FULL);

    /* CSK_I2C_TRANSMIT_MODE:  arg0 = 1, means DMA mode ;  arg0 = 0, means Interrupt mode */
    I2C_Control(I2C_Handle[i2c_index], CSK_I2C_TRANSMIT_MODE, 0);
    I2C_Control(I2C_Handle[i2c_index], CSK_I2C_BUS_SPEED, CSK_I2C_BUS_SPEED_STANDARD);  // CSK_I2C_BUS_SPEED_STANDARD/CSK_I2C_BUS_SPEED_FAST
    I2C_Control(I2C_Handle[i2c_index], CSK_I2C_BUS_CLEAR, 0);

    CLR_XFER_DONE(i2c_index);

    return 0;
}


/**
  * @brief  Camera writes single data.
  * @param  Addr: I2C address
  * @param  Reg: Register address
  * @param  Value: Data to be written
  * @retval None
  */
static int csk_i2c_write_reg8(uint8_t i2c_index, uint16_t addr, uint8_t reg, uint8_t value)
{
    int32_t ret = 0;
    uint8_t i2c_data[2];

    if (i2c_index > 1) {
        VIDEO_LOG("[%s:%d] i2c_index=%d error, must be 0/1", __func__, __LINE__, i2c_index);
        return -1;
    }

    if (I2C_Handle[i2c_index] == NULL) {
        VIDEO_LOG("[%s:%d] i2c_index=%d handle is NULL, not init", __func__, __LINE__, i2c_index);
        return -1;
    }

    CLR_XFER_DONE(i2c_index);

    i2c_data[0] = reg;
    i2c_data[1] = value;
    ret = I2C_MasterTransmit(I2C_Handle[i2c_index], addr, i2c_data, 2, 0);
    if (CSK_DRIVER_OK != ret) {
        VIDEO_LOG("[%s:%d] I2C error ret=%d", __func__, __LINE__, ret);
        return -1;
    }
    if (!wait_xfer_done_timeout(i2c_index, 1000)) { // 1000ms
        VIDEO_LOG("[%s:%d] I2C transfer is timeout!!", __func__, __LINE__);
        return -1;
    }

    return 0;
}


/**
  * @brief  Camera reads single data.
  * @param  Addr: I2C address
  * @param  Reg: Register address
  * @retval Read data number
  */
static int csk_i2c_read(uint8_t i2c_index, uint16_t addr, uint8_t reg, uint8_t *data, uint16_t num)
{
    int32_t ret = 0;

    if (i2c_index > 1) {
        VIDEO_LOG("[%s:%d] i2c_index=%d error, must be 0/1", __func__, __LINE__, i2c_index);
        return -1;
    }

    if (I2C_Handle[i2c_index] == NULL) {
        VIDEO_LOG("[%s:%d] i2c_index=%d handle is NULL, not init", __func__, __LINE__, i2c_index);
        return -1;
    }

    CLR_XFER_DONE(i2c_index);

    ret = I2C_MasterTransmit(I2C_Handle[i2c_index], addr, &reg, 1, 1);
    if (CSK_DRIVER_OK != ret) {
        VIDEO_LOG("[%s:%d] I2C error ret=%d", __func__, __LINE__, ret);
        return -1;
    }
    if (!wait_xfer_done_timeout(i2c_index, 1000)) { // 1000ms
        VIDEO_LOG("[%s:%d] I2C transfer is timeout!!", __func__, __LINE__);
        return -1;
    }

    ret = I2C_MasterReceive(I2C_Handle[i2c_index], addr, data, num, 0);
    if (CSK_DRIVER_OK != ret) {
        VIDEO_LOG("[%s:%d] I2C error ret=%d", __func__, __LINE__, ret);
        return -1;
    }
    if (!wait_xfer_done_timeout(i2c_index, 1000)) { // 1000ms
        VIDEO_LOG("[%s:%d] I2C transfer is timeout!!", __func__, __LINE__);
        return -1;
    }

    return num;
}


/**
  * @brief  Camera reads single data.
  * @param  Addr: I2C address
  * @param  Reg: Register address
  * @retval Read data number
  */
static uint8_t csk_i2c_read_reg8(uint8_t i2c_index, uint16_t addr, uint8_t reg)
{
    uint8_t data;

    if(csk_i2c_read(i2c_index, addr, reg, &data, 1)) {
        return data;
    } else {
        return 0;
    }
}


static void gc0328_init(uint8_t colorbar_enable)
{
    uint8_t data = 0;
    uint8_t i2c_index = 0;

    csk_i2c_init(i2c_index);
    DELAY_MS(10);

    /* chip id */
    data = csk_i2c_read_reg8(i2c_index, 0x21, 0xf0);
    if(data != 0x9D) {
        VIDEO_LOG("[%s:%d] not detect gc0328, read=0x%x", __func__, __LINE__, data);
    } else {
        VIDEO_LOG("[%s:%d] detect gc0328", __func__, __LINE__);
    }

    /* sw reset */
    csk_i2c_write_reg8(i2c_index, 0x21, 0xfe, 0xf0);
    DELAY_MS(100);

    csk_i2c_write_reg8(i2c_index, 0x21, 0xfe, 0x80);
    csk_i2c_write_reg8(i2c_index, 0x21, 0xfe, 0x80);
    csk_i2c_write_reg8(i2c_index, 0x21, 0xfc, 0x16);
    csk_i2c_write_reg8(i2c_index, 0x21, 0xfc, 0x16);
    csk_i2c_write_reg8(i2c_index, 0x21, 0xfc, 0x16);
    csk_i2c_write_reg8(i2c_index, 0x21, 0xfc, 0x16);
    csk_i2c_write_reg8(i2c_index, 0x21, 0xfe, 0x0 );
    csk_i2c_write_reg8(i2c_index, 0x21, 0x4f, 0x0 );
    csk_i2c_write_reg8(i2c_index, 0x21, 0x42, 0x0 );
    csk_i2c_write_reg8(i2c_index, 0x21, 0x3 ,0x0  );
    csk_i2c_write_reg8(i2c_index, 0x21, 0x4 ,0xc0 );
    csk_i2c_write_reg8(i2c_index, 0x21, 0x77, 0x62);
    csk_i2c_write_reg8(i2c_index, 0x21, 0x78, 0x40);
    csk_i2c_write_reg8(i2c_index, 0x21, 0x79, 0x4d);
    csk_i2c_write_reg8(i2c_index, 0x21, 0x5 ,0x1  );
    csk_i2c_write_reg8(i2c_index, 0x21, 0x6 ,0x32 );
    csk_i2c_write_reg8(i2c_index, 0x21, 0x7 ,0x0  );
    csk_i2c_write_reg8(i2c_index, 0x21, 0x8 ,0xc  );
    csk_i2c_write_reg8(i2c_index, 0x21, 0xfe, 0x1 );
    csk_i2c_write_reg8(i2c_index, 0x21, 0x29, 0x0 );
    csk_i2c_write_reg8(i2c_index, 0x21, 0x2a, 0x78);
    csk_i2c_write_reg8(i2c_index, 0x21, 0x2b, 0x1 );
    csk_i2c_write_reg8(i2c_index, 0x21, 0x2c, 0xe0);
    csk_i2c_write_reg8(i2c_index, 0x21, 0x2d, 0x1 );
    csk_i2c_write_reg8(i2c_index, 0x21, 0x2e, 0xe0);
    csk_i2c_write_reg8(i2c_index, 0x21, 0x2f, 0x1 );
    csk_i2c_write_reg8(i2c_index, 0x21, 0x30, 0xe0);
    csk_i2c_write_reg8(i2c_index, 0x21, 0x31, 0x1 );
    csk_i2c_write_reg8(i2c_index, 0x21, 0x32, 0xe0);
    csk_i2c_write_reg8(i2c_index, 0x21, 0xfe, 0x0 );
    csk_i2c_write_reg8(i2c_index, 0x21, 0xfe, 0x1 );
    csk_i2c_write_reg8(i2c_index, 0x21, 0x4f, 0x0 );
    csk_i2c_write_reg8(i2c_index, 0x21, 0x4c, 0x1 );
    csk_i2c_write_reg8(i2c_index, 0x21, 0xfe, 0x0 );
    csk_i2c_write_reg8(i2c_index, 0x21, 0xfe, 0x1 );
    csk_i2c_write_reg8(i2c_index, 0x21, 0x51, 0x80);
    csk_i2c_write_reg8(i2c_index, 0x21, 0x52, 0x12);
    csk_i2c_write_reg8(i2c_index, 0x21, 0x53, 0x80);
    csk_i2c_write_reg8(i2c_index, 0x21, 0x54, 0x60);
    csk_i2c_write_reg8(i2c_index, 0x21, 0x55, 0x1 );
    csk_i2c_write_reg8(i2c_index, 0x21, 0x56, 0x6 );
    csk_i2c_write_reg8(i2c_index, 0x21, 0x5b, 0x2 );
    csk_i2c_write_reg8(i2c_index, 0x21, 0x61, 0xdc);
    csk_i2c_write_reg8(i2c_index, 0x21, 0x62, 0xdc);
    csk_i2c_write_reg8(i2c_index, 0x21, 0x7c, 0x71);
    csk_i2c_write_reg8(i2c_index, 0x21, 0x7d, 0x0 );
    csk_i2c_write_reg8(i2c_index, 0x21, 0x76, 0x0 );
    csk_i2c_write_reg8(i2c_index, 0x21, 0x79, 0x20);
    csk_i2c_write_reg8(i2c_index, 0x21, 0x7b, 0x0 );
    csk_i2c_write_reg8(i2c_index, 0x21, 0x70, 0xff);
    csk_i2c_write_reg8(i2c_index, 0x21, 0x71, 0x0 );
    csk_i2c_write_reg8(i2c_index, 0x21, 0x72, 0x10);
    csk_i2c_write_reg8(i2c_index, 0x21, 0x73, 0x40);
    csk_i2c_write_reg8(i2c_index, 0x21, 0x74, 0x40);
    csk_i2c_write_reg8(i2c_index, 0x21, 0x50, 0x0 );
    csk_i2c_write_reg8(i2c_index, 0x21, 0xfe, 0x1 );
    csk_i2c_write_reg8(i2c_index, 0x21, 0x4f, 0x0 );
    csk_i2c_write_reg8(i2c_index, 0x21, 0x4c, 0x1 );
    csk_i2c_write_reg8(i2c_index, 0x21, 0x4f, 0x0 );
    csk_i2c_write_reg8(i2c_index, 0x21, 0x4f, 0x0 );
    csk_i2c_write_reg8(i2c_index, 0x21, 0x4f, 0x0 );
    csk_i2c_write_reg8(i2c_index, 0x21, 0x4d, 0x36);
    csk_i2c_write_reg8(i2c_index, 0x21, 0x4e, 0x2 );
    csk_i2c_write_reg8(i2c_index, 0x21, 0x4e, 0x2 );
    csk_i2c_write_reg8(i2c_index, 0x21, 0x4d, 0x44);
    csk_i2c_write_reg8(i2c_index, 0x21, 0x4e, 0x2 );
    csk_i2c_write_reg8(i2c_index, 0x21, 0x4e, 0x2 );
    csk_i2c_write_reg8(i2c_index, 0x21, 0x4e, 0x2 );
    csk_i2c_write_reg8(i2c_index, 0x21, 0x4e, 0x2 );
    csk_i2c_write_reg8(i2c_index, 0x21, 0x4d, 0x53);
    csk_i2c_write_reg8(i2c_index, 0x21, 0x4e, 0x8 );
    csk_i2c_write_reg8(i2c_index, 0x21, 0x4e, 0x8 );
    csk_i2c_write_reg8(i2c_index, 0x21, 0x4e, 0x2 );
    csk_i2c_write_reg8(i2c_index, 0x21, 0x4d, 0x63);
    csk_i2c_write_reg8(i2c_index, 0x21, 0x4e, 0x8 );
    csk_i2c_write_reg8(i2c_index, 0x21, 0x4e, 0x8 );
    csk_i2c_write_reg8(i2c_index, 0x21, 0x4d, 0x73);
    csk_i2c_write_reg8(i2c_index, 0x21, 0x4e, 0x20);
    csk_i2c_write_reg8(i2c_index, 0x21, 0x4d, 0x83);
    csk_i2c_write_reg8(i2c_index, 0x21, 0x4e, 0x20);
    csk_i2c_write_reg8(i2c_index, 0x21, 0x4f, 0x1 );
    csk_i2c_write_reg8(i2c_index, 0x21, 0x50, 0x88);
    csk_i2c_write_reg8(i2c_index, 0x21, 0xfe, 0x0 );
    csk_i2c_write_reg8(i2c_index, 0x21, 0x27, 0x0 );
    csk_i2c_write_reg8(i2c_index, 0x21, 0x2a, 0x40);
    csk_i2c_write_reg8(i2c_index, 0x21, 0x2b, 0x40);
    csk_i2c_write_reg8(i2c_index, 0x21, 0x2c, 0x40);
    csk_i2c_write_reg8(i2c_index, 0x21, 0x2d, 0x40);
    csk_i2c_write_reg8(i2c_index, 0x21, 0xfe, 0x0 );
    csk_i2c_write_reg8(i2c_index, 0x21, 0x9 ,0x0  );
    csk_i2c_write_reg8(i2c_index, 0x21, 0xa ,0x0  );
    csk_i2c_write_reg8(i2c_index, 0x21, 0xb ,0x0  );
    csk_i2c_write_reg8(i2c_index, 0x21, 0xc ,0x0  );
    csk_i2c_write_reg8(i2c_index, 0x21, 0xd ,0x1  );
    csk_i2c_write_reg8(i2c_index, 0x21, 0xe ,0xe8 );
    csk_i2c_write_reg8(i2c_index, 0x21, 0xf ,0x2  );
    csk_i2c_write_reg8(i2c_index, 0x21, 0x10, 0x88);
    csk_i2c_write_reg8(i2c_index, 0x21, 0x16, 0x0 );
    csk_i2c_write_reg8(i2c_index, 0x21, 0x17, 0x14);
    csk_i2c_write_reg8(i2c_index, 0x21, 0x18, 0xe );
    csk_i2c_write_reg8(i2c_index, 0x21, 0x19, 0x6 );
    csk_i2c_write_reg8(i2c_index, 0x21, 0x1b, 0x48);
    csk_i2c_write_reg8(i2c_index, 0x21, 0x1f, 0xc8);
    csk_i2c_write_reg8(i2c_index, 0x21, 0x20, 0x1 );
    csk_i2c_write_reg8(i2c_index, 0x21, 0x21, 0x78);
    csk_i2c_write_reg8(i2c_index, 0x21, 0x22, 0xb0);
    csk_i2c_write_reg8(i2c_index, 0x21, 0x23, 0x4 );
    csk_i2c_write_reg8(i2c_index, 0x21, 0x24, 0x3f);
    csk_i2c_write_reg8(i2c_index, 0x21, 0x26, 0x0 );
    csk_i2c_write_reg8(i2c_index, 0x21, 0x50, 0x1 );
    csk_i2c_write_reg8(i2c_index, 0x21, 0x70, 0x85);
    csk_i2c_write_reg8(i2c_index, 0x21, 0x40, 0x7f);
    csk_i2c_write_reg8(i2c_index, 0x21, 0x41, 0x26);
    csk_i2c_write_reg8(i2c_index, 0x21, 0x42, 0xff);
    csk_i2c_write_reg8(i2c_index, 0x21, 0x45, 0x0 );
    csk_i2c_write_reg8(i2c_index, 0x21, 0x44, 0x6 );
    csk_i2c_write_reg8(i2c_index, 0x21, 0x46, 0x2 );
    csk_i2c_write_reg8(i2c_index, 0x21, 0x4b, 0x1 );
    csk_i2c_write_reg8(i2c_index, 0x21, 0x50, 0x1 );
    csk_i2c_write_reg8(i2c_index, 0x21, 0x7e, 0xa );
    csk_i2c_write_reg8(i2c_index, 0x21, 0x7f, 0x3 );
    csk_i2c_write_reg8(i2c_index, 0x21, 0x80, 0x27);
    csk_i2c_write_reg8(i2c_index, 0x21, 0x81, 0x15);
    csk_i2c_write_reg8(i2c_index, 0x21, 0x82, 0x90);
    csk_i2c_write_reg8(i2c_index, 0x21, 0x83, 0x2 );
    csk_i2c_write_reg8(i2c_index, 0x21, 0x84, 0x23);
    csk_i2c_write_reg8(i2c_index, 0x21, 0x90, 0x2c);
    csk_i2c_write_reg8(i2c_index, 0x21, 0x92, 0x2 );
    csk_i2c_write_reg8(i2c_index, 0x21, 0x94, 0x2 );
    csk_i2c_write_reg8(i2c_index, 0x21, 0x95, 0x35);
    csk_i2c_write_reg8(i2c_index, 0x21, 0xd1, 0x32);
    csk_i2c_write_reg8(i2c_index, 0x21, 0xd2, 0x32);
    csk_i2c_write_reg8(i2c_index, 0x21, 0xdd, 0x18);
    csk_i2c_write_reg8(i2c_index, 0x21, 0xde, 0x32);
    csk_i2c_write_reg8(i2c_index, 0x21, 0xe4, 0x88);
    csk_i2c_write_reg8(i2c_index, 0x21, 0xe5, 0x40);
    csk_i2c_write_reg8(i2c_index, 0x21, 0xd7, 0xe );
    csk_i2c_write_reg8(i2c_index, 0x21, 0xfe, 0x0 );
    csk_i2c_write_reg8(i2c_index, 0x21, 0xbf, 0x10);
    csk_i2c_write_reg8(i2c_index, 0x21, 0xc0, 0x1c);
    csk_i2c_write_reg8(i2c_index, 0x21, 0xc1, 0x33);
    csk_i2c_write_reg8(i2c_index, 0x21, 0xc2, 0x48);
    csk_i2c_write_reg8(i2c_index, 0x21, 0xc3, 0x5a);
    csk_i2c_write_reg8(i2c_index, 0x21, 0xc4, 0x6b);
    csk_i2c_write_reg8(i2c_index, 0x21, 0xc5, 0x7b);
    csk_i2c_write_reg8(i2c_index, 0x21, 0xc6, 0x95);
    csk_i2c_write_reg8(i2c_index, 0x21, 0xc7, 0xab);
    csk_i2c_write_reg8(i2c_index, 0x21, 0xc8, 0xbf);
    csk_i2c_write_reg8(i2c_index, 0x21, 0xc9, 0xcd);
    csk_i2c_write_reg8(i2c_index, 0x21, 0xca, 0xd9);
    csk_i2c_write_reg8(i2c_index, 0x21, 0xcb, 0xe3);
    csk_i2c_write_reg8(i2c_index, 0x21, 0xcc, 0xeb);
    csk_i2c_write_reg8(i2c_index, 0x21, 0xcd, 0xf7);
    csk_i2c_write_reg8(i2c_index, 0x21, 0xce, 0xfd);
    csk_i2c_write_reg8(i2c_index, 0x21, 0xcf, 0xff);
    csk_i2c_write_reg8(i2c_index, 0x21, 0xfe, 0x0 );
    csk_i2c_write_reg8(i2c_index, 0x21, 0x63, 0x0 );
    csk_i2c_write_reg8(i2c_index, 0x21, 0x64, 0x5 );
    csk_i2c_write_reg8(i2c_index, 0x21, 0x65, 0xc );
    csk_i2c_write_reg8(i2c_index, 0x21, 0x66, 0x1a);
    csk_i2c_write_reg8(i2c_index, 0x21, 0x67, 0x29);
    csk_i2c_write_reg8(i2c_index, 0x21, 0x68, 0x39);
    csk_i2c_write_reg8(i2c_index, 0x21, 0x69, 0x4b);
    csk_i2c_write_reg8(i2c_index, 0x21, 0x6a, 0x5e);
    csk_i2c_write_reg8(i2c_index, 0x21, 0x6b, 0x82);
    csk_i2c_write_reg8(i2c_index, 0x21, 0x6c, 0xa4);
    csk_i2c_write_reg8(i2c_index, 0x21, 0x6d, 0xc5);
    csk_i2c_write_reg8(i2c_index, 0x21, 0x6e, 0xe5);
    csk_i2c_write_reg8(i2c_index, 0x21, 0x6f, 0xff);
    csk_i2c_write_reg8(i2c_index, 0x21, 0xfe, 0x1 );
    csk_i2c_write_reg8(i2c_index, 0x21, 0x18, 0x2 );
    csk_i2c_write_reg8(i2c_index, 0x21, 0xfe, 0x0 );
    csk_i2c_write_reg8(i2c_index, 0x21, 0x98, 0x0 );
    csk_i2c_write_reg8(i2c_index, 0x21, 0x9b, 0x20);
    csk_i2c_write_reg8(i2c_index, 0x21, 0x9c, 0x80);
    csk_i2c_write_reg8(i2c_index, 0x21, 0xa4, 0x10);
    csk_i2c_write_reg8(i2c_index, 0x21, 0xa8, 0xb0);
    csk_i2c_write_reg8(i2c_index, 0x21, 0xaa, 0x40);
    csk_i2c_write_reg8(i2c_index, 0x21, 0xa2, 0x23);
    csk_i2c_write_reg8(i2c_index, 0x21, 0xad, 0x1 );
    csk_i2c_write_reg8(i2c_index, 0x21, 0xfe, 0x1 );
    csk_i2c_write_reg8(i2c_index, 0x21, 0x9c, 0x2 );
    csk_i2c_write_reg8(i2c_index, 0x21, 0x8 ,0xa0 );
    csk_i2c_write_reg8(i2c_index, 0x21, 0x9 ,0xe8 );
    csk_i2c_write_reg8(i2c_index, 0x21, 0x10, 0x0 );
    csk_i2c_write_reg8(i2c_index, 0x21, 0x11, 0x11);
    csk_i2c_write_reg8(i2c_index, 0x21, 0x12, 0x10);
    csk_i2c_write_reg8(i2c_index, 0x21, 0x13, 0x80);
    csk_i2c_write_reg8(i2c_index, 0x21, 0x15, 0xfc);
    csk_i2c_write_reg8(i2c_index, 0x21, 0x18, 0x3 );
    csk_i2c_write_reg8(i2c_index, 0x21, 0x21, 0xc0);
    csk_i2c_write_reg8(i2c_index, 0x21, 0x22, 0x60);
    csk_i2c_write_reg8(i2c_index, 0x21, 0x23, 0x30);
    csk_i2c_write_reg8(i2c_index, 0x21, 0x25, 0x0 );
    csk_i2c_write_reg8(i2c_index, 0x21, 0x24, 0x14);
    csk_i2c_write_reg8(i2c_index, 0x21, 0xfe, 0x1 );
    csk_i2c_write_reg8(i2c_index, 0x21, 0xc0, 0x10);
    csk_i2c_write_reg8(i2c_index, 0x21, 0xc1, 0xc );
    csk_i2c_write_reg8(i2c_index, 0x21, 0xc2, 0xa );
    csk_i2c_write_reg8(i2c_index, 0x21, 0xc6, 0xe );
    csk_i2c_write_reg8(i2c_index, 0x21, 0xc7, 0xb );
    csk_i2c_write_reg8(i2c_index, 0x21, 0xc8, 0xa );
    csk_i2c_write_reg8(i2c_index, 0x21, 0xba, 0x26);
    csk_i2c_write_reg8(i2c_index, 0x21, 0xbb, 0x1c);
    csk_i2c_write_reg8(i2c_index, 0x21, 0xbc, 0x1d);
    csk_i2c_write_reg8(i2c_index, 0x21, 0xb4, 0x23);
    csk_i2c_write_reg8(i2c_index, 0x21, 0xb5, 0x1c);
    csk_i2c_write_reg8(i2c_index, 0x21, 0xb6, 0x1a);
    csk_i2c_write_reg8(i2c_index, 0x21, 0xc3, 0x0 );
    csk_i2c_write_reg8(i2c_index, 0x21, 0xc4, 0x0 );
    csk_i2c_write_reg8(i2c_index, 0x21, 0xc5, 0x0 );
    csk_i2c_write_reg8(i2c_index, 0x21, 0xc9, 0x0 );
    csk_i2c_write_reg8(i2c_index, 0x21, 0xca, 0x0 );
    csk_i2c_write_reg8(i2c_index, 0x21, 0xcb, 0x0 );
    csk_i2c_write_reg8(i2c_index, 0x21, 0xbd, 0x0 );
    csk_i2c_write_reg8(i2c_index, 0x21, 0xbe, 0x0 );
    csk_i2c_write_reg8(i2c_index, 0x21, 0xbf, 0x0 );
    csk_i2c_write_reg8(i2c_index, 0x21, 0xb7, 0x7 );
    csk_i2c_write_reg8(i2c_index, 0x21, 0xb8, 0x5 );
    csk_i2c_write_reg8(i2c_index, 0x21, 0xb9, 0x5 );
    csk_i2c_write_reg8(i2c_index, 0x21, 0xa8, 0x7 );
    csk_i2c_write_reg8(i2c_index, 0x21, 0xa9, 0x6 );
    csk_i2c_write_reg8(i2c_index, 0x21, 0xaa, 0x0 );
    csk_i2c_write_reg8(i2c_index, 0x21, 0xab, 0x4 );
    csk_i2c_write_reg8(i2c_index, 0x21, 0xac, 0x0 );
    csk_i2c_write_reg8(i2c_index, 0x21, 0xad, 0x2 );
    csk_i2c_write_reg8(i2c_index, 0x21, 0xae, 0xd );
    csk_i2c_write_reg8(i2c_index, 0x21, 0xaf, 0x5 );
    csk_i2c_write_reg8(i2c_index, 0x21, 0xb0, 0x0 );
    csk_i2c_write_reg8(i2c_index, 0x21, 0xb1, 0x7 );
    csk_i2c_write_reg8(i2c_index, 0x21, 0xb2, 0x3 );
    csk_i2c_write_reg8(i2c_index, 0x21, 0xb3, 0x0 );
    csk_i2c_write_reg8(i2c_index, 0x21, 0xa4, 0x0 );
    csk_i2c_write_reg8(i2c_index, 0x21, 0xa5, 0x0 );
    csk_i2c_write_reg8(i2c_index, 0x21, 0xa6, 0x0 );
    csk_i2c_write_reg8(i2c_index, 0x21, 0xa7, 0x0 );
    csk_i2c_write_reg8(i2c_index, 0x21, 0xa1, 0x3c);
    csk_i2c_write_reg8(i2c_index, 0x21, 0xa2, 0x50);
    csk_i2c_write_reg8(i2c_index, 0x21, 0xfe, 0x0 );
    csk_i2c_write_reg8(i2c_index, 0x21, 0xb1, 0x4 );
    csk_i2c_write_reg8(i2c_index, 0x21, 0xb2, 0xfd);
    csk_i2c_write_reg8(i2c_index, 0x21, 0xb3, 0xfc);
    csk_i2c_write_reg8(i2c_index, 0x21, 0xb4, 0xf0);
    csk_i2c_write_reg8(i2c_index, 0x21, 0xb5, 0x5 );
    csk_i2c_write_reg8(i2c_index, 0x21, 0xb6, 0xf0);
    csk_i2c_write_reg8(i2c_index, 0x21, 0xfe, 0x0 );
    csk_i2c_write_reg8(i2c_index, 0x21, 0x27, 0xf7);
    csk_i2c_write_reg8(i2c_index, 0x21, 0x28, 0x7f);
    csk_i2c_write_reg8(i2c_index, 0x21, 0x29, 0x20);
    csk_i2c_write_reg8(i2c_index, 0x21, 0x33, 0x20);
    csk_i2c_write_reg8(i2c_index, 0x21, 0x34, 0x20);
    csk_i2c_write_reg8(i2c_index, 0x21, 0x35, 0x20);
    csk_i2c_write_reg8(i2c_index, 0x21, 0x36, 0x20);
    csk_i2c_write_reg8(i2c_index, 0x21, 0x32, 0x8 );
    csk_i2c_write_reg8(i2c_index, 0x21, 0x47, 0x0 );
    csk_i2c_write_reg8(i2c_index, 0x21, 0x48, 0x0 );
    csk_i2c_write_reg8(i2c_index, 0x21, 0xfe, 0x1 );
    csk_i2c_write_reg8(i2c_index, 0x21, 0x79, 0x0 );
    csk_i2c_write_reg8(i2c_index, 0x21, 0x7d, 0x0 );
    csk_i2c_write_reg8(i2c_index, 0x21, 0x50, 0x88);
    csk_i2c_write_reg8(i2c_index, 0x21, 0x5b, 0xc );
    csk_i2c_write_reg8(i2c_index, 0x21, 0x76, 0x8f);
    csk_i2c_write_reg8(i2c_index, 0x21, 0x80, 0x70);
    csk_i2c_write_reg8(i2c_index, 0x21, 0x81, 0x70);
    csk_i2c_write_reg8(i2c_index, 0x21, 0x82, 0xb0);
    csk_i2c_write_reg8(i2c_index, 0x21, 0x70, 0xff);
    csk_i2c_write_reg8(i2c_index, 0x21, 0x71, 0x0 );
    csk_i2c_write_reg8(i2c_index, 0x21, 0x72, 0x28);
    csk_i2c_write_reg8(i2c_index, 0x21, 0x73, 0xb );
    csk_i2c_write_reg8(i2c_index, 0x21, 0x74, 0xb );
    csk_i2c_write_reg8(i2c_index, 0x21, 0xfe, 0x0 );
    csk_i2c_write_reg8(i2c_index, 0x21, 0x70, 0x45);
    csk_i2c_write_reg8(i2c_index, 0x21, 0x4f, 0x1 );
    csk_i2c_write_reg8(i2c_index, 0x21, 0xf1, 0x7 );
    csk_i2c_write_reg8(i2c_index, 0x21, 0xf2, 0x1 );
    DELAY_MS(100);
    csk_i2c_write_reg8(i2c_index, 0x21, 0xfe, 0x0);


#if 1
    /* Setting frame size to 320x240 */
    csk_i2c_write_reg8(i2c_index, 0x21, 0xfe, 0x0);
    csk_i2c_write_reg8(i2c_index, 0x21, 0x5 ,0x0  );  // HB high  // 0x01
    csk_i2c_write_reg8(i2c_index, 0x21, 0x6 ,0x80 );  // HB low  // 0x32
    csk_i2c_write_reg8(i2c_index, 0x21, 0x7 ,0x0  );  // VB  // 0x00
    csk_i2c_write_reg8(i2c_index, 0x21, 0x8 ,0xc  );  // VB  // 0x0c
    csk_i2c_write_reg8(i2c_index, 0x21, 0x9 ,0x0  );  // Row start  // 0x00
    csk_i2c_write_reg8(i2c_index, 0x21, 0xa ,0x0  );  // Row start  // 0x00
    csk_i2c_write_reg8(i2c_index, 0x21, 0xb ,0x0  );  // Col start  // 0x00
    csk_i2c_write_reg8(i2c_index, 0x21, 0xc ,0x0  );  // Col start  // 0x00
    csk_i2c_write_reg8(i2c_index, 0x21, 0xd ,0x1  );  // Window height  // 0x01
    csk_i2c_write_reg8(i2c_index, 0x21, 0xe ,0x00 );  // Window height  // 0xe8
    csk_i2c_write_reg8(i2c_index, 0x21, 0xf ,0x1  );  // Window width  // 0x02
    csk_i2c_write_reg8(i2c_index, 0x21, 0x10,0x60);   // Window width  // 0x88

    csk_i2c_write_reg8(i2c_index, 0x21, 0xfe, 0x0);
    csk_i2c_write_reg8(i2c_index, 0x21, 0x51, 0x0);  // Crop _win_y1
    csk_i2c_write_reg8(i2c_index, 0x21, 0x52, 0x0);  // Crop _win_y1  // 0x78
    csk_i2c_write_reg8(i2c_index, 0x21, 0x53, 0x0);  // Crop _win_x1
    csk_i2c_write_reg8(i2c_index, 0x21, 0x54, 0x0);  // Crop _win_x1  // 0xa0
    csk_i2c_write_reg8(i2c_index, 0x21, 0x55, 0x0);  // Crop_win_height
    csk_i2c_write_reg8(i2c_index, 0x21, 0x56, 0xf0); // Crop_win_height
    csk_i2c_write_reg8(i2c_index, 0x21, 0x57, 0x1);  // Crop_win_width
    csk_i2c_write_reg8(i2c_index, 0x21, 0x58, 0x40); // Crop_win_width
#else
    /* Setting frame size to 640x480 */
    csk_i2c_write_reg8(i2c_index, 0x21, 0xfe, 0x0);
    csk_i2c_write_reg8(i2c_index, 0x21, 0x5 ,0x1  );
    csk_i2c_write_reg8(i2c_index, 0x21, 0x6 ,0x32 );
    csk_i2c_write_reg8(i2c_index, 0x21, 0x7 ,0x0  );
    csk_i2c_write_reg8(i2c_index, 0x21, 0x8 ,0xc  );
    csk_i2c_write_reg8(i2c_index, 0x21, 0x9 ,0x0  );
    csk_i2c_write_reg8(i2c_index, 0x21, 0xa ,0x0  );
    csk_i2c_write_reg8(i2c_index, 0x21, 0xb ,0x0  );
    csk_i2c_write_reg8(i2c_index, 0x21, 0xc ,0x0  );
    csk_i2c_write_reg8(i2c_index, 0x21, 0xd ,0x1  );
    csk_i2c_write_reg8(i2c_index, 0x21, 0xe ,0xe8 );
    csk_i2c_write_reg8(i2c_index, 0x21, 0xf ,0x2  );
    csk_i2c_write_reg8(i2c_index, 0x21, 0x10, 0x88);

    csk_i2c_write_reg8(i2c_index, 0x21, 0xfe, 0x0);
    csk_i2c_write_reg8(i2c_index, 0x21, 0x51, 0x0);
    csk_i2c_write_reg8(i2c_index, 0x21, 0x52, 0x0);
    csk_i2c_write_reg8(i2c_index, 0x21, 0x53, 0x0);
    csk_i2c_write_reg8(i2c_index, 0x21, 0x54, 0x0);
    csk_i2c_write_reg8(i2c_index, 0x21, 0x55, 0x01);
    csk_i2c_write_reg8(i2c_index, 0x21, 0x56, 0xe0);
    csk_i2c_write_reg8(i2c_index, 0x21, 0x57, 0x02);
    csk_i2c_write_reg8(i2c_index, 0x21, 0x58, 0x80);
#endif    

    /* Set pixel format */
    csk_i2c_write_reg8(i2c_index, 0x21, 0xfe, 0x0);
    csk_i2c_write_reg8(i2c_index, 0x21, 0x44, 0x2);   // 0x00:YUV422-CbYCrY 0x01:CrYCbY 0x02:YCbYCr 0x03:YCrYCb  0x06:RGB565

    /* colorbar 0x4C bit0 */
    if(colorbar_enable) {
        csk_i2c_write_reg8(i2c_index, 0x21, 0xfe, 0x0);
        csk_i2c_write_reg8(i2c_index, 0x21, 0x4c, 0x1);
    }
}

