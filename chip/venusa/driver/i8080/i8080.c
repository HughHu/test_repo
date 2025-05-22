#if 0
#include <string.h>
#include <stdint.h>
#include "chip.h"
#include "ClockManager.h"
#include "log_print.h"
#include "i8080.h"


#define DEBUG_LOG   1
#if DEBUG_LOG
#define DEV_LOG(format, ...)   CLOGD(format, ##__VA_ARGS__)
#else
#define DEV_LOG(format, ...)
#endif // DEBUG_LOG


// driver version
#define CSK_I8080_DRV_VERSION CSK_DRIVER_VERSION_MAJOR_MINOR(1,0)

static const
CSK_DRIVER_VERSION i8080_driver_version = { CSK_I8080_API_VERSION, CSK_I8080_DRV_VERSION };

//------------------------------------------------------------------------------------------


_FAST_DATA_VI static I8080_DEV i80800_dev = {
        ((I8080_OUT_RegDef *)(I8080_BASE)),
        {0},
        NULL,
        0,
        0,
};


//------------------------------------------------------------------------------------
static void I8080_Reset(void)
{
//    IP_AP_CFG->REG_CLK_CFG0.bit.ENA_VIDEO_CLK = 1;  // bit15
//    IP_AP_CFG->REG_CLK_CFG0.bit.ENA_I8080_CLK = 1;    // bit23
//    IP_AP_CFG->REG_SW_RESET.bit.I8080_RESET = 0x1;    // bit12
//    IP_AP_CFG->REG_DMA_SEL.bit.DMA_SEL = 0;         // bit0  0:i8080  1:qspi_out
}


static I8080_DEV * safe_i8080_dev(void *i8080_dev)
{
    //TODO: safe check of I8080 device parameter
    if (i8080_dev == I80800()) {
        if (((I8080_DEV *)i8080_dev)->Instance != ((I8080_OUT_RegDef *)(I8080_BASE))) {
            //CLOGW("I80800 device context has been tampered illegally!!\n");
            DEV_LOG("[%s:%d] Error i8080_dev=%#x ", __func__, __LINE__, i8080_dev);
            return NULL;
        }
    } else {
        DEV_LOG("[%s:%d] Error i8080_dev=%#x ", __func__, __LINE__, i8080_dev);
        return NULL;
    }

    return (I8080_DEV *)i8080_dev;
}

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/**
  * @brief  Handles I8080 interrupt request.
  * @retval None
  */
_FAST_FUNC_RO void I8080_IRQ_Handler()
{
    I8080_DEV *i8080_dev = (I8080_DEV *)I80800();
    uint32_t status = i8080_dev->Instance->REG_I8080_INT_STATUS.all;
    DEV_LOG("[%s:%d] status=0x%x", __func__, __LINE__, status);

     /* Frame complete interrupt management */
    if((status & I8080_OUT_I8080_INT_STATUS_TRANS_END_ISR_Msk) == I8080_OUT_I8080_INT_STATUS_TRANS_END_ISR_Msk)
    {
        i8080_dev->Instance->REG_I8080_INTR_CLR.bit.TRANS_END_CLR = 1;

        if(i8080_dev->cb_event)
            i8080_dev->cb_event(I8080_IRQ_EVENT_SOF, 0);
    }

    if((status & I8080_OUT_I8080_INT_STATUS_TXFIFO_RD_EMPTY_ISR_Msk) == I8080_OUT_I8080_INT_STATUS_TXFIFO_RD_EMPTY_ISR_Msk)
    {
        i8080_dev->Instance->REG_I8080_INTR_CLR.bit.TXFIFO_RD_EMPTY_CLR = 1;

        /* Update error code */
        i8080_dev->ErrorCode |= I8080_ERROR_FIFO_WR_FULL;

        /* Change the state */
        i8080_dev->State = I8080_STATE_ERROR;

        if(i8080_dev->cb_event)
            i8080_dev->cb_event(I8080_IRQ_EVENT_FIFO_WR_FULL, 0);
    }

    if((status & I8080_OUT_I8080_INT_STATUS_TXFIFO_WR_FULL_ISR_Msk) == I8080_OUT_I8080_INT_STATUS_TXFIFO_WR_FULL_ISR_Msk)
    {
        i8080_dev->Instance->REG_I8080_INTR_CLR.bit.TXFIFO_WR_FULL_CLR = 1;

        /* Update error code */
        i8080_dev->ErrorCode |= I8080_ERROR_FIFO_WR_FULL;

        /* Change the state */
        i8080_dev->State = I8080_STATE_ERROR;

        if(i8080_dev->cb_event)
            i8080_dev->cb_event(I8080_IRQ_EVENT_FIFO_WR_FULL, 0);
    }

    if((status & I8080_OUT_I8080_INT_STATUS_TXFIFO_UNDERFLOW_ISR_Msk) == I8080_OUT_I8080_INT_STATUS_TXFIFO_UNDERFLOW_ISR_Msk)
    {
        i8080_dev->Instance->REG_I8080_INTR_CLR.bit.TXFIFO_UNDERFLOW_CLR = 1;

        /* Update error code */
        i8080_dev->ErrorCode |= I8080_ERROR_FIFO_WR_EMPTY;

        /* Change the state */
        i8080_dev->State = I8080_STATE_ERROR;

        if(i8080_dev->cb_event)
            i8080_dev->cb_event(I8080_IRQ_EVENT_FIFO_WR_EMPTY, 0);
    }

    if((status & I8080_OUT_I8080_INT_STATUS_TXFIFO_OVERFLOW_ISR_Msk) == I8080_OUT_I8080_INT_STATUS_TXFIFO_OVERFLOW_ISR_Msk)
    {
        i8080_dev->Instance->REG_I8080_INTR_CLR.bit.TXFIFO_OVERFLOW_CLR = 1;

        /* Update error code */
        i8080_dev->ErrorCode |= I8080_ERROR_FIFO_RD_EMPTY;

        /* Change the state */
        i8080_dev->State = I8080_STATE_ERROR;

        if(i8080_dev->cb_event)
            i8080_dev->cb_event(I8080_IRQ_EVENT_FIFO_RD_EMPTY, 0);
    }

    if((status & I8080_OUT_I8080_INT_STATUS_DMA_REQ_ISR_Msk) == I8080_OUT_I8080_INT_STATUS_DMA_REQ_ISR_Msk)
    {
        i8080_dev->Instance->REG_I8080_INTR_CLR.bit.DMA_REQ_CLR = 1;
    }

    if((status & I8080_OUT_I8080_INT_STATUS_DMA_SINGLE_ISR_Msk) == I8080_OUT_I8080_INT_STATUS_DMA_SINGLE_ISR_Msk)
    {
        i8080_dev->Instance->REG_I8080_INTR_CLR.bit.DMA_SINGLE_CLR = 1;
    }

}

/* Exported functions --------------------------------------------------------*/


/**
  * @brief  Return I8080 driver version.
  *
  * @return CSK_DRIVER_VERSION
  */
CSK_DRIVER_VERSION
I8080_GetVersion(void)
{
    return i8080_driver_version;
}


/**
  * @brief  Return I8080 instance.
  *
  * @return Instance of I8080
  */
void* I80800(void)
{
    return &i80800_dev;
}


/**
  * @brief  Return I8080 BUF instance.
  *
  * @return Instance of I8080
  */
uint32_t I80800_Buf(void)
{
    return I8080_TXBUF;
}


/**
 * @brief Initialize the I8080 (Digital Video Processor) device.
 *
 * @param pRgbDev A pointer to the I8080 device structure.
 * @param pCallback The callback function for I8080 events.
 * @param pcfg The configuration structure for the I8080 device.
 * @return int32_t Returns CSK_DRIVER_OK if initialization is successful, otherwise returns an error code.
 */
int32_t I8080_Initialize(void *pRgbDev, void *pCallback, I8080_InitTypeDef *pcfg)
{
    I8080_DEV *i8080_dev = safe_i8080_dev(pRgbDev);

    /* Check the I8080 instance */
    if(i8080_dev == NULL)
    {
        DEV_LOG("[%s:%d] Error input: pRgbDev is NULL", __func__, __LINE__);
        return CSK_DRIVER_ERROR_PARAMETER;
    }

    if(pcfg == NULL)
    {
        DEV_LOG("[%s:%d] Error input: pCfg is NULL", __func__, __LINE__);
        return CSK_DRIVER_ERROR_PARAMETER;
    }

    I8080_Reset();

    memcpy(&i8080_dev->Init, pcfg, sizeof(I8080_InitTypeDef));

    i8080_dev->Instance->REG_I8080_CONTROL0.bit.I8080_EN = 0;
    i8080_dev->Instance->REG_I8080_CONTROL1.bit.I8080_SOFT_RST = 1;

    switch(pcfg->frms)
    {
        case I8080_FRAME_ONCE:
            i8080_dev->Instance->REG_I8080_CONTROL0.bit.FRMS_EN = 0;
            break;

        case I8080_FRAME_CONTINUE:
            i8080_dev->Instance->REG_I8080_CONTROL0.bit.FRMS_EN = 1;
            break;

        default:
            CLOG("[%s:%d] frms=%d is error", __func__, __LINE__, pcfg->frms);
            return CSK_DRIVER_ERROR_PARAMETER;
    }

    switch(pcfg->wires)
    {
        case I8080_OUTPUT_WIRES_24:
            i8080_dev->Instance->REG_I8080_CONTROL1.bit.I8080_SERIAL_MODE = 0;
            break;

        case I8080_OUTPUT_WIRES_8:
            i8080_dev->Instance->REG_I8080_CONTROL1.bit.I8080_SERIAL_MODE = 1;
            break;

        default:
            CLOG("[%s:%d] wires=%d is error", __func__, __LINE__, pcfg->wires);
            return CSK_DRIVER_ERROR_PARAMETER;
    }

    switch(pcfg->format_in)
    {
        case I8080_INPUT_FORMAT_I8080888:
            i8080_dev->Instance->REG_I8080_CONTROL1.bit.I8080_INPUT_FORMAT_SEL = 0;
            break;

        case I8080_INPUT_FORMAT_XI80808888:
            i8080_dev->Instance->REG_I8080_CONTROL1.bit.I8080_INPUT_FORMAT_SEL = 1;
            break;

        case I8080_INPUT_FORMAT_I8080565:
            i8080_dev->Instance->REG_I8080_CONTROL1.bit.I8080_INPUT_FORMAT_SEL = 2;
            break;

        default:
            CLOG("[%s:%d] format_in=%d is error", __func__, __LINE__, pcfg->format_in);
            return CSK_DRIVER_ERROR_PARAMETER;
    }

    switch(pcfg->format_out)
    {
        case I8080_OUTPUT_FORMAT_I8080888:
            i8080_dev->Instance->REG_I8080_CONTROL1.bit.I8080_OUTPUT_FORMAT_SEL = 0;
            i8080_dev->Instance->REG_SEQUENTIAL_CONTROL0.bit.SBGR = 0;
            break;

        case I8080_OUTPUT_FORMAT_I8080666:
            i8080_dev->Instance->REG_I8080_CONTROL1.bit.I8080_OUTPUT_FORMAT_SEL = 1;
            i8080_dev->Instance->REG_SEQUENTIAL_CONTROL0.bit.SBGR = 0;
            break;

        case I8080_OUTPUT_FORMAT_I8080565:
            i8080_dev->Instance->REG_I8080_CONTROL1.bit.I8080_OUTPUT_FORMAT_SEL = 2;
            i8080_dev->Instance->REG_SEQUENTIAL_CONTROL0.bit.SBGR = 0;
            break;

        case I8080_OUTPUT_FORMAT_BGR888:
            i8080_dev->Instance->REG_I8080_CONTROL1.bit.I8080_OUTPUT_FORMAT_SEL = 0;
            i8080_dev->Instance->REG_SEQUENTIAL_CONTROL0.bit.SBGR = 1;
            break;

        case I8080_OUTPUT_FORMAT_BGR666:
            i8080_dev->Instance->REG_I8080_CONTROL1.bit.I8080_OUTPUT_FORMAT_SEL = 1;
            i8080_dev->Instance->REG_SEQUENTIAL_CONTROL0.bit.SBGR = 1;
            break;

        case I8080_OUTPUT_FORMAT_BGR565:
            i8080_dev->Instance->REG_I8080_CONTROL1.bit.I8080_OUTPUT_FORMAT_SEL = 2;
            i8080_dev->Instance->REG_SEQUENTIAL_CONTROL0.bit.SBGR = 1;
            break;

        default:
            CLOG("[%s:%d] format_out=%d is error", __func__, __LINE__, pcfg->format_out);
            return CSK_DRIVER_ERROR_PARAMETER;
    }

    if(true == pcfg->out_lsb) {
        i8080_dev->Instance->REG_I8080_CONTROL0.bit.DATA_ALIGN = 1;
    } else {
        i8080_dev->Instance->REG_I8080_CONTROL0.bit.DATA_ALIGN = 0;
    }

    if(true == pcfg->de_continue) {
        i8080_dev->Instance->REG_SEQUENTIAL_CONTROL1.bit.DE_SEQUENTIAL_SEL = 0;
    } else {
        i8080_dev->Instance->REG_SEQUENTIAL_CONTROL1.bit.DE_SEQUENTIAL_SEL = 1;
    }

    switch(pcfg->VSPolarity)
    {
        case I8080_POLARITY_POSITIVE:
            i8080_dev->Instance->REG_SEQUENTIAL_CONTROL0.bit.VDPOL = 0;
            break;

        case I8080_POLARITY_NEGATIVE:
            i8080_dev->Instance->REG_SEQUENTIAL_CONTROL0.bit.VDPOL = 1;
            break;

        default:
            CLOG("[%s:%d] VSPolarity=%d is error", __func__, __LINE__, pcfg->VSPolarity);
            return CSK_DRIVER_ERROR_PARAMETER;
    }

    switch(pcfg->HSPolarity)
    {
        case I8080_POLARITY_POSITIVE:
            i8080_dev->Instance->REG_SEQUENTIAL_CONTROL0.bit.HDPOL = 0;
            break;

        case I8080_POLARITY_NEGATIVE:
            i8080_dev->Instance->REG_SEQUENTIAL_CONTROL0.bit.HDPOL = 1;
            break;

        default:
            CLOG("[%s:%d] HSPolarity=%d is error", __func__, __LINE__, pcfg->HSPolarity);
            return CSK_DRIVER_ERROR_PARAMETER;
    }

    switch(pcfg->DEPolarity)
    {
        case I8080_POLARITY_POSITIVE:
            i8080_dev->Instance->REG_SEQUENTIAL_CONTROL0.bit.DEPOL = 0;
            break;

        case I8080_POLARITY_NEGATIVE:
            i8080_dev->Instance->REG_SEQUENTIAL_CONTROL0.bit.DEPOL = 1;
            break;

        default:
            CLOG("[%s:%d] DEPolarity=%d is error", __func__, __LINE__, pcfg->DEPolarity);
            return CSK_DRIVER_ERROR_PARAMETER;
    }

    switch(pcfg->CLKPolarity)
    {
        case I8080_POLARITY_POSITIVE:
            i8080_dev->Instance->REG_SEQUENTIAL_CONTROL0.bit.DCLKPOL = 0;
            i8080_dev->Instance->REG_I8080_CONTROL0.bit.SEL_I8080_INV_CLK = 1;  // bit30  0:normal  1:inv
            break;

        case I8080_POLARITY_NEGATIVE:
            i8080_dev->Instance->REG_SEQUENTIAL_CONTROL0.bit.DCLKPOL = 1;
            i8080_dev->Instance->REG_I8080_CONTROL0.bit.SEL_I8080_INV_CLK = 0;  // bit30  0:normal  1:inv
            break;

        default:
            CLOG("[%s:%d] CLKPolarity=%d is error", __func__, __LINE__, pcfg->CLKPolarity);
            return CSK_DRIVER_ERROR_PARAMETER;
    }

    switch(pcfg->sync)
    {
        case I8080_SYNC_MODE_SYNC:
            i8080_dev->Instance->REG_I8080_CONTROL1.bit.I8080_MODE_SEL = 0;
            break;

        case I8080_SYNC_MODE_SYNC_DE:
            i8080_dev->Instance->REG_I8080_CONTROL1.bit.I8080_MODE_SEL = 1;
            break;

        default:
            CLOG("[%s:%d] sync=%d is error", __func__, __LINE__, pcfg->sync);
            return CSK_DRIVER_ERROR_PARAMETER;
    }

#if 0
    CLOG("pcfg->img_width=%d 0x%x", pcfg->img_width, pcfg->img_width);
    CLOG("pcfg->img_height=%d 0x%x", pcfg->img_height, pcfg->img_height);
    CLOG("pcfg->v_pulse_width=%d 0x%x", pcfg->v_pulse_width, pcfg->v_pulse_width);
    CLOG("pcfg->h_pulse_width=%d 0x%x", pcfg->h_pulse_width, pcfg->h_pulse_width);
    CLOG("pcfg->v_front_blanking=%d 0x%x", pcfg->v_front_blanking, pcfg->v_front_blanking);
    CLOG("pcfg->h_front_blanking=%d 0x%x", pcfg->h_front_blanking, pcfg->h_front_blanking);
    CLOG("pcfg->v_back_blanking=%d 0x%x", pcfg->v_back_blanking, pcfg->v_back_blanking);
    CLOG("pcfg->h_back_blanking=%d 0x%x", pcfg->h_back_blanking, pcfg->h_back_blanking);
#endif

#if 0
    i8080_dev->Instance->REG_IMAGE_SIZE.bit.WIDTH_IN = pcfg->img_width & 0xFFFF;
    i8080_dev->Instance->REG_IMAGE_SIZE.bit.HEIGHT_IN = pcfg->img_height & 0xFFFF;
    i8080_dev->Instance->REG_SEQUENTIAL_CONTROL0.bit.V_PULSE_WIDTH = pcfg->v_pulse_width & 0xF;
    i8080_dev->Instance->REG_SEQUENTIAL_CONTROL0.bit.H_PULSE_WIDTH = pcfg->h_pulse_width & 0xF;
    i8080_dev->Instance->REG_SEQUENTIAL_CONTROL1.bit.V_FRONT_BLANKING = (uint32_t)pcfg->v_front_blanking & 0xFF;
    i8080_dev->Instance->REG_SEQUENTIAL_CONTROL1.bit.H_FRONT_BLANKING = (uint32_t)pcfg->h_front_blanking & 0xFF;
    i8080_dev->Instance->REG_SEQUENTIAL_CONTROL0.bit.V_BLANKING = pcfg->v_back_blanking & 0xFF;
    i8080_dev->Instance->REG_SEQUENTIAL_CONTROL0.bit.H_BLANKING = pcfg->h_back_blanking & 0xFF;
#else
    uint32_t value = 0;
    value = (pcfg->v_pulse_width << 20) | (pcfg->h_pulse_width << 16) | (pcfg->v_back_blanking << 8) | pcfg->h_back_blanking;
    i8080_dev->Instance->REG_SEQUENTIAL_CONTROL0.all &= ~0xFFFFFF;
    i8080_dev->Instance->REG_SEQUENTIAL_CONTROL0.all |= (value & 0xFFFFFF);

    value = (pcfg->v_front_blanking << 8) | pcfg->h_front_blanking;
    i8080_dev->Instance->REG_SEQUENTIAL_CONTROL1.all &= ~0xFFFF;
    i8080_dev->Instance->REG_SEQUENTIAL_CONTROL1.all |= (value & 0xFFFF);

    value = (pcfg->img_height << 16) | pcfg->img_width;
    i8080_dev->Instance->REG_IMAGE_SIZE.all = value;
#endif

    i8080_dev->Instance->REG_SEQUENTIAL_CONTROL0.bit.I8080_BURST_THD = 3;      // 0:1byte  1:2byte  2:4byte  3:8byte
    i8080_dev->Instance->REG_I8080_CONTROL1.bit.FIFO_WR_CLR = 1;
    i8080_dev->Instance->REG_I8080_CONTROL1.bit.FIFO_RD_CLR = 1;

    /* IRQ mask, but enable sof and eof */
    i8080_dev->Instance->REG_I8080_INTR_MSK.bit.EOF_FLAG_MSK = 1;
    i8080_dev->Instance->REG_I8080_INTR_MSK.bit.SOF_FLAG_MSK = 1;
    i8080_dev->Instance->REG_I8080_INTR_MSK.bit.FIFO_RD_EMPTY_MSK = 1;
    i8080_dev->Instance->REG_I8080_INTR_MSK.bit.FIFO_RD_FULL_MSK = 1;
    i8080_dev->Instance->REG_I8080_INTR_MSK.bit.FIFO_WR_EMPTY_MSK = 1;
    i8080_dev->Instance->REG_I8080_INTR_MSK.bit.FIFO_WR_FULL_MSK = 1;

    /* IRQ Clear */
    i8080_dev->Instance->REG_I8080_INTR_CLR.bit.EOF_FLAG_CLR = 1;
    i8080_dev->Instance->REG_I8080_INTR_CLR.bit.SOF_FLAG_CLR = 1;
    i8080_dev->Instance->REG_I8080_INTR_CLR.bit.FIFO_RD_EMPTY_CLR = 1;
    i8080_dev->Instance->REG_I8080_INTR_CLR.bit.FIFO_RD_FULL_CLR = 1;
    i8080_dev->Instance->REG_I8080_INTR_CLR.bit.FIFO_WR_EMPTY_CLR = 1;
    i8080_dev->Instance->REG_I8080_INTR_CLR.bit.FIFO_WR_FULL_CLR = 1;

    /* Set up callback */
    i8080_dev->cb_event = (CSK_I8080_SignalEvent_t)pCallback;

    /* Update error code */
    i8080_dev->ErrorCode = I8080_ERROR_NONE;

    /* Init the low level hardware and interrupt */
    register_ISR(IRQ_I8080_VECTOR, (ISR)I8080_IRQ_Handler, NULL);
    clear_IRQ(IRQ_I8080_VECTOR);
    enable_IRQ(IRQ_I8080_VECTOR);

    /* Initialize the I8080 state*/
    i8080_dev->State  = I8080_STATE_READY;

    return CSK_DRIVER_OK;
}


/**
 * @brief Uninitializes the I8080 device.
 *
 * This function uninitializes the I8080 device by performing a reset and disabling the VI.
 *
 * @param pRgbDev A pointer to the I8080 device structure.
 * @return CSK_DRIVER_OK if the operation is successful, otherwise returns an error code.
 */
int32_t I8080_Uninitialize(void *pRgbDev)
{
    I8080_DEV *i8080_dev = safe_i8080_dev(pRgbDev);

    /* Check the I8080 instance */
    if(i8080_dev == NULL)
    {
        DEV_LOG("[%s:%d] Error input: pRgbDev is NULL", __func__, __LINE__);
        return CSK_DRIVER_ERROR_PARAMETER;
    }

    /* Disable I8080 */
    i8080_dev->Instance->REG_I8080_CONTROL0.bit.FRMS_EN = 0;
    i8080_dev->Instance->REG_I8080_CONTROL0.bit.I8080_EN = 0;
    i8080_dev->Instance->REG_I8080_CONTROL1.bit.FIFO_WR_CLR = 1;
    i8080_dev->Instance->REG_I8080_CONTROL1.bit.FIFO_RD_CLR = 1;
    i8080_dev->Instance->REG_I8080_CONTROL1.bit.I8080_SOFT_RST = 1;

    disable_IRQ(IRQ_I8080_VECTOR);

    /* IRQ mask, but enable sof and eof */
    i8080_dev->Instance->REG_I8080_INTR_MSK.bit.EOF_FLAG_MSK = 1;
    i8080_dev->Instance->REG_I8080_INTR_MSK.bit.SOF_FLAG_MSK = 1;
    i8080_dev->Instance->REG_I8080_INTR_MSK.bit.FIFO_RD_EMPTY_MSK = 1;
    i8080_dev->Instance->REG_I8080_INTR_MSK.bit.FIFO_RD_FULL_MSK = 1;
    i8080_dev->Instance->REG_I8080_INTR_MSK.bit.FIFO_WR_EMPTY_MSK = 1;
    i8080_dev->Instance->REG_I8080_INTR_MSK.bit.FIFO_WR_FULL_MSK = 1;

    /* IRQ Clear */
    i8080_dev->Instance->REG_I8080_INTR_CLR.bit.EOF_FLAG_CLR = 1;
    i8080_dev->Instance->REG_I8080_INTR_CLR.bit.SOF_FLAG_CLR = 1;
    i8080_dev->Instance->REG_I8080_INTR_CLR.bit.FIFO_RD_EMPTY_CLR = 1;
    i8080_dev->Instance->REG_I8080_INTR_CLR.bit.FIFO_RD_FULL_CLR = 1;
    i8080_dev->Instance->REG_I8080_INTR_CLR.bit.FIFO_WR_EMPTY_CLR = 1;
    i8080_dev->Instance->REG_I8080_INTR_CLR.bit.FIFO_WR_FULL_CLR = 1;

    I8080_Reset();

    i8080_dev->ErrorCode = I8080_ERROR_NONE;
    i8080_dev->State  = I8080_STATE_RESET;

    return CSK_DRIVER_OK;
}


/**
 * @brief Start the I8080 to capture video frames.
 *
 * @param pRgbDev A pointer to the I8080 device structure.
 * @return int32_t Returns CSK_DRIVER_OK if successful, otherwise returns an error code.
 */
int32_t I8080_Start(void *pRgbDev)
{
    I8080_DEV *i8080_dev = safe_i8080_dev(pRgbDev);

    /* Check the I8080 instance */
    if(i8080_dev == NULL)
    {
        DEV_LOG("[%s:%d] Error input: pRgbDev is NULL", __func__, __LINE__);
        return CSK_DRIVER_ERROR_PARAMETER;
    }

    if(i8080_dev->State != I8080_STATE_READY)
    {
        DEV_LOG("[%s:%d] Error state: i8080 state not ready, now is %d, ", __func__, __LINE__, i8080_dev->State);
        return CSK_DRIVER_ERROR;
    }

    /* Lock the I8080 peripheral state */
    i8080_dev->State = I8080_STATE_BUSY;

    //DEV_LOG("[%s:%d] 0x%x", __func__, __LINE__, &i8080_dev->Instance->REG_I8080_CONTROL0.all);

    /* Enable I8080 */
    i8080_dev->Instance->REG_I8080_CONTROL1.bit.FIFO_WR_CLR = 1;
    i8080_dev->Instance->REG_I8080_CONTROL1.bit.FIFO_RD_CLR = 1;
    i8080_dev->Instance->REG_I8080_CONTROL0.bit.I8080_EN = 1;
    i8080_dev->Instance->REG_I8080_CONTROL0.bit.I8080_START = 1;

    /* Return function status */
    return CSK_DRIVER_OK;
}


/**
 * @brief Stops the I8080 and releases its resources.
 *
 * @param pRgbDev A pointer to the I8080 device structure.
 * @return int32_t Returns CSK_DRIVER_OK if successful, otherwise returns an error code.
 */
int32_t I8080_Stop(void *pRgbDev)
{
    I8080_DEV *i8080_dev = safe_i8080_dev(pRgbDev);

    /* Check the I8080 instance */
    if(i8080_dev == NULL)
    {
        DEV_LOG("[%s:%d] Error input: pRgbDev is NULL", __func__, __LINE__);
        return CSK_DRIVER_ERROR_PARAMETER;
    }

    /* Lock the I8080 peripheral state */
    i8080_dev->State = I8080_STATE_BUSY;

    /* Disable I8080 */
    i8080_dev->Instance->REG_I8080_CONTROL0.bit.FRMS_EN = 0;
    i8080_dev->Instance->REG_I8080_CONTROL0.bit.I8080_EN = 0;
    i8080_dev->Instance->REG_I8080_CONTROL1.bit.FIFO_WR_CLR = 1;
    i8080_dev->Instance->REG_I8080_CONTROL1.bit.FIFO_RD_CLR = 1;
    i8080_dev->Instance->REG_I8080_CONTROL1.bit.I8080_SOFT_RST = 1;

    /* Update error code */
    i8080_dev->ErrorCode = I8080_ERROR_NONE;

    /* Change I8080 state */
    i8080_dev->State = I8080_STATE_READY;

    return CSK_DRIVER_OK;
}


static uint32_t clk_div_get(uint32_t src_clk, uint32_t freq_hz)
{
    uint32_t divider = 0;
    uint32_t freq_big = 0;
    uint32_t freq_lit = 0;

    divider = src_clk / freq_hz;
    if(divider == 0) {
        return 1;
    }

    freq_big = src_clk / divider;
    freq_lit = src_clk / (divider + 1);

    if(((freq_big - freq_hz) > (freq_hz - freq_lit))) {
        return (divider + 1);
    } else {
        return divider;
    }
}

/**
 * @brief Enables the I8080 clock output.
 *
 * @param freq_hz The desired frequency of the I8080 clock output in Hz.
 * @return int32_t Returns CSK_DRIVER_OK if successful, otherwise returns an error code.
 */
int32_t I8080_EnableClockout(uint32_t freq_hz)
{
    uint32_t div_24MHz = 0;
    uint32_t freq_24MHz = 0;
    uint32_t div_100MHz = 0;
    uint32_t freq_100MHz = 0;

    if(freq_hz < I8080_CLK_OUT_HZ_MIN) {
        freq_hz = I8080_CLK_OUT_HZ_MIN;
    } else if(freq_hz > I8080_CLK_OUT_HZ_MAX) {
        freq_hz = I8080_CLK_OUT_HZ_MAX;
    } else {
    }

    div_24MHz = clk_div_get(I8080_CLK_OUT_SEL_24MHz, freq_hz);
    freq_24MHz = I8080_CLK_OUT_SEL_24MHz / div_24MHz;

    div_100MHz = clk_div_get(I8080_CLK_OUT_SEL_100MHz, freq_hz);
    freq_100MHz = I8080_CLK_OUT_SEL_100MHz / div_100MHz;

    if(freq_24MHz >= freq_hz) {
        freq_24MHz = freq_24MHz - freq_hz;
    } else {
        freq_24MHz = freq_hz - freq_24MHz;
    }

    if(freq_100MHz >= freq_hz) {
        freq_100MHz = freq_100MHz - freq_hz;
    } else {
        freq_100MHz = freq_hz - freq_100MHz;
    }

    if(freq_24MHz <= freq_100MHz) {
        IP_SYSCTRL->REG_PERI_CLK_CFG5.bit.SEL_I8080_CLK = 0;                    // bit20  0:24MHz  1:syspll_peri_clk
        IP_SYSCTRL->REG_PERI_CLK_CFG5.bit.DIV_I8080_CLK_M = (div_24MHz & 0xF);  // bit21~24
        DEV_LOG("[%s:%d] src=24MHz div=%d clk=%dHz", __func__, __LINE__, div_24MHz, (I8080_CLK_OUT_SEL_24MHz / div_24MHz));
    } else {
        IP_SYSCTRL->REG_PERI_CLK_CFG5.bit.SEL_I8080_CLK = 1;  // bit20  0:24MHz  1:syspll_peri_clk
        IP_SYSCTRL->REG_PERI_CLK_CFG5.bit.DIV_I8080_CLK_M = (div_100MHz & 0xF);  // bit21~24
        DEV_LOG("[%s:%d] src=100MHz div=%d clk=%dHz", __func__, __LINE__, div_100MHz, (I8080_CLK_OUT_SEL_100MHz / div_100MHz));
    }
    IP_SYSCTRL->REG_PERI_CLK_CFG5.bit.DIV_I8080_CLK_LD = 0x1;  // bit25  0:normal  1:inv

    return CSK_DRIVER_OK;
}


/**
 * @brief Disables the I8080 clock output.
 *
 * @return int32_t Returns CSK_DRIVER_OK if successful, otherwise returns an error code.
 */
int32_t I8080_DisableClockout(void)
{
    return CSK_DRIVER_OK;
}


/**
 * @brief Get the current state of the I8080 device.
 *
 * @param pRgbDev A pointer to the I8080 device structure.
 * @param pState A pointer to a I8080_emState variable where the current state will be stored.
 * @return int32_t Returns CSK_DRIVER_OK if successful, otherwise returns an error code.
 */
int32_t I8080_GetState(void *pRgbDev, I8080_emState *pState)
{
    I8080_DEV *i8080_dev = safe_i8080_dev(pRgbDev);

    /* Check the I8080 instance */
    if(i8080_dev == NULL)
    {
        DEV_LOG("[%s:%d] Error input: pRgbDev is NULL", __func__, __LINE__);
        return CSK_DRIVER_ERROR_PARAMETER;
    }

    if(pState == NULL)
    {
        DEV_LOG("[%s:%d] Error input: pState is NULL", __func__, __LINE__);
        return CSK_DRIVER_ERROR_PARAMETER;
    }

    *pState = i8080_dev->State;

    return CSK_DRIVER_OK;
}


/**
 * @brief Get the error code from a I8080 device.
 *
 * @param pRgbDev A pointer to the I8080 device structure.
 * @param pError A pointer to a variable where the error code will be stored.
 * @return int32_t Returns CSK_DRIVER_OK if successful, otherwise returns an error code.
 */
int32_t I8080_GetError(void *pRgbDev, uint32_t *pError)
{
    I8080_DEV *i8080_dev = safe_i8080_dev(pRgbDev);

    /* Check the I8080 instance */
    if(i8080_dev == NULL)
    {
        DEV_LOG("[%s:%d] Error input: pRgbDev is NULL", __func__, __LINE__);
        return CSK_DRIVER_ERROR_PARAMETER;
    }

    if(pError == NULL)
    {
        DEV_LOG("[%s:%d] Error input: pError is NULL", __func__, __LINE__);
        return CSK_DRIVER_ERROR_PARAMETER;
    }

    *pError = i8080_dev->ErrorCode;

    return CSK_DRIVER_OK;
}

#endif





