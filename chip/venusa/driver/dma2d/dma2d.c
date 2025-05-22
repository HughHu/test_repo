#include <string.h> // for memset etc.
#include <stdio.h> // for memset etc.
#include "ClockManager.h"
#include "log_print.h"
#include "venusa_ap.h"

#include "Driver_DMA2D.h"

// DMA2D Control register
#define DMA2D_CH_CTRL_HS_SEL_OFFSET         28
#define DMA2D_CH_CTRL_ONLINE_D_DST_OFFSET   26
#define DMA2D_CH_CTRL_QSPI_EN_OFFSE T       25
#define DMA2D_CH_CTRL_DVP_EN_OFFSET         24
#define DMA2D_CH_CTRL_LINE_STR_OFFSET       23
#define DMA2D_CH_CTRL_CFG_SGEN_OFFSET       22
#define DMA2D_CH_CTRL_CFG_DSEN_OFFSET       21
#define DMA2D_CH_CTRL_CH_PRIO_OFFSET        19
#define DMA2D_CH_CTRL_ONLINE_D_SRC_OFFSET   18
#define DMA2D_CH_CTRL_FLOW_CTRL_OFFSET      17
#define DMA2D_CH_CTRL_DST_BURST_OFFSET      15
#define DMA2D_CH_CTRL_SRC_BURST_OFFSET      13
#define DMA2D_CH_CTRL_DST_INC_MODE_OFFSET   10
#define DMA2D_CH_CTRL_SRC_INC_MODE_OFFSET   8
#define DMA2D_CH_CTRL_AHB_LOCK_OFFSET       7
#define DMA2D_CH_CTRL_DST_BASE_UNIT_OFFSET  5
#define DMA2D_CH_CTRL_SRC_BASE_UNIT_OFFSET  3
#define DMA2D_CH_CTRL_TFR_MODE_OFFSET       1
#define DMA2D_CH_CTRL_CH_EN_OFFSET          0

// DMA2D image format register
#define DMA2D_IMAGE_RGB_FORMAT_OUT_OFFSET   13
#define DMA2D_IMAGE_RGB_FORMAT_IN_OFFSET    12
#define DMA2D_IMAGE_YUV422_FORMAT_OUT_OFFSET 10
#define DMA2D_IMAGE_YUV422_FORMAT_IN_OFFSET 8
#define DMA2D_IMAGE_OUT_FORMAT_OFFSET       4
#define DMA2D_IMAGE_IN_FORMAT_OFFSET        0

// DMA2D image jpeg register
#define DMA2D_JPEG_DATA_TYPE_OFFSET         2
#define DMA2D_JPEG_ENC_EN_OFFSET            1
#define DMA2D_JPEG_DEC_EN_OFFSET            0

// DMA2D image rotation register
#define DMA2D_ROTA_TILE_MODE_OFFSET         4
#define DMA2D_ROTA_MODE_OFFSET              1
#define DMA2D_ROTA_EN_OFFSET                0

// DMA2D image size register
#define DMA2D_SIZE_H_OFFSET                 16
#define DMA2D_SIZE_W_OFFSET                 0

// DMA2D image line stride register
#define DMA2D_LINE_STR_OUT_OFFSET           16
#define DMA2D_LINE_STR_IN_OFFSET            0

// DMA2D image mirror register
#define DMA2D_MIRROR_MODE_OFFSET            1
#define DMA2D_MIRROR_EN_OFFSET              0

// DMA2D image copy register
#define DMA2D_COPY_EN_OFFSET                24
#define DMA2D_COPY_ROW_OFFSET               12
#define DMA2D_COPY_COL_OFFSET               0

// DMA2D image scaler register
#define DMA2D_SCALER_LAST_BAND_OFFSET       14
#define DMA2D_SCALER_LAST_TILE_OFFSET       13
#define DMA2D_SCALER_MERGE_VIRT_MODE_OFFSET 9
#define DMA2D_SCALER_MERGE_HOR_MODE_OFFSET  5
#define DMA2D_SCALER_MERGE_FLAG_OFFSET      4
#define DMA2D_SCALER_FLAG_OFFSET            3
#define DMA2D_SCALER_VERT_DIRT_OFFSET       2
#define DMA2D_SCALER_HOR_DIRT_OFFSET        1
#define DMA2D_SCALER_EN_OFFSET              0

#define DMA2D_SCALER_PHASESTEP_VIRT_OFFSET  16
#define DMA2D_SCALER_PHASESTEP_HOR_OFFSET   0

#define DMA2D_SCALER_STARTPHASE_VIRT_OFFSET 16
#define DMA2D_SCALER_STARTPHASE_HOR_OFFSET  0


// DMA2D DIAGRAM CONFIGURE
#define DMA2D_DIAG_SEL_TOG_FLAG_MODE        0
#define DMA2D_DIAG_SEL_REMAIN_CNT_MODE      0
#define DMA2D_DIAG_SEL_SRC_ADDRESS_MODE     4
#define DMA2D_DIAG_SEL_DST_ADDRESS_MODE     5


#define DMA2D_IN_FORMAT_YUV444_BYTES_SIZE   4
#define DMA2D_IN_FORMAT_ARGB_BYTES_SIZE       4
#define DMA2D_IN_FORMAT_ABGR_BYTES_SIZE       4
#define DMA2D_IN_FORMAT_XRGB_BYTES_SIZE       3
#define DMA2D_IN_FORMAT_XBGR_BYTES_SIZE       3
#define DMA2D_IN_FORMAT_YUV422_BYTES_SIZE   2
#define DMA2D_IN_FORMAT_YUV420_BYTES_SIZE   2

#define DMA2D_IMAGE_CROPPED                  1
#define DMA2D_IMAGE_NOT_CROPPED              0

#define DMA2D_IMAGER_ZOOM_SCALE             1
#define DMA2D_IMAGER_NOT_ZOOM_SCALE         0

// DMA2D MAX BLOCK
#define DMA2D_CH_MAX_BLOCK_LENGTH           0xFFFFFF //24bit
#define DMA2D_CH_MAX_BLOCK_LENGTH_MASK      0xFFFFFF

#define DMA2D_CH_IMG_MAX_WIDTH              0x7FF //11bit
#define DMA2D_CH_IMG_MAX_HEIGHT             0x7FF

typedef struct _csk_dma2d_ch_info {
    CSK_DMA2D_SignalEvent_t cb_event;
    void* workspace;
    csk_dma2d_status_t status;
    // In normal mode: total length
    // In PiPo mode: buffer length
    uint32_t length;
    // Current transfer length
    uint32_t xfer_length;
    //crop offset
    // transfer mode
    //uint32_t mode;
    csk_jpeg_codec_mode_t mode_2d;
    uint8_t block_in_size;
    uint8_t block_out_size;
    csk_inc_mode_t src_inc_mode;
    csk_inc_mode_t dst_inc_mode;
} csk_dma2d_ch_info_t;

static csk_dma2d_ch_info_t dma2d_ch_info[CSK_DMA2D_MAX_CHANNEL_NUM] = {0};


int32_t DMA2D_Stop(csk_dma2d_ch_t chn) {
    uint32_t *ch_control = (uint32_t*)&IP_GPDMA2D->REG_DMA_CTRL_00.all + chn;
    uint32_t *ch_irq_clr = (uint32_t*)&IP_GPDMA2D->REG_DMA_CH_IRQ_CLR_00.all + chn;

    // clear interrupt and config
    *ch_irq_clr |= 0x1F;
    IP_GPDMA2D->REG_DMA_CH_CLR.bit.CFG_CH_CLR |= (0x1 << chn);

    // delay
    __ASM volatile("nop");
    __ASM volatile("nop");
    __ASM volatile("nop");
    __ASM volatile("nop");
    __ASM volatile("nop");

    // disable chn
    *ch_control &= ~(0x1 << DMA2D_CH_CTRL_CH_EN_OFFSET);

    dma2d_ch_info[chn].status = dma2d_status_config;
    
    return CSK_DRIVER_OK;
}

int32_t DMA2D_Config(csk_dma2d_init_t* res, CSK_DMA2D_SignalEvent_t cb_event, void* workspace) {
    if ((dma2d_ch_info[res->dma_ch].status == dma2d_status_none) || (dma2d_ch_info[res->dma_ch].status == dma2d_status_busy))
    {
        return CSK_DMA2D_STATUS_ERROR;
    }

    dma2d_ch_info[res->dma_ch].cb_event = cb_event;
    dma2d_ch_info[res->dma_ch].workspace = workspace;
    dma2d_ch_info[res->dma_ch].src_inc_mode = res->src_inc_mode;
    dma2d_ch_info[res->dma_ch].dst_inc_mode = res->dst_inc_mode;

    uint8_t chn = (uint8_t)res->dma_ch;
    if (chn >= CSK_DMA2D_MAX_CHANNEL_NUM)
    {
        return CSK_DRIVER_ERROR_PARAMETER;
    }

    // Get channel control base address
    uint32_t* ch_ctrl = (uint32_t*)&IP_GPDMA2D->REG_DMA_CTRL_00.all + chn;
    uint32_t ch_ctrl_para = 0;

    // transfer mode
    ch_ctrl_para |= ((res->tfr_mode&0x3) << DMA2D_CH_CTRL_TFR_MODE_OFFSET);
    if (tfr_mode_p2m == res->tfr_mode || tfr_mode_m2p == res->tfr_mode)
    {
        // handshake sel
        if (apc_rx3_hs_num15 < res->handshake || rgb_hs_num0 > res->handshake)
            return CSK_DRIVER_ERROR_PARAMETER;
        ch_ctrl_para |= ((res->handshake&0xF) << DMA2D_CH_CTRL_HS_SEL_OFFSET);
        
        // flow control
        if (dma2d_flow_ctrl_peripheral == res->flow_ctrl)
        {
            ch_ctrl_para |= ((res->flow_ctrl&0x1) << DMA2D_CH_CTRL_FLOW_CTRL_OFFSET);
        }
    }

    // basic unit
    ch_ctrl_para |= (((res->src_basic_unit&0x3) << DMA2D_CH_CTRL_SRC_BASE_UNIT_OFFSET) | ((res->dst_basic_unit&0x3) << DMA2D_CH_CTRL_DST_BASE_UNIT_OFFSET));

    // address mode configure
    ch_ctrl_para |= (((res->src_inc_mode&0x3) << DMA2D_CH_CTRL_SRC_INC_MODE_OFFSET) | ((res->dst_inc_mode&0x3) << DMA2D_CH_CTRL_DST_INC_MODE_OFFSET));

    // burst length config
    ch_ctrl_para |= (((res->src_burst_len&0x3) << DMA2D_CH_CTRL_SRC_BURST_OFFSET) | ((res->dst_burst_len&0x3) << DMA2D_CH_CTRL_DST_BURST_OFFSET));

    // prio configure
    ch_ctrl_para |= ((res->prio_lvl&0x3) << DMA2D_CH_CTRL_CH_PRIO_OFFSET);

    // src gather
    ch_ctrl_para |= ((res->src_gather.enable&0x1)<<DMA2D_CH_CTRL_CFG_SGEN_OFFSET);
    if (csk_func_enable == res->src_gather.enable)
    {
        uint32_t *ch_gather_num = (uint32_t*)&IP_GPDMA2D->REG_DMA_SRC_GATHER_NUM_00.all + chn;
        uint32_t *ch_gather_itv = (uint32_t*)&IP_GPDMA2D->REG_DMA_SRC_GATHER_ITV_00.all + chn;
        *ch_gather_num = (res->src_gather.counter&0xFFFFFF);
        *ch_gather_itv = (res->src_gather.interval&0xFFFFFF);
    }

    // dst scatter
    ch_ctrl_para |= ((res->dst_scatter.enable&0x1)<<DMA2D_CH_CTRL_CFG_DSEN_OFFSET);
    if (csk_func_enable == res->dst_scatter.enable)
    {
        uint32_t *ch_scatter_num = (uint32_t*)&IP_GPDMA2D->REG_DMA_DST_SCATTER_NUM_00.all + chn;
        uint32_t *ch_scatter_itv = (uint32_t*)&IP_GPDMA2D->REG_DMA_DST_SCATTER_ITV_00.all + chn;
        *ch_scatter_num = (res->dst_scatter.counter&0xFFFFFF);
        *ch_scatter_itv = (res->dst_scatter.interval&0xFFFFFF);
    }

    *ch_ctrl = ch_ctrl_para;

    dma2d_ch_info[res->dma_ch].status = dma2d_status_config;

    return CSK_DRIVER_OK;
}

int32_t DMA2D_Start_Normal(csk_dma2d_ch_t chn, void* src, void* dst, uint32_t img_in_len) {
    // Only in configure status can start normal transmits
    if ((dma2d_ch_info[chn].status != dma2d_status_config) && (dma2d_ch_info[chn].status != dma2d_status_config_img))
    {
        //return CSK_DMA2D_STATUS_ERROR;
    }

    // Get channel control register
    uint32_t* ch_ctrl = (uint32_t*)&IP_GPDMA2D->REG_DMA_CTRL_00.all + chn;

    // Get channel length register
    uint32_t* ch_img_in_len_reg = (uint32_t*)&IP_GPDMA2D->REG_DMA_BLK_LEN_00.all + chn;
    uint32_t* src_address = (uint32_t*)&IP_GPDMA2D->REG_DMA_SRC_ADDR_00.all + chn;
    uint32_t* dst_address = (uint32_t*)&IP_GPDMA2D->REG_DMA_DST_ADDR_00.all + chn;
    uint32_t *ch_shd = (uint32_t*)&IP_GPDMA2D->REG_DMA_CH_SHD_RDY_00.all + chn;
    uint32_t *ch_irq_en = (uint32_t*)&IP_GPDMA2D->REG_DMA_CH_IRQ_EN_00.all + chn;

    // Configure source and destination
    *src_address = (uint32_t)src;
    *dst_address = (uint32_t)dst;

    // Configure length
    dma2d_ch_info[chn].length = img_in_len;
    dma2d_ch_info[chn].xfer_length = 0;
    if (img_in_len > DMA2D_CH_MAX_BLOCK_LENGTH)
        *ch_img_in_len_reg = DMA2D_CH_MAX_BLOCK_LENGTH;
    else
        *ch_img_in_len_reg = img_in_len;

    dma2d_ch_info[chn].status = dma2d_status_busy;

    // Enable interrupt
    *ch_irq_en = 0x1;

    // Enable
    if (!(*ch_ctrl & 0x1))
    {
        *ch_ctrl |= 0x1;
    }

    // Enable shadow ready register and start update
    *ch_shd = 0x1;

    return CSK_DRIVER_OK;
}

static void DMA2D_IRQ_Handler(csk_dma2d_ch_t chn) {
    volatile uint32_t int_status = *((uint32_t*)&IP_GPDMA2D->REG_DMA_CH_IRQ_STATUS_00.all + chn);
    uint32_t* int_clr = (uint32_t*)&IP_GPDMA2D->REG_DMA_CH_IRQ_CLR_00.all + chn;

    // Clear interrupt pending
    *int_clr = int_status;

    if (int_status&0x1) //blk done irq
    {
        uint32_t event = 0;
        csk_dma2d_ch_info_t* p_channel = &dma2d_ch_info[chn];
        
        // Need transfer residue data
        if ((p_channel->length - p_channel->xfer_length) > DMA2D_CH_MAX_BLOCK_LENGTH)
        {
            uint32_t* channel_control = (uint32_t*)&IP_GPDMA2D->REG_DMA_CTRL_00.all + chn;
            uint32_t* channel_length = (uint32_t*)&IP_GPDMA2D->REG_DMA_BLK_LEN_00.all + chn;
            uint32_t* ch_shd = (uint32_t*)&IP_GPDMA2D->REG_DMA_CH_SHD_RDY_00.all + chn;

            p_channel->xfer_length += *channel_length;

            // Calculate residue size
            if ((p_channel->length - p_channel->xfer_length) >= DMA2D_CH_MAX_BLOCK_LENGTH)
                *channel_length = DMA2D_CH_MAX_BLOCK_LENGTH;
            else
                *channel_length = (p_channel->length - p_channel->xfer_length);

            uint32_t* src_address0 = (uint32_t*)&IP_GPDMA2D->REG_DMA_SRC_ADDR_00.all + chn;
            uint32_t* dst_address0 = (uint32_t*)&IP_GPDMA2D->REG_DMA_DST_ADDR_00.all + chn;
            if (inc_mode_increase == p_channel->src_inc_mode)
                *src_address0 += DMA2D_CH_MAX_BLOCK_LENGTH;
            else if (inc_mode_decrease == p_channel->src_inc_mode)
                *src_address0 -= DMA2D_CH_MAX_BLOCK_LENGTH;

            if (inc_mode_increase == p_channel->dst_inc_mode)
                *dst_address0 += DMA2D_CH_MAX_BLOCK_LENGTH;
            else if (inc_mode_decrease == p_channel->dst_inc_mode)
                *dst_address0 -= DMA2D_CH_MAX_BLOCK_LENGTH;

            // Enable shadow ready register
            *ch_shd = 0x1;
            
            // Enable
            if (!(*channel_control & 0x1))
            {
                *channel_control |= 0x1;
            }
        }
        else // It's the last block
        {
            // Transmit complete
            p_channel->xfer_length = p_channel->length;

            event |= CSK_DMA2D_EVENT_TRANSFER_DONE;

            // Change status to <config>
            p_channel->status = dma2d_status_config;
        }

        // Callback
        if (p_channel->cb_event != NULL && event != 0) 
        {
            p_channel->cb_event(event, p_channel->workspace);
        }
    }
}

static void DMA2D_CHN0_IRQ_Handler(void) {
    return DMA2D_IRQ_Handler(dma_2d_ch0);
}

static void DMA2D_CHN1_IRQ_Handler(void) {
    return DMA2D_IRQ_Handler(dma_2d_ch1);
}

static void DMA2D_CHN2_IRQ_Handler(void) {
    return DMA2D_IRQ_Handler(dma_2d_ch2);
}

static void DMA2D_CHN3_IRQ_Handler(void) {
    return DMA2D_IRQ_Handler(dma_2d_ch3);
}

static void DMA2D_CHN4_IRQ_Handler(void) {
    return DMA2D_IRQ_Handler(dma_2d_ch4);
}

static void DMA2D_CHN5_IRQ_Handler(void) {
    return DMA2D_IRQ_Handler(dma_2d_ch5);
}

static void DMA2D_COM_IRQ_Handler(void) {
    volatile uint32_t int_status = IP_GPDMA2D->REG_DMA_COM_IRQ_STATUS.all;

    // Clear interrupt pending
    IP_GPDMA2D->REG_DMA_COM_IRQ_CLR.all = int_status;

    return;
}

int32_t DMA2D_Initialize(void) {
    uint8_t i = 0;
    uint32_t *ch_irq_clr = NULL;
    uint32_t *ch_irq_en = NULL;    
    static volatile uint8_t init_flg = 0;
    static void (*chn_irq_handler[CSK_DMA2D_MAX_CHANNEL_NUM])(void) = 
    {
        DMA2D_CHN0_IRQ_Handler,
        DMA2D_CHN1_IRQ_Handler,
        DMA2D_CHN2_IRQ_Handler,
        DMA2D_CHN3_IRQ_Handler,
        DMA2D_CHN4_IRQ_Handler,
        DMA2D_CHN5_IRQ_Handler
    };

    if (init_flg == 0)
    {
        memset(dma2d_ch_info, 0, sizeof(dma2d_ch_info));

        // enable clock
        __HAL_CRM_GPDMA_CLK_ENABLE();

        // reset
        IP_SYSCTRL->REG_SW_RESET_CFG2.bit.GPDMA2D_RESET = 1;

        // clear chn interrupt and config、disable chn interrupt
        ch_irq_clr = (uint32_t*)&IP_GPDMA2D->REG_DMA_CH_IRQ_CLR_00.all;
        ch_irq_en = (uint32_t*)&IP_GPDMA2D->REG_DMA_CH_IRQ_EN_00.all;
        for (i = 0; i < CSK_DMA2D_MAX_CHANNEL_NUM; i++)
        {
            *(ch_irq_clr + i) = 0xF;
            *(ch_irq_en + i) = 0x0;
        }
        IP_GPDMA2D->REG_DMA_CH_CLR.bit.CFG_CH_CLR = 0x3F;
        
        // register chn global interrupt
        for (i = 0; i < CSK_DMA2D_MAX_CHANNEL_NUM; i++)
        {
            register_ISR(IRQ_GPDMA_0_VECTOR+i, chn_irq_handler[i], NULL);
            enable_IRQ(IRQ_GPDMA_0_VECTOR+i);
            dma2d_ch_info[i].status = dma2d_status_init;
        }

        // clear com interrupt and cmd fifo
        IP_GPDMA2D->REG_DMA_COM_IRQ_CLR.all = 0xF;
        IP_GPDMA2D->REG_DMA_CMD_FIFO0_CTRL.bit.CFG_CMD_FIFO0_CLR = 0x1;
        IP_GPDMA2D->REG_DMA_CMD_FIFO1_CTRL.bit.CFG_CMD_FIFO1_CLR = 0x1;
        
        // disable com interrupt
        IP_GPDMA2D->REG_DMA_COM_IRQ_EN.bit.CFG_COM_IRQ_EN = 0x0;

        // register com global interrupt
        register_ISR(IRQ_GPDMA_COM_VECTOR, DMA2D_COM_IRQ_Handler, NULL);
        enable_IRQ(IRQ_GPDMA_COM_VECTOR);

        init_flg = 1;
    }

    return CSK_DRIVER_OK;
}

void DMA2D_Image_Scaler_Register_Convert(uint16_t in_value, uint16_t out_value, csk_scaler_mode_t *scaler_mode, csk_merge_ratio_t *merge_mode, uint16_t *phase_step)
{
    float ratio = 0;

    *scaler_mode = csk_scaler_null;
    *merge_mode = csk_merge_null;

    if (in_value < out_value)
    {
        *scaler_mode = csk_scaler_up;
    }
    else if (in_value > out_value)
    {
        ratio = (float)in_value/out_value;
        if (8 <= ratio)
        {
            *merge_mode = csk_merge_8to1;
            in_value /= 8;
        }
        else if (4 <= ratio)
        {
            *merge_mode = csk_merge_4to1;
            in_value /= 4;
        }
        else if (2 <= ratio)
        {
            *merge_mode = csk_merge_2to1;
            in_value /= 2;
        }

        if (0 != in_value%out_value)
        {
            *scaler_mode = csk_scaler_down;
        }
    }

    *phase_step = (float)in_value/out_value*4096;

    //CLOGE("[%s][%d]in=%u,out=%u,scaler=%d,merge=%d,step=%u", __FUNCTION__, __LINE__, in_value, out_value, *scaler_mode, *merge_mode, *phase_step);

    return;
}

int32_t DMA2D_Image_Config_Extend(csk_dma2d_ch_t chn, csk_dma_2d_image_cfg_t* img_cfg) {
    uint32_t *ch_ctrl = (uint32_t*)&IP_GPDMA2D->REG_DMA_CTRL_00.all + chn;

    // Validate channel range
    if (chn < dma_2d_ch0 || chn > dma_2d_ch5)
    {
        return CSK_DRIVER_ERROR_PARAMETER;
    }

    // Validate channel status
    if (dma2d_ch_info[chn].status != dma2d_status_config)
    {
        return CSK_DRIVER_ERROR;
    }

    // ch0 support scaler
    //*********************************************scaler
    if (dma_2d_ch0 == chn)
    {
        IP_GPDMA2D->REG_DMA_SCALER_CTRL0.all = 0x0;
        IP_GPDMA2D->REG_DMA_SCALER_CTRL1.all = 0x0;
        IP_GPDMA2D->REG_DMA_SCALER_CTRL2.all = 0x0;
        //if ((csk_func_enable == img_cfg->img_scaler_en) && (img_cfg->img_input.img_width != img_cfg->img_output.img_width || img_cfg->img_input.img_height != img_cfg->img_output.img_height))
        if (csk_func_enable == img_cfg->img_scaler_en)
        {
            uint8_t last_band = 1;
            uint8_t last_tile = 1;
            uint8_t merge_flg = 0;
            csk_merge_ratio_t merge_h = csk_merge_null;
            csk_merge_ratio_t merge_v = csk_merge_null;
            uint8_t scaler_flg = 0;
            csk_scaler_mode_t scaler_h = csk_scaler_null;
            csk_scaler_mode_t scaler_v = csk_scaler_null;
            uint16_t phase_step_h = 0;
            uint16_t phase_step_v = 0;

            DMA2D_Image_Scaler_Register_Convert(img_cfg->img_input.img_width, img_cfg->img_output.img_width, &scaler_h, &merge_h, &phase_step_h);
            DMA2D_Image_Scaler_Register_Convert(img_cfg->img_input.img_height, img_cfg->img_output.img_height, &scaler_v, &merge_v, &phase_step_v);
            if (csk_scaler_null != scaler_h || csk_scaler_null != scaler_v)
            {
                scaler_flg = 1;
            }
            if (csk_merge_null != merge_h || csk_merge_null != merge_v)
            {
                merge_flg = 1;
            }

            IP_GPDMA2D->REG_DMA_SCALER_CTRL0.all = ((last_band&0x1)<<DMA2D_SCALER_LAST_BAND_OFFSET)|((last_tile&0x1)<<DMA2D_SCALER_LAST_TILE_OFFSET)|((merge_v&0xF)<<DMA2D_SCALER_MERGE_VIRT_MODE_OFFSET)
                                                    |((merge_h&0xF)<<DMA2D_SCALER_MERGE_HOR_MODE_OFFSET)|((merge_flg&0x1)<<DMA2D_SCALER_MERGE_FLAG_OFFSET)|((scaler_flg&0x1)<<DMA2D_SCALER_FLAG_OFFSET)
                                                    |((scaler_v&0x1)<<DMA2D_SCALER_VERT_DIRT_OFFSET)|((scaler_h&0x1)<<DMA2D_SCALER_HOR_DIRT_OFFSET)|((img_cfg->img_scaler_en&0x1)<<DMA2D_SCALER_EN_OFFSET);

            IP_GPDMA2D->REG_DMA_SCALER_CTRL1.all = ((phase_step_v&0xFFFF)<<DMA2D_SCALER_PHASESTEP_VIRT_OFFSET)|((phase_step_h&0xFFFF)<<DMA2D_SCALER_PHASESTEP_HOR_OFFSET);

            //CLOGE("[%s][%d]ctrl0=%#x,ctrl1=%#x,ctrl2=%#x", __FUNCTION__, __LINE__, IP_GPDMA2D->REG_DMA_SCALER_CTRL0.all, IP_GPDMA2D->REG_DMA_SCALER_CTRL1.all, IP_GPDMA2D->REG_DMA_SCALER_CTRL2.all);

            //reserved for scene of frame tile(big resolution exceeds scaler-mode 256*3 linebuffer)
            //uint16_t start_phase_h = 0;
            //uint16_t start_phase_v = 0;
            //IP_GPDMA2D->REG_DMA_SCALER_CTRL2.all = ((start_phase_v&0xFFF)<<DMA2D_SCALER_STARTPHASE_VIRT_OFFSET)|((start_phase_h&0xFFF)<<DMA2D_SCALER_STARTPHASE_HOR_OFFSET);
        }
    }

    // ch1/ch2 support jpeg encode/decode and rotation/mirror
    if (dma_2d_ch1 == chn || dma_2d_ch2 == chn)
    {
        //*********************************************jepg encode and decode
        uint32_t jpeg_image_type = 0;
        uint32_t *ch_jpeg_ctrl = (uint32_t*)&IP_GPDMA2D->REG_DMA_JPEG_CTRL_01.all + (chn - dma_2d_ch1);
        *ch_jpeg_ctrl &= ~0xF;
        if (csk_image_format_yuv422_packed == img_cfg->img_input.img_format)
            jpeg_image_type = 0x0;
        else if (csk_image_format_yuv444_packed == img_cfg->img_input.img_format || csk_image_format_rgb888 == img_cfg->img_input.img_format)
            jpeg_image_type = 0x1;
        else //y8
            jpeg_image_type = 0x2;
        *ch_jpeg_ctrl |= (((img_cfg->img_jpeg_codec_mode&0x3)<<DMA2D_JPEG_DEC_EN_OFFSET)|((jpeg_image_type&0x3)<<DMA2D_JPEG_DATA_TYPE_OFFSET));

        //*********************************************rotation
        uint32_t *ch_rota_ctrl = (uint32_t*)&IP_GPDMA2D->REG_DMA_ROTA_CTRL_01.all + (chn - dma_2d_ch1);
        *ch_rota_ctrl &= ~0x1F;
        *ch_rota_ctrl |= (((img_cfg->img_rota_en&0x1)<<DMA2D_ROTA_EN_OFFSET)|((img_cfg->img_rota_mode&0x3)<<DMA2D_ROTA_MODE_OFFSET)|((img_cfg->img_rota_tile_en&0x1)<<DMA2D_ROTA_TILE_MODE_OFFSET));

        //*********************************************mirror
        uint32_t *ch_mirror_ctrl = (uint32_t*)&IP_GPDMA2D->REG_DMA_MIRROR_CTRL_01.all + (chn - dma_2d_ch1);
        *ch_mirror_ctrl &= ~0x3;
        *ch_mirror_ctrl |= (((img_cfg->img_mirror_en&0x1)<<DMA2D_MIRROR_EN_OFFSET)|((img_cfg->img_mirror_mode&0x1)<<DMA2D_MIRROR_MODE_OFFSET));
    }

    //*********************************************image format
    uint32_t *ch_img_format = (uint32_t*)&IP_GPDMA2D->REG_DMA_IMAGE_FORMA_00.all + chn;
    *ch_img_format &= ~0x3FFF;
    *ch_img_format |= (((img_cfg->img_input.img_format&0x7)<<DMA2D_IMAGE_IN_FORMAT_OFFSET)|((img_cfg->img_output.img_format&0x7)<<DMA2D_IMAGE_OUT_FORMAT_OFFSET)|((img_cfg->img_input.img_yuv422_format&0x3)<<DMA2D_IMAGE_YUV422_FORMAT_IN_OFFSET)
                       |((img_cfg->img_output.img_yuv422_format&0x1)<<DMA2D_IMAGE_YUV422_FORMAT_OUT_OFFSET)|((img_cfg->img_input.img_rgb_format&0x1)<<DMA2D_IMAGE_RGB_FORMAT_IN_OFFSET)|((img_cfg->img_output.img_rgb_format&0x1)<<DMA2D_IMAGE_RGB_FORMAT_OUT_OFFSET));

    //*********************************************image size
    uint32_t *ch_img_size_in = (uint32_t*)&IP_GPDMA2D->REG_DMA_IN_IMAGE_SIZE_00.all + chn;
    uint32_t *ch_img_size_out = (uint32_t*)&IP_GPDMA2D->REG_DMA_OUT_IMAGE_SIZE_00.all + chn;
    uint32_t *ch_img_size_stride = (uint32_t*)&IP_GPDMA2D->REG_DMA_LINE_STRIDE_SIZE_00.all + chn;
    *ch_img_size_in = 0x0;
    *ch_img_size_in |= (((img_cfg->img_input.img_width&0x7FF)<<DMA2D_SIZE_W_OFFSET)|((img_cfg->img_input.img_height&0x7FF)<<DMA2D_SIZE_H_OFFSET));
    *ch_img_size_out = 0x0;
    *ch_img_size_out |= (((img_cfg->img_output.img_width&0x7FF)<<DMA2D_SIZE_W_OFFSET)|((img_cfg->img_output.img_height&0x7FF)<<DMA2D_SIZE_H_OFFSET));
    *ch_img_size_stride = 0x0;
    *ch_img_size_stride |= (((img_cfg->img_input.img_line_stride&0x3FFF)<<DMA2D_LINE_STR_IN_OFFSET)|((img_cfg->img_output.img_line_stride&0x3FFF)<<DMA2D_LINE_STR_OUT_OFFSET));

    //two-dimensional mode set(scaler/format convertion/crop/jpeg enc/jpeg dec/rotation/mirror/memory copy),one-dimensional mode(normal/scatter/gather)
    if (csk_func_enable == img_cfg->img_scaler_en || csk_jpeg_decode == img_cfg->img_jpeg_codec_mode || csk_jpeg_encode == img_cfg->img_jpeg_codec_mode || csk_func_enable == img_cfg->img_rota_en
        || csk_func_enable == img_cfg->img_mirror_en || csk_func_enable == img_cfg->img_copy_en || csk_func_enable == img_cfg->img_crop_en || img_cfg->img_input.img_format != img_cfg->img_output.img_format)
        *ch_ctrl |= (0x1<<DMA2D_CH_CTRL_LINE_STR_OFFSET);
    else
        *ch_ctrl &= ~(0x1<<DMA2D_CH_CTRL_LINE_STR_OFFSET);

    dma2d_ch_info[chn].status |= dma2d_status_config_img;

    return CSK_DRIVER_OK;
}

