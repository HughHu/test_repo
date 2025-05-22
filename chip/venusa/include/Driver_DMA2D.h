#ifndef __DRIVER_DMA2D_H__
#define __DRIVER_DMA2D_H__

#include "Driver_Common.h"


#ifdef __cplusplus
extern "C" {
#endif

#define CSK_DMA2D_MAX_CHANNEL_NUM              6
#define CSK_DMA2D_EVENT_TRANSFER_DONE          (1UL << 0)
#define CSK_DMA2D_STATUS_ERROR                 (CSK_DRIVER_ERROR_SPECIFIC - 1)
#define CSK_DMA2D_MODE_ERROR                   (CSK_DRIVER_ERROR_SPECIFIC - 2)
#define CSK_DMA2D_UNKNOWN_ERROR                (CSK_DRIVER_ERROR_SPECIFIC - 3)


typedef enum _csk_dma2d_status {
    dma2d_status_none = 0UL,
    dma2d_status_init = 1UL,
    dma2d_status_config,
    dma2d_status_config_img,
    dma2d_status_busy,
    dma2d_status_stopped,
    dma2d_status_err,
} csk_dma2d_status_t;

typedef enum _csk_dma2d_ch {
    dma_2d_ch0 = 0UL,
    dma_2d_ch1,
    dma_2d_ch2,
    dma_2d_ch3,
    dma_2d_ch4,
    dma_2d_ch5,
} csk_dma2d_ch_t;

typedef enum _csk_dma2d_burst_len {
    dma2d_burst_len_1spl = 0UL,
    dma2d_burst_len_4spl,
    dma2d_burst_len_8spl,
    dma2d_burst_len_16spl,
} csk_dma2d_burst_len_t;

typedef enum _csk_dma2d_sample_unit {
    dma2d_sample_unit_byte = 0UL,
    dma2d_sample_unit_halfword,
    dma2d_sample_unit_word,
} csk_dma2d_sample_unit_t;

typedef enum _csk_dma2d_flow_ctrl {
    dma2d_flow_ctrl_dma = 0UL,
    dma2d_flow_ctrl_peripheral,
} csk_dma2d_flow_ctrl_t;

typedef enum _csk_tfr_mode {
    tfr_mode_p2m = 0UL,
    tfr_mode_m2p,
    tfr_mode_m2m,
} csk_tfr_mode_t;

typedef enum _csk_inc_mode {
    inc_mode_increase = 0UL,
    inc_mode_decrease,
    inc_mode_fix,
} csk_inc_mode_t;

typedef enum _csk_prio_mode {
    prio_mode_vhigh = 0UL,
    prio_mode_high,
    prio_mode_normal,
    prio_mode_low,
} csk_prio_mode_t;

typedef enum _csk_handshake_num {
    rgb_hs_num0 = 0x0,
    qspi_out_hs_num1,
    qspi_in_hs_num2,
    dvp_hs_num3,
    jpg_e_hs_num4,
    jpg_p_hs_num5,
    uart0_rx_hs_num6,
    uart0_tx_hs_num7,
    apc_tx0_hs_num8,
    apc_tx1_hs_num9,
    apc_tx2_hs_num10,
    i8080_hs_num11,
    apc_rx0_hs_num12,
    apc_rx1_hs_num13,
    apc_rx2_hs_num14,
    apc_rx3_hs_num15,

    hs_none = 0xff,
} csk_handshake_num_t;

typedef enum _csk_image_format {
    csk_image_format_rgb888 = 0x0,
    csk_image_format_rgb565,
    csk_image_format_yuv444_packed,
    csk_image_format_yuv422_packed,
    csk_image_format_y8,
} csk_image_format_t;

typedef enum _csk_image_yuv422_format {
    csk_image_yuv422_format_y0cby1cr = 0x0,
    csk_image_yuv422_format_cby0cry1,
    csk_image_yuv422_format_y0cry1cb,
    csk_image_yuv422_format_cry0cby1,
} csk_image_yuv422_format_t;

typedef enum _csk_image_rgb_format {
    csk_image_bgr_format = 0x0,
    csk_image_rgb_format,
} csk_image_rgb_format_t;

typedef enum _csk_jpeg_codec_mode {
    csk_jpeg_bypass = 0x0,
    csk_jpeg_decode,
    csk_jpeg_encode,
} csk_jpeg_codec_mode_t;

typedef enum _csk_rotation_mode {
    csk_rota_cw90 = 0x0,        //clockwise 90°
    csk_rota_ccw90,             //counterclockwise 90°
    csk_rota_cw180,             //clockwise 180°
    csk_rota_transpose,         //transpose
} csk_rotation_mode_t;

typedef enum _csk_mirror_mode {
    csk_mirror_hor = 0x0,       //mirror horizontal
    csk_mirror_vert,            //mirror vertical
} csk_mirror_mode_t;

typedef enum _csk_scaler_mode {
    csk_scaler_down = 0x0,      //scaler down
    csk_scaler_up,              //scaler up
    csk_scaler_null,            //remain unchange
} csk_scaler_mode_t;

typedef enum _csk_merge_ratio {
    csk_merge_null = 0x1,      //remain unchange
    csk_merge_2to1 = 0x2,      //two datas are averaged
    csk_merge_4to1 = 0x4,      //four datas are averaged
    csk_merge_8to1 = 0x8,      //eight datas are averaged
} csk_merge_ratio_t;

typedef enum _csk_func_switch {
    csk_func_disable = 0x0,       //function disable
    csk_func_enable,              //function enable
} csk_func_switch_t;

typedef struct _csk_image_info {
    uint16_t img_width;                             // width of the image (unit:pixel)
    uint16_t img_height;                            // height of the image (unit:pixel)
    uint16_t img_line_stride;                       // stride of the image (unit:byte)
    csk_image_format_t img_format;                  // format of image
    csk_image_yuv422_format_t img_yuv422_format;    // specific format for YUV422
    csk_image_rgb_format_t img_rgb_format;          // specific format for RGB
} csk_image_info_t;

typedef struct _csk_gather_scatter_info {
    csk_func_switch_t enable;                       // function switch
    uint32_t interval;                              // data interval(unit:byte)
    uint32_t counter;                               // data counter(unit:byte)
} csk_gather_scatter_info_t;

typedef struct _csk_dma2d_init {
    csk_dma2d_ch_t dma_ch;
    csk_tfr_mode_t tfr_mode;
    csk_dma2d_sample_unit_t src_basic_unit;
    csk_dma2d_sample_unit_t dst_basic_unit;
    csk_inc_mode_t src_inc_mode;
    csk_inc_mode_t dst_inc_mode;
    csk_dma2d_burst_len_t src_burst_len;
    csk_dma2d_burst_len_t dst_burst_len;
    csk_dma2d_flow_ctrl_t flow_ctrl;
    csk_prio_mode_t prio_lvl;
    csk_handshake_num_t handshake;
    csk_gather_scatter_info_t src_gather;
    csk_gather_scatter_info_t dst_scatter;
} csk_dma2d_init_t;

typedef struct _csk_dma_2d_image_cfg {
    csk_image_info_t img_input;                     // input image info
    csk_image_info_t img_output;                    // output image info

    csk_jpeg_codec_mode_t img_jpeg_codec_mode;      // jpeg codec mode

    csk_func_switch_t img_rota_en;                  // rotation switch
    csk_func_switch_t img_rota_tile_en;             // rotation tile mode switch
    csk_rotation_mode_t img_rota_mode;              // rotation mode

    csk_func_switch_t img_mirror_en;                // mirror switch
    csk_mirror_mode_t img_mirror_mode;              // mirror mode

    csk_func_switch_t img_copy_en;                  // memcpy copy switch

    csk_func_switch_t img_crop_en;                  // memcpy crop switch

    csk_func_switch_t img_scaler_en;                // scaler switch
} csk_dma_2d_image_cfg_t;

typedef struct _csk_dma_2d_rotate_cfg {
    csk_dma2d_init_t dma2d_init;
    csk_dma_2d_image_cfg_t dma2d_img_cfg;
} csk_dma_2d_rotate_cfg_t;


typedef void (*CSK_DMA2D_SignalEvent_t)(uint32_t event, void* workspace);


int32_t DMA2D_Initialize(void);

int32_t DMA2D_Config(csk_dma2d_init_t* res, CSK_DMA2D_SignalEvent_t cb_event, void* workspace);

int32_t DMA2D_Start_Normal(csk_dma2d_ch_t chn, void* src, void* dst, uint32_t img_in_len);

int32_t DMA2D_Stop(csk_dma2d_ch_t chn);

int32_t DMA2D_Image_Config_Extend(csk_dma2d_ch_t chn, csk_dma_2d_image_cfg_t* img_cfg);

#ifdef __cplusplus
}
#endif

#endif /* __DRIVER_DMA2D_H__ */
