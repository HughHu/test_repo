#if 0
#ifndef __INCLUDE_DRIVER_I8080_H
#define __INCLUDE_DRIVER_I8080_H

#include "Driver_Common.h"

#define CSK_I8080_API_VERSION CSK_DRIVER_VERSION_MAJOR_MINOR(1,0)         /*!< API version */


/*----- SPI Control Codes: Mode -----*/
#define CSK_QSPI_LCD_MODE_Pos                0
#define CSK_QSPI_LCD_MODE_Msk                (0xFUL << CSK_QSPI_LCD_MODE_Pos)     // bit[3:0]
#define CSK_QSPI_LCD_MODE_UNSET              (0x00UL << CSK_QSPI_LCD_MODE_Pos)    ///< SPI Mode is kept unchanged
#define CSK_QSPI_LCD_MODE_MASTER             (0x01UL << CSK_QSPI_LCD_MODE_Pos)    ///< SPI Master (Output on MOSI, Input on MISO); arg = Bus Speed in bps
#define CSK_QSPI_LCD_MODE_SLAVE              (0x02UL << CSK_QSPI_LCD_MODE_Pos)    ///< SPI Slave  (Output on MISO, Input on MOSI)

typedef enum
{
    I8080_DATA_LEN_8BIT         = 0x00U,
    I8080_DATA_LEN_24BIT        = 0x01U,
    I8080_DATA_LEN_32BIT        = 0x02U,
    I8080_DATA_LEN_BUTT
}i8080_emDataLen;

typedef enum
{
    I8080_TRANS_MODE_TX       = 0x00U,
    I8080_TRANS_MODE_RX        = 0x01U,
    I8080_TRANS_MODE_BUTT
}i8080_emTransMode;

typedef enum
{

}i8080_emDataMerge;

typedef enum
{

}i8080_emLSB;

typedef enum
{

}i8080_emByteConv;

typedef enum
{
    I8080_OUTPUT_HIGH_ACTIVE    = 0x00U,
    I8080_OUTPUT_LOW_ACTIVE     = 0x01U,
    I8080_OUTPUT_NONE           = 0x01U,
    I8080_POLARITY_BUTT
}i8080_emOutput;

/**
  * @brief  I8080 State enum definition
  */
typedef enum
{
    I8080_STATE_RESET             = 0x00U,  /*!< I8080 not yet initialized or disabled  */
    I8080_STATE_READY             = 0x01U,  /*!< I8080 initialized and ready for use    */
    I8080_STATE_BUSY              = 0x02U,  /*!< I8080 internal processing is ongoing   */
    I8080_STATE_TIMEOUT           = 0x03U,  /*!< I8080 timeout state                    */
    I8080_STATE_ERROR             = 0x04U,  /*!< I8080 error state                      */
    I8080_STATE_SUSPENDED         = 0x05U,  /*!< I8080 suspend state                    */
    I8080_STATE_BUTT
}I8080_emState;


/**
  * @brief  I8080 error enum definition
  */
typedef enum
{
    I8080_ERROR_NONE              = 0x00U,  /*!< I8080 not yet initialized or disabled  */
    I8080_ERROR_FIFO_RD_EMPTY     = 0x04U,  /*!< I8080 fifo_read_empty                  */
    I8080_ERROR_FIFO_RD_FULL      = 0x08U,  /*!< I8080 fifo_read_full                   */
    I8080_ERROR_FIFO_WR_EMPTY     = 0x10U,  /*!< I8080 fifo_write_empty                 */
    I8080_ERROR_FIFO_WR_FULL      = 0x20U,  /*!< I8080 fifo_write_full                  */
    I8080_ERROR_BUTT
}I8080_emError;


/**
  * @brief  I8080 interrupt status enum definition
  */
typedef enum
{
    I8080_IRQ_EVENT_EOF                 = 0x00U,  /*!< I8080 eof               */
    I8080_IRQ_EVENT_SOF                 = 0x01U,  /*!< I8080 sof               */
    I8080_IRQ_EVENT_FIFO_RD_EMPTY       = 0x02U,  /*!< I8080 fifo_read_empty   */
    I8080_IRQ_EVENT_FIFO_RD_FULL        = 0x03U,  /*!< I8080 fifo_read_full    */
    I8080_IRQ_EVENT_FIFO_WR_EMPTY       = 0x04U,  /*!< I8080 fifo_write_empty  */
    I8080_IRQ_EVENT_FIFO_WR_FULL        = 0x05U,  /*!< I8080 fifo_write_full   */
    I8080_IRQ_EVENT_BUTT
}I8080_emIrqEvent;


/**
  * @brief   I8080 Init structure definition
  */
typedef struct
{
    i8080_emFrms      frms;
    i8080_emWires     wires;
    i8080_emSync      sync;
    bool            de_continue;
    i8080_emFormatIn  format_in;
    i8080_emFormatOut format_out;
    bool            out_lsb;
    i8080_emPol       VSPolarity;
    i8080_emPol       HSPolarity;
    i8080_emPol       DEPolarity;
    i8080_emPol       CLKPolarity;
    uint32_t        clk_hz;
    uint16_t        img_width;          // 0~0xFFFF
    uint16_t        img_height;         // 0~0xFFFF
    uint8_t         v_pulse_width;      // 0~0xF
    uint8_t         h_pulse_width;      // 0~0xF
    uint8_t         v_front_blanking;   // 0~0xFF
    uint8_t         h_front_blanking;   // 0~0xFF
    uint8_t         v_back_blanking;    // 0~0xFF
    uint8_t         h_back_blanking;    // 0~0xFF
}I8080_InitTypeDef;


/**
 \fn          void CSK_I8080_SignalEvent_t (I8080_emIrqEvent event, uint32_t usr_param)
 \brief       Signal I8080 Events.
 \param[in]   event        I8080 event notification mask
 \param[in]   usr_param    user parameter
 \return      none
*/
typedef void (*CSK_I8080_SignalEvent_t)(I8080_emIrqEvent event, uint32_t param);



//------------------------------------------------------------------------------------------
/**
  * @brief  Return I8080 driver version.
  *
  * @return CSK_DRIVER_VERSION
  */
CSK_DRIVER_VERSION
I8080_GetVersion(void);

/**
 * @brief Initialize the I8080 (Digital Video Processor) device.
 *
 * @param pRgbDev A pointer to the I8080 device structure.
 * @param pCallback The callback function for I8080 events.
 * @param pCfg The configuration structure for the I8080 device.
 * @return int32_t Returns CSK_DRIVER_OK if initialization is successful, otherwise returns an error code.
 */
int32_t
I8080_Initialize(void *pRgbDev, void *pCallback, I8080_InitTypeDef *pcfg);

/**
 * @brief Uninitializes the I8080 device.
 *
 * This function uninitializes the I8080 device by performing a reset and disabling the VI.
 *
 * @param pRgbDev A pointer to the I8080 device structure.
 * @return CSK_DRIVER_OK if the operation is successful, otherwise returns an error code.
 */
int32_t
I8080_Uninitialize(void *pRgbDev);

/**
 * @brief Start the I8080 to capture video frames.
 *
 * @param pRgbDev A pointer to the I8080 device structure.
 * @return int32_t Returns CSK_DRIVER_OK if successful, otherwise returns an error code.
 */
int32_t
I8080_Start(void *pRgbDev);

/**
 * @brief Stops the I8080 and releases its resources.
 *
 * @param pRgbDev A pointer to the I8080 device structure.
 * @return int32_t Returns CSK_DRIVER_OK if successful, otherwise returns an error code.
 */
int32_t
I8080_Stop(void *pRgbDev);

/**
 * @brief Enables the I8080 clock output.
 *
 * @param freq_hz The desired frequency of the I8080 clock output in Hz.
 * @return int32_t Returns CSK_DRIVER_OK if successful, otherwise returns an error code.
 */
int32_t
I8080_EnableClockout(uint32_t freq_hz);

/**
 * @brief Disables the I8080 clock output.
 *
 * @return int32_t Returns CSK_DRIVER_OK if successful, otherwise returns an error code.
 */
int32_t
I8080_DisableClockout(void);

/**
 * @brief Get the current state of the I8080 device.
 *
 * @param pRgbDev A pointer to the I8080 device structure.
 * @param pState A pointer to a I8080_emState variable where the current state will be stored.
 * @return int32_t Returns CSK_DRIVER_OK if successful, otherwise returns an error code.
 */
int32_t
I8080_GetState(void *pRgbDev, I8080_emState *pState);

/**
 * @brief Get the error code from a I8080 device.
 *
 * @param pRgbDev A pointer to the I8080 device structure.
 * @param pError A pointer to a variable where the error code will be stored.
 * @return int32_t Returns CSK_DRIVER_OK if successful, otherwise returns an error code.
 */
int32_t
I8080_GetError(void *pRgbDev, uint32_t *pError);

/**
  * @brief  Return I8080 instance.
  *
  * @return Instance of I8080
  */
void* I80800(void);

/**
  * @brief  Return I8080 BUF instance.
  *
  * @return Instance of I8080
  */
uint32_t I80800_Buf(void);


#endif /* __DRIVER_I8080_H */

#endif
