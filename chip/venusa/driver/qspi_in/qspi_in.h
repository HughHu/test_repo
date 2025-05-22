/*
 * qspi_in.h
 *
 *  Created on: Oct 09, 2023
 *
 */

#ifndef __DRIVER_QSPI_IN_INTERNAL_H
#define __DRIVER_QSPI_IN_INTERNAL_H

#include "chip.h"
#include "Driver_QSPI_IN.h"

/*----- QSPI Transfer mode -----*/
#define CSK_QSPI_IN_TRANSMODE_SINGLE       (0x0)
#define CSK_QSPI_IN_TRANSMODE_DUAL         (0x1)
#define CSK_QSPI_IN_TRANSMODE_QUAD         (0x2)

#define CSK_QSPI_IN_BUF      (QSPI_IN_BASE + 0x2C)

#define QSPI_IN_INTRST_LINE_START_INT_Pos       10
#define QSPI_IN_INTRST_LINE_START_INT_Msk       0x400
#define QSPI_IN_INTRST_LINE_END_INT_Pos         9
#define QSPI_IN_INTRST_LINE_END_INT_Msk         0x200
#define QSPI_IN_INTRST_FRAME_END_INT_Pos        8
#define QSPI_IN_INTRST_FRAME_END_INT_Msk        0x100
#define QSPI_IN_INTRST_FRAME_START_INT_Pos      7
#define QSPI_IN_INTRST_FRAME_START_INT_Msk      0x80
#define QSPI_IN_INTRST_CRC_ERR_INT_Pos          6
#define QSPI_IN_INTRST_CRC_ERR_INT_Msk          0x40
#define QSPI_IN_INTRST_SLVCMDINT_Pos            5
#define QSPI_IN_INTRST_SLVCMDINT_Msk            0x20
#define QSPI_IN_INTRST_ENDINT_Pos               4
#define QSPI_IN_INTRST_ENDINT_Msk               0x10
#define QSPI_IN_INTRST_MTK_TRANS_ERR_INT_Pos    3
#define QSPI_IN_INTRST_MTK_TRANS_ERR_INT_Msk    0x8
#define QSPI_IN_INTRST_RXFIFOINT_Pos            2
#define QSPI_IN_INTRST_RXFIFOINT_Msk            0x4
#define QSPI_IN_INTRST_RXFIFOORINT_Pos          0
#define QSPI_IN_INTRST_RXFIFOORINT_Msk          0x1


/**
  * @brief  QSPI_IN handle Structure definition
  */
typedef struct __QSPI_IN_DEV
{
    QSPI_SENSOR_IN_RegDef      *Instance;           /*!< QSPI_IN Register base address  */

    QSPI_IN_InitTypeDef        Init;                /*!< QSPI_IN parameters             */

    QSPI_IN_SignalEvent_t      cb_event;            /*!< QSPI_IN callback               */
}QSPI_IN_DEV;


#endif /* __DRIVER_QSPI_IN_INTERNAL_H */
