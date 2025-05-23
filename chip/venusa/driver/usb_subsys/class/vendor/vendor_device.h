/* 
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Ha Thach (tinyusb.org)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * This file is part of the TinyUSB stack.
 */

#ifndef _TUSB_VENDOR_DEVICE_H_
#define _TUSB_VENDOR_DEVICE_H_

#include "common/tusb_common.h"
#include "device/usbd.h"

#ifndef CFG_TUD_VENDOR_EPSIZE
#define CFG_TUD_VENDOR_EPSIZE     64
#endif

//BSD: TX ZLP automatically if TX size is multiple of MPS
//#define AUTO_TX_ZLP          1

#ifdef __cplusplus
 extern "C" {
#endif

//--------------------------------------------------------------------+
// Application API (Multiple Interfaces)
//--------------------------------------------------------------------+
bool     tud_vendor_n_mounted         (uint8_t itf);

#if CFG_TUD_VENDOR_USE_FIFO

uint32_t tud_vendor_n_available       (uint8_t itf);
uint32_t tud_vendor_n_read            (uint8_t itf, void* buffer, uint32_t bufsize);
bool     tud_vendor_n_peek            (uint8_t itf, int pos, uint8_t* u8);

uint32_t tud_vendor_n_write           (uint8_t itf, void const* buffer, uint32_t bufsize);
uint32_t tud_vendor_n_write_available (uint8_t itf);

static inline
uint32_t tud_vendor_n_write_str       (uint8_t itf, char const* str);

#else // !CFG_TUD_VENDOR_USE_FIFO

// Read asynchronously received bytes from OUT EP, return value:
// 0: indicate that read operation is pending, later asynchronous callback
//      will be called to notify caller of the read result (completion or failure)
// > 0: indicate that part of or all read has been done,
//      all done if return_value == bufsize, and
//      part done and asynchronous callback invoked if return_value < bufsize
// < 0: indicate that read operation failed
//
int32_t tud_vendor_n_read_immed (uint8_t itf, uint8_t ep_addr, void* buffer, uint32_t bufsize);

// Write asynchronously bytes to IN EP, return value:
// 0: indicate that write operation is pending, later asynchronous callback
//      will be called to notify caller of the write result (completion or failure)
// > 0: indicate that part of or all write has been done,
//      all done if return_value == bufsize, and
//      part done and asynchronous callback invoked if return_value < bufsize
// < 0: indicate that write operation failed//
//
int32_t tud_vendor_n_write_immed (uint8_t itf, uint8_t ep_addr, void const* buffer, uint32_t bufsize);

// Write ZLP (Zero-Length-Packet) to specified EP of the interface (index)
#define tud_vendor_write_zlp(itf, ep_addr)     tud_vendor_n_write_immed (itf, ep_addr, NULL, 0)

#endif // CFG_TUD_VENDOR_USE_FIFO

//--------------------------------------------------------------------+
// Application API (Single Port)
//--------------------------------------------------------------------+
static inline bool     tud_vendor_mounted         (void);

#if CFG_TUD_VENDOR_USE_FIFO
static inline uint32_t tud_vendor_available       (void);
static inline uint32_t tud_vendor_read            (void* buffer, uint32_t bufsize);
static inline bool     tud_vendor_peek            (int pos, uint8_t* u8);
static inline uint32_t tud_vendor_write           (void const* buffer, uint32_t bufsize);
static inline uint32_t tud_vendor_write_str       (char const* str);
static inline uint32_t tud_vendor_write_available (void);
#endif // CFG_TUD_VENDOR_USE_FIFO

//--------------------------------------------------------------------+
// Application Callback API (weak is optional)
//--------------------------------------------------------------------+

#if !CFG_TUD_VENDOR_USE_FIFO

#if !DIRECT_RW_EP
//BSD: set EP buffer and its length, Invoked when vendord_open is called. EP buffer CANNOT be set to NULL!!
//void tud_vendor_ep_buf_cb(uint8_t itf, uint8_t ep_addr, uint8_t **ep_buf_pp, uint16_t *ep_buf_len_p);
void tud_vendor_ep_buf_cb(uint8_t itf, uint8_t ep_addr, uint8_t **ep_buf_pp, uint32_t *ep_buf_len_p);
#endif

// status: 0 = successful, -1 = failed,  -2 = aborted
#define VENDOR_OP_STATUS_SUCCESS   0
#define VENDOR_OP_STATUS_FAILED    -1
#define VENDOR_OP_STATUS_ABORTED   -2

TU_ATTR_WEAK void
tud_vendor_read_done_cb (uint8_t itf, uint8_t ep_addr, uint8_t status, void* buffer, uint32_t xferred_bytes);

TU_ATTR_WEAK void
tud_vendor_write_done_cb (uint8_t itf, uint8_t ep_addr, uint8_t status, void const* buffer, uint32_t xferred_bytes);

#endif // !CFG_TUD_VENDOR_USE_FIFO

// Invoked when received new data
TU_ATTR_WEAK void tud_vendor_rx_cb(uint8_t itf);

//--------------------------------------------------------------------+
// Inline Functions
//--------------------------------------------------------------------+

static inline bool tud_vendor_mounted (void)
{
  return tud_vendor_n_mounted(0);
}

#if CFG_TUD_VENDOR_USE_FIFO

static inline uint32_t tud_vendor_n_write_str (uint8_t itf, char const* str)
{
  return tud_vendor_n_write(itf, str, strlen(str));
}

static inline uint32_t tud_vendor_available (void)
{
  return tud_vendor_n_available(0);
}

static inline uint32_t tud_vendor_read (void* buffer, uint32_t bufsize)
{
  return tud_vendor_n_read(0, buffer, bufsize);
}

static inline bool tud_vendor_peek (int pos, uint8_t* u8)
{
  return tud_vendor_n_peek(0, pos, u8);
}

static inline uint32_t tud_vendor_write (void const* buffer, uint32_t bufsize)
{
  return tud_vendor_n_write(0, buffer, bufsize);
}

static inline uint32_t tud_vendor_write_str (char const* str)
{
  return tud_vendor_n_write_str(0, str);
}

static inline uint32_t tud_vendor_write_available (void)
{
  return tud_vendor_n_write_available(0);
}

#endif // CFG_TUD_VENDOR_USE_FIFO

//--------------------------------------------------------------------+
// Internal Class Driver API
//--------------------------------------------------------------------+
void     vendord_init(void);
void     vendord_reset(uint8_t rhport);
uint16_t vendord_open(uint8_t rhport, tusb_desc_interface_t const * itf_desc, uint16_t max_len);
bool     vendord_xfer_cb(uint8_t rhport, uint8_t ep_addr, xfer_result_t event, uint32_t xferred_bytes);

#ifdef __cplusplus
 }
#endif

#endif /* _TUSB_VENDOR_DEVICE_H_ */
