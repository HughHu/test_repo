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

/** \ingroup group_usbd
 *  @{ */

#ifndef _TUSB_USBD_H_
#define _TUSB_USBD_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "common/tusb_common.h"

//--------------------------------------------------------------------+
// Application API
//--------------------------------------------------------------------+

// Init device stack
bool tud_init (void);

/* //BSD:
// Task function should be called in main/rtos loop
void tud_task (void);

// Check if there is pending events need proccessing by tud_task()
bool tud_task_event_ready(void);
*/

// Interrupt handler, name alias to DCD
extern void dcd_int_handler(uint8_t rhport);
#define tud_int_handler   dcd_int_handler

// Get current bus speed
tusb_speed_t tud_speed_get(void);

// Check if device is connected (may not mounted/configured yet)
// True if just got out of Bus Reset and received the very first data from host
bool tud_connected(void);

// Check if device is connected and configured
bool tud_mounted(void);

// Check if device is suspended
bool tud_suspended(void);

// Check if device is ready to transfer
static inline bool tud_ready(void)
{
  return tud_mounted() && !tud_suspended();
}

// Remote wake up host, only if suspended and enabled by host
bool tud_remote_wakeup(void);

// Enable pull-up resistor on D+ D-
// Return false on unsupported MCUs
bool tud_disconnect(void);

// Disable pull-up resistor on D+ D-
// Return false on unsupported MCUs
bool tud_connect(void);

//BSD: Notify that VBus level is changed
// Return false on unsupported MCUs
bool tud_notify_vbus_level_changed(uint8_t vbus_level);

// Carry out Data and Status stage of control transfer
// - If len = 0, it is equivalent to sending status only
// - If len > wLength : it will be truncated
bool tud_control_xfer(uint8_t rhport, tusb_control_request_t const * request, void* buffer, uint16_t len);

// Send STATUS (zero length) packet
bool tud_control_status(uint8_t rhport, tusb_control_request_t const * request);

//--------------------------------------------------------------------+
// Application Callbacks (WEAK is optional)
//--------------------------------------------------------------------+

// Invoked when received GET DEVICE DESCRIPTOR request
// Application return pointer to descriptor
uint8_t const * tud_descriptor_device_cb(void);

// Invoked when received GET BOS DESCRIPTOR request
// Application return pointer to descriptor
TU_ATTR_WEAK uint8_t const * tud_descriptor_bos_cb(void);

// Invoked when received GET CONFIGURATION DESCRIPTOR request
// Application return pointer to descriptor, whose contents must exist long enough for transfer to complete
uint8_t const * tud_descriptor_configuration_cb(uint8_t index);

// Invoked when received GET STRING DESCRIPTOR request
// Application return pointer to descriptor, whose contents must exist long enough for transfer to complete
uint16_t const* tud_descriptor_string_cb(uint8_t index, uint16_t langid);

// Invoked when received GET DEVICE QUALIFIER DESCRIPTOR request
// Application return pointer to descriptor, whose contents must exist long enough for transfer to complete
TU_ATTR_WEAK uint8_t const* tud_descriptor_device_qualifier_cb(void);

// Invoked when bus reset is detected
TU_ATTR_WEAK void tud_bus_reset_cb(void);

// Invoked when device is mounted (configured)
TU_ATTR_WEAK void tud_mount_cb(void);

// Invoked when device is unmounted
TU_ATTR_WEAK void tud_umount_cb(void);

// Invoked when usb bus is suspended
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
TU_ATTR_WEAK void tud_suspend_cb(bool remote_wakeup_en);

// Invoked when usb bus is resumed
TU_ATTR_WEAK void tud_resume_cb(void);

// Invoked when received control request with VENDOR TYPE
TU_ATTR_WEAK bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const * request);

//--------------------------------------------------------------------+
// Binary Device Object Store (BOS) Descriptor Templates
//--------------------------------------------------------------------+

#define TUD_BOS_DESC_LEN      5

// total length, number of device caps
#define TUD_BOS_DESCRIPTOR(_total_len, _caps_num) \
  5, TUSB_DESC_BOS, U16_TO_U8S_LE(_total_len), _caps_num

// Device Capability Platform 128-bit UUID + Data
#define TUD_BOS_PLATFORM_DESCRIPTOR(...) \
  4+TU_ARGS_NUM(__VA_ARGS__), TUSB_DESC_DEVICE_CAPABILITY, DEVICE_CAPABILITY_PLATFORM, 0x00, __VA_ARGS__

//------------- WebUSB BOS Platform -------------//

// Descriptor Length
#define TUD_BOS_WEBUSB_DESC_LEN         24

// Vendor Code, iLandingPage
#define TUD_BOS_WEBUSB_DESCRIPTOR(_vendor_code, _ipage) \
  TUD_BOS_PLATFORM_DESCRIPTOR(TUD_BOS_WEBUSB_UUID, U16_TO_U8S_LE(0x0100), _vendor_code, _ipage)

#define TUD_BOS_WEBUSB_UUID   \
  0x38, 0xB6, 0x08, 0x34, 0xA9, 0x09, 0xA0, 0x47, \
  0x8B, 0xFD, 0xA0, 0x76, 0x88, 0x15, 0xB6, 0x65

//------------- Microsoft OS 2.0 Platform -------------//
#define TUD_BOS_MICROSOFT_OS_DESC_LEN   28

// Total Length of descriptor set, vendor code
#define TUD_BOS_MS_OS_20_DESCRIPTOR(_desc_set_len, _vendor_code) \
  TUD_BOS_PLATFORM_DESCRIPTOR(TUD_BOS_MS_OS_20_UUID, U32_TO_U8S_LE(0x06030000), U16_TO_U8S_LE(_desc_set_len), _vendor_code, 0)

#define TUD_BOS_MS_OS_20_UUID \
    0xDF, 0x60, 0xDD, 0xD8, 0x89, 0x45, 0xC7, 0x4C, \
  0x9C, 0xD2, 0x65, 0x9D, 0x9E, 0x64, 0x8A, 0x9F

//--------------------------------------------------------------------+
// Configuration & Interface Descriptor Templates
//--------------------------------------------------------------------+

//------------- Configuration -------------//
#define TUD_CONFIG_DESC_LEN   (9)

// Config number, interface count, string index, total length, attribute, power in mA
#define TUD_CONFIG_DESCRIPTOR(config_num, _itfcount, _stridx, _total_len, _attribute, _power_ma) \
  9, TUSB_DESC_CONFIGURATION, U16_TO_U8S_LE(_total_len), _itfcount, config_num, _stridx, TU_BIT(7) | _attribute, (_power_ma)/2

//------------- CDC -------------//

// Length of template descriptor: 66 or 59 bytes
#define TUD_CDC_DESC_NOTIF_LEN      (8+9+5+5+4+5+7+9+7+7) // with notification EP
#define TUD_CDC_DESC_LEN            (8+9+5+5+4+5+9+7+7) // NO notification EP

// CDC Descriptor Template, with notification EP
// Interface number, string index, EP notification address and size, EP data address (out, in) and size.
#define TUD_CDC_DESCRIPTOR_NOTIF(_itfnum, _stridx, _ep_notif, _ep_notif_size, _epout, _epin, _epsize) \
  /* Interface Associate */\
  8, TUSB_DESC_INTERFACE_ASSOCIATION, _itfnum, 2, TUSB_CLASS_CDC, CDC_COMM_SUBCLASS_ABSTRACT_CONTROL_MODEL, CDC_COMM_PROTOCOL_NONE, 0,\
  /* CDC Control Interface */\
  9, TUSB_DESC_INTERFACE, _itfnum, 0, 1, TUSB_CLASS_CDC, CDC_COMM_SUBCLASS_ABSTRACT_CONTROL_MODEL, CDC_COMM_PROTOCOL_NONE, _stridx,\
  /* CDC Header */\
  5, TUSB_DESC_CS_INTERFACE, CDC_FUNC_DESC_HEADER, U16_TO_U8S_LE(0x0120),\
  /* CDC Call */\
  5, TUSB_DESC_CS_INTERFACE, CDC_FUNC_DESC_CALL_MANAGEMENT, 0, (uint8_t)((_itfnum) + 1),\
  /* CDC ACM: support line request */\
  4, TUSB_DESC_CS_INTERFACE, CDC_FUNC_DESC_ABSTRACT_CONTROL_MANAGEMENT, 2,\
  /* CDC Union */\
  5, TUSB_DESC_CS_INTERFACE, CDC_FUNC_DESC_UNION, _itfnum, (uint8_t)((_itfnum) + 1),\
  /* Endpoint Notification */\
  7, TUSB_DESC_ENDPOINT, _ep_notif, TUSB_XFER_INTERRUPT, U16_TO_U8S_LE(_ep_notif_size), 16,\
  /* CDC Data Interface */\
  9, TUSB_DESC_INTERFACE, (uint8_t)((_itfnum)+1), 0, 2, TUSB_CLASS_CDC_DATA, 0, 0, 0,\
  /* Endpoint Out */\
  7, TUSB_DESC_ENDPOINT, _epout, TUSB_XFER_BULK, U16_TO_U8S_LE(_epsize), 0,\
  /* Endpoint In */\
  7, TUSB_DESC_ENDPOINT, _epin, TUSB_XFER_BULK, U16_TO_U8S_LE(_epsize), 0

// CDC Descriptor Template, without notification EP
// Interface number, string index, EP data address (out, in) and size.
#define TUD_CDC_DESCRIPTOR(_itfnum, _stridx, _epout, _epin, _epsize) \
  /* Interface Associate */\
  8, TUSB_DESC_INTERFACE_ASSOCIATION, _itfnum, 2, TUSB_CLASS_CDC, CDC_COMM_SUBCLASS_ABSTRACT_CONTROL_MODEL, CDC_COMM_PROTOCOL_NONE, 0,\
  /* CDC Control Interface */\
  9, TUSB_DESC_INTERFACE, _itfnum, 0, 0/*1*/, TUSB_CLASS_CDC, CDC_COMM_SUBCLASS_ABSTRACT_CONTROL_MODEL, CDC_COMM_PROTOCOL_NONE, _stridx,\
  /* CDC Header */\
  5, TUSB_DESC_CS_INTERFACE, CDC_FUNC_DESC_HEADER, U16_TO_U8S_LE(0x0120),\
  /* CDC Call */\
  5, TUSB_DESC_CS_INTERFACE, CDC_FUNC_DESC_CALL_MANAGEMENT, 0, (uint8_t)((_itfnum) + 1),\
  /* CDC ACM: support line request */\
  4, TUSB_DESC_CS_INTERFACE, CDC_FUNC_DESC_ABSTRACT_CONTROL_MANAGEMENT, 2,\
  /* CDC Union */\
  5, TUSB_DESC_CS_INTERFACE, CDC_FUNC_DESC_UNION, _itfnum, (uint8_t)((_itfnum) + 1),\
  /* CDC Data Interface */\
  9, TUSB_DESC_INTERFACE, (uint8_t)((_itfnum)+1), 0, 2, TUSB_CLASS_CDC_DATA, 0, 0, 0,\
  /* Endpoint Out */\
  7, TUSB_DESC_ENDPOINT, _epout, TUSB_XFER_BULK, U16_TO_U8S_LE(_epsize), 0,\
  /* Endpoint In */\
  7, TUSB_DESC_ENDPOINT, _epin, TUSB_XFER_BULK, U16_TO_U8S_LE(_epsize), 0

//------------- MSC -------------//

// Length of template descriptor: 23 bytes
#define TUD_MSC_DESC_LEN    (9 + 7 + 7)

// Interface number, string index, EP Out & EP In address, EP size
#define TUD_MSC_DESCRIPTOR(_itfnum, _stridx, _epout, _epin, _epsize) \
  /* Interface */\
  9, TUSB_DESC_INTERFACE, _itfnum, 0, 2, TUSB_CLASS_MSC, MSC_SUBCLASS_SCSI, MSC_PROTOCOL_BOT, _stridx,\
  /* Endpoint Out */\
  7, TUSB_DESC_ENDPOINT, _epout, TUSB_XFER_BULK, U16_TO_U8S_LE(_epsize), 0,\
  /* Endpoint In */\
  7, TUSB_DESC_ENDPOINT, _epin, TUSB_XFER_BULK, U16_TO_U8S_LE(_epsize), 0

//------------- HID -------------//

//BSD: for HID-related macros, e.g. HID_SUBCLASS_BOOT, HID_DESC_TYPE_HID etc.
#if CFG_TUD_HID
#include "class/hid/hid.h"
#endif

// Length of template descriptor: 25 bytes
#define TUD_HID_DESC_LEN    (9 + 9 + 7)

// HID Input only descriptor
// Interface number, string index, protocol, report descriptor len, EP In address, size & polling interval
#define TUD_HID_DESCRIPTOR(_itfnum, _stridx, _boot_protocol, _report_desc_len, _epin, _epsize, _ep_interval) \
  /* Interface */\
  9, TUSB_DESC_INTERFACE, _itfnum, 0, 1, TUSB_CLASS_HID, (uint8_t)((_boot_protocol) ? HID_SUBCLASS_BOOT : 0), _boot_protocol, _stridx,\
  /* HID descriptor */\
  9, HID_DESC_TYPE_HID, U16_TO_U8S_LE(0x0111), 0, 1, HID_DESC_TYPE_REPORT, U16_TO_U8S_LE(_report_desc_len),\
  /* Endpoint In */\
  7, TUSB_DESC_ENDPOINT, _epin, TUSB_XFER_INTERRUPT, U16_TO_U8S_LE(_epsize), _ep_interval

// Length of template descriptor: 32 bytes
#define TUD_HID_INOUT_DESC_LEN    (9 + 9 + 7 + 7)

// HID Input & Output descriptor
// Interface number, string index, protocol, report descriptor len, EP OUT & IN address, size & polling interval
#define TUD_HID_INOUT_DESCRIPTOR(_itfnum, _stridx, _boot_protocol, _report_desc_len, _epout, _epin, _epsize, _ep_interval) \
  /* Interface */\
  9, TUSB_DESC_INTERFACE, _itfnum, 0, 2, TUSB_CLASS_HID, (uint8_t)((_boot_protocol) ? HID_SUBCLASS_BOOT : 0), _boot_protocol, _stridx,\
  /* HID descriptor */\
  9, HID_DESC_TYPE_HID, U16_TO_U8S_LE(0x0111), 0, 1, HID_DESC_TYPE_REPORT, U16_TO_U8S_LE(_report_desc_len),\
  /* Endpoint Out */\
  7, TUSB_DESC_ENDPOINT, _epout, TUSB_XFER_INTERRUPT, U16_TO_U8S_LE(_epsize), _ep_interval, \
  /* Endpoint In */\
  7, TUSB_DESC_ENDPOINT, _epin, TUSB_XFER_INTERRUPT, U16_TO_U8S_LE(_epsize), _ep_interval

#define TUD_HID_INOUT_DESCRIPTOR2(_itfnum, _stridx, _boot_protocol, _report_desc_len, _epout, _epin, \
                                _oepsize, _iepsize, _oep_interval, _iep_interval) \
  /* Interface */\
  9, TUSB_DESC_INTERFACE, _itfnum, 0, 2, TUSB_CLASS_HID, (uint8_t)((_boot_protocol) ? HID_SUBCLASS_BOOT : 0), _boot_protocol, _stridx,\
  /* HID descriptor */\
  9, HID_DESC_TYPE_HID, U16_TO_U8S_LE(0x0111), 0, 1, HID_DESC_TYPE_REPORT, U16_TO_U8S_LE(_report_desc_len),\
  /* Endpoint Out */\
  7, TUSB_DESC_ENDPOINT, _epout, TUSB_XFER_INTERRUPT, U16_TO_U8S_LE(_oepsize), _oep_interval, \
  /* Endpoint In */\
  7, TUSB_DESC_ENDPOINT, _epin, TUSB_XFER_INTERRUPT, U16_TO_U8S_LE(_iepsize), _iep_interval

//------------- MIDI -------------//
// MIDI v1.0 is based on Audio v1.0

#define TUD_MIDI_DESC_HEAD_LEN (9 + 9 + 9 + 7)
#define TUD_MIDI_DESC_HEAD(_itfnum,  _stridx, _numcables) \
  /* Audio Control (AC) Interface */\
  9, TUSB_DESC_INTERFACE, _itfnum, 0, 0, TUSB_CLASS_AUDIO, AUDIO_SUBCLASS_CONTROL, AUDIO_FUNC_PROTOCOL_CODE_UNDEF, _stridx,\
  /* AC Header */\
  9, TUSB_DESC_CS_INTERFACE, AUDIO_CS_AC_INTERFACE_HEADER, U16_TO_U8S_LE(0x0100), U16_TO_U8S_LE(0x0009), 1, (uint8_t)((_itfnum) + 1),\
  /* MIDI Streaming (MS) Interface */\
  9, TUSB_DESC_INTERFACE, (uint8_t)((_itfnum) + 1), 0, 2, TUSB_CLASS_AUDIO, AUDIO_SUBCLASS_MIDI_STREAMING, AUDIO_FUNC_PROTOCOL_CODE_UNDEF, 0,\
  /* MS Header */\
  7, TUSB_DESC_CS_INTERFACE, MIDI_CS_INTERFACE_HEADER, U16_TO_U8S_LE(0x0100), U16_TO_U8S_LE(7 + (_numcables) * TUD_MIDI_DESC_JACK_LEN)

#define TUD_MIDI_JACKID_IN_EMB(_cablenum) \
  (uint8_t)(((_cablenum) - 1) * 4 + 1)

#define TUD_MIDI_JACKID_IN_EXT(_cablenum) \
  (uint8_t)(((_cablenum) - 1) * 4 + 2)

#define TUD_MIDI_JACKID_OUT_EMB(_cablenum) \
  (uint8_t)(((_cablenum) - 1) * 4 + 3)

#define TUD_MIDI_JACKID_OUT_EXT(_cablenum) \
  (uint8_t)(((_cablenum) - 1) * 4 + 4)

#define TUD_MIDI_DESC_JACK_LEN (6 + 6 + 9 + 9)
#define TUD_MIDI_DESC_JACK(_cablenum) \
  /* MS In Jack (Embedded) */\
  6, TUSB_DESC_CS_INTERFACE, MIDI_CS_INTERFACE_IN_JACK, MIDI_JACK_EMBEDDED, TUD_MIDI_JACKID_IN_EMB(_cablenum), 0,\
  /* MS In Jack (External) */\
  6, TUSB_DESC_CS_INTERFACE, MIDI_CS_INTERFACE_IN_JACK, MIDI_JACK_EXTERNAL, TUD_MIDI_JACKID_IN_EXT(_cablenum), 0,\
  /* MS Out Jack (Embedded), connected to In Jack External */\
  9, TUSB_DESC_CS_INTERFACE, MIDI_CS_INTERFACE_OUT_JACK, MIDI_JACK_EMBEDDED, TUD_MIDI_JACKID_OUT_EMB(_cablenum), 1, TUD_MIDI_JACKID_IN_EXT(_cablenum), 1, 0,\
  /* MS Out Jack (External), connected to In Jack Embedded */\
  9, TUSB_DESC_CS_INTERFACE, MIDI_CS_INTERFACE_OUT_JACK, MIDI_JACK_EXTERNAL, TUD_MIDI_JACKID_OUT_EXT(_cablenum), 1, TUD_MIDI_JACKID_IN_EMB(_cablenum), 1, 0

#define TUD_MIDI_DESC_EP_LEN(_numcables) (9 + 4 + (_numcables))
#define TUD_MIDI_DESC_EP(_epout, _epsize, _numcables) \
  /* Endpoint: Note Audio v1.0's endpoint has 9 bytes instead of 7 */\
  9, TUSB_DESC_ENDPOINT, _epout, TUSB_XFER_BULK, U16_TO_U8S_LE(_epsize), 0, 0, 0, \
  /* MS Endpoint (connected to embedded jack) */\
  (uint8_t)(4 + (_numcables)), TUSB_DESC_CS_ENDPOINT, MIDI_CS_ENDPOINT_GENERAL, _numcables

// Length of template descriptor (88 bytes)
#define TUD_MIDI_DESC_LEN (TUD_MIDI_DESC_HEAD_LEN + TUD_MIDI_DESC_JACK_LEN + TUD_MIDI_DESC_EP_LEN(1) * 2)

// MIDI simple descriptor
// - 1 Embedded Jack In connected to 1 External Jack Out
// - 1 Embedded Jack out connected to 1 External Jack In
#define TUD_MIDI_DESCRIPTOR(_itfnum, _stridx, _epout, _epin, _epsize) \
  TUD_MIDI_DESC_HEAD(_itfnum, _stridx, 1),\
  TUD_MIDI_DESC_JACK(1),\
  TUD_MIDI_DESC_EP(_epout, _epsize, 1),\
  TUD_MIDI_JACKID_IN_EMB(1),\
  TUD_MIDI_DESC_EP(_epin, _epsize, 1),\
  TUD_MIDI_JACKID_OUT_EMB(1)

//------------- AUDIO -------------//
//
// BSD NOTE:
// IAD, Clock Source etc. descriptors are specific to USB/UAC 2.0, NOT used by USB1.1 / UAC1.0.
// ADD _V1 suffix to indicate macros are used for USB1.1 / UAC1.0 only.
// ADD _V2 suffix to indicate macros are used for USB2.0 / UAC2.0 only.
// Those macros without any suffix COULD be used for both UAC1.0 and UAC2.0.
//

/* Standard Interface Association Descriptor (IAD) */ // IAD FOR both USB/UAC 2.0 & USB1.1/UAC1.0!!
#define TUD_AUDIO_DESC_IAD_LEN 8

#define TUD_AUDIO_DESC_IAD_LEN_V2  TUD_AUDIO_DESC_IAD_LEN
#define TUD_AUDIO_DESC_IAD_V2(_firstitfs, _nitfs, _stridx) \
  TUD_AUDIO_DESC_IAD_LEN_V2, TUSB_DESC_INTERFACE_ASSOCIATION, _firstitfs, _nitfs, \
  TUSB_CLASS_AUDIO, AUDIO_FUNCTION_SUBCLASS_UNDEFINED, AUDIO_FUNC_PROTOCOL_CODE_V2, _stridx

#define TUD_AUDIO_DESC_IAD_LEN_V1  TUD_AUDIO_DESC_IAD_LEN
#define TUD_AUDIO_DESC_IAD_V1(_firstitfs, _nitfs, _stridx) \
  TUD_AUDIO_DESC_IAD_LEN_V1, TUSB_DESC_INTERFACE_ASSOCIATION, _firstitfs, _nitfs, \
  TUSB_CLASS_AUDIO, AUDIO_FUNCTION_SUBCLASS_UNDEFINED, AUDIO_FUNC_PROTOCOL_CODE_UNDEF, _stridx

/* V2 Standard AC Interface Descriptor(4.7.1) */
#define TUD_AUDIO_DESC_STD_AC_LEN 9
#define TUD_AUDIO_DESC_STD_AC_V2(_itfnum, _nEPs, _stridx) /* _nEPs is 0 or 1 */\
  TUD_AUDIO_DESC_STD_AC_LEN, TUSB_DESC_INTERFACE, _itfnum, /* fixed to zero */ 0x00, _nEPs, \
  TUSB_CLASS_AUDIO, AUDIO_SUBCLASS_CONTROL, AUDIO_INT_PROTOCOL_CODE_V2, _stridx

/* V1 Standard AC Interface Descriptor(4.3.1) */
#define TUD_AUDIO_DESC_STD_AC_V1(_itfnum, _nEPs, _stridx) /* _nEPs is 0 or 1 */\
  TUD_AUDIO_DESC_STD_AC_LEN, TUSB_DESC_INTERFACE, _itfnum, /* fixed to zero */ 0x00, _nEPs, \
  TUSB_CLASS_AUDIO, AUDIO_SUBCLASS_CONTROL, AUDIO_INT_PROTOCOL_CODE_UNDEF, _stridx

/* V2 Class-Specific AC Interface Header Descriptor(4.7.2) */
/* _bcdADC : Audio Device Class Specification Release Number in Binary-Coded Decimal,
 * _category : see audio_function_t,
 * _totallen : Total number of bytes returned for the class-specific AudioControl interface
 *   i.e. Clock Source, Unit and Terminal descriptors - Do not include TUD_AUDIO_DESC_CS_AC_LEN, we already do this here */
#define TUD_AUDIO_DESC_CS_AC_LEN_V2 9
#define TUD_AUDIO_DESC_CS_AC_V2(_bcdADC, _category, _totallen, _ctrl) \
  TUD_AUDIO_DESC_CS_AC_LEN_V2, TUSB_DESC_CS_INTERFACE, AUDIO_CS_AC_INTERFACE_HEADER, U16_TO_U8S_LE(_bcdADC), \
  _category, U16_TO_U8S_LE(_totallen + TUD_AUDIO_DESC_CS_AC_LEN), _ctrl

/* V1 Class-Specific AC Interface Header Descriptor(4.3.2) */
#define TUD_AUDIO_DESC_CS_AC_1AS_LEN_V1 9 // 1 AS for MIC or SPK
#define TUD_AUDIO_DESC_CS_AC_2AS_LEN_V1 10 // 2 AS for MIC and SPK
#define TUD_AUDIO_DESC_CS_AC_1AS_V1(_bcdADC, _totallen, _as_itfnum) /* _totallen excludes length of this structure*/ \
  TUD_AUDIO_DESC_CS_AC_1AS_LEN_V1, TUSB_DESC_CS_INTERFACE, AUDIO_CS_AC_INTERFACE_HEADER, U16_TO_U8S_LE(_bcdADC), \
  U16_TO_U8S_LE(_totallen + TUD_AUDIO_DESC_CS_AC_1AS_LEN_V1), 0x1, _as_itfnum
#define TUD_AUDIO_DESC_CS_AC_2AS_V1(_bcdADC, _totallen, _as1_itfnum, _as2_itfnum) /* _totallen excludes length of this structure*/ \
  TUD_AUDIO_DESC_CS_AC_2AS_LEN_V1, TUSB_DESC_CS_INTERFACE, AUDIO_CS_AC_INTERFACE_HEADER, U16_TO_U8S_LE(_bcdADC), \
  U16_TO_U8S_LE(_totallen + TUD_AUDIO_DESC_CS_AC_2AS_LEN_V1), 0x2, _as1_itfnum, _as2_itfnum

/* V2 Clock Source Descriptor(4.7.2.1) */
/* The actual sampling frequency of the Clock Source can be retrieved through a Get Sampling Frequency request.
 *  In the programmable frequency case, the sampling frequency for this Clock Source can be set through
 *   a Set Sampling Frequency request. Extracted by BSD, Audio20.final.pdf, P49.
 */
#define TUD_AUDIO_DESC_CLK_SRC_LEN_V2 8
#define TUD_AUDIO_DESC_CLK_SRC_V2(_clkid, _attr, _ctrl, _assocTerm, _stridx) \
  TUD_AUDIO_DESC_CLK_SRC_LEN_V2, TUSB_DESC_CS_INTERFACE, AUDIO_CS_AC_INTERFACE_CLOCK_SOURCE, _clkid, _attr, _ctrl, _assocTerm, _stridx

/* V2 Input Terminal Descriptor(4.7.2.4) */
#define TUD_AUDIO_DESC_INPUT_TERM_LEN_V2 17
#define TUD_AUDIO_DESC_INPUT_TERM_V2(_termid, _termtype, _assocTerm, _clkid, _nchannelslogical, _channelcfg, _idxchannelnames, _ctrl, _stridx) \
  TUD_AUDIO_DESC_INPUT_TERM_LEN_V2, TUSB_DESC_CS_INTERFACE, AUDIO_CS_AC_INTERFACE_INPUT_TERMINAL, _termid, U16_TO_U8S_LE(_termtype), _assocTerm, \
  _clkid, _nchannelslogical, U32_TO_U8S_LE(_channelcfg), _idxchannelnames, U16_TO_U8S_LE(_ctrl), _stridx

/* V1 Input Terminal Descriptor(4.3.2.1) */ //FIXME: NOTE macro definitions for _channelcfg!! it could be 0...
#define TUD_AUDIO_DESC_INPUT_TERM_LEN_V1 12
#define TUD_AUDIO_DESC_INPUT_TERM_V1(_termid, _termtype, _assocTerm, _nchannelslogical, _channelcfg, _idxchannelnames, _stridx) \
  TUD_AUDIO_DESC_INPUT_TERM_LEN_V1, TUSB_DESC_CS_INTERFACE, AUDIO_CS_AC_INTERFACE_INPUT_TERMINAL, _termid, U16_TO_U8S_LE(_termtype), _assocTerm, \
  _nchannelslogical, U16_TO_U8S_LE(_channelcfg), _idxchannelnames, _stridx

/* V2 Output Terminal Descriptor(4.7.2.5) */
#define TUD_AUDIO_DESC_OUTPUT_TERM_LEN_V2 12
#define TUD_AUDIO_DESC_OUTPUT_TERM_V2(_termid, _termtype, _assocTerm, _srcid, _clkid, _ctrl, _stridx) \
  TUD_AUDIO_DESC_OUTPUT_TERM_LEN_V2, TUSB_DESC_CS_INTERFACE, AUDIO_CS_AC_INTERFACE_OUTPUT_TERMINAL, _termid, U16_TO_U8S_LE(_termtype), _assocTerm, \
  _srcid, _clkid, U16_TO_U8S_LE(_ctrl), _stridx

/* V1 Output Terminal Descriptor(4.3.2.2) */
#define TUD_AUDIO_DESC_OUTPUT_TERM_LEN_V1 9
#define TUD_AUDIO_DESC_OUTPUT_TERM_V1(_termid, _termtype, _assocTerm, _srcid, _stridx) \
  TUD_AUDIO_DESC_OUTPUT_TERM_LEN_V1, TUSB_DESC_CS_INTERFACE, AUDIO_CS_AC_INTERFACE_OUTPUT_TERMINAL, _termid, U16_TO_U8S_LE(_termtype), _assocTerm, \
  _srcid, _stridx

/* V2 Feature Unit Descriptor(4.7.2.8) */
#define TUD_AUDIO_DESC_FEATURE_UNIT_ONE_CHANNEL_LEN_V2      (6+(1+1)*4)
#define TUD_AUDIO_DESC_FEATURE_UNIT_TWO_CHANNEL_LEN_V2      (6+(2+1)*4)
#define TUD_AUDIO_DESC_FEATURE_UNIT_N_CHANNEL_LEN_V2(n)     (6+(n+1)*4)

// 1 - Channel
#define TUD_AUDIO_DESC_FEATURE_UNIT_ONE_CHANNEL_V2(_unitid, _srcid, _ctrlch0master, _ctrlch1, _stridx) \
  TUD_AUDIO_DESC_FEATURE_UNIT_ONE_CHANNEL_LEN_V2, TUSB_DESC_CS_INTERFACE, AUDIO_CS_AC_INTERFACE_FEATURE_UNIT, _unitid, _srcid, \
  U32_TO_U8S_LE(_ctrlch0master), U32_TO_U8S_LE(_ctrlch1), _stridx

// 2 - Channels
#define TUD_AUDIO_DESC_FEATURE_UNIT_TWO_CHANNEL_V2(_unitid, _srcid, _ctrlch0master, _ctrlch1, _ctrlch2, _stridx) \
  TUD_AUDIO_DESC_FEATURE_UNIT_TWO_CHANNEL_LEN_V2, TUSB_DESC_CS_INTERFACE, AUDIO_CS_AC_INTERFACE_FEATURE_UNIT, _unitid, _srcid, \
  U32_TO_U8S_LE(_ctrlch0master), U32_TO_U8S_LE(_ctrlch1), U32_TO_U8S_LE(_ctrlch2), _stridx

// For more channels, add definitions here

// MIC V2 (IN, TX)
#if (CFG_TUD_AUDIO_N_CHANNELS_TX == 1)
#define MIC_FU_CTRLS_ARRAY_V2(_ctrlch)  \
    U32_TO_U8S_LE(_ctrlch)
#elif (CFG_TUD_AUDIO_N_CHANNELS_TX == 2)
#define MIC_FU_CTRLS_ARRAY_V2(_ctrlch)  \
    U32_TO_U8S_LE(_ctrlch), U32_TO_U8S_LE(_ctrlch)
#elif (CFG_TUD_AUDIO_N_CHANNELS_TX == 3)
#define MIC_FU_CTRLS_ARRAY_V2(_ctrlch)  \
    U32_TO_U8S_LE(_ctrlch), U32_TO_U8S_LE(_ctrlch), U32_TO_U8S_LE(_ctrlch)
#elif (CFG_TUD_AUDIO_N_CHANNELS_TX == 4)
#define MIC_FU_CTRLS_ARRAY_V2(_ctrlch)  \
    U32_TO_U8S_LE(_ctrlch), U32_TO_U8S_LE(_ctrlch), U32_TO_U8S_LE(_ctrlch), U32_TO_U8S_LE(_ctrlch)
#elif (CFG_TUD_AUDIO_N_CHANNELS_TX == 5)
#define MIC_FU_CTRLS_ARRAY_V2(_ctrlch)  \
    U32_TO_U8S_LE(_ctrlch), U32_TO_U8S_LE(_ctrlch), U32_TO_U8S_LE(_ctrlch), \
    U32_TO_U8S_LE(_ctrlch), U32_TO_U8S_LE(_ctrlch)
#elif (CFG_TUD_AUDIO_N_CHANNELS_TX == 6)
#define MIC_FU_CTRLS_ARRAY_V2(_ctrlch)  \
    U32_TO_U8S_LE(_ctrlch), U32_TO_U8S_LE(_ctrlch), U32_TO_U8S_LE(_ctrlch), \
    U32_TO_U8S_LE(_ctrlch), U32_TO_U8S_LE(_ctrlch), U32_TO_U8S_LE(_ctrlch)
#elif (CFG_TUD_AUDIO_N_CHANNELS_TX == 7)
#define MIC_FU_CTRLS_ARRAY_V2(_ctrlch)  \
    U32_TO_U8S_LE(_ctrlch), U32_TO_U8S_LE(_ctrlch), U32_TO_U8S_LE(_ctrlch), U32_TO_U8S_LE(_ctrlch), \
    U32_TO_U8S_LE(_ctrlch), U32_TO_U8S_LE(_ctrlch), U32_TO_U8S_LE(_ctrlch)
#elif (CFG_TUD_AUDIO_N_CHANNELS_TX == 8)
#define MIC_FU_CTRLS_ARRAY_V2(_ctrlch)  \
    U32_TO_U8S_LE(_ctrlch), U32_TO_U8S_LE(_ctrlch), U32_TO_U8S_LE(_ctrlch), U32_TO_U8S_LE(_ctrlch), \
    U32_TO_U8S_LE(_ctrlch), U32_TO_U8S_LE(_ctrlch), U32_TO_U8S_LE(_ctrlch), U32_TO_U8S_LE(_ctrlch)
#endif

// n - Channels (with same bmaControls) for MIC V2
#define TUD_AUDIO_DESC_FEATURE_UNIT_MIC_V2(_unitid, _srcid, _ctrlch0master, _ctrlch, _stridx) \
  TUD_AUDIO_DESC_FEATURE_UNIT_N_CHANNEL_LEN_V2(CFG_TUD_AUDIO_N_CHANNELS_TX), TUSB_DESC_CS_INTERFACE, \
  AUDIO_CS_AC_INTERFACE_FEATURE_UNIT, _unitid, _srcid, U32_TO_U8S_LE(_ctrlch0master), MIC_FU_CTRLS_ARRAY_V2(_ctrlch), _stridx

//SPK V2 (OUT, RX)
#if (CFG_TUD_AUDIO_N_CHANNELS_RX == 1)
#define SPK_FU_CTRLS_ARRAY_V2(_ctrlch)  \
    U32_TO_U8S_LE(_ctrlch)
#elif (CFG_TUD_AUDIO_N_CHANNELS_RX == 2)
#define SPK_FU_CTRLS_ARRAY_V2(_ctrlch)  \
    U32_TO_U8S_LE(_ctrlch), U32_TO_U8S_LE(_ctrlch)
#elif (CFG_TUD_AUDIO_N_CHANNELS_RX == 3)
#define SPK_FU_CTRLS_ARRAY_V2(_ctrlch)  \
    U32_TO_U8S_LE(_ctrlch), U32_TO_U8S_LE(_ctrlch), U32_TO_U8S_LE(_ctrlch)
#elif (CFG_TUD_AUDIO_N_CHANNELS_RX == 4)
#define SPK_FU_CTRLS_ARRAY_V2(_ctrlch)  \
    U32_TO_U8S_LE(_ctrlch), U32_TO_U8S_LE(_ctrlch), U32_TO_U8S_LE(_ctrlch), U32_TO_U8S_LE(_ctrlch)
#elif (CFG_TUD_AUDIO_N_CHANNELS_RX == 5)
#define SPK_FU_CTRLS_ARRAY_V2(_ctrlch)  \
    U32_TO_U8S_LE(_ctrlch), U32_TO_U8S_LE(_ctrlch), U32_TO_U8S_LE(_ctrlch), \
    U32_TO_U8S_LE(_ctrlch), U32_TO_U8S_LE(_ctrlch)
#elif (CFG_TUD_AUDIO_N_CHANNELS_RX == 6)
#define SPK_FU_CTRLS_ARRAY_V2(_ctrlch)  \
    U32_TO_U8S_LE(_ctrlch), U32_TO_U8S_LE(_ctrlch), U32_TO_U8S_LE(_ctrlch), \
    U32_TO_U8S_LE(_ctrlch), U32_TO_U8S_LE(_ctrlch), U32_TO_U8S_LE(_ctrlch)
#elif (CFG_TUD_AUDIO_N_CHANNELS_RX == 7)
#define SPK_FU_CTRLS_ARRAY_V2(_ctrlch)  \
    U32_TO_U8S_LE(_ctrlch), U32_TO_U8S_LE(_ctrlch), U32_TO_U8S_LE(_ctrlch), U32_TO_U8S_LE(_ctrlch), \
    U32_TO_U8S_LE(_ctrlch), U32_TO_U8S_LE(_ctrlch), U32_TO_U8S_LE(_ctrlch)
#elif (CFG_TUD_AUDIO_N_CHANNELS_RX == 8)
#define SPK_FU_CTRLS_ARRAY_V2(_ctrlch)  \
    U32_TO_U8S_LE(_ctrlch), U32_TO_U8S_LE(_ctrlch), U32_TO_U8S_LE(_ctrlch), U32_TO_U8S_LE(_ctrlch), \
    U32_TO_U8S_LE(_ctrlch), U32_TO_U8S_LE(_ctrlch), U32_TO_U8S_LE(_ctrlch), U32_TO_U8S_LE(_ctrlch)
#endif

// n - Channels (with same bmaControls) for SPK V2
#define TUD_AUDIO_DESC_FEATURE_UNIT_SPK_V2(_unitid, _srcid, _ctrlch0master, _ctrlch, _stridx) \
  TUD_AUDIO_DESC_FEATURE_UNIT_N_CHANNEL_LEN_V2(CFG_TUD_AUDIO_N_CHANNELS_RX), TUSB_DESC_CS_INTERFACE, \
  AUDIO_CS_AC_INTERFACE_FEATURE_UNIT, _unitid, _srcid, U32_TO_U8S_LE(_ctrlch0master), SPK_FU_CTRLS_ARRAY_V2(_ctrlch), _stridx


/* V1 Feature Unit Descriptor(4.3.2.5) */
#define TUD_AUDIO_DESC_FEATURE_UNIT_1CH_LEN_V1      (7+(1+1)*2)
#define TUD_AUDIO_DESC_FEATURE_UNIT_2CH_LEN_V1      (7+(2+1)*2)
#define TUD_AUDIO_DESC_FEATURE_UNIT_nCH_LEN_V1(n)   (7+(n+1)*2)

// 1 - Channel
#define TUD_AUDIO_DESC_FEATURE_UNIT_1CH_V1(_unitid, _srcid, _ctrlch0master, _ctrlch1, _stridx) \
  TUD_AUDIO_DESC_FEATURE_UNIT_1CH_LEN_V1, TUSB_DESC_CS_INTERFACE, AUDIO_CS_AC_INTERFACE_FEATURE_UNIT, _unitid, _srcid, 2, \
  U16_TO_U8S_LE(_ctrlch0master), U16_TO_U8S_LE(_ctrlch1), _stridx

// 2 - Channels
#define TUD_AUDIO_DESC_FEATURE_UNIT_2CH_V1(_unitid, _srcid, _ctrlch0master, _ctrlch1, _ctrlch2, _stridx) \
  TUD_AUDIO_DESC_FEATURE_UNIT_2CH_LEN_V1, TUSB_DESC_CS_INTERFACE, AUDIO_CS_AC_INTERFACE_FEATURE_UNIT, _unitid, _srcid, 2, \
  U16_TO_U8S_LE(_ctrlch0master), U16_TO_U8S_LE(_ctrlch1), U16_TO_U8S_LE(_ctrlch2), _stridx

// For more channels, add definitions here

// MIC V1 (IN, TX)
#if (CFG_TUD_AUDIO_N_CHANNELS_TX == 1)
#define MIC_FU_CTRLS_ARRAY_V1(_ctrlch)  \
    U16_TO_U8S_LE(_ctrlch)
#elif (CFG_TUD_AUDIO_N_CHANNELS_TX == 2)
#define MIC_FU_CTRLS_ARRAY_V1(_ctrlch)  \
    U16_TO_U8S_LE(_ctrlch), U16_TO_U8S_LE(_ctrlch)
#elif (CFG_TUD_AUDIO_N_CHANNELS_TX == 3)
#define MIC_FU_CTRLS_ARRAY_V1(_ctrlch)  \
    U16_TO_U8S_LE(_ctrlch), U16_TO_U8S_LE(_ctrlch), U16_TO_U8S_LE(_ctrlch)
#elif (CFG_TUD_AUDIO_N_CHANNELS_TX == 4)
#define MIC_FU_CTRLS_ARRAY_V1(_ctrlch)  \
    U16_TO_U8S_LE(_ctrlch), U16_TO_U8S_LE(_ctrlch), U16_TO_U8S_LE(_ctrlch), U16_TO_U8S_LE(_ctrlch)
#elif (CFG_TUD_AUDIO_N_CHANNELS_TX == 5)
#define MIC_FU_CTRLS_ARRAY_V1(_ctrlch)  \
    U16_TO_U8S_LE(_ctrlch), U16_TO_U8S_LE(_ctrlch), U16_TO_U8S_LE(_ctrlch), \
    U16_TO_U8S_LE(_ctrlch), U16_TO_U8S_LE(_ctrlch)
#elif (CFG_TUD_AUDIO_N_CHANNELS_TX == 6)
#define MIC_FU_CTRLS_ARRAY_V1(_ctrlch)  \
    U16_TO_U8S_LE(_ctrlch), U16_TO_U8S_LE(_ctrlch), U16_TO_U8S_LE(_ctrlch), \
    U16_TO_U8S_LE(_ctrlch), U16_TO_U8S_LE(_ctrlch), U16_TO_U8S_LE(_ctrlch)
#elif (CFG_TUD_AUDIO_N_CHANNELS_TX == 7)
#define MIC_FU_CTRLS_ARRAY_V1(_ctrlch)  \
    U16_TO_U8S_LE(_ctrlch), U16_TO_U8S_LE(_ctrlch), U16_TO_U8S_LE(_ctrlch), U16_TO_U8S_LE(_ctrlch), \
    U16_TO_U8S_LE(_ctrlch), U16_TO_U8S_LE(_ctrlch), U16_TO_U8S_LE(_ctrlch)
#elif (CFG_TUD_AUDIO_N_CHANNELS_TX == 8)
#define MIC_FU_CTRLS_ARRAY_V1(_ctrlch)  \
    U16_TO_U8S_LE(_ctrlch), U16_TO_U8S_LE(_ctrlch), U16_TO_U8S_LE(_ctrlch), U16_TO_U8S_LE(_ctrlch), \
    U16_TO_U8S_LE(_ctrlch), U16_TO_U8S_LE(_ctrlch), U16_TO_U8S_LE(_ctrlch), U16_TO_U8S_LE(_ctrlch)
#endif

// n - Channels (with same bmaControls) for MIC V1
#define TUD_AUDIO_DESC_FEATURE_UNIT_MIC_V1(_unitid, _srcid, _ctrlch0master, _ctrlch, _stridx) \
  TUD_AUDIO_DESC_FEATURE_UNIT_nCH_LEN_V1(CFG_TUD_AUDIO_N_CHANNELS_TX), TUSB_DESC_CS_INTERFACE, \
  AUDIO_CS_AC_INTERFACE_FEATURE_UNIT, _unitid, _srcid, 2, U16_TO_U8S_LE(_ctrlch0master), MIC_FU_CTRLS_ARRAY_V1(_ctrlch), _stridx

// SPK V1 (OUT, RX)
#if (CFG_TUD_AUDIO_N_CHANNELS_RX == 1)
#define SPK_FU_CTRLS_ARRAY_V1(_ctrlch)  \
    U16_TO_U8S_LE(_ctrlch)
#elif (CFG_TUD_AUDIO_N_CHANNELS_RX == 2)
#define SPK_FU_CTRLS_ARRAY_V1(_ctrlch)  \
    U16_TO_U8S_LE(_ctrlch), U16_TO_U8S_LE(_ctrlch)
#elif (CFG_TUD_AUDIO_N_CHANNELS_RX == 3)
#define SPK_FU_CTRLS_ARRAY_V1(_ctrlch)  \
    U16_TO_U8S_LE(_ctrlch), U16_TO_U8S_LE(_ctrlch), U16_TO_U8S_LE(_ctrlch)
#elif (CFG_TUD_AUDIO_N_CHANNELS_RX == 4)
#define SPK_FU_CTRLS_ARRAY_V1(_ctrlch)  \
    U16_TO_U8S_LE(_ctrlch), U16_TO_U8S_LE(_ctrlch), U16_TO_U8S_LE(_ctrlch), U16_TO_U8S_LE(_ctrlch)
#elif (CFG_TUD_AUDIO_N_CHANNELS_RX == 5)
#define SPK_FU_CTRLS_ARRAY_V1(_ctrlch)  \
    U16_TO_U8S_LE(_ctrlch), U16_TO_U8S_LE(_ctrlch), U16_TO_U8S_LE(_ctrlch), \
    U16_TO_U8S_LE(_ctrlch), U16_TO_U8S_LE(_ctrlch)
#elif (CFG_TUD_AUDIO_N_CHANNELS_RX == 6)
#define SPK_FU_CTRLS_ARRAY_V1(_ctrlch)  \
    U16_TO_U8S_LE(_ctrlch), U16_TO_U8S_LE(_ctrlch), U16_TO_U8S_LE(_ctrlch), \
    U16_TO_U8S_LE(_ctrlch), U16_TO_U8S_LE(_ctrlch), U16_TO_U8S_LE(_ctrlch)
#elif (CFG_TUD_AUDIO_N_CHANNELS_RX == 7)
#define SPK_FU_CTRLS_ARRAY_V1(_ctrlch)  \
    U16_TO_U8S_LE(_ctrlch), U16_TO_U8S_LE(_ctrlch), U16_TO_U8S_LE(_ctrlch), U16_TO_U8S_LE(_ctrlch), \
    U16_TO_U8S_LE(_ctrlch), U16_TO_U8S_LE(_ctrlch), U16_TO_U8S_LE(_ctrlch)
#elif (CFG_TUD_AUDIO_N_CHANNELS_RX == 8)
#define SPK_FU_CTRLS_ARRAY_V1(_ctrlch)  \
    U16_TO_U8S_LE(_ctrlch), U16_TO_U8S_LE(_ctrlch), U16_TO_U8S_LE(_ctrlch), U16_TO_U8S_LE(_ctrlch), \
    U16_TO_U8S_LE(_ctrlch), U16_TO_U8S_LE(_ctrlch), U16_TO_U8S_LE(_ctrlch), U16_TO_U8S_LE(_ctrlch)
#endif

// n - Channels (with same bmaControls) for SPK V1
#define TUD_AUDIO_DESC_FEATURE_UNIT_SPK_V1(_unitid, _srcid, _ctrlch0master, _ctrlch, _stridx) \
  TUD_AUDIO_DESC_FEATURE_UNIT_nCH_LEN_V1(CFG_TUD_AUDIO_N_CHANNELS_RX), TUSB_DESC_CS_INTERFACE, \
  AUDIO_CS_AC_INTERFACE_FEATURE_UNIT, _unitid, _srcid, 2, U16_TO_U8S_LE(_ctrlch0master), SPK_FU_CTRLS_ARRAY_V1(_ctrlch), _stridx


/* V2 Standard AS Interface Descriptor(4.9.1) */
#define TUD_AUDIO_DESC_STD_AS_INT_LEN 9
#define TUD_AUDIO_DESC_STD_AS_INT_V2(_itfnum, _altset, _nEPs, _stridx) \
  TUD_AUDIO_DESC_STD_AS_INT_LEN, TUSB_DESC_INTERFACE, _itfnum, _altset, _nEPs, TUSB_CLASS_AUDIO, AUDIO_SUBCLASS_STREAMING, AUDIO_INT_PROTOCOL_CODE_V2, _stridx

/* V1 Standard AS Interface Descriptor(4.5.1) */
#define TUD_AUDIO_DESC_STD_AS_INT_V1(_itfnum, _altset, _nEPs, _stridx) \
  TUD_AUDIO_DESC_STD_AS_INT_LEN, TUSB_DESC_INTERFACE, _itfnum, _altset, _nEPs, TUSB_CLASS_AUDIO, AUDIO_SUBCLASS_STREAMING, AUDIO_INT_PROTOCOL_CODE_UNDEF, _stridx

/* V2 Class-Specific AS Interface Descriptor(4.9.2) */
#define TUD_AUDIO_DESC_CS_AS_INT_LEN_V2 16
#define TUD_AUDIO_DESC_CS_AS_INT_V2(_termid, _ctrl, _formattype, _formats, _nchannelsphysical, _channelcfg, _stridx) \
  TUD_AUDIO_DESC_CS_AS_INT_LEN_V2, TUSB_DESC_CS_INTERFACE, AUDIO_CS_AS_INTERFACE_AS_GENERAL, _termid, _ctrl, _formattype, U32_TO_U8S_LE(_formats), _nchannelsphysical, U32_TO_U8S_LE(_channelcfg), _stridx

/* V1 Class-Specific AS Interface Descriptor(4.5.2) */
#define TUD_AUDIO_DESC_CS_AS_INT_LEN_V1 7
#define TUD_AUDIO_DESC_CS_AS_INT_V1(_termid, _delay, _formattag) \
  TUD_AUDIO_DESC_CS_AS_INT_LEN_V1, TUSB_DESC_CS_INTERFACE, AUDIO_CS_AS_INTERFACE_AS_GENERAL, _termid, _delay, U16_TO_U8S_LE(_formattag)

/* V2 Type I Format Type Descriptor(2.3.1.6 - Audio Formats) */
#define TUD_AUDIO_DESC_TYPE_I_FORMAT_LEN_V2 6
/* _subslotsize is number of bytes per sample (i.e. subslot) and can be 1,2,3, or 4 */
#define TUD_AUDIO_DESC_TYPE_I_FORMAT_V2(_subslotsize, _bitresolution) \
  TUD_AUDIO_DESC_TYPE_I_FORMAT_LEN_V2, TUSB_DESC_CS_INTERFACE, AUDIO_CS_AS_INTERFACE_FORMAT_TYPE, AUDIO_FORMAT_TYPE_I, \
  _subslotsize, _bitresolution

/* V1 Type I Format Type Descriptor(2.2.5 - Audio Formats) */
#define TUD_AUDIO_DESC_TYPE_I_FORMAT_LEN_V1         (8 + 1 * 3) // 1 sampling frequency (default)
#define TUD_AUDIO_DESC_TYPE_I_FORMAT_nSF_LEN_V1(n)  (8 + n * 3) // n sampling frequencies

/* ch_bytes is number of bytes per sample and can be 1,2,3, or 4 */
#define TUD_AUDIO_DESC_TYPE_I_FORMAT_V1(ch_cnt, ch_bytes, ch_bits, samp_freq) \
  TUD_AUDIO_DESC_TYPE_I_FORMAT_LEN_V1, TUSB_DESC_CS_INTERFACE, AUDIO_CS_AS_INTERFACE_FORMAT_TYPE, AUDIO_FORMAT_TYPE_I, \
  ch_cnt, ch_bytes, ch_bits, 1, U24_TO_U8S_LE(samp_freq)

#define TUD_AUDIO_DESC_TYPE_I_FORMAT_2SF_V1(ch_cnt, ch_bytes, ch_bits, samp_freq1, samp_freq2) \
  TUD_AUDIO_DESC_TYPE_I_FORMAT_2SF_LEN_V1, TUSB_DESC_CS_INTERFACE, AUDIO_CS_AS_INTERFACE_FORMAT_TYPE, AUDIO_FORMAT_TYPE_I, \
  ch_cnt, ch_bytes, ch_bits, 2, U24_TO_U8S_LE(samp_freq1), U24_TO_U8S_LE(samp_freq2)

// For more sampling frequencies, add definitions here

/* V2 Standard AS Isochronous Audio Data Endpoint Descriptor(4.10.1.1) */
#define TUD_AUDIO_DESC_STD_AS_ISO_EP_LEN_V2 7
#define TUD_AUDIO_DESC_STD_AS_ISO_EP_V2(_ep, _attr, _maxEPsize, _interval) \
  TUD_AUDIO_DESC_STD_AS_ISO_EP_LEN_V2, TUSB_DESC_ENDPOINT, _ep, _attr, U16_TO_U8S_LE(_maxEPsize), _interval

/* V1 Standard AS Isochronous Audio Data Endpoint Descriptor(4.6.1.1) */
#define TUD_AUDIO_DESC_STD_AS_ISO_EP_LEN_V1 9
#define TUD_AUDIO_DESC_STD_AS_ISO_EP_V1(_ep, _attr, _maxEPsize, _interval) \
  TUD_AUDIO_DESC_STD_AS_ISO_EP_LEN_V1, TUSB_DESC_ENDPOINT, _ep, _attr, U16_TO_U8S_LE(_maxEPsize), _interval, 0, 0

/* V2 Class-Specific AS Isochronous Audio Data Endpoint Descriptor(4.10.1.2) */
#define TUD_AUDIO_DESC_CS_AS_ISO_EP_LEN_V2 8
#define TUD_AUDIO_DESC_CS_AS_ISO_EP_V2(_attr, _ctrl, _lockdelayunit, _lockdelay) \
  TUD_AUDIO_DESC_CS_AS_ISO_EP_LEN_V2, TUSB_DESC_CS_ENDPOINT, AUDIO_CS_EP_SUBTYPE_GENERAL, _attr, _ctrl, \
  _lockdelayunit, U16_TO_U8S_LE(_lockdelay)

/* V1 Class-Specific AS Isochronous Audio Data Endpoint Descriptor(4.6.1.2) */
#define TUD_AUDIO_DESC_CS_AS_ISO_EP_LEN_V1 7
#define TUD_AUDIO_DESC_CS_AS_ISO_EP_V1(_attr, _lockdelayunit, _lockdelay) \
  TUD_AUDIO_DESC_CS_AS_ISO_EP_LEN_V1, TUSB_DESC_CS_ENDPOINT, AUDIO_CS_EP_SUBTYPE_GENERAL, _attr, _lockdelayunit, U16_TO_U8S_LE(_lockdelay)

/* V2 Standard AS Isochronous Feedback Endpoint Descriptor(4.10.2.1) */
#define TUD_AUDIO_DESC_STD_AS_ISO_FB_EP_LEN_V2 7
#define TUD_AUDIO_DESC_STD_AS_ISO_FB_EP_V2(_ep, _interval) \
  TUD_AUDIO_DESC_STD_AS_ISO_FB_EP_LEN_V2, TUSB_DESC_ENDPOINT, _ep, \
  (TUSB_XFER_ISOCHRONOUS | TUSB_ISO_EP_ATT_NO_SYNC | TUSB_ISO_EP_ATT_EXPLICIT_FB), U16_TO_U8S_LE(4), _interval

/* Standard AS Isochronous Feedback Endpoint Descriptor(4.6.2.1) */
#define TUD_AUDIO_DESC_STD_AS_ISO_FB_EP_LEN_V1 9
#define TUD_AUDIO_DESC_STD_AS_ISO_FB_EP_V1(_ep, _interval, _refresh) \
  TUD_AUDIO_DESC_STD_AS_ISO_FB_EP_LEN_V1, TUSB_DESC_ENDPOINT, _ep, \
  (TUSB_XFER_ISOCHRONOUS | TUSB_ISO_EP_ATT_NO_SYNC), U16_TO_U8S_LE(4), _interval, _refresh /*1:2ms ~ 9:512ms*/, 0

#define UAC1_STREAM_COUNT(ac_itf)  ((uint8_t*)(ac_itf))[TUD_AUDIO_DESC_STD_AC_LEN + 7]
#define UAC1_STREAM_INTFNO(ac_itf, i)  ((uint8_t*)(ac_itf))[TUD_AUDIO_DESC_STD_AC_LEN + 7 + i]

//------------------------------------------------------------------------------------------------
// AUDIO simple descriptor (UAC2) for 1 microphone input
// - 1 Input Terminal, 1 Feature Unit (Mute and Volume Control), 1 Output Terminal, 1 Clock Source

//#define TUD_AUDIO_MIC_DESC_N_AS_INT 1 	// Number of AS interfaces

//#define TUD_AUDIO_MIC_DESC_LEN (TUD_AUDIO_DESC_IAD_LEN\
//  + TUD_AUDIO_DESC_STD_AC_LEN\
//  + TUD_AUDIO_DESC_CS_AC_LEN\
//  + TUD_AUDIO_DESC_CLK_SRC_LEN\
//  + TUD_AUDIO_DESC_INPUT_TERM_LEN\
//  + TUD_AUDIO_DESC_OUTPUT_TERM_LEN\
//  + TUD_AUDIO_DESC_FEATURE_UNIT_ONE_CHANNEL_LEN\
//  + TUD_AUDIO_DESC_STD_AS_INT_LEN\
//  + TUD_AUDIO_DESC_STD_AS_INT_LEN\
//  + TUD_AUDIO_DESC_CS_AS_INT_LEN\
//  + TUD_AUDIO_DESC_TYPE_I_FORMAT_LEN\
//  + TUD_AUDIO_DESC_STD_AS_ISO_EP_LEN\
//  + TUD_AUDIO_DESC_CS_AS_ISO_EP_LEN)

//#define TUD_AUDIO_MIC_DESCRIPTOR(_itfnum, _stridx, _nBytesPerSample, _nBitsUsedPerSample, _epin, _epsize) \
//  /* Standard Interface Association Descriptor (IAD) */\
//  TUD_AUDIO_DESC_IAD(/*_firstitfs*/ _itfnum, /*_nitfs*/ 0x02, /*_stridx*/ 0x00),\
//  /* Standard AC Interface Descriptor(4.7.1) */\
//  TUD_AUDIO_DESC_STD_AC(/*_itfnum*/ _itfnum, /*_nEPs*/ 0x00, /*_stridx*/ _stridx),\
//  /* Class-Specific AC Interface Header Descriptor(4.7.2) */\
//  TUD_AUDIO_DESC_CS_AC(/*_bcdADC*/ 0x0200, /*_category*/ AUDIO_FUNC_MICROPHONE, /*_totallen*/ TUD_AUDIO_DESC_CLK_SRC_LEN+TUD_AUDIO_DESC_INPUT_TERM_LEN+TUD_AUDIO_DESC_OUTPUT_TERM_LEN+TUD_AUDIO_DESC_FEATURE_UNIT_ONE_CHANNEL_LEN, /*_ctrl*/ AUDIO_CS_AS_INTERFACE_CTRL_LATENCY_POS),\
//  /* Clock Source Descriptor(4.7.2.1) */\
//  TUD_AUDIO_DESC_CLK_SRC(/*_clkid*/ 0x04, /*_attr*/ AUDIO_CLOCK_SOURCE_ATT_INT_FIX_CLK, /*_ctrl*/ (AUDIO_CTRL_R << AUDIO_CLOCK_SOURCE_CTRL_CLK_FRQ_POS), /*_assocTerm*/ 0x01,  /*_stridx*/ 0x00),\
//  /* Input Terminal Descriptor(4.7.2.4) */\
//  TUD_AUDIO_DESC_INPUT_TERM(/*_termid*/ 0x01, /*_termtype*/ AUDIO_TERM_TYPE_IN_GENERIC_MIC, /*_assocTerm*/ 0x03, /*_clkid*/ 0x04, /*_nchannelslogical*/ 0x01, /*_channelcfg*/ AUDIO_CHANNEL_CONFIG_NON_PREDEFINED, /*_idxchannelnames*/ 0x00, /*_ctrl*/ AUDIO_CTRL_R << AUDIO_IN_TERM_CTRL_CONNECTOR_POS, /*_stridx*/ 0x00),\
//  /* Output Terminal Descriptor(4.7.2.5) */\
//  TUD_AUDIO_DESC_OUTPUT_TERM(/*_termid*/ 0x03, /*_termtype*/ AUDIO_TERM_TYPE_USB_STREAMING, /*_assocTerm*/ 0x01, /*_srcid*/ 0x02, /*_clkid*/ 0x04, /*_ctrl*/ 0x0000, /*_stridx*/ 0x00),\
//  /* Feature Unit Descriptor(4.7.2.8) */\
//  TUD_AUDIO_DESC_FEATURE_UNIT_ONE_CHANNEL(/*_unitid*/ 0x02, /*_srcid*/ 0x01, /*_ctrlch0master*/ AUDIO_CTRL_RW << AUDIO_FEATURE_UNIT_CTRL_MUTE_POS | AUDIO_CTRL_RW << AUDIO_FEATURE_UNIT_CTRL_VOLUME_POS, /*_ctrlch1*/ AUDIO_CTRL_RW << AUDIO_FEATURE_UNIT_CTRL_MUTE_POS | AUDIO_CTRL_RW << AUDIO_FEATURE_UNIT_CTRL_VOLUME_POS, /*_stridx*/ 0x00),\
//  /* Standard AS Interface Descriptor(4.9.1) */\
//  /* Interface 1, Alternate 0 - default alternate setting with 0 bandwidth */\
//  TUD_AUDIO_DESC_STD_AS_INT(/*_itfnum*/ (uint8_t)((_itfnum)+1), /*_altset*/ 0x00, /*_nEPs*/ 0x00, /*_stridx*/ 0x00),\
//  /* Standard AS Interface Descriptor(4.9.1) */\
//  /* Interface 1, Alternate 1 - alternate interface for data streaming */\
//  TUD_AUDIO_DESC_STD_AS_INT(/*_itfnum*/ (uint8_t)((_itfnum)+1), /*_altset*/ 0x01, /*_nEPs*/ 0x01, /*_stridx*/ 0x00),\
//  /* Class-Specific AS Interface Descriptor(4.9.2) */\
//  TUD_AUDIO_DESC_CS_AS_INT(/*_termid*/ 0x03, /*_ctrl*/ AUDIO_CTRL_NONE, /*_formattype*/ AUDIO_FORMAT_TYPE_I, /*_formats*/ AUDIO_DATA_FORMAT_TYPE_I_PCM, /*_nchannelsphysical*/ 0x01, /*_channelcfg*/ AUDIO_CHANNEL_CONFIG_NON_PREDEFINED, /*_stridx*/ 0x00),\
//  /* Type I Format Type Descriptor(2.3.1.6 - Audio Formats) */\
//  TUD_AUDIO_DESC_TYPE_I_FORMAT(_nBytesPerSample, _nBitsUsedPerSample),\
//  /* Standard AS Isochronous Audio Data Endpoint Descriptor(4.10.1.1) */\
//  TUD_AUDIO_DESC_STD_AS_ISO_EP(/*_ep*/ _epin, /*_attr*/ (TUSB_XFER_ISOCHRONOUS | TUSB_ISO_EP_ATT_ASYNCHRONOUS | TUSB_ISO_EP_ATT_DATA), /*_maxEPsize*/ _epsize, /*_interval*/ (CFG_TUSB_RHPORT0_MODE & OPT_MODE_HIGH_SPEED) ? 0x04 : 0x01),\
//  /* Class-Specific AS Isochronous Audio Data Endpoint Descriptor(4.10.1.2) */\
//  TUD_AUDIO_DESC_CS_AS_ISO_EP(/*_attr*/ AUDIO_CS_AS_ISO_DATA_EP_ATT_NON_MAX_PACKETS_OK, /*_ctrl*/ AUDIO_CTRL_NONE, /*_lockdelayunit*/ AUDIO_CS_AS_ISO_DATA_EP_LOCK_DELAY_UNIT_UNDEFINED, /*_lockdelay*/ 0x0000)

// V2: MIC, N channel
#define TUD_AUDIO_MIC_DESC_LEN_V2 ( TUD_AUDIO_DESC_IAD_LEN_V2 \
  + TUD_AUDIO_DESC_STD_AC_LEN \
  + TUD_AUDIO_DESC_CS_AC_LEN_V2 \
  + TUD_AUDIO_DESC_CLK_SRC_LEN_V2 \
  + TUD_AUDIO_DESC_INPUT_TERM_LEN_V2 \
  + TUD_AUDIO_DESC_OUTPUT_TERM_LEN_V2 \
  + TUD_AUDIO_DESC_FEATURE_UNIT_N_CHANNEL_LEN_V2(CFG_TUD_AUDIO_N_CHANNELS_TX) \
  + TUD_AUDIO_DESC_STD_AS_INT_LEN \
  + TUD_AUDIO_DESC_STD_AS_INT_LEN \
  + TUD_AUDIO_DESC_CS_AS_INT_LEN_V2 \
  + TUD_AUDIO_DESC_TYPE_I_FORMAT_LEN_V2 \
  + TUD_AUDIO_DESC_STD_AS_ISO_EP_LEN_V2 \
  + TUD_AUDIO_DESC_CS_AS_ISO_EP_LEN_V2 )

#define TUD_AUDIO_MIC_DESCRIPTOR_V2(_itfnum, _stridx, _nBytesPerSample, _nBitsUsedPerSample, _epin, _epsize) \
  /* Standard Interface Association Descriptor (IAD) */\
  TUD_AUDIO_DESC_IAD_V2(/*_firstitfs*/ _itfnum, /*_nitfs*/ 0x02, /*_stridx*/ 0x00),\
  /* Standard AC Interface Descriptor(4.7.1) */\
  TUD_AUDIO_DESC_STD_AC_V2(/*_itfnum*/ _itfnum, /*_nEPs*/ 0x00, /*_stridx*/ _stridx),\
  /* Class-Specific AC Interface Header Descriptor(4.7.2) */\
  TUD_AUDIO_DESC_CS_AC_V2(/*_bcdADC*/ 0x0200, /*_category*/ AUDIO_FUNC_MICROPHONE, \
          /*_totallen*/ TUD_AUDIO_DESC_CLK_SRC_LEN_V2 + TUD_AUDIO_DESC_INPUT_TERM_LEN_V2 + \
          TUD_AUDIO_DESC_OUTPUT_TERM_LEN_V2 + TUD_AUDIO_DESC_FEATURE_UNIT_N_CHANNEL_LEN_V2(CFG_TUD_AUDIO_N_CHANNELS_TX), \
          /*_ctrl*/ AUDIO_CS_AS_INTERFACE_CTRL_LATENCY_POS),\
  /* Clock Source Descriptor(4.7.2.1) */\
  TUD_AUDIO_DESC_CLK_SRC_V2(/*_clkid*/ 0x04, /*_attr*/ AUDIO_CLOCK_SOURCE_ATT_INT_FIX_CLK, \
          /*_ctrl*/ (AUDIO_CTRL_R << AUDIO_CLOCK_SOURCE_CTRL_CLK_FRQ_POS), /*_assocTerm*/ 0x01,  /*_stridx*/ 0x00),\
  /* Input Terminal Descriptor(4.7.2.4) */\
  TUD_AUDIO_DESC_INPUT_TERM_V2(/*_termid*/ 0x01, /*_termtype*/ AUDIO_TERM_TYPE_IN_GENERIC_MIC, /*_assocTerm*/ 0x03, \
          /*_clkid*/ 0x04, /*_nchannelslogical*/ CFG_TUD_AUDIO_N_CHANNELS_TX, /*_channelcfg*/ AUDIO_CHANNEL_CONFIG_NON_PREDEFINED, \
          /*_idxchannelnames*/ 0x00, /*_ctrl*/ AUDIO_CTRL_R << AUDIO_IN_TERM_CTRL_CONNECTOR_POS, /*_stridx*/ 0x00),\
  /* Output Terminal Descriptor(4.7.2.5) */\
  TUD_AUDIO_DESC_OUTPUT_TERM_V2(/*_termid*/ 0x03, /*_termtype*/ AUDIO_TERM_TYPE_USB_STREAMING, /*_assocTerm*/ 0x01, \
          /*_srcid*/ 0x02, /*_clkid*/ 0x04, /*_ctrl*/ 0x0000, /*_stridx*/ 0x00),\
  /* Feature Unit Descriptor(4.7.2.8) */\
  TUD_AUDIO_DESC_FEATURE_UNIT_MIC_V2(/*_unitid*/ 0x02, /*_srcid*/ 0x01, \
          /*_ctrlch0master*/ (AUDIO_CTRL_RW << AUDIO_FEATURE_UNIT_CTRL_MUTE_POS) | (AUDIO_CTRL_RW << AUDIO_FEATURE_UNIT_CTRL_VOLUME_POS), \
          /*_ctrlch*/ (AUDIO_CTRL_RW << AUDIO_FEATURE_UNIT_CTRL_MUTE_POS) | (AUDIO_CTRL_RW << AUDIO_FEATURE_UNIT_CTRL_VOLUME_POS), /*_stridx*/ 0x00),\
  /* Standard AS Interface Descriptor(4.9.1) */\
  /* Interface 1, Alternate 0 - default alternate setting with 0 bandwidth */\
  TUD_AUDIO_DESC_STD_AS_INT_V2(/*_itfnum*/ (uint8_t)((_itfnum)+1), /*_altset*/ 0x00, /*_nEPs*/ 0x00, /*_stridx*/ 0x00),\
  /* Standard AS Interface Descriptor(4.9.1) */\
  /* Interface 1, Alternate 1 - alternate interface for data streaming */\
  TUD_AUDIO_DESC_STD_AS_INT_V2(/*_itfnum*/ (uint8_t)((_itfnum)+1), /*_altset*/ 0x01, /*_nEPs*/ 0x01, /*_stridx*/ 0x00),\
  /* Class-Specific AS Interface Descriptor(4.9.2) */\
  TUD_AUDIO_DESC_CS_AS_INT_V2(/*_termid*/ 0x03, /*_ctrl*/ AUDIO_CTRL_NONE, /*_formattype*/ AUDIO_FORMAT_TYPE_I, \
          /*_formats*/ AUDIO_DATA_FORMAT_TYPE_I_PCM, /*_nchannelsphysical*/ 0x01, /*_channelcfg*/ AUDIO_CHANNEL_CONFIG_NON_PREDEFINED, /*_stridx*/ 0x00),\
  /* Type I Format Type Descriptor(2.3.1.6 - Audio Formats) */\
  TUD_AUDIO_DESC_TYPE_I_FORMAT_V2(_nBytesPerSample, _nBitsUsedPerSample),\
  /* Standard AS Isochronous Audio Data Endpoint Descriptor(4.10.1.1) */\
  TUD_AUDIO_DESC_STD_AS_ISO_EP_V2(/*_ep*/ _epin, /*_attr*/ (TUSB_XFER_ISOCHRONOUS | TUSB_ISO_EP_ATT_ASYNCHRONOUS | TUSB_ISO_EP_ATT_DATA), \
          /*_maxEPsize*/ _epsize, /*_interval*/ (CFG_TUSB_RHPORT0_MODE & OPT_MODE_HIGH_SPEED) ? 0x04 : 0x01),\
  /* Class-Specific AS Isochronous Audio Data Endpoint Descriptor(4.10.1.2) */\
  TUD_AUDIO_DESC_CS_AS_ISO_EP_V2(/*_attr*/ AUDIO_CS_AS_ISO_DATA_EP_ATT_NON_MAX_PACKETS_OK, /*_ctrl*/ AUDIO_CTRL_NONE, \
          /*_lockdelayunit*/ AUDIO_CS_AS_ISO_DATA_EP_LOCK_DELAY_UNIT_UNDEFINED, /*_lockdelay*/ 0x0000)


// AUDIO simple descriptor (UAC2) for mono speaker
// - 1 Input Terminal, 2 Feature Unit (Mute and Volume Control), 3 Output Terminal, 4 Clock Source

//#define TUD_AUDIO_SPEAKER_MONO_FB_DESC_LEN (TUD_AUDIO_DESC_IAD_LEN\
//  + TUD_AUDIO_DESC_STD_AC_LEN\
//  + TUD_AUDIO_DESC_CS_AC_LEN\
//  + TUD_AUDIO_DESC_CLK_SRC_LEN\
//  + TUD_AUDIO_DESC_INPUT_TERM_LEN\
//  + TUD_AUDIO_DESC_OUTPUT_TERM_LEN\
//  + TUD_AUDIO_DESC_FEATURE_UNIT_ONE_CHANNEL_LEN\
//  + TUD_AUDIO_DESC_STD_AS_INT_LEN\
//  + TUD_AUDIO_DESC_STD_AS_INT_LEN\
//  + TUD_AUDIO_DESC_CS_AS_INT_LEN\
//  + TUD_AUDIO_DESC_TYPE_I_FORMAT_LEN\
//  + TUD_AUDIO_DESC_STD_AS_ISO_EP_LEN\
//  + TUD_AUDIO_DESC_CS_AS_ISO_EP_LEN\
//  + TUD_AUDIO_DESC_STD_AS_ISO_FB_EP_LEN)

//#define TUD_AUDIO_SPEAKER_MONO_FB_DESCRIPTOR(_itfnum, _stridx, _nBytesPerSample, _nBitsUsedPerSample, _epout, _epsize, _epfb) \
//  /* Standard Interface Association Descriptor (IAD) */\
//  TUD_AUDIO_DESC_IAD(/*_firstitfs*/ _itfnum, /*_nitfs*/ 0x02, /*_stridx*/ 0x00),\
//  /* Standard AC Interface Descriptor(4.7.1) */\
//  TUD_AUDIO_DESC_STD_AC(/*_itfnum*/ _itfnum, /*_nEPs*/ 0x00, /*_stridx*/ _stridx),\
//  /* Class-Specific AC Interface Header Descriptor(4.7.2) */\
//  TUD_AUDIO_DESC_CS_AC(/*_bcdADC*/ 0x0200, /*_category*/ AUDIO_FUNC_DESKTOP_SPEAKER, /*_totallen*/ TUD_AUDIO_DESC_CLK_SRC_LEN+TUD_AUDIO_DESC_INPUT_TERM_LEN+TUD_AUDIO_DESC_OUTPUT_TERM_LEN+TUD_AUDIO_DESC_FEATURE_UNIT_ONE_CHANNEL_LEN, /*_ctrl*/ AUDIO_CS_AS_INTERFACE_CTRL_LATENCY_POS),\
//  /* Clock Source Descriptor(4.7.2.1) */\
//  TUD_AUDIO_DESC_CLK_SRC(/*_clkid*/ 0x04, /*_attr*/ AUDIO_CLOCK_SOURCE_ATT_INT_FIX_CLK, /*_ctrl*/ (AUDIO_CTRL_R << AUDIO_CLOCK_SOURCE_CTRL_CLK_FRQ_POS), /*_assocTerm*/ 0x01,  /*_stridx*/ 0x00),\
//  /* Input Terminal Descriptor(4.7.2.4) */\
//  TUD_AUDIO_DESC_INPUT_TERM(/*_termid*/ 0x01, /*_termtype*/ AUDIO_TERM_TYPE_USB_STREAMING, /*_assocTerm*/ 0x00, /*_clkid*/ 0x04, /*_nchannelslogical*/ 0x01, /*_channelcfg*/ AUDIO_CHANNEL_CONFIG_NON_PREDEFINED, /*_idxchannelnames*/ 0x00, /*_ctrl*/ 0 * (AUDIO_CTRL_R << AUDIO_IN_TERM_CTRL_CONNECTOR_POS), /*_stridx*/ 0x00),\
//  /* Output Terminal Descriptor(4.7.2.5) */\
//  TUD_AUDIO_DESC_OUTPUT_TERM(/*_termid*/ 0x03, /*_termtype*/ AUDIO_TERM_TYPE_OUT_DESKTOP_SPEAKER, /*_assocTerm*/ 0x01, /*_srcid*/ 0x02, /*_clkid*/ 0x04, /*_ctrl*/ 0x0000, /*_stridx*/ 0x00),\
//  /* Feature Unit Descriptor(4.7.2.8) */\
//  TUD_AUDIO_DESC_FEATURE_UNIT_ONE_CHANNEL(/*_unitid*/ 0x02, /*_srcid*/ 0x01, /*_ctrlch0master*/ 0 * (AUDIO_CTRL_RW << AUDIO_FEATURE_UNIT_CTRL_MUTE_POS | AUDIO_CTRL_RW << AUDIO_FEATURE_UNIT_CTRL_VOLUME_POS), /*_ctrlch1*/ 0 * (AUDIO_CTRL_RW << AUDIO_FEATURE_UNIT_CTRL_MUTE_POS | AUDIO_CTRL_RW << AUDIO_FEATURE_UNIT_CTRL_VOLUME_POS), /*_stridx*/ 0x00),\
//  /* Standard AS Interface Descriptor(4.9.1) */\
//  /* Interface 1, Alternate 0 - default alternate setting with 0 bandwidth */\
//  TUD_AUDIO_DESC_STD_AS_INT(/*_itfnum*/ (uint8_t)((_itfnum) + 1), /*_altset*/ 0x00, /*_nEPs*/ 0x00, /*_stridx*/ 0x00),\
//  /* Standard AS Interface Descriptor(4.9.1) */\
//  /* Interface 1, Alternate 1 - alternate interface for data streaming */\
//  TUD_AUDIO_DESC_STD_AS_INT(/*_itfnum*/ (uint8_t)((_itfnum) + 1), /*_altset*/ 0x01, /*_nEPs*/ 0x02, /*_stridx*/ 0x00),\
//  /* Class-Specific AS Interface Descriptor(4.9.2) */\
//  TUD_AUDIO_DESC_CS_AS_INT(/*_termid*/ 0x01, /*_ctrl*/ AUDIO_CTRL_NONE, /*_formattype*/ AUDIO_FORMAT_TYPE_I, /*_formats*/ AUDIO_DATA_FORMAT_TYPE_I_PCM, /*_nchannelsphysical*/ 0x01, /*_channelcfg*/ AUDIO_CHANNEL_CONFIG_NON_PREDEFINED, /*_stridx*/ 0x00),\
//  /* Type I Format Type Descriptor(2.3.1.6 - Audio Formats) */\
//  TUD_AUDIO_DESC_TYPE_I_FORMAT(_nBytesPerSample, _nBitsUsedPerSample),\
//  /* Standard AS Isochronous Audio Data Endpoint Descriptor(4.10.1.1) */\
//  TUD_AUDIO_DESC_STD_AS_ISO_EP(/*_ep*/ _epout, /*_attr*/ (TUSB_XFER_ISOCHRONOUS | TUSB_ISO_EP_ATT_ASYNCHRONOUS | TUSB_ISO_EP_ATT_DATA), /*_maxEPsize*/ _epsize, /*_interval*/ (CFG_TUSB_RHPORT0_MODE & OPT_MODE_HIGH_SPEED) ? 0x04 : 0x01),\
//  /* Class-Specific AS Isochronous Audio Data Endpoint Descriptor(4.10.1.2) */\
//  TUD_AUDIO_DESC_CS_AS_ISO_EP(/*_attr*/ AUDIO_CS_AS_ISO_DATA_EP_ATT_NON_MAX_PACKETS_OK, /*_ctrl*/ AUDIO_CTRL_NONE, /*_lockdelayunit*/ AUDIO_CS_AS_ISO_DATA_EP_LOCK_DELAY_UNIT_UNDEFINED, /*_lockdelay*/ 0x0000),\
//  /* Standard AS Isochronous Feedback Endpoint Descriptor(4.10.2.1) */\
//  TUD_AUDIO_DESC_STD_AS_ISO_FB_EP(/*_ep*/ _epfb, /*_interval*/ 1)\

// V2: SPK, N Channels, FB
#define TUD_AUDIO_SPK_FB_DESC_LEN_V2 (TUD_AUDIO_DESC_IAD_LEN_V2 \
  + TUD_AUDIO_DESC_STD_AC_LEN \
  + TUD_AUDIO_DESC_CS_AC_LEN_V2 \
  + TUD_AUDIO_DESC_CLK_SRC_LEN_V2 \
  + TUD_AUDIO_DESC_INPUT_TERM_LEN_V2 \
  + TUD_AUDIO_DESC_OUTPUT_TERM_LEN_V2 \
  + TUD_AUDIO_DESC_FEATURE_UNIT_N_CHANNEL_LEN_V2(CFG_TUD_AUDIO_N_CHANNELS_RX) /*BSD NOTE*/ \
  + TUD_AUDIO_DESC_STD_AS_INT_LEN \
  + TUD_AUDIO_DESC_STD_AS_INT_LEN \
  + TUD_AUDIO_DESC_CS_AS_INT_LEN_V2 \
  + TUD_AUDIO_DESC_TYPE_I_FORMAT_LEN_V2 \
  + TUD_AUDIO_DESC_STD_AS_ISO_EP_LEN_V2 \
  + TUD_AUDIO_DESC_CS_AS_ISO_EP_LEN_V2 \
  + TUD_AUDIO_DESC_STD_AS_ISO_FB_EP_LEN_V2)

#define TUD_AUDIO_SPK_FB_DESCRIPTOR_V2(_itfnum, _stridx, _nBytesPerSample, _nBitsUsedPerSample, _epout, _epsize, _epfb) \
  /* Standard Interface Association Descriptor (IAD) */\
  TUD_AUDIO_DESC_IAD_V2(/*_firstitfs*/ _itfnum, /*_nitfs*/ 0x02, /*_stridx*/ 0x00),\
  /* Standard AC Interface Descriptor(4.7.1) */\
  TUD_AUDIO_DESC_STD_AC_V2(/*_itfnum*/ _itfnum, /*_nEPs*/ 0x00, /*_stridx*/ _stridx),\
  /* Class-Specific AC Interface Header Descriptor(4.7.2) */\
  TUD_AUDIO_DESC_CS_AC_V2(/*_bcdADC*/ 0x0200, /*_category*/ AUDIO_FUNC_DESKTOP_SPEAKER, \
          /*_totallen*/ TUD_AUDIO_DESC_CLK_SRC_LEN_V2 + TUD_AUDIO_DESC_INPUT_TERM_LEN_V2 + \
          TUD_AUDIO_DESC_OUTPUT_TERM_LEN_V2 + TUD_AUDIO_DESC_FEATURE_UNIT_N_CHANNEL_LEN_V2(CFG_TUD_AUDIO_N_CHANNELS_RX), /*BSD NOTE*/ \
          /*_ctrl*/ AUDIO_CS_AS_INTERFACE_CTRL_LATENCY_POS),\
  /* Clock Source Descriptor(4.7.2.1) */\
  TUD_AUDIO_DESC_CLK_SRC_V2(/*_clkid*/ 0x04, /*_attr*/ AUDIO_CLOCK_SOURCE_ATT_INT_FIX_CLK, \
          /*_ctrl*/ (AUDIO_CTRL_R << AUDIO_CLOCK_SOURCE_CTRL_CLK_FRQ_POS), /*_assocTerm*/ 0x01,  /*_stridx*/ 0x00),\
  /* Input Terminal Descriptor(4.7.2.4) */\
  TUD_AUDIO_DESC_INPUT_TERM_V2(/*_termid*/ 0x01, /*_termtype*/ AUDIO_TERM_TYPE_USB_STREAMING, /*_assocTerm*/ 0x00, \
          /*_clkid*/ 0x04, /*_nchannelslogical*/ CFG_TUD_AUDIO_N_CHANNELS_RX, /*_channelcfg*/ AUDIO_CHANNEL_CONFIG_NON_PREDEFINED, \
          /*_idxchannelnames*/ 0x00, /*_ctrl*/ 0 * (AUDIO_CTRL_R << AUDIO_IN_TERM_CTRL_CONNECTOR_POS), /*_stridx*/ 0x00),\
  /* Output Terminal Descriptor(4.7.2.5) */\
  TUD_AUDIO_DESC_OUTPUT_TERM_V2(/*_termid*/ 0x03, /*_termtype*/ AUDIO_TERM_TYPE_OUT_DESKTOP_SPEAKER, /*_assocTerm*/ 0x01, \
          /*_srcid*/ 0x02, /*_clkid*/ 0x04, /*_ctrl*/ 0x0000, /*_stridx*/ 0x00),\
  /* Feature Unit Descriptor(4.7.2.8) */ /*BSD NOTE*/ \
  TUD_AUDIO_DESC_FEATURE_UNIT_SPK_V2(/*_unitid*/ 0x02, /*_srcid*/ 0x01, \
          /*_ctrlch0master*/ (AUDIO_CTRL_RW << AUDIO_FEATURE_UNIT_CTRL_MUTE_POS) | (AUDIO_CTRL_RW << AUDIO_FEATURE_UNIT_CTRL_VOLUME_POS), \
          /*_ctrlch*/ (AUDIO_CTRL_RW << AUDIO_FEATURE_UNIT_CTRL_MUTE_POS) | (AUDIO_CTRL_RW << AUDIO_FEATURE_UNIT_CTRL_VOLUME_POS), /*_stridx*/ 0x00),\
  /* Standard AS Interface Descriptor(4.9.1) */\
  /* Interface 1, Alternate 0 - default alternate setting with 0 bandwidth */\
  TUD_AUDIO_DESC_STD_AS_INT_V2(/*_itfnum*/ (uint8_t)((_itfnum) + 1), /*_altset*/ 0x00, /*_nEPs*/ 0x00, /*_stridx*/ 0x00),\
  /* Standard AS Interface Descriptor(4.9.1) */\
  /* Interface 1, Alternate 1 - alternate interface for data streaming */\
  TUD_AUDIO_DESC_STD_AS_INT_V2(/*_itfnum*/ (uint8_t)((_itfnum) + 1), /*_altset*/ 0x01, /*_nEPs*/ 0x02, /*_stridx*/ 0x00),\
  /* Class-Specific AS Interface Descriptor(4.9.2) */\
  TUD_AUDIO_DESC_CS_AS_INT_V2(/*_termid*/ 0x01, /*_ctrl*/ AUDIO_CTRL_NONE, /*_formattype*/ AUDIO_FORMAT_TYPE_I, \
          /*_formats*/ AUDIO_DATA_FORMAT_TYPE_I_PCM, /*_nchannelsphysical*/ 0x01, /*_channelcfg*/ AUDIO_CHANNEL_CONFIG_NON_PREDEFINED, /*_stridx*/ 0x00),\
  /* Type I Format Type Descriptor(2.3.1.6 - Audio Formats) */\
  TUD_AUDIO_DESC_TYPE_I_FORMAT_V2(_nBytesPerSample, _nBitsUsedPerSample),\
  /* Standard AS Isochronous Audio Data Endpoint Descriptor(4.10.1.1) */\
  TUD_AUDIO_DESC_STD_AS_ISO_EP_V2(/*_ep*/ _epout, /*_attr*/ (TUSB_XFER_ISOCHRONOUS | TUSB_ISO_EP_ATT_ASYNCHRONOUS | TUSB_ISO_EP_ATT_DATA), \
          /*_maxEPsize*/ _epsize, /*_interval*/ (CFG_TUSB_RHPORT0_MODE & OPT_MODE_HIGH_SPEED) ? 0x04 : 0x01),\
  /* Class-Specific AS Isochronous Audio Data Endpoint Descriptor(4.10.1.2) */\
  TUD_AUDIO_DESC_CS_AS_ISO_EP_V2(/*_attr*/ AUDIO_CS_AS_ISO_DATA_EP_ATT_NON_MAX_PACKETS_OK, /*_ctrl*/ AUDIO_CTRL_NONE, \
          /*_lockdelayunit*/ AUDIO_CS_AS_ISO_DATA_EP_LOCK_DELAY_UNIT_UNDEFINED, /*_lockdelay*/ 0x0000),\
  /* Standard AS Isochronous Feedback Endpoint Descriptor(4.10.2.1) */\
  TUD_AUDIO_DESC_STD_AS_ISO_FB_EP_V2(/*_ep*/ _epfb, /*_interval*/ 1)\


//------------------------------------------------------------------------------------------------
#define ID_MIC_INPUT_TERMINAL       0x01
#define ID_MIC_FEATURE_UNIT         0x02
#define ID_MIC_OUTPUT_TERMINAL      0x03

#define ID_SPK_INPUT_TERMINAL       0x04
#define ID_SPK_FEATURE_UNIT         0x05
#define ID_SPK_OUTPUT_TERMINAL      0x06

// Add IAD in prior to the first audio interface descriptor for UAC1.0?
#if CFG_TUD_AUDIO_USE_IAD
#define AUDIO_IAD_LEN_V1    TUD_AUDIO_DESC_IAD_LEN_V1
#define AUDIO_IAD_V1(...)   TUD_AUDIO_DESC_IAD_V1(__VA_ARGS__),
#else
#define AUDIO_IAD_LEN_V1    0
#define AUDIO_IAD_V1(...)
#endif

// Support feedback EP for OUT EP?
#if CFG_TUD_AUDIO_ENABLE_FEEDBACK_EP
#define nEP_SPK         2 // OUT EP + FB (IN) EP
#define FB_EP_LEN_V1    TUD_AUDIO_DESC_STD_AS_ISO_FB_EP_LEN_V1
#define FB_EP_V1(_epfb) \
        /* Standard AS Isochronous Feedback Endpoint Descriptor(4.6.2.1)*/ \
        ,TUD_AUDIO_DESC_STD_AS_ISO_FB_EP_V1(/*_ep*/ _epfb, /*_interval*/ 1, /*_refresh, 2ms*/ 1)
#else
#define nEP_SPK         1 // OUT EP only
#define FB_EP_LEN_V1    0
#define FB_EP_V1(_epfb)
#endif // CFG_TUD_AUDIO_ENABLE_FEEDBACK_EP

// _channelcfg for MIC (Set it L/R Front only for stereo, otherwise 0)
#if (CFG_TUD_AUDIO_N_CHANNELS_TX == 2) // stereo
#define MIC_CHANNEL_CONFIG_V1   (UAC1_CC_D0 | UAC1_CC_D1)
//#elif (CFG_TUD_AUDIO_N_CHANNELS_TX == 1) // mono
//#define MIC_CHANNEL_CONFIG_V1   (UAC1_CC_D0)
#else // other MIC array
#define MIC_CHANNEL_CONFIG_V1   (UAC1_CC_NONE)
#endif

// _channelcfg for SPK (Set it L/R Front only for stereo, otherwise 0)
#if (CFG_TUD_AUDIO_N_CHANNELS_RX == 2) // stereo
#define SPK_CHANNEL_CONFIG_V1   (UAC1_CC_D0 | UAC1_CC_D1)
//#elif (CFG_TUD_AUDIO_N_CHANNELS_RX == 1) // mono
//#define SPK_CHANNEL_CONFIG_V1   (UAC1_CC_D0)
#else // other SPK array
#define SPK_CHANNEL_CONFIG_V1   (UAC1_CC_NONE)
#endif


// V1: MIC, N Channel
#define TUD_AUDIO_MIC_DESC_LEN_V1 ( AUDIO_IAD_LEN_V1 + \
  + TUD_AUDIO_DESC_STD_AC_LEN \
  + TUD_AUDIO_DESC_CS_AC_1AS_LEN_V1 \
  + TUD_AUDIO_DESC_INPUT_TERM_LEN_V1 \
  + TUD_AUDIO_DESC_OUTPUT_TERM_LEN_V1 \
  + TUD_AUDIO_DESC_FEATURE_UNIT_nCH_LEN_V1(CFG_TUD_AUDIO_N_CHANNELS_TX) \
  + TUD_AUDIO_DESC_STD_AS_INT_LEN \
  + TUD_AUDIO_DESC_STD_AS_INT_LEN \
  + TUD_AUDIO_DESC_CS_AS_INT_LEN_V1 \
  + TUD_AUDIO_DESC_TYPE_I_FORMAT_LEN_V1 \
  + TUD_AUDIO_DESC_STD_AS_ISO_EP_LEN_V1 \
  + TUD_AUDIO_DESC_CS_AS_ISO_EP_LEN_V1 )

#define TUD_AUDIO_MIC_DESCRIPTOR_V1(_itfnum, _stridx, _nBytesPerSample, _nBitsUsedPerSample, _nSampFreq, _epin, _epsize) \
  /* Standard Interface Association Descriptor (IAD) */\
  AUDIO_IAD_V1(/*_firstitfs*/ _itfnum, /*_nitfs*/ 0x02, /*_stridx*/ 0x00) \
  /* Standard AC Interface Descriptor(4.3.1) */\
  TUD_AUDIO_DESC_STD_AC_V1(/*_itfnum*/ _itfnum, /*_nEPs*/ 0x00, /*_stridx*/ _stridx),\
  /* Class-Specific AC Interface Header Descriptor(4.3.2) */\
  TUD_AUDIO_DESC_CS_AC_1AS_V1(/*_bcdADC*/ 0x0100, /*_totallen*/ TUD_AUDIO_DESC_INPUT_TERM_LEN_V1 + TUD_AUDIO_DESC_OUTPUT_TERM_LEN_V1 + \
          TUD_AUDIO_DESC_FEATURE_UNIT_nCH_LEN_V1(CFG_TUD_AUDIO_N_CHANNELS_TX), /*_as_itfnum*/ (uint8_t)((_itfnum)+1)), /*BSD NOTE*/\
  /* Input Terminal Descriptor(4.3.2.1) */\
  TUD_AUDIO_DESC_INPUT_TERM_V1(/*_termid*/ ID_MIC_INPUT_TERMINAL, /*_termtype*/ AUDIO_TERM_TYPE_IN_GENERIC_MIC, /*_assocTerm, BSD NOTE:ID_MIC_OUTPUT_TERMINAL*/ 0x00, \
          /*_nchannelslogical*/ CFG_TUD_AUDIO_N_CHANNELS_TX, /*_channelcfg*/ MIC_CHANNEL_CONFIG_V1, /*_idxchannelnames*/ 0x00, /*_stridx*/ 0x00), \
  /* Output Terminal Descriptor(4.3.2.2) */\
  TUD_AUDIO_DESC_OUTPUT_TERM_V1(/*_termid*/ ID_MIC_OUTPUT_TERMINAL, /*_termtype*/ AUDIO_TERM_TYPE_USB_STREAMING, /*_assocTerm, BSD NOTE:ID_MIC_INPUT_TERMINAL*/ 0x00, \
          /*_srcid*/ ID_MIC_FEATURE_UNIT, /*_stridx*/ 0x00),\
  /* Feature Unit Descriptor(4.3.2.5) */\
  TUD_AUDIO_DESC_FEATURE_UNIT_MIC_V1(/*_unitid*/ ID_MIC_FEATURE_UNIT, /*_srcid*/ ID_MIC_INPUT_TERMINAL, \
          /*_ctrlch0master*/ UAC1_FU_CTRL_MUTE | UAC1_FU_CTRL_VOLUME, \
          /*_ctrlch, UAC1_FU_CTRL_MUTE | UAC1_FU_CTRL_VOLUME*/ 0, /*_stridx*/ 0x00),\
  /* Standard AS Interface Descriptor(4.5.1) */\
  /* Interface 1, Alternate 0 - default alternate setting with 0 bandwidth */\
  TUD_AUDIO_DESC_STD_AS_INT_V1(/*_itfnum*/ (uint8_t)((_itfnum)+1), /*_altset*/ 0x00, /*_nEPs*/ 0x00, /*_stridx*/ 0x00), \
  /* Standard AS Interface Descriptor(4.5.1) */\
  /* Interface 1, Alternate 1 - alternate interface for data streaming */\
  TUD_AUDIO_DESC_STD_AS_INT_V1(/*_itfnum*/ (uint8_t)((_itfnum)+1), /*_altset*/ 0x01, /*_nEPs*/ 0x01, /*_stridx*/ 0x00), \
  /* Class-Specific AS Interface Descriptor(4.5.2) */\
  TUD_AUDIO_DESC_CS_AS_INT_V1(/*_termid*/ ID_MIC_OUTPUT_TERMINAL, /*_delay, default 1ms*/ 1, /*_formattag*/ AUDIO_DATA_FORMAT_TYPE_I_PCM),\
  /* Type I Format Type Descriptor(2.2.5 - Audio Formats) */\
  TUD_AUDIO_DESC_TYPE_I_FORMAT_V1(CFG_TUD_AUDIO_N_CHANNELS_TX, _nBytesPerSample, _nBitsUsedPerSample, _nSampFreq), /*BSD NOTE: default 1SF*/\
  /* Standard AS Isochronous Audio Data Endpoint Descriptor(4.6.1.1) */\
  TUD_AUDIO_DESC_STD_AS_ISO_EP_V1(/*_ep*/ _epin, /*_attr*/ (TUSB_XFER_ISOCHRONOUS | TUSB_ISO_EP_ATT_ASYNCHRONOUS | TUSB_ISO_EP_ATT_DATA), \
          /*_maxEPsize*/ _epsize, /*_interval*/ 0x01),\
  /* Class-Specific AS Isochronous Audio Data Endpoint Descriptor(4.6.1.2) */\
  TUD_AUDIO_DESC_CS_AS_ISO_EP_V1(/*_attr*/ AUDIO_CS_AS_ISO_DATA_EP_ATT_NON_MAX_PACKETS_OK, \
          /*_lockdelayunit*/ AUDIO_CS_AS_ISO_DATA_EP_LOCK_DELAY_UNIT_MILLISEC, /*_lockdelay*/ 0x0005/*BSD NOTE: default 5ms?*/)

// V1: SPK, N Channel, FB
#define TUD_AUDIO_SPK_DESC_LEN_V1 ( AUDIO_IAD_LEN_V1 + \
  + TUD_AUDIO_DESC_STD_AC_LEN \
  + TUD_AUDIO_DESC_CS_AC_1AS_LEN_V1 \
  + TUD_AUDIO_DESC_INPUT_TERM_LEN_V1 \
  + TUD_AUDIO_DESC_OUTPUT_TERM_LEN_V1 \
  + TUD_AUDIO_DESC_FEATURE_UNIT_nCH_LEN_V1(CFG_TUD_AUDIO_N_CHANNELS_RX) \
  + TUD_AUDIO_DESC_STD_AS_INT_LEN \
  + TUD_AUDIO_DESC_STD_AS_INT_LEN \
  + TUD_AUDIO_DESC_CS_AS_INT_LEN_V1 \
  + TUD_AUDIO_DESC_TYPE_I_FORMAT_LEN_V1 \
  + TUD_AUDIO_DESC_STD_AS_ISO_EP_LEN_V1 \
  + TUD_AUDIO_DESC_CS_AS_ISO_EP_LEN_V1 \
  + FB_EP_LEN_V1 )

#define TUD_AUDIO_SPK_DESCRIPTOR_V1(_itfnum, _stridx, _nBytesPerSample, _nBitsUsedPerSample, _nSampFreq, _epout, _epsize, _epfb) \
  /* Standard Interface Association Descriptor (IAD) */\
  AUDIO_IAD_V1(/*_firstitfs*/ _itfnum, /*_nitfs*/ 0x02, /*_stridx*/ 0x00) \
  /* Standard AC Interface Descriptor(4.3.1) */\
  TUD_AUDIO_DESC_STD_AC_V1(/*_itfnum*/ _itfnum, /*_nEPs*/ 0x00, /*_stridx*/ _stridx),\
  /* Class-Specific AC Interface Header Descriptor(4.3.2) */\
  TUD_AUDIO_DESC_CS_AC_1AS_V1(/*_bcdADC*/ 0x0100, /*_totallen*/ TUD_AUDIO_DESC_INPUT_TERM_LEN_V1 + TUD_AUDIO_DESC_OUTPUT_TERM_LEN_V1 + \
          TUD_AUDIO_DESC_FEATURE_UNIT_nCH_LEN_V1(CFG_TUD_AUDIO_N_CHANNELS_RX), /*_as_itfnum*/ (uint8_t)((_itfnum)+1)), /*BSD NOTE*/\
  /* Input Terminal Descriptor(4.3.2.1) */\
  TUD_AUDIO_DESC_INPUT_TERM_V1(/*_termid*/ ID_SPK_INPUT_TERMINAL, /*_termtype*/ AUDIO_TERM_TYPE_USB_STREAMING, /*_assocTerm*/ 0x00, \
          /*_nchannelslogical*/ CFG_TUD_AUDIO_N_CHANNELS_RX, /*_channelcfg*/ SPK_CHANNEL_CONFIG_V1, /*_idxchannelnames*/ 0x00, /*_stridx*/ 0x00), \
  /* Output Terminal Descriptor(4.3.2.2) */\
  TUD_AUDIO_DESC_OUTPUT_TERM_V1(/*_termid*/ ID_SPK_OUTPUT_TERMINAL, /*_termtype*/ AUDIO_TERM_TYPE_OUT_DESKTOP_SPEAKER, /*_assocTerm, BSD NOTE:ID_SPK_INPUT_TERMINAL*/ 0x00, \
          /*_srcid*/ ID_SPK_FEATURE_UNIT, /*_stridx*/ 0x00),\
  /* Feature Unit Descriptor(4.3.2.5) */\
  TUD_AUDIO_DESC_FEATURE_UNIT_SPK_V1(/*_unitid*/ ID_SPK_FEATURE_UNIT, /*_srcid*/ ID_SPK_INPUT_TERMINAL, \
          /*_ctrlch0master*/ UAC1_FU_CTRL_MUTE | UAC1_FU_CTRL_VOLUME, \
          /*_ctrlch, UAC1_FU_CTRL_MUTE | UAC1_FU_CTRL_VOLUME*/ 0, /*_stridx*/ 0x00),\
  /* Standard AS Interface Descriptor(4.5.1) */\
  /* Interface 1, Alternate 0 - default alternate setting with 0 bandwidth */\
  TUD_AUDIO_DESC_STD_AS_INT_V1(/*_itfnum*/ (uint8_t)((_itfnum)+1), /*_altset*/ 0x00, /*_nEPs*/ 0x00, /*_stridx*/ 0x00), \
  /* Standard AS Interface Descriptor(4.5.1) */\
  /* Interface 1, Alternate 1 - alternate interface for data streaming */\
  TUD_AUDIO_DESC_STD_AS_INT_V1(/*_itfnum*/ (uint8_t)((_itfnum)+1), /*_altset*/ 0x01, /*_nEPs*/ nEP_SPK, /*_stridx*/ 0x00), \
  /* Class-Specific AS Interface Descriptor(4.5.2) */\
  TUD_AUDIO_DESC_CS_AS_INT_V1(/*_termid*/ ID_SPK_INPUT_TERMINAL, /*_delay, default 1ms*/ 1, /*_formattag*/ AUDIO_DATA_FORMAT_TYPE_I_PCM),\
  /* Type I Format Type Descriptor(2.2.5 - Audio Formats) */\
  TUD_AUDIO_DESC_TYPE_I_FORMAT_V1(CFG_TUD_AUDIO_N_CHANNELS_RX, _nBytesPerSample, _nBitsUsedPerSample, _nSampFreq), /*BSD NOTE: default 1SF*/\
  /* Standard AS Isochronous Audio Data Endpoint Descriptor(4.6.1.1) */\
  TUD_AUDIO_DESC_STD_AS_ISO_EP_V1(/*_ep*/ _epout, /*_attr*/ (TUSB_XFER_ISOCHRONOUS | TUSB_ISO_EP_ATT_ASYNCHRONOUS), \
          /*_maxEPsize*/ _epsize, /*_interval*/ 0x01),\
  /* Class-Specific AS Isochronous Audio Data Endpoint Descriptor(4.6.1.2) */\
  TUD_AUDIO_DESC_CS_AS_ISO_EP_V1(/*_attr*/ AUDIO_CS_AS_ISO_DATA_EP_ATT_NON_MAX_PACKETS_OK, \
          /*_lockdelayunit*/ AUDIO_CS_AS_ISO_DATA_EP_LOCK_DELAY_UNIT_MILLISEC, /*_lockdelay*/ 0x0005/*BSD NOTE: default 5ms?*/) \
  FB_EP_V1(_epfb)

// V1: MIC, N Channel + SPK, N Channel, FB
#define TUD_AUDIO_IO_DESC_LEN_V1 ( AUDIO_IAD_LEN_V1 + \
  + TUD_AUDIO_DESC_STD_AC_LEN \
  + TUD_AUDIO_DESC_CS_AC_2AS_LEN_V1 \
  + TUD_AUDIO_DESC_INPUT_TERM_LEN_V1 * 2 \
  + TUD_AUDIO_DESC_OUTPUT_TERM_LEN_V1 * 2 \
  + TUD_AUDIO_DESC_FEATURE_UNIT_nCH_LEN_V1(CFG_TUD_AUDIO_N_CHANNELS_TX) \
  + TUD_AUDIO_DESC_FEATURE_UNIT_nCH_LEN_V1(CFG_TUD_AUDIO_N_CHANNELS_RX) \
  + TUD_AUDIO_DESC_STD_AS_INT_LEN * 2 \
  + TUD_AUDIO_DESC_STD_AS_INT_LEN * 2 \
  + TUD_AUDIO_DESC_CS_AS_INT_LEN_V1 * 2 \
  + TUD_AUDIO_DESC_TYPE_I_FORMAT_LEN_V1 * 2 \
  + TUD_AUDIO_DESC_STD_AS_ISO_EP_LEN_V1 * 2 \
  + TUD_AUDIO_DESC_CS_AS_ISO_EP_LEN_V1 * 2 \
  + FB_EP_LEN_V1 )

#define TUD_AUDIO_IO_DESCRIPTOR_V1(_itfnum, _stridx, _nBytesPerSample_in, _nBitsUsedPerSample_in, _nSampFreq_in, _epin, _epsize_in, \
                                                    _nBytesPerSample_out, _nBitsUsedPerSample_out, _nSampFreq_out, _epout, _epsize_out, _epfb) \
  /* Standard Interface Association Descriptor (IAD) */\
  AUDIO_IAD_V1(/*_firstitfs*/ _itfnum, /*_nitfs*/ 0x03, /*_stridx*/ 0x00) \
  /* Standard AC Interface Descriptor(4.3.1) */\
  TUD_AUDIO_DESC_STD_AC_V1(/*_itfnum*/ _itfnum, /*_nEPs*/ 0x00, /*_stridx*/ _stridx), \
  /* Class-Specific AC Interface Header Descriptor(4.3.2) */\
  TUD_AUDIO_DESC_CS_AC_2AS_V1(/*_bcdADC*/ 0x0100, /*_totallen*/ (TUD_AUDIO_DESC_INPUT_TERM_LEN_V1 + TUD_AUDIO_DESC_OUTPUT_TERM_LEN_V1) * 2 + \
          TUD_AUDIO_DESC_FEATURE_UNIT_nCH_LEN_V1(CFG_TUD_AUDIO_N_CHANNELS_TX) + TUD_AUDIO_DESC_FEATURE_UNIT_nCH_LEN_V1(CFG_TUD_AUDIO_N_CHANNELS_RX), \
          /*_as_itfnum*/ (uint8_t)((_itfnum)+1), /*_as_itfnum*/ (uint8_t)((_itfnum)+2)), \
  /* Input Terminal Descriptor(4.3.2.1) */\
  TUD_AUDIO_DESC_INPUT_TERM_V1(/*_termid*/ ID_MIC_INPUT_TERMINAL, /*_termtype*/ AUDIO_TERM_TYPE_IN_GENERIC_MIC, /*_assocTerm, BSD NOTE:ID_MIC_OUTPUT_TERMINAL*/ 0, \
          /*_nchannelslogical*/ CFG_TUD_AUDIO_N_CHANNELS_TX, /*_channelcfg*/ MIC_CHANNEL_CONFIG_V1, /*_idxchannelnames*/ 0x00, /*_stridx*/ 0x00), \
  /* Output Terminal Descriptor(4.3.2.2) */\
  TUD_AUDIO_DESC_OUTPUT_TERM_V1(/*_termid*/ ID_MIC_OUTPUT_TERMINAL, /*_termtype*/ AUDIO_TERM_TYPE_USB_STREAMING, /*_assocTerm, BSD NOTE:ID_MIC_INPUT_TERMINAL*/ 0, \
          /*_srcid*/ ID_MIC_FEATURE_UNIT, /*_stridx*/ 0x00),\
  /* Feature Unit Descriptor(4.3.2.5) */\
  TUD_AUDIO_DESC_FEATURE_UNIT_MIC_V1(/*_unitid*/ ID_MIC_FEATURE_UNIT, /*_srcid*/ ID_MIC_INPUT_TERMINAL, \
          /*_ctrlch0master*/ UAC1_FU_CTRL_MUTE | UAC1_FU_CTRL_VOLUME, \
          /*_ctrlch, UAC1_FU_CTRL_MUTE | UAC1_FU_CTRL_VOLUME*/ 0, /*_stridx*/ 0x00),\
  /* Input Terminal Descriptor(4.3.2.1) */\
  TUD_AUDIO_DESC_INPUT_TERM_V1(/*_termid*/ ID_SPK_INPUT_TERMINAL, /*_termtype*/ AUDIO_TERM_TYPE_USB_STREAMING, /*_assocTerm*/ 0x00, \
          /*_nchannelslogical*/ CFG_TUD_AUDIO_N_CHANNELS_RX, /*_channelcfg*/ SPK_CHANNEL_CONFIG_V1, /*_idxchannelnames*/ 0x00, /*_stridx*/ 0x00), \
  /* Output Terminal Descriptor(4.3.2.2) */\
  TUD_AUDIO_DESC_OUTPUT_TERM_V1(/*_termid*/ ID_SPK_OUTPUT_TERMINAL, /*_termtype*/ AUDIO_TERM_TYPE_OUT_DESKTOP_SPEAKER, /*_assocTerm, BSD NOTE:ID_SPK_INPUT_TERMINAL*/ 0, \
          /*_srcid*/ ID_SPK_FEATURE_UNIT, /*_stridx*/ 0x00),\
  /* Feature Unit Descriptor(4.3.2.5) */\
  TUD_AUDIO_DESC_FEATURE_UNIT_SPK_V1(/*_unitid*/ ID_SPK_FEATURE_UNIT, /*_srcid*/ ID_SPK_INPUT_TERMINAL, \
          /*_ctrlch0master*/ UAC1_FU_CTRL_MUTE | UAC1_FU_CTRL_VOLUME, \
          /*_ctrlch, UAC1_FU_CTRL_MUTE | UAC1_FU_CTRL_VOLUME*/ 0, /*_stridx*/ 0x00), \
  /* Standard AS Interface Descriptor(4.5.1) */ \
  /* Interface 1, Alternate 0 - default alternate setting with 0 bandwidth */ \
  TUD_AUDIO_DESC_STD_AS_INT_V1(/*_itfnum*/ (uint8_t)((_itfnum)+1), /*_altset*/ 0x00, /*_nEPs*/ 0x00, /*_stridx*/ 0x00), \
  /* Standard AS Interface Descriptor(4.5.1) */\
  /* Interface 1, Alternate 1 - alternate interface for data streaming */\
  TUD_AUDIO_DESC_STD_AS_INT_V1(/*_itfnum*/ (uint8_t)((_itfnum)+1), /*_altset*/ 0x01, /*_nEPs*/ 0x01, /*_stridx*/ 0x00), \
  /* Class-Specific AS Interface Descriptor(4.5.2) */\
  TUD_AUDIO_DESC_CS_AS_INT_V1(/*_termid*/ ID_MIC_OUTPUT_TERMINAL, /*_delay, default 1ms*/ 1, /*_formattag*/ AUDIO_DATA_FORMAT_TYPE_I_PCM),\
  /* Type I Format Type Descriptor(2.2.5 - Audio Formats) */\
  TUD_AUDIO_DESC_TYPE_I_FORMAT_V1(CFG_TUD_AUDIO_N_CHANNELS_TX, _nBytesPerSample_in, _nBitsUsedPerSample_in, _nSampFreq_in), /*BSD NOTE: default 1SF*/\
  /* Standard AS Isochronous Audio Data Endpoint Descriptor(4.6.1.1) */\
  TUD_AUDIO_DESC_STD_AS_ISO_EP_V1(/*_ep*/ _epin, /*_attr*/ (TUSB_XFER_ISOCHRONOUS | TUSB_ISO_EP_ATT_ASYNCHRONOUS | TUSB_ISO_EP_ATT_DATA), \
          /*_maxEPsize*/ _epsize_in, /*_interval*/ 0x01),\
  /* Class-Specific AS Isochronous Audio Data Endpoint Descriptor(4.6.1.2) */\
  TUD_AUDIO_DESC_CS_AS_ISO_EP_V1(/*_attr*/ AUDIO_CS_AS_ISO_DATA_EP_ATT_NON_MAX_PACKETS_OK, \
          /*_lockdelayunit*/ AUDIO_CS_AS_ISO_DATA_EP_LOCK_DELAY_UNIT_MILLISEC, /*_lockdelay*/ 0x0005/*BSD NOTE: default 5ms?*/), \
  /* Standard AS Interface Descriptor(4.5.1) */ \
  /* Interface 1, Alternate 0 - default alternate setting with 0 bandwidth */ \
  TUD_AUDIO_DESC_STD_AS_INT_V1(/*_itfnum*/ (uint8_t)((_itfnum)+2), /*_altset*/ 0x00, /*_nEPs*/ 0x00, /*_stridx*/ 0x00), \
  /* Standard AS Interface Descriptor(4.5.1) */\
  /* Interface 1, Alternate 1 - alternate interface for data streaming */\
  TUD_AUDIO_DESC_STD_AS_INT_V1(/*_itfnum*/ (uint8_t)((_itfnum)+2), /*_altset*/ 0x01, /*_nEPs*/ nEP_SPK, /*_stridx*/ 0x00), \
  /* Class-Specific AS Interface Descriptor(4.5.2) */\
  TUD_AUDIO_DESC_CS_AS_INT_V1(/*_termid*/ ID_SPK_INPUT_TERMINAL, /*_delay, default 1ms*/ 1, /*_formattag*/ AUDIO_DATA_FORMAT_TYPE_I_PCM),\
  /* Type I Format Type Descriptor(2.2.5 - Audio Formats) */\
  TUD_AUDIO_DESC_TYPE_I_FORMAT_V1(CFG_TUD_AUDIO_N_CHANNELS_RX, _nBytesPerSample_out, _nBitsUsedPerSample_out, _nSampFreq_out), /*BSD NOTE: default 1SF*/\
  /* Standard AS Isochronous Audio Data Endpoint Descriptor(4.6.1.1) */\
  TUD_AUDIO_DESC_STD_AS_ISO_EP_V1(/*_ep*/ _epout, /*_attr*/ (TUSB_XFER_ISOCHRONOUS | TUSB_ISO_EP_ATT_ASYNCHRONOUS | TUSB_ISO_EP_ATT_DATA), \
          /*_maxEPsize*/ _epsize_out, /*_interval*/ 0x01),\
  /* Class-Specific AS Isochronous Audio Data Endpoint Descriptor(4.6.1.2) */\
  TUD_AUDIO_DESC_CS_AS_ISO_EP_V1(/*_attr*/ AUDIO_CS_AS_ISO_DATA_EP_ATT_NON_MAX_PACKETS_OK, \
          /*_lockdelayunit*/ AUDIO_CS_AS_ISO_DATA_EP_LOCK_DELAY_UNIT_MILLISEC, /*_lockdelay*/ 0x0005/*BSD NOTE: default 5ms?*/) \
  FB_EP_V1(_epfb)

//------------- TUD_USBTMC/USB488 -------------//
#define TUD_USBTMC_APP_CLASS    (TUSB_CLASS_APPLICATION_SPECIFIC)
#define TUD_USBTMC_APP_SUBCLASS 0x03u

#define TUD_USBTMC_PROTOCOL_STD    0x00u
#define TUD_USBTMC_PROTOCOL_USB488 0x01u

//   Interface number, number of endpoints, EP string index, USB_TMC_PROTOCOL*, bulk-out endpoint ID,
//   bulk-in endpoint ID
#define TUD_USBTMC_IF_DESCRIPTOR(_itfnum, _bNumEndpoints, _stridx, _itfProtocol) \
  /* Interface */ \
  0x09, TUSB_DESC_INTERFACE, _itfnum, 0x00, _bNumEndpoints, TUD_USBTMC_APP_CLASS, TUD_USBTMC_APP_SUBCLASS, _itfProtocol, _stridx

#define TUD_USBTMC_IF_DESCRIPTOR_LEN 9u

#define TUD_USBTMC_BULK_DESCRIPTORS(_epout, _epin, _bulk_epsize) \
  /* Endpoint Out */ \
  7, TUSB_DESC_ENDPOINT, _epout, TUSB_XFER_BULK, U16_TO_U8S_LE(_bulk_epsize), 0u, \
  /* Endpoint In */ \
  7, TUSB_DESC_ENDPOINT, _epin, TUSB_XFER_BULK, U16_TO_U8S_LE(_bulk_epsize), 0u

#define TUD_USBTMC_BULK_DESCRIPTORS_LEN (7u+7u)

/* optional interrupt endpoint */ \
// _int_pollingInterval : for LS/FS, expressed in frames (1ms each). 16 may be a good number?
#define TUD_USBTMC_INT_DESCRIPTOR(_ep_interrupt, _ep_interrupt_size, _int_pollingInterval ) \
  7, TUSB_DESC_ENDPOINT, _ep_interrupt, TUSB_XFER_INTERRUPT, U16_TO_U8S_LE(_ep_interrupt_size), 0x16

#define TUD_USBTMC_INT_DESCRIPTOR_LEN (7u)


//------------- Vendor -------------//
#define TUD_VENDOR_DESC_LEN  (9+7+7)

// Interface number, string index, EP Out & IN address, EP size
#define TUD_VENDOR_DESCRIPTOR(_itfnum, _stridx, _epout, _epin, _epsize) \
  /* Interface */\
  9, TUSB_DESC_INTERFACE, _itfnum, 0, 2, TUSB_CLASS_VENDOR_SPECIFIC, 0x00, 0x00, _stridx,\
  /* Endpoint Out */\
  7, TUSB_DESC_ENDPOINT, _epout, TUSB_XFER_BULK, U16_TO_U8S_LE(_epsize), 0,\
  /* Endpoint In */\
  7, TUSB_DESC_ENDPOINT, _epin, TUSB_XFER_BULK, U16_TO_U8S_LE(_epsize), 0

//BSD: Length of Interface & Endpoint Descriptor
#define TUD_STD_ITF_DESC_LEN    9
#define TUD_STD_EP_DESC_LEN     7

//BSD: Vendor Interface
// Interface number, string index, EP count
#define TUD_VENDOR_ITF_DESCRIPTOR(_itfnum, _stridx, _ep_count) \
  9, TUSB_DESC_INTERFACE, _itfnum, 0, _ep_count, TUSB_CLASS_VENDOR_SPECIFIC, 0x00, 0x00, _stridx

//BSD: Vendor Endpoint
// EP address, EP type, EP size (MPS), interval (for ISO & INTERRUPT)
#define TUD_VENDOR_EP_EDESCRIPTOR(_ep_addr, _ep_type, _epsize, _interval) \
  7, TUSB_DESC_ENDPOINT, _ep_addr, _ep_type, U16_TO_U8S_LE(_epsize), _interval

//------------- DFU Runtime -------------//
#define TUD_DFU_APP_CLASS    (TUSB_CLASS_APPLICATION_SPECIFIC)
#define TUD_DFU_APP_SUBCLASS 0x01u

// Length of template descriptr: 18 bytes
#define TUD_DFU_RT_DESC_LEN (9 + 9)

// DFU runtime descriptor
// Interface number, string index, attributes, detach timeout, transfer size
#define TUD_DFU_RT_DESCRIPTOR(_itfnum, _stridx, _attr, _timeout, _xfer_size) \
  /* Interface */ \
  9, TUSB_DESC_INTERFACE, _itfnum, 0, 0, TUD_DFU_APP_CLASS, TUD_DFU_APP_SUBCLASS, DFU_PROTOCOL_RT, _stridx, \
  /* Function */ \
  9, DFU_DESC_FUNCTIONAL, _attr, U16_TO_U8S_LE(_timeout), U16_TO_U8S_LE(_xfer_size), U16_TO_U8S_LE(0x0101)


//------------- CDC-ECM -------------//

// Length of template descriptor: 71 bytes
#define TUD_CDC_ECM_DESC_LEN  (8+9+5+5+13+7+9+9+7+7)

// CDC-ECM Descriptor Template
// Interface number, description string index, MAC address string index, EP notification address and size, EP data address (out, in), and size, max segment size.
#define TUD_CDC_ECM_DESCRIPTOR(_itfnum, _desc_stridx, _mac_stridx, _ep_notif, _ep_notif_size, _epout, _epin, _epsize, _maxsegmentsize) \
  /* Interface Association */\
  8, TUSB_DESC_INTERFACE_ASSOCIATION, _itfnum, 2, TUSB_CLASS_CDC, CDC_COMM_SUBCLASS_ETHERNET_CONTROL_MODEL, 0, 0,\
  /* CDC Control Interface */\
  9, TUSB_DESC_INTERFACE, _itfnum, 0, 1, TUSB_CLASS_CDC, CDC_COMM_SUBCLASS_ETHERNET_CONTROL_MODEL, 0, _desc_stridx,\
  /* CDC-ECM Header */\
  5, TUSB_DESC_CS_INTERFACE, CDC_FUNC_DESC_HEADER, U16_TO_U8S_LE(0x0120),\
  /* CDC-ECM Union */\
  5, TUSB_DESC_CS_INTERFACE, CDC_FUNC_DESC_UNION, _itfnum, (uint8_t)((_itfnum) + 1),\
  /* CDC-ECM Functional Descriptor */\
  13, TUSB_DESC_CS_INTERFACE, CDC_FUNC_DESC_ETHERNET_NETWORKING, _mac_stridx, 0, 0, 0, 0, U16_TO_U8S_LE(_maxsegmentsize), U16_TO_U8S_LE(0), 0,\
  /* Endpoint Notification */\
  7, TUSB_DESC_ENDPOINT, _ep_notif, TUSB_XFER_INTERRUPT, U16_TO_U8S_LE(_ep_notif_size), 1,\
  /* CDC Data Interface (default inactive) */\
  9, TUSB_DESC_INTERFACE, (uint8_t)((_itfnum)+1), 0, 0, TUSB_CLASS_CDC_DATA, 0, 0, 0,\
  /* CDC Data Interface (alternative active) */\
  9, TUSB_DESC_INTERFACE, (uint8_t)((_itfnum)+1), 1, 2, TUSB_CLASS_CDC_DATA, 0, 0, 0,\
  /* Endpoint In */\
  7, TUSB_DESC_ENDPOINT, _epin, TUSB_XFER_BULK, U16_TO_U8S_LE(_epsize), 0,\
  /* Endpoint Out */\
  7, TUSB_DESC_ENDPOINT, _epout, TUSB_XFER_BULK, U16_TO_U8S_LE(_epsize), 0


//------------- RNDIS -------------//

#if 0
/* Windows XP */
#define TUD_RNDIS_ITF_CLASS    TUSB_CLASS_CDC
#define TUD_RNDIS_ITF_SUBCLASS CDC_COMM_SUBCLASS_ABSTRACT_CONTROL_MODEL
#define TUD_RNDIS_ITF_PROTOCOL 0xFF /* CDC_COMM_PROTOCOL_MICROSOFT_RNDIS */
#else
/* Windows 7+ */
#define TUD_RNDIS_ITF_CLASS    TUSB_CLASS_WIRELESS_CONTROLLER
#define TUD_RNDIS_ITF_SUBCLASS 0x01
#define TUD_RNDIS_ITF_PROTOCOL 0x03
#endif

// Length of template descriptor: 66 bytes
#define TUD_RNDIS_DESC_LEN  (8+9+5+5+4+5+7+9+7+7)

// RNDIS Descriptor Template
// Interface number, string index, EP notification address and size, EP data address (out, in) and size.
#define TUD_RNDIS_DESCRIPTOR(_itfnum, _stridx, _ep_notif, _ep_notif_size, _epout, _epin, _epsize) \
  /* Interface Association */\
  8, TUSB_DESC_INTERFACE_ASSOCIATION, _itfnum, 2, TUD_RNDIS_ITF_CLASS, TUD_RNDIS_ITF_SUBCLASS, TUD_RNDIS_ITF_PROTOCOL, 0,\
  /* CDC Control Interface */\
  9, TUSB_DESC_INTERFACE, _itfnum, 0, 1, TUD_RNDIS_ITF_CLASS, TUD_RNDIS_ITF_SUBCLASS, TUD_RNDIS_ITF_PROTOCOL, _stridx,\
  /* CDC-ACM Header */\
  5, TUSB_DESC_CS_INTERFACE, CDC_FUNC_DESC_HEADER, U16_TO_U8S_LE(0x0110),\
  /* CDC Call Management */\
  5, TUSB_DESC_CS_INTERFACE, CDC_FUNC_DESC_CALL_MANAGEMENT, 0, (uint8_t)((_itfnum) + 1),\
  /* ACM */\
  4, TUSB_DESC_CS_INTERFACE, CDC_FUNC_DESC_ABSTRACT_CONTROL_MANAGEMENT, 0,\
  /* CDC Union */\
  5, TUSB_DESC_CS_INTERFACE, CDC_FUNC_DESC_UNION, _itfnum, (uint8_t)((_itfnum) + 1),\
  /* Endpoint Notification */\
  7, TUSB_DESC_ENDPOINT, _ep_notif, TUSB_XFER_INTERRUPT, U16_TO_U8S_LE(_ep_notif_size), 1,\
  /* CDC Data Interface */\
  9, TUSB_DESC_INTERFACE, (uint8_t)((_itfnum)+1), 0, 2, TUSB_CLASS_CDC_DATA, 0, 0, 0,\
  /* Endpoint In */\
  7, TUSB_DESC_ENDPOINT, _epin, TUSB_XFER_BULK, U16_TO_U8S_LE(_epsize), 0,\
  /* Endpoint Out */\
  7, TUSB_DESC_ENDPOINT, _epout, TUSB_XFER_BULK, U16_TO_U8S_LE(_epsize), 0

//------------- BT Radio -------------//
#define TUD_BT_APP_CLASS                    (TUSB_CLASS_WIRELESS_CONTROLLER)
#define TUD_BT_APP_SUBCLASS                 0x01
#define TUD_BT_PROTOCOL_PRIMARY_CONTROLLER  0x01
#define TUD_BT_PROTOCOL_AMP_CONTROLLER      0x02

#ifndef CFG_TUD_BTH_ISO_ALT_COUNT
#define CFG_TUD_BTH_ISO_ALT_COUNT 0
#endif

// Length of template descriptor: 30 bytes + number of ISO alternatives * 23
#define TUD_BTH_DESC_LEN (9 + 7 + 7 + 7 + (CFG_TUD_BTH_ISO_ALT_COUNT) * (9 + 7 + 7))

/* Primary Interface */
#define TUD_BTH_PRI_ITF(_itfnum, _stridx, _ep_evt, _ep_evt_size, _ep_evt_interval, _ep_in, _ep_out, _ep_size) \
  /* 9, TUSB_DESC_INTERFACE, _itfnum, _stridx, 3, TUD_BT_APP_CLASS, TUD_BT_APP_SUBCLASS, TUD_BT_PROTOCOL_PRIMARY_CONTROLLER, 0, */ \
  9, TUSB_DESC_INTERFACE, _itfnum, 0, 3, TUD_BT_APP_CLASS, TUD_BT_APP_SUBCLASS, TUD_BT_PROTOCOL_PRIMARY_CONTROLLER, _stridx, \
  /* Endpoint In for events */ \
  7, TUSB_DESC_ENDPOINT, _ep_evt, TUSB_XFER_INTERRUPT, U16_TO_U8S_LE(_ep_evt_size), _ep_evt_interval, \
  /* Endpoint In for ACL data */ \
  7, TUSB_DESC_ENDPOINT, _ep_in, TUSB_XFER_BULK, U16_TO_U8S_LE(_ep_size), 1, \
  /* Endpoint Out for ACL data */ \
  7, TUSB_DESC_ENDPOINT, _ep_out, TUSB_XFER_BULK, U16_TO_U8S_LE(_ep_size), 1

#define TUD_BTH_ISO_ITF(_itfnum, _alt, _ep_in, _ep_out, _n) ,\
  /* Interface with 2 endpoints */ \
  9, TUSB_DESC_INTERFACE, _itfnum, _alt, 2, TUD_BT_APP_CLASS, TUD_BT_APP_SUBCLASS, TUD_BT_PROTOCOL_PRIMARY_CONTROLLER, 0, \
  /* Isochronous endpoints */ \
  7, TUSB_DESC_ENDPOINT, _ep_in, TUSB_XFER_ISOCHRONOUS, U16_TO_U8S_LE(_n), 1, \
  7, TUSB_DESC_ENDPOINT, _ep_out, TUSB_XFER_ISOCHRONOUS, U16_TO_U8S_LE(_n), 1

#define _FIRST(a, ...) a
#define _REST(a, ...) __VA_ARGS__

#define TUD_BTH_ISO_ITF_0(_itfnum, ...)
#define TUD_BTH_ISO_ITF_1(_itfnum, _ep_in, _ep_out, ...) TUD_BTH_ISO_ITF(_itfnum, (CFG_TUD_BTH_ISO_ALT_COUNT) - 1, _ep_in, _ep_out, _FIRST(__VA_ARGS__))
#define TUD_BTH_ISO_ITF_2(_itfnum, _ep_in, _ep_out, ...) TUD_BTH_ISO_ITF(_itfnum, (CFG_TUD_BTH_ISO_ALT_COUNT) - 2, _ep_in, _ep_out, _FIRST(__VA_ARGS__)) \
  TUD_BTH_ISO_ITF_1(_itfnum, _ep_in, _ep_out, _REST(__VA_ARGS__))
#define TUD_BTH_ISO_ITF_3(_itfnum, _ep_in, _ep_out, ...) TUD_BTH_ISO_ITF(_itfnum, (CFG_TUD_BTH_ISO_ALT_COUNT) - 3, _ep_in, _ep_out, _FIRST(__VA_ARGS__)) \
  TUD_BTH_ISO_ITF_2(_itfnum, _ep_in, _ep_out, _REST(__VA_ARGS__))
#define TUD_BTH_ISO_ITF_4(_itfnum, _ep_in, _ep_out, ...) TUD_BTH_ISO_ITF(_itfnum, (CFG_TUD_BTH_ISO_ALT_COUNT) - 4, _ep_in, _ep_out, _FIRST(__VA_ARGS__)) \
  TUD_BTH_ISO_ITF_3(_itfnum, _ep_in, _ep_out, _REST(__VA_ARGS__))
#define TUD_BTH_ISO_ITF_5(_itfnum, _ep_in, _ep_out, ...) TUD_BTH_ISO_ITF(_itfnum, (CFG_TUD_BTH_ISO_ALT_COUNT) - 5, _ep_in, _ep_out, _FIRST(__VA_ARGS__)) \
  TUD_BTH_ISO_ITF_4(_itfnum, _ep_in, _ep_out, _REST(__VA_ARGS__))
#define TUD_BTH_ISO_ITF_6(_itfnum, _ep_in, _ep_out, ...) TUD_BTH_ISO_ITF(_itfnum, (CFG_TUD_BTH_ISO_ALT_COUNT) - 6, _ep_in, _ep_out, _FIRST(__VA_ARGS__)) \
  TUD_BTH_ISO_ITF_5(_itfnum, _ep_in, _ep_out, _REST(__VA_ARGS__))
// Added by BSD2022.1.22.
#define TUD_BTH_ISO_ITF_7(_itfnum, _ep_in, _ep_out, ...) TUD_BTH_ISO_ITF(_itfnum, (CFG_TUD_BTH_ISO_ALT_COUNT) - 7, _ep_in, _ep_out, _FIRST(__VA_ARGS__)) \
  TUD_BTH_ISO_ITF_6(_itfnum, _ep_in, _ep_out, _REST(__VA_ARGS__))

#define TUD_BTH_ISO_ITFS(_itfnum, _ep_in, _ep_out, ...) \
  TU_XSTRCAT(TUD_BTH_ISO_ITF_, CFG_TUD_BTH_ISO_ALT_COUNT)(_itfnum, _ep_in, _ep_out, __VA_ARGS__)

// BT Primary controller descriptor
// Interface number, string index, event endpoint, event endpoint size, interval, data in, data out, data endpoint size, iso endpoint sizes
#define TUD_BTH_DESCRIPTOR(_itfnum, _stridx, _ep_evt, _ep_evt_size, _ep_evt_interval, _ep_in, _ep_out, _ep_size,...) \
  TUD_BTH_PRI_ITF(_itfnum, _stridx, _ep_evt, _ep_evt_size, _ep_evt_interval, _ep_in, _ep_out, _ep_size) \
  TUD_BTH_ISO_ITFS(_itfnum + 1, _ep_in + 1, _ep_out + 1, __VA_ARGS__)

#ifdef __cplusplus
}
#endif

#endif /* _TUSB_USBD_H_ */

/** @} */
