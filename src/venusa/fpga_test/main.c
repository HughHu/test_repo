/*
    FreeRTOS V9.0.0 - Copyright (C) 2016 Real Time Engineers Ltd.
    All rights reserved

    VISIT http://www.FreeRTOS.org TO ENSURE YOU ARE USING THE LATEST VERSION.

    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation >>>> AND MODIFIED BY <<<< the FreeRTOS exception.

    ***************************************************************************
    >>!   NOTE: The modification to the GPL is included to allow you to     !<<
    >>!   distribute a combined work that includes FreeRTOS without being   !<<
    >>!   obliged to provide the source code for proprietary components     !<<
    >>!   outside of the FreeRTOS kernel.                                   !<<
    ***************************************************************************

    FreeRTOS is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
    FOR A PARTICULAR PURPOSE.  Full license text is available on the following
    link: http://www.freertos.org/a00114.html

    ***************************************************************************
     *                                                                       *
     *    FreeRTOS provides completely free yet professionally developed,    *
     *    robust, strictly quality controlled, supported, and cross          *
     *    platform software that is more than just the market leader, it     *
     *    is the industry's de facto standard.                               *
     *                                                                       *
     *    Help yourself get started quickly while simultaneously helping     *
     *    to support the FreeRTOS project by purchasing a FreeRTOS           *
     *    tutorial book, reference manual, or both:                          *
     *    http://www.FreeRTOS.org/Documentation                              *
     *                                                                       *
    ***************************************************************************

    http://www.FreeRTOS.org/FAQHelp.html - Having a problem?  Start by reading
    the FAQ page "My application does not run, what could be wrong?".  Have you
    defined configASSERT()?

    http://www.FreeRTOS.org/support - In return for receiving this top quality
    embedded software for free we request you assist our global community by
    participating in the support forum.

    http://www.FreeRTOS.org/training - Investing in training allows your team to
    be as productive as possible as early as possible.  Now you can receive
    FreeRTOS training directly from Richard Barry, CEO of Real Time Engineers
    Ltd, and the world's leading authority on the world's leading RTOS.

    http://www.FreeRTOS.org/plus - A selection of FreeRTOS ecosystem products,
    including FreeRTOS+Trace - an indispensable productivity tool, a DOS
    compatible FAT file system, and our tiny thread aware UDP/IP stack.

    http://www.FreeRTOS.org/labs - Where new FreeRTOS products go to incubate.
    Come and try FreeRTOS+TCP, our new open source TCP/IP stack for FreeRTOS.

    http://www.OpenRTOS.com - Real Time Engineers ltd. license FreeRTOS to High
    Integrity Systems ltd. to sell under the OpenRTOS brand.  Low cost OpenRTOS
    licenses offer ticketed support, indemnification and commercial middleware.

    http://www.SafeRTOS.com - High Integrity Systems also provide a safety
    engineered and independently SIL3 certified version for use in safety and
    mission critical applications that require provable dependability.

    1 tab == 4 spaces!
*/

/* Kernel includes. */
#include <stdio.h>
#include <stdlib.h>

#include "FreeRTOS.h" /* Must come first. */
#include "queue.h"    /* RTOS queue related API prototypes. */
#include "semphr.h"   /* Semaphore related API prototypes. */
#include "task.h"     /* RTOS task related API prototypes. */
#include "timers.h"   /* Software timer related API prototypes. */
#include "log_print.h"
#include "chip.h"

#include "Driver_JPEG.h"
#include "Driver_DMA2D.h"
#include "Driver_DVP.h"
#include "Driver_GPIO.h"

#include "log_print.h"
#include "chip.h"
#include "systick.h"
#include "ClockManager.h"
//#include "PSRAMManager.h"
#include "jpeg_check.h"
#include "ls_jpeg.h"

/* The period of the example software timer, specified in milliseconds, and
converted to ticks using the pdMS_TO_TICKS() macro. */
#define mainSOFTWARE_TIMER_PERIOD_MS    pdMS_TO_TICKS(1000)
#define TASKDLYMS                       pdMS_TO_TICKS(100)
#define mainQUEUE_LENGTH                (1)

#define TEST_JPEG_DVP_OUT_DMA2D_CH                 dma_2d_ch5


static void prvSetupHardware(void);
extern void idle_task(void);
static void vExampleTimerCallback(TimerHandle_t xTimer);

/* The queue used by the queue send and queue receive tasks. */
static QueueHandle_t xQueue = NULL;

static TaskHandle_t StartTask1_Handler;
static TaskHandle_t StartTask2_Handler;
static TaskHandle_t StartTask3_Handler;

static Jpeg_CodecCfg enc_cfg = 
{
    .width = 320,
    .height = 240,
    .pixel_format = JPEG_PIXEL_FORMAT_YUV422,
};

uint8_t codec_output_buffer[49152];

/* dvp encode */
static uint8_t enc_yuv422_320x240[153600];
//static uint8_t *enc_yuv422_320x240 = 0x38000000;
uint32_t enc_output_jpeg_size = 0;

void prvSetupHardware(void)
{

}

void usb_task(void* pvParameters);
void dvp_task(void* pvParameters);
void jpeg_task(void* pvParameters);

static volatile uint32_t dvp_dma2d_output_finish_cnt = 0;

extern int insmod_uvc_module();
extern int32_t test_jpeg_dvp_start(Jpeg_CodecCfg *enc_cfg);
extern void test_jpeg_dvp_stop();



void dma2d_reg_dump(csk_dma2d_ch_t chn)
{
    //GPDMA2D_RegDef;
    CLOGD("--------------------dma2d-%d------------------------", chn);
    CLOGD("0x000 CONTRL                  *0x%08x=0x%08x", (uint32_t*)&IP_GPDMA2D->REG_DMA_CTRL_00.all+chn, *((uint32_t*)&IP_GPDMA2D->REG_DMA_CTRL_00.all+chn));
    CLOGD("0x0A0 SRC_ADDR                *0x%08x=0x%08x", (uint32_t*)&IP_GPDMA2D->REG_DMA_SRC_ADDR_00.all+chn, *((uint32_t*)&IP_GPDMA2D->REG_DMA_SRC_ADDR_00.all+chn));
    CLOGD("0x0C0 DST_ADDR                *0x%08x=0x%08x", (uint32_t*)&IP_GPDMA2D->REG_DMA_DST_ADDR_00.all+chn, *((uint32_t*)&IP_GPDMA2D->REG_DMA_DST_ADDR_00.all+chn));
    CLOGD("0x0E0 BLK_LEN                 *0x%08x=0x%08x", (uint32_t*)&IP_GPDMA2D->REG_DMA_BLK_LEN_00.all+chn, *((uint32_t*)&IP_GPDMA2D->REG_DMA_BLK_LEN_00.all+chn));
    CLOGD("0x100 IMG_FMT                 *0x%08x=0x%08x", (uint32_t*)&IP_GPDMA2D->REG_DMA_IMAGE_FORMA_00.all+chn, *((uint32_t*)&IP_GPDMA2D->REG_DMA_IMAGE_FORMA_00.all+chn));
    CLOGD("0x120 IMG_IN_SIZE             *0x%08x=0x%08x", (uint32_t*)&IP_GPDMA2D->REG_DMA_IN_IMAGE_SIZE_00.all+chn, *((uint32_t*)&IP_GPDMA2D->REG_DMA_IN_IMAGE_SIZE_00.all+chn));
    CLOGD("0x140 IMG_OUT_SIZE            *0x%08x=0x%08x", (uint32_t*)&IP_GPDMA2D->REG_DMA_OUT_IMAGE_SIZE_00.all+chn, *((uint32_t*)&IP_GPDMA2D->REG_DMA_OUT_IMAGE_SIZE_00.all+chn));
    CLOGD("0x160 IMG_LINE_STRIDE         *0x%08x=0x%08x", (uint32_t*)&IP_GPDMA2D->REG_DMA_LINE_STRIDE_SIZE_00.all+chn, *((uint32_t*)&IP_GPDMA2D->REG_DMA_LINE_STRIDE_SIZE_00.all+chn));
    CLOGD("0x180 IMG_JPEG_1              *0x%08x=0x%08x", (uint32_t*)&IP_GPDMA2D->REG_DMA_JPEG_CTRL_01.all, *((uint32_t*)&IP_GPDMA2D->REG_DMA_JPEG_CTRL_01.all));
    CLOGD("0x184 IMG_JPEG_2              *0x%08x=0x%08x", (uint32_t*)&IP_GPDMA2D->REG_DMA_JPEG_CTRL_02.all, *((uint32_t*)&IP_GPDMA2D->REG_DMA_JPEG_CTRL_02.all));
    CLOGD("0x380 WK_REG0_RPT             *0x%08x=0x%08x", (uint32_t*)&IP_GPDMA2D->REG_DMA_CH_WK_REG0_RPT_00.all+chn, *((uint32_t*)&IP_GPDMA2D->REG_DMA_CH_WK_REG0_RPT_00.all+chn));
    CLOGD("0x3A0 WK_REG1_RPT             *0x%08x=0x%08x", (uint32_t*)&IP_GPDMA2D->REG_DMA_CH_WK_REG1_RPT_00.all+chn, *((uint32_t*)&IP_GPDMA2D->REG_DMA_CH_WK_REG1_RPT_00.all+chn));
    CLOGD("0x3C0 WK_REG2_RPT             *0x%08x=0x%08x", (uint32_t*)&IP_GPDMA2D->REG_DMA_CH_WK_REG2_RPT_00.all+chn, *((uint32_t*)&IP_GPDMA2D->REG_DMA_CH_WK_REG2_RPT_00.all+chn));
    CLOGD("0x3E0 WK_REG3_RPT             *0x%08x=0x%08x", (uint32_t*)&IP_GPDMA2D->REG_DMA_CH_WK_REG3_RPT_00.all+chn, *((uint32_t*)&IP_GPDMA2D->REG_DMA_CH_WK_REG3_RPT_00.all+chn));
    CLOGD("0x400 WK_REG4_RPT             *0x%08x=0x%08x", (uint32_t*)&IP_GPDMA2D->REG_DMA_CH_WK_REG4_RPT_00.all+chn, *((uint32_t*)&IP_GPDMA2D->REG_DMA_CH_WK_REG4_RPT_00.all+chn));
    CLOGD("0x420 WK_REG5_RPT             *0x%08x=0x%08x", (uint32_t*)&IP_GPDMA2D->REG_DMA_CH_WK_REG5_RPT_00.all+chn, *((uint32_t*)&IP_GPDMA2D->REG_DMA_CH_WK_REG5_RPT_00.all+chn));
    CLOGD("0x440 WK_REG6_RPT             *0x%08x=0x%08x", (uint32_t*)&IP_GPDMA2D->REG_DMA_CH_WK_REG6_RPT_00.all+chn, *((uint32_t*)&IP_GPDMA2D->REG_DMA_CH_WK_REG6_RPT_00.all+chn));
    CLOGD("0x460 WK_REG7_RPT             *0x%08x=0x%08x", (uint32_t*)&IP_GPDMA2D->REG_DMA_CH_WK_REG7_RPT_00.all+chn, *((uint32_t*)&IP_GPDMA2D->REG_DMA_CH_WK_REG7_RPT_00.all+chn));
    CLOGD("0x480 WK_REG8_RPT             *0x%08x=0x%08x", (uint32_t*)&IP_GPDMA2D->REG_DMA_CH_WK_REG8_RPT_00.all+chn, *((uint32_t*)&IP_GPDMA2D->REG_DMA_CH_WK_REG8_RPT_00.all+chn));
    CLOGD("0x4A0 WK_REG9_RPT             *0x%08x=0x%08x", (uint32_t*)&IP_GPDMA2D->REG_DMA_CH_WK_REG9_RPT_00.all+chn, *((uint32_t*)&IP_GPDMA2D->REG_DMA_CH_WK_REG9_RPT_00.all+chn));

}



int main(void)
{
#if ICACHE_ENABLE
    EnableICache();
#else
    DisableICache();
#endif

#if DCACHE_ENABLE
    EnableDCache();
#else
    DisableDCache();
#endif
    __RWMB();
    __FENCE_I();

#if UART_ENABLE
    logInit(0, 115200); // uart0, baudrate=115200
#endif

    //csk_timer_start();
    VIDEO_LOG("[%s:%d] %s %s", __func__, __LINE__, __DATE__, __TIME__);

    TimerHandle_t xExampleSoftwareTimer = NULL;

    /* Configure the system ready to run the demo.  The clock configuration
    can be done here if it was not done before main() was called. */
    prvSetupHardware();

    xQueue = xQueueCreate(/* The number of items the queue can hold. */
                 mainQUEUE_LENGTH,
                 /* The size of each item the queue holds. */
                 sizeof(uint32_t));

    if (xQueue == NULL) {
        CLOGD("Unable to create xQueue due to low memory.\n");
        while (1);
    }
    xTaskCreate((TaskFunction_t)usb_task, (const char*)"usb_task",
                (uint16_t)256, (void*)NULL, (UBaseType_t)2,
                (TaskHandle_t*)&StartTask1_Handler);

    xTaskCreate((TaskFunction_t)dvp_task, (const char*)"dvp_task",
                (uint16_t)256, (void*)NULL, (UBaseType_t)1,
                (TaskHandle_t*)&StartTask2_Handler);

    xTaskCreate((TaskFunction_t)jpeg_task, (const char*)"jpeg_task",
                (uint16_t)256, (void*)NULL, (UBaseType_t)1,
                (TaskHandle_t*)&StartTask2_Handler);


    xExampleSoftwareTimer =
        xTimerCreate((const char*)"ExTimer", mainSOFTWARE_TIMER_PERIOD_MS,
                     pdTRUE, (void*)0, vExampleTimerCallback);

    xTimerStart(xExampleSoftwareTimer, 0);
    CLOGD("Before StartScheduler\r\n");

    vTaskStartScheduler();

    CLOGD("OS should never run to here\r\n");

    while (1);
}

void usb_task(void* pvParameters)
{
    int cnt = 0;
    
    CLOGD("Enter to usb_task\r\n");

    /* load uvc module */
    insmod_uvc_module();

    while (1) {
        //CLOGD("usb_task is running %d.....\r\n", cnt++);
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

void dvp_task(void* pvParameters)
{
    int cnt = 0;

    
    vTaskDelay(pdMS_TO_TICKS(2000));
    CLOGD("Enter to dvp_task\r\n");

    /* start dvp recv frame */
    test_jpeg_dvp_start(&enc_cfg);

    while (1) {
        //CLOGD("dvp_task is running %d.....\r\n", cnt++);
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
    
    test_jpeg_dvp_stop();
}

uint32_t test_jpeg_image_size(uint16_t width, uint16_t height, Jpeg_emPixelFormat format)
{
    uint32_t size = 0;

    switch(format)
    {
        case JPEG_PIXEL_FORMAT_YUV444:
        case JPEG_PIXEL_FORMAT_RGB888:
            size = width * height * 3;
            break;

        case JPEG_PIXEL_FORMAT_YUV422:
        case JPEG_PIXEL_FORMAT_RGB565:
            size = width * height * 2;
            break;

        case JPEG_PIXEL_FORMAT_YUV420:
        case JPEG_PIXEL_FORMAT_YUV411:
            size = width * height * 3 / 2;
            break;

        case JPEG_PIXEL_FORMAT_GRAY:
            size = width * height;
            break;

        default:
            size = width * height * 3 / 2;
            break;
    }

    return size;
}

static void test_jpeg_dma2d_callback(uint32_t event, void* workspace)
{
    int32_t ret = FAILURE;
    Jpeg_CodecCfg *enc_cfg = (Jpeg_CodecCfg*)workspace;

    //CLOGD("[%s:%d] event=%d", __func__, __LINE__, event);
    if (CSK_DMA2D_EVENT_TRANSFER_DONE == event && NULL != enc_cfg)
    {
        ret = DMA2D_Start_Normal(TEST_JPEG_DVP_OUT_DMA2D_CH, (void*)DVP0_Buf(), (void*)enc_yuv422_320x240, test_jpeg_image_size(enc_cfg->width, enc_cfg->height, enc_cfg->pixel_format));
    }    
    dvp_dma2d_output_finish_cnt = event;
}


void jpeg_task(void* pvParameters)
{
    int32_t ret = SUCCESS;
    uint32_t timeout = 0;
    csk_dma2d_init_t jpeg_enc_input = {
            .dma_ch = TEST_JPEG_DVP_OUT_DMA2D_CH,
            .tfr_mode = tfr_mode_p2m,
            .src_basic_unit = dma2d_sample_unit_word,
            .dst_basic_unit = dma2d_sample_unit_word,
            .src_inc_mode = inc_mode_fix,
            .dst_inc_mode = inc_mode_increase,
            .src_burst_len = dma2d_burst_len_8spl,
            .dst_burst_len = dma2d_burst_len_8spl,
            .flow_ctrl = dma2d_flow_ctrl_dma,
            .prio_lvl = prio_mode_vhigh,
            .handshake = dvp_hs_num3,
            .src_gather.enable = csk_func_disable,
            .dst_scatter.enable = csk_func_disable,
    };

    vTaskDelay(pdMS_TO_TICKS(4000));

    CLOGD("Enter to jpeg_task\r\n");

    /* dma2d init */
    ret = DMA2D_Initialize();
    //CHECK_RET_EQ(ret, CSK_DRIVER_OK);
    
    ret = DMA2D_Config(&jpeg_enc_input, test_jpeg_dma2d_callback, &enc_cfg);
    //CHECK_RET_EQ(ret, CSK_DRIVER_OK);

    /* dma2d start */
    ret = DMA2D_Start_Normal(jpeg_enc_input.dma_ch, (void*)DVP0_Buf(), (void*)enc_yuv422_320x240, test_jpeg_image_size(enc_cfg.width, enc_cfg.height, enc_cfg.pixel_format));
    //CHECK_RET_EQ(ret, CSK_DRIVER_OK);

    while (1)
    {
        /* wait done */
        timeout = 1000000; // us
        CHECK_EQ_TIMEOUT_EXIT(dvp_dma2d_output_finish_cnt, 0, timeout, error0);
        dvp_dma2d_output_finish_cnt = 0;
        
        /* put frame to jpeg */
        if (SUCCESS == (ret = jpeg_encoder_ext(enc_yuv422_320x240, codec_output_buffer, &enc_output_jpeg_size, &enc_cfg)))
            VIDEO_LOG("[%s:%d] test encode_ext yuv422 packed SUCCESS", __FUNCTION__, __LINE__);
        else
            VIDEO_LOG("[%s:%d] test encode_ext yuv422 packed FAILED", __FUNCTION__, __LINE__);
error0:
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}


static void vExampleTimerCallback(TimerHandle_t xTimer)
{
    /* The timer has expired.  Count the number of times this happens.  The
    timer that calls this function is an auto re-load timer, so it will
    execute periodically. */
    static int cnt = 0;
    CLOGD("timers Callback %d\r\n", cnt++);
}

#if 0
void vApplicationTickHook(void)
{
    // BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    /* The RTOS tick hook function is enabled by setting configUSE_TICK_HOOK to
    1 in FreeRTOSConfig.h.

    "Give" the semaphore on every 500th tick interrupt. */

    /* If xHigherPriorityTaskWoken is pdTRUE then a context switch should
    normally be performed before leaving the interrupt (because during the
    execution of the interrupt a task of equal or higher priority than the
    running task was unblocked).  The syntax required to context switch from
    an interrupt is port dependent, so check the documentation of the port you
    are using.

    In this case, the function is running in the context of the tick interrupt,
    which will automatically check for the higher priority task to run anyway,
    so no further action is required. */
}
/*-----------------------------------------------------------*/

void vApplicationMallocFailedHook(void)
{
    /* The malloc failed hook is enabled by setting
    configUSE_MALLOC_FAILED_HOOK to 1 in FreeRTOSConfig.h.

    Called if a call to pvPortMalloc() fails because there is insufficient
    free memory available in the FreeRTOS heap.  pvPortMalloc() is called
    internally by FreeRTOS API functions that create tasks, queues, software
    timers, and semaphores.  The size of the FreeRTOS heap is set by the
    configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */
    CLOGD("malloc failed\n");
    while (1);
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook(TaskHandle_t xTask, char* pcTaskName)
{
    /* Run time stack overflow checking is performed if
    configconfigCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
    function is called if a stack overflow is detected.  pxCurrentTCB can be
    inspected in the debugger if the task name passed into this function is
    corrupt. */
    CLOGD("Stack Overflow\n");
    while (1);
}
/*-----------------------------------------------------------*/

extern UBaseType_t uxCriticalNesting;
void vApplicationIdleHook(void)
{
    // volatile size_t xFreeStackSpace;
    /* The idle task hook is enabled by setting configUSE_IDLE_HOOK to 1 in
    FreeRTOSConfig.h.

    This function is called on each cycle of the idle task.  In this case it
    does nothing useful, other than report the amount of FreeRTOS heap that
    remains unallocated. */
    /* By now, the kernel has allocated everything it is going to, so
    if there is a lot of heap remaining unallocated then
    the value of configTOTAL_HEAP_SIZE in FreeRTOSConfig.h can be
    reduced accordingly. */
}
#endif
