/* Standard includes. */
#include <stdio.h>
#include <string.h>

#include "log_print.h"
#include "chip.h"
#include <FreeRTOS.h>
#include <semphr.h>
#include <queue.h>
#include "Driver_DUAL_TIMER.h"
#include "Driver_JPEG.h"
#include "ls_jpeg.h"
#include "jpeg_check.h"


#define LOG_MESSAGE_MAX_LENGTH 128

typedef struct {
    char message[LOG_MESSAGE_MAX_LENGTH];
} LogMessage_t;

#undef  CLOGD
#define CLOGD(fmt, ...) do { \
   LogMessage_t logMessage; \
   if (snprintf(logMessage.message, LOG_MESSAGE_MAX_LENGTH, fmt, ##__VA_ARGS__) > 0) { \
       xQueueSend(logQueue, &logMessage, 0); \
   } \
} while (0)


QueueHandle_t logQueue;
QueueHandle_t rawImageQueue;
QueueHandle_t encImageQueue;

TaskHandle_t hLogTask;
SemaphoreHandle_t dvpSemaphore;

typedef struct {
    uint8_t* image;
    uint32_t size;
} encImageInfo_t;


/* YUV422 or RGB565 */
uint8_t dvp_image[DVP_IMAGE_WIDTH * DVP_IMAGE_HEIGHT * 2];
uint8_t dvp_image_shadow[DVP_IMAGE_WIDTH * DVP_IMAGE_HEIGHT * 2];
uint8_t jpeg_buffer[49152];
uint8_t jpeg_buffer_shadow[49152];


volatile uint8_t dvp_image_in_use = 0; // 0: dvp_image, 1: dvp_image_shadow
volatile uint8_t dvp_image_shadow_in_use = 0; // 0: dvp_image, 1: dvp_image_shadow
volatile uint8_t jpeg_buffer_in_use = 0; // 0: jpeg_buffer, 1: jpeg_buffer_shadow
volatile uint8_t jpeg_buffer_shadow_in_use = 0; // 0: jpeg_buffer, 1: jpeg_buffer_shadow

Jpeg_CodecCfg enc_cfg = 
{
    .width = 320,
    .height = 240,
    .pixel_format = JPEG_PIXEL_FORMAT_YUV422,
};


void usb_task(void* pvParameters);
void dvp_task(void* pvParameters);
void jpeg_task(void* pvParameters);
void log_task(void *pvParameters);

extern int insmod_uvc_module();
extern int32_t test_jpeg_dvp_start(Jpeg_CodecCfg *enc_cfg, uint8_t *dvp_image, uint32_t imgae_size);
extern void test_jpeg_dvp_stop();
extern void test_jpeg_dvp_restart(uint8_t *dvp_image, uint32_t imgae_size);
extern void video_send_frame(uint8_t *encoded_frame, uint32_t encoded_size);


void log_task(void *pvParameters)
{
    LogMessage_t logMessage;

    while (1) {
        if (xQueueReceive(logQueue, &logMessage, portMAX_DELAY) == pdPASS) {
            CLOG("%s, logging by core-%d", logMessage.message, (__RV_CSR_READ(CSR_MHARTID) & 0xFF));
        }
        
        static uint32_t tick_prev, tick_curr = 0;
        tick_curr = xTaskGetTickCount();
        if (tick_curr - tick_prev > 1000) {
            tick_prev = tick_curr;
            CLOG("Free Heap: %d bytes\n", xPortGetFreeHeapSize());

            uint64_t ulTotalRunTime = 0;
            uint32_t ulTotalTask = uxTaskGetNumberOfTasks();
            TaskStatus_t *pxTaskStatusArray = pvPortMalloc(ulTotalTask * sizeof(TaskStatus_t));

            uxTaskGetSystemState(pxTaskStatusArray, ulTotalTask, &ulTotalRunTime);
            for (int i=0; i < ulTotalTask; i++) {
                float cpuUsage = (pxTaskStatusArray[i].ulRunTimeCounter * 100.0) / ulTotalRunTime;
                int usage = (int)(cpuUsage * 10);
                CLOG("Task: %s, CPU Usage: %d.%d%%\n", pxTaskStatusArray[i].pcTaskName, (int)(usage/10), (int)(usage % 10));
            }
            vPortFree(pxTaskStatusArray);
        }
    }
}


void usb_task(void* pvParameters)
{
    int cnt = 0;

    encImageInfo_t image_info;

    CLOG("Enter to usb_task\r\n");

    /* Load UVC module */
    insmod_uvc_module();

    while (1) {
        CLOGD("[Core-%u] usb_task is running %d.....", (__RV_CSR_READ(CSR_MHARTID) & 0xFF), cnt++);
        if(xQueueReceive(encImageQueue, &image_info, portMAX_DELAY) == pdPASS) {
            CLOGD("[Core-%u] usb_task received jpeg buffer %p with size %d.....", (__RV_CSR_READ(CSR_MHARTID) & 0xFF), image_info.image, image_info.size);

            video_send_frame(image_info.image, image_info.size);

            // Wait for the USB transfer to complete
            while (!tud_video_n_streaming(0, 0)) {
                vTaskDelay(10);
            }

            // Release the jpeg buffer
            if (image_info.image == jpeg_buffer) {
                jpeg_buffer_in_use = 0;
            } else {
                jpeg_buffer_shadow_in_use = 0;
            }
            
        }
    }
}


void jpeg_task(void* pvParameters)
{
    int cnt = 0;
    uint8_t *raw_image_in_use = NULL;
    uint8_t *jpeg_image_in_use = NULL;

    CLOGD("Enter to jpeg_task\r\n");

    while (1) {
        CLOGD("[Core-%u] jpeg_task is running %d.....", (__RV_CSR_READ(CSR_MHARTID) & 0xFF), cnt++);
        if(xQueueReceive(rawImageQueue, &raw_image_in_use, portMAX_DELAY) == pdPASS) {
            CLOGD("[Core-%u] jpeg_task received dvp image %p.....", (__RV_CSR_READ(CSR_MHARTID) & 0xFF), raw_image_in_use);

            if (jpeg_buffer_in_use == 0) {
                jpeg_buffer_in_use = 1;
                jpeg_image_in_use = jpeg_buffer;                             
            } else if(jpeg_buffer_shadow_in_use == 0) {
                jpeg_buffer_shadow_in_use = 1;
                jpeg_image_in_use = jpeg_buffer_shadow; 
            } else {
                CLOGD("[Core-%u] jpeg buffer was not consumed by usb task",  (__RV_CSR_READ(CSR_MHARTID) & 0xFF));
                // skip this frame due to jpeg buffer full
                if (raw_image_in_use == dvp_image) {
                    dvp_image_in_use = 0;
                } else {
                    dvp_image_shadow_in_use = 0;
                }
                continue;
            }

            uint32_t enc_output_jpeg_size = 0;
            jpeg_encoder_ext(raw_image_in_use, jpeg_image_in_use, &enc_output_jpeg_size, &enc_cfg);
            CLOGD("[Core-%u] jpeg encoded size:  %dbyte.....", (__RV_CSR_READ(CSR_MHARTID) & 0xFF), enc_output_jpeg_size);

            encImageInfo_t enc_image_info;
            enc_image_info.image = jpeg_image_in_use;
            enc_image_info.size = enc_output_jpeg_size;
            if(xQueueSend(encImageQueue, &enc_image_info, 0) != pdPASS) {
                CLOGD("[Core-%u] jpeg_task failed to send jpeg buffer to usb task due to full of the queue", (__RV_CSR_READ(CSR_MHARTID) & 0xFF));
                // Release the jpeg buffer if sending failed
                if(jpeg_image_in_use == jpeg_buffer) {
                    jpeg_buffer_in_use = 0;
                } else {
                    jpeg_buffer_shadow_in_use = 0;
                }

            }

            // Release the raw image buffer
            if (raw_image_in_use == dvp_image) {
                dvp_image_in_use = 0;
            } else {
                dvp_image_shadow_in_use = 0;
            }

        }
        

    }
}


void dvp_task(void* pvParameters)
{
    int cnt = 0;
    uint8_t *raw_image_in_use = NULL;

    CLOG("Enter to dvp_task\r\n");

    /* Start DVP to receive frames */
    test_jpeg_dvp_start(&enc_cfg, dvp_image, DVP_IMAGE_WIDTH * DVP_IMAGE_HEIGHT * 2);
    raw_image_in_use = dvp_image;

    while (1) {
        CLOGD("[Core-%u] dvp_task is running %d.....", (__RV_CSR_READ(CSR_MHARTID) & 0xFF), cnt++);
        if (xSemaphoreTake(dvpSemaphore, portMAX_DELAY) == pdTRUE) {
            // Stop DVP to process the current frame
            test_jpeg_dvp_stop();

            xQueueSend(rawImageQueue, &raw_image_in_use, 0);

            // Wait for one of the dvp_image buffers to be free
            while(dvp_image_in_use == 1 && dvp_image_shadow_in_use == 1) {
                vTaskDelay(10);
            }
            
            // Switch to the shadow buffer if the main buffer is in use
            if (dvp_image_in_use == 0) {
                dvp_image_in_use = 1;
                raw_image_in_use = dvp_image;
                // Restart DVP
                test_jpeg_dvp_restart(dvp_image, DVP_IMAGE_WIDTH * DVP_IMAGE_HEIGHT * 2);                                
            } else if(dvp_image_shadow_in_use == 0) {
                dvp_image_shadow_in_use = 1;
                raw_image_in_use = dvp_image_shadow;
                // Restart DVP
                test_jpeg_dvp_restart(dvp_image_shadow, DVP_IMAGE_WIDTH * DVP_IMAGE_HEIGHT * 2);
            } else {
                CLOGD("[Core-%u] dvp_task should not reach here",  (__RV_CSR_READ(CSR_MHARTID) & 0xFF));
                while(1) {
                    vTaskDelay(10);
                }
            }
        }
       
    }
}


int main( void )
{
    logInit(0, 921600);

    CLOG("Gimbal Demo");

    BaseType_t xResult;

    logQueue = xQueueCreate(10, sizeof(LogMessage_t) + 16);
    if (logQueue == NULL) {
        CLOG("failed to create log queue.\n");
    }

    rawImageQueue = xQueueCreate(2, sizeof(uint32_t));
    if (rawImageQueue == NULL) {
        CLOG("failed to create raw image queue.\n");
    }

    encImageQueue = xQueueCreate(2, sizeof(encImageInfo_t));
    if (encImageQueue == NULL) {
        CLOG("failed to create encoded image queue.\n");
    }

    dvpSemaphore = xSemaphoreCreateBinary();
    if (dvpSemaphore == NULL) {
        CLOG("failed to create semaphore.\n");
    }

    // Create the log task with affinity. So it can only run on the selected core.
    // xResult = xTaskCreateAffinitySet(
    //     log_task,  /* The function that implements the task. */
    //     "LogTask", /* Text name for the task. */
    //     256,       /* Stack depth in words. */
    //     NULL,      /* Task parameters. */
    //     2,         /* Priority and mode (user in this case). */
    //     (1<<1),    /* Affinity. Set to core 1. */
    //     &hLogTask  /* Handle. */
    // );

    // Create the log task without affinity. So it can run on any core.
    xResult = xTaskCreate(
            log_task,  /* The function that implements the task. */
            "LogTask", /* Text name for the task. */
            512,       /* Stack depth in words. */
            NULL,      /* Task parameters. */
            1,         /* Priority and mode (user in this case). */
            &hLogTask  /* Handle. */
        );

    if (xResult != pdPASS) {
        CLOG("failed to create LogTask.\n");
    }
    
    xResult = xTaskCreate((TaskFunction_t)usb_task, (const char*)"usb_task",
                (uint16_t)512, (void*)NULL, (UBaseType_t)3,
                NULL);
    if( xResult != pdPASS ) {
        CLOG("failed to create usb_task.\n");
    }

    xResult = xTaskCreate((TaskFunction_t)dvp_task, (const char*)"dvp_task",
                (uint16_t)512, (void*)NULL, (UBaseType_t)2,
                NULL);
    if( xResult != pdPASS ) {
        CLOG("failed to create dvp_task.\n");
    }

   xResult = xTaskCreate((TaskFunction_t)jpeg_task, (const char*)"jpeg_task",
               (uint16_t)512, (void*)NULL, (UBaseType_t)2,
               NULL);
   if( xResult != pdPASS ) {
       CLOG("failed to create jpeg_task.\n");
   }


    /* Start the Core-1 */
    extern uint32_t _start;
    start_core1((uint32_t)&_start);


    /* Start the scheduler. */
    vTaskStartScheduler();

    /* Will only get here if there was insufficient memory to create the idle
    task. */
    for( ;; );
}


void eclic_inter_core_int_handler()
{
    uint32_t sender_id = 0;
    unsigned long hartid = __get_hart_id();

    uint32_t val = CIDU_QueryCoreIntSenderMask(hartid);

    if(0 != val) {
        CIDU_ClearInterCoreIntReq(hartid == 0 ? 1 : 0, hartid);
        SysTimer_SetHartSWIRQ(hartid);
    }
}


void smp_main(void)
{
    if(__RV_CSR_READ(CSR_MHARTID) & 0xFF)
    {
        disable_GINT();
        CIDU_ClearInterCoreIntReq(0, 1);
        clear_IRQ(IRQ_IDU_VECTOR);
        enable_IRQ(IRQ_IDU_VECTOR);
    
        xPortStartScheduler();
    }
    else
    {
        disable_GINT();
        CIDU_ClearInterCoreIntReq(1, 0);
        clear_IRQ(IRQ_IDU_VECTOR);
        register_ISR(IRQ_IDU_VECTOR, (void*)eclic_inter_core_int_handler, NULL);
        enable_IRQ(IRQ_IDU_VECTOR);
    
        main();
    }
}

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
    CLOG("malloc failed\n");
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
    CLOG("Stack Overflow\n");
    while (1);
}
/*-----------------------------------------------------------*/

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
