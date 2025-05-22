/*
 *
 *
 */

#include <stdarg.h>
#include <assert.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include "chip.h"
#include "log_print.h"
#include "Driver_SPI.h"
#include "IOMuxManager.h"
#include "ClockManager.h" 
#include "Driver_GPIO.h"
#include "unity.h"

#ifndef SPI_INSTANCE
#define SPI_INSTANCE           0
#endif

#ifndef SPI_SCK_PIN
#define SPI_SCK_PIN            16
#endif

#ifndef SPI_CS_PIN
#define SPI_CS_PIN             17
#endif

#ifndef SPI_MISO_PIN
#define SPI_MISO_PIN           18
#endif

#ifndef SPI_MOSI_PIN
#define SPI_MOSI_PIN           19
#endif

#ifndef SPI_SCK_PAD
#define SPI_SCK_PAD           CSK_IOMUX_PAD_A
#endif

#ifndef SPI_CS_PAD
#define SPI_CS_PAD            CSK_IOMUX_PAD_A
#endif

#ifndef SPI_MISO_PAD
#define SPI_MISO_PAD          CSK_IOMUX_PAD_A
#endif

#ifndef SPI_MOSI_PAD
#define SPI_MOSI_PAD          CSK_IOMUX_PAD_A
#endif

#ifndef SPI_SCK_FREQ
#define SPI_SCK_FREQ          100000
#endif

#ifndef SPI_BUF_SIZE
#define SPI_BUF_SIZE          8
#endif


uint8_t buf_send[SPI_BUF_SIZE];
uint8_t buf_recv[SPI_BUF_SIZE];  

void* spi_dev = NULL;    

static void config_spi()
{

#if (SPI_INSTANCE == 0)   
    IOMuxManager_PinConfigure(SPI_SCK_PAD, SPI_SCK_PIN, CSK_IOMUX_FUNC_ALTER5);    // SCK
    IOMuxManager_PinConfigure(SPI_CS_PAD, SPI_CS_PIN, CSK_IOMUX_FUNC_ALTER5);      // CS
    IOMuxManager_PinConfigure(SPI_MOSI_PAD, SPI_MOSI_PIN, CSK_IOMUX_FUNC_ALTER5);  // MOSI
    IOMuxManager_PinConfigure(SPI_MISO_PAD, SPI_MISO_PIN, CSK_IOMUX_FUNC_ALTER5);  // MISO

    __HAL_CRM_SPI0_CLK_ENABLE();

    spi_dev = SPI0();

#elif (SPI_INSTANCE == 1)
    IOMuxManager_PinConfigure(SPI_SCK_PAD, SPI_SCK_PIN, CSK_IOMUX_FUNC_ALTER6);    // SCK
    IOMuxManager_PinConfigure(SPI_CS_PAD, SPI_CS_PIN, CSK_IOMUX_FUNC_ALTER6);      // CS
    IOMuxManager_PinConfigure(SPI_MOSI_PAD, SPI_MOSI_PIN, CSK_IOMUX_FUNC_ALTER6);  // MOSI
    IOMuxManager_PinConfigure(SPI_MISO_PAD, SPI_MISO_PIN, CSK_IOMUX_FUNC_ALTER6);  // MISO

    __HAL_CRM_SPI1_CLK_ENABLE();

    spi_dev = SPI1();
#else
    #error "SPI_INSTANCE not valid"
#endif

}


static void close_spi(void *spi_dev)
{
    if (spi_dev != NULL) {
        SPI_PowerControl (spi_dev, CSK_POWER_OFF);
        SPI_Uninitialize(spi_dev);
    }
}

//SPI master event callback
static void SPI_DrvEvent_m (uint32_t event, uint32_t usr_param)
{
    void *spi_dev = (void *)usr_param;
    assert(spi_dev != NULL);

    if( !(event & CSK_SPI_EVENT_TRANSFER_COMPLETE) ) {
        CSK_SPI_STATUS status;
        status.all = 0;
        SPI_GetStatus(spi_dev, &status);
        CLOGD("%s: event = 0x%x (status = 0x%x), other than TRANSFER_COMPLETE!\r\n",
                __func__, event, status.all);
        return;
    }

    return;
}

void setUp(void) {
    // test configure
}

void tearDown(void) {
    // test clear
}

void test_spi_pio(void);
void test_spi_dma(void);

int main()
{
    logInit(0, 115200);
    CLOGD("\r\n++++++++++ SPI%d Master POL=0 PHA=0 +++++++++++\r\n\r\n", SPI_INSTANCE);

    config_spi();

    UNITY_BEGIN();  
    RUN_TEST(test_spi_pio);
    RUN_TEST(test_spi_dma);
    UNITY_END();
    
    close_spi(spi_dev);

    return 0;
}

void test_spi_pio(void)
{
    int ret;

    memset(buf_send, 0x55, SPI_BUF_SIZE);

    ret = SPI_Initialize(spi_dev, SPI_DrvEvent_m, (uint32_t)spi_dev);
    TEST_ASSERT(ret == CSK_DRIVER_OK);

    ret = SPI_PowerControl(spi_dev, CSK_POWER_FULL);
    TEST_ASSERT(ret == CSK_DRIVER_OK);

    ret = SPI_Control(spi_dev, CSK_SPI_MODE_MASTER | CSK_SPI_TXIO_PIO | CSK_SPI_RXIO_PIO |
                 CSK_SPI_CPOL0_CPHA0 |
                 CSK_SPI_DATA_BITS(8) |
                 CSK_SPI_MSB_LSB, SPI_SCK_FREQ);
    TEST_ASSERT(ret == CSK_DRIVER_OK);


    // send SPI_BUF_SIZE bytes
    ret = SPI_Send(spi_dev, buf_send, SPI_BUF_SIZE);
    TEST_ASSERT(ret == CSK_DRIVER_OK);

    // receive SPI_BUF_SIZE bytes
    ret = SPI_Receive(spi_dev, buf_recv, SPI_BUF_SIZE);
    TEST_ASSERT(ret == CSK_DRIVER_OK);
    // check received data
    TEST_ASSERT(memcmp(buf_send, buf_recv, SPI_BUF_SIZE) == 0);

    SPI_PowerControl (spi_dev, CSK_POWER_OFF);
    SPI_Uninitialize(spi_dev);
}

void test_spi_dma(void)
{
    int ret;

    memset(buf_send, 0x55, SPI_BUF_SIZE);

    ret = SPI_Initialize(spi_dev, SPI_DrvEvent_m, (uint32_t)spi_dev);
    TEST_ASSERT(ret == CSK_DRIVER_OK);

    ret = SPI_PowerControl(spi_dev, CSK_POWER_FULL);
    TEST_ASSERT(ret == CSK_DRIVER_OK);

    ret = SPI_Control(spi_dev, CSK_SPI_MODE_MASTER | CSK_SPI_TXIO_DMA | CSK_SPI_RXIO_DMA |
                 CSK_SPI_CPOL0_CPHA0 |
                 CSK_SPI_DATA_BITS(8) |
                 CSK_SPI_MSB_LSB, SPI_SCK_FREQ);
    TEST_ASSERT(ret == CSK_DRIVER_OK);

    // send SPI_BUF_SIZE bytes
    ret = SPI_Send(spi_dev, buf_send, SPI_BUF_SIZE);
    TEST_ASSERT(ret == CSK_DRIVER_OK);

    // receive SPI_BUF_SIZE bytes
    ret = SPI_Receive(spi_dev, buf_recv, SPI_BUF_SIZE);
    TEST_ASSERT(ret == CSK_DRIVER_OK);
    // check received data
    TEST_ASSERT(memcmp(buf_send, buf_recv, SPI_BUF_SIZE) == 0);

    SPI_PowerControl (spi_dev, CSK_POWER_OFF);
    SPI_Uninitialize(spi_dev);
}
