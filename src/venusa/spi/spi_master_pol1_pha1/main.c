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
#define SPI_BUF_SIZE          8 // Original buffer size, can be used for some legacy tests or as a small test case
#endif

#ifndef MAX_TEST_BUF_SIZE
#define MAX_TEST_BUF_SIZE     128 // Define a larger buffer for various test lengths
#endif

uint8_t buf_send[MAX_TEST_BUF_SIZE];
uint8_t buf_recv[MAX_TEST_BUF_SIZE];  

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

// Helper function to run a specific SPI test case
static void run_spi_test_case(const char* test_name, uint32_t io_flags, uint32_t data_length, 
                              const uint8_t* p_send_data, const uint8_t* p_expected_receive_data)
{
    TEST_ASSERT_MESSAGE(data_length <= MAX_TEST_BUF_SIZE, "Test data_length exceeds MAX_TEST_BUF_SIZE.");
    TEST_ASSERT_MESSAGE(p_send_data != NULL, "p_send_data cannot be NULL.");
    // p_expected_receive_data can be NULL if we don't want to verify received content (e.g., for a simple loopback where send = expected)

    CLOGD("Running SPI Test Case: %s, Length: %u bytes, IO Flags: 0x%X", test_name, data_length, io_flags);

    int ret;

    // Prepare send buffer
    memcpy(buf_send, p_send_data, data_length);
    memset(buf_recv, 0, MAX_TEST_BUF_SIZE); // Clear receive buffer before test

    ret = SPI_Initialize(spi_dev, SPI_DrvEvent_m, (uint32_t)spi_dev);
    TEST_ASSERT_EQUAL_INT_MESSAGE(CSK_DRIVER_OK, ret, "SPI_Initialize failed.");

    ret = SPI_PowerControl(spi_dev, CSK_POWER_FULL);
    TEST_ASSERT_EQUAL_INT_MESSAGE(CSK_DRIVER_OK, ret, "SPI_PowerControl (FULL) failed.");

    ret = SPI_Control(spi_dev, CSK_SPI_MODE_MASTER | io_flags |
                 CSK_SPI_CPOL1_CPHA1 |      // Mode 3
                 CSK_SPI_DATA_BITS(8) |
                 CSK_SPI_MSB_LSB, SPI_SCK_FREQ);
    TEST_ASSERT_EQUAL_INT_MESSAGE(CSK_DRIVER_OK, ret, "SPI_Control failed.");

    // For SPI, Send and Receive are often sequential for master half-duplex type operations,
    // or can be simultaneous (Transfer). The current driver API implies sequential.
    // The Pico slave is designed to receive first, then send.

    CLOGD("Sending data...");
    for(uint32_t i=0; i < data_length; i++) { CLOGD("TX[%u]: 0x%02X", i, buf_send[i]); }

    ret = SPI_Send(spi_dev, buf_send, data_length);
    TEST_ASSERT_EQUAL_INT_MESSAGE(CSK_DRIVER_OK, ret, "SPI_Send failed.");

    // Add a small delay if necessary, though slave should be ready.
    // For example, if slave needs time to process received data before preparing its reply.
    // CSK_DelayMs(1); // If issues arise, can try a small delay.

    CLOGD("Receiving data...");
    ret = SPI_Receive(spi_dev, buf_recv, data_length);
    TEST_ASSERT_EQUAL_INT_MESSAGE(CSK_DRIVER_OK, ret, "SPI_Receive failed.");

    for(uint32_t i=0; i < data_length; i++) { CLOGD("RX[%u]: 0x%02X", i, buf_recv[i]); }

    if (p_expected_receive_data != NULL) {
        TEST_ASSERT_EQUAL_UINT8_ARRAY_MESSAGE(p_expected_receive_data, buf_recv, data_length, "Received data does not match expected data.");
        CLOGD("Data verification successful.");
    } else {
        // If no expected pattern, common for loopback, check if sent data equals received data
        TEST_ASSERT_EQUAL_UINT8_ARRAY_MESSAGE(buf_send, buf_recv, data_length, "Received data does not match sent data (loopback check).");
        CLOGD("Loopback data verification successful.");
    }
    
    // Power off and Uninitialize after each test case to ensure clean state for the next
    ret = SPI_PowerControl (spi_dev, CSK_POWER_OFF);
    TEST_ASSERT_EQUAL_INT_MESSAGE(CSK_DRIVER_OK, ret, "SPI_PowerControl (OFF) failed.");
    
    ret = SPI_Uninitialize(spi_dev);
    TEST_ASSERT_EQUAL_INT_MESSAGE(CSK_DRIVER_OK, ret, "SPI_Uninitialize failed.");
    
    CLOGD("Test Case %s PASSED.", test_name);
}


void setUp(void) {
    // test configure
    // config_spi() is called once in main before UNITY_BEGIN.
    // Individual tests re-initialize/de-initialize SPI.
}

void tearDown(void) {
    // test clear
    // close_spi() is called once in main after UNITY_END.
}

void test_spi_pio_variable_lengths_and_patterns(void); // Renamed for clarity
void test_spi_dma_variable_lengths_and_patterns(void); // Renamed for clarity

int main()
{
    logInit(0, 115200);
    CLOGD("\r\n++++++++++ SPI%d Master POL=1 PHA=1 Variable Length Tests +++++++++++\r\n\r\n", SPI_INSTANCE);

    config_spi(); // Initial hardware pin configuration

    UNITY_BEGIN();  
    RUN_TEST(test_spi_pio_variable_lengths_and_patterns);
    // RUN_TEST(test_spi_dma_variable_lengths_and_patterns); // Can enable DMA tests later
    UNITY_END();
    
    close_spi(spi_dev); // Final cleanup of SPI pins/clocks

    return 0;
}

// Test function for PIO mode with various scenarios
void test_spi_pio_variable_lengths_and_patterns(void)
{
    uint8_t pattern_aa[MAX_TEST_BUF_SIZE];
    uint8_t pattern_55[MAX_TEST_BUF_SIZE];
    uint8_t pattern_inc[MAX_TEST_BUF_SIZE];
    uint8_t pattern_pico_send[] = {0xA1, 0xB2, 0xC3, 0xD4}; // Master sends this
    uint8_t pattern_pico_expect[] = {0x1A, 0x2B, 0x3C, 0x4D}; // Master expects this from Pico

    memset(pattern_aa, 0xAA, MAX_TEST_BUF_SIZE);
    memset(pattern_55, 0x55, MAX_TEST_BUF_SIZE);
    for(uint32_t i=0; i < MAX_TEST_BUF_SIZE; i++) { pattern_inc[i] = i; }

    // Scenario 1: 1 byte, pattern 0xAA, loopback (Pico slave should be set to echo)
    run_spi_test_case("PIO_1_byte_0xAA_loopback", CSK_SPI_TXIO_PIO | CSK_SPI_RXIO_PIO, 1, pattern_aa, pattern_aa);

    // Scenario 2: 5 bytes, pattern 0x55, loopback
    run_spi_test_case("PIO_5_bytes_0x55_loopback", CSK_SPI_TXIO_PIO | CSK_SPI_RXIO_PIO, 5, pattern_55, pattern_55);
    
    // Scenario 3: 16 bytes, incremental pattern, loopback
    run_spi_test_case("PIO_16_bytes_inc_loopback", CSK_SPI_TXIO_PIO | CSK_SPI_RXIO_PIO, 16, pattern_inc, pattern_inc);

    // Scenario 4: 32 bytes, pattern 0xAA, loopback
    run_spi_test_case("PIO_32_bytes_0xAA_loopback", CSK_SPI_TXIO_PIO | CSK_SPI_RXIO_PIO, 32, pattern_aa, pattern_aa);

    // Scenario 5: MAX_TEST_BUF_SIZE bytes (e.g. 128), incremental pattern, loopback
    run_spi_test_case("PIO_MAX_bytes_inc_loopback", CSK_SPI_TXIO_PIO | CSK_SPI_RXIO_PIO, MAX_TEST_BUF_SIZE, pattern_inc, pattern_inc);

    // Scenario 6: Test with Pico custom pattern (Pico expects 0xA1,B2,C3,D4 and sends 0x1A,2B,3C,4D)
    // Ensure Pico slave script is configured for this specific exchange.
    TEST_ASSERT_MESSAGE(sizeof(pattern_pico_send) == sizeof(pattern_pico_expect), "Pico send/expect pattern length mismatch in test code.");
    run_spi_test_case("PIO_Pico_Custom_Pattern_4bytes", CSK_SPI_TXIO_PIO | CSK_SPI_RXIO_PIO, 
                      sizeof(pattern_pico_send), pattern_pico_send, pattern_pico_expect);
    
    // Scenario 7: Test with a different length for Pico custom pattern (e.g. 2 bytes from the defined 4)
    // Pico should be configured to expect [0xA1, 0xB2] and send [0x1A, 0x2B]
    // For this, we would need another set of pico_send_short/pico_expect_short or adjust Pico config.
    // For now, this demonstrates using a portion of a larger defined pattern.
    // This requires the Pico slave to be configured for this specific 2-byte exchange.
    // If Pico is expecting 4 bytes (from Scenario 6) and gets 2, it might fail or behave unexpectedly.
    // For a robust test, ensure slave configuration matches *each* test case.
    // Let's assume Pico is configured for: expect [0xA1,0xB2], send [0x1A,0x2B] for this specific test.
    uint8_t pattern_pico_send_short[] = {0xA1, 0xB2};
    uint8_t pattern_pico_expect_short[] = {0x1A, 0x2B};
     run_spi_test_case("PIO_Pico_Custom_Pattern_2bytes", CSK_SPI_TXIO_PIO | CSK_SPI_RXIO_PIO, 
                       sizeof(pattern_pico_send_short), pattern_pico_send_short, pattern_pico_expect_short);

    // Add more test cases as needed
}

// Test function for DMA mode with various scenarios (similar structure to PIO)
void test_spi_dma_variable_lengths_and_patterns(void)
{
    // Optional: Implement if time permits and DMA testing is required with variable lengths.
    // Structure would be very similar to test_spi_pio_variable_lengths_and_patterns,
    // just using (CSK_SPI_TXIO_DMA | CSK_SPI_RXIO_DMA) as io_flags.
    
    // Example:
    uint8_t pattern_dma[MAX_TEST_BUF_SIZE];
    memset(pattern_dma, 0xDD, MAX_TEST_BUF_SIZE);

    // run_spi_test_case("DMA_16_bytes_0xDD_loopback", CSK_SPI_TXIO_DMA | CSK_SPI_RXIO_DMA, 16, pattern_dma, pattern_dma);
    
    CLOGW("DMA tests are not fully implemented in this iteration.");
    // To make this test pass if uncommented, we need at least one assertion or call to a test function.
    // For now, keeping it minimal. If RUN_TEST calls this, it needs to do something.
    TEST_MESSAGE("DMA variable length tests are placeholders.");

}
