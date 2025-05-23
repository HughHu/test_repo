# Pico PIO SPI Slave Testing Guide

## 1. Overview

This guide describes how to use the Raspberry Pi Pico as a PIO-based SPI slave for testing SPI master implementations, specifically targeting the Venusa SPI master applications.

The `test_spi_pio.py` script allows the Pico to emulate an SPI slave device that can operate in any of the four standard SPI modes (0, 1, 2, or 3). It offers flexibility in configuring expected data from the master and the data it should send in response, making it a useful tool for verifying SPI communication.

## 2. Hardware Setup

Connect the Raspberry Pi Pico to the Venusa device (or other SPI master) as follows:

*   **MOSI:** Pico GP3 (Master Out, Slave In) to Venusa MOSI
*   **MISO:** Pico GP4 (Master In, Slave Out) to Venusa MISO
*   **SCK:** Pico GP2 (Serial Clock) to Venusa SCK
*   **CS:** Pico GP5 (Chip Select) to Venusa CS
*   **GND:** Pico GND to Venusa GND

**Note:** These pin assignments are defined at the top of `test_spi_pio.py` (`PIN_MOSI`, `PIN_MISO`, `PIN_SCK`, `PIN_CS`). You can modify them if your hardware setup requires different pins. Ensure `SCK_PIN_FOR_PIO` global constant is also updated if `PIN_SCK` is changed, as the PIO programs use this.

## 3. Pico Slave (`test_spi_pio.py`) Configuration

The `test_spi_pio.py` script needs to be configured before running it on the Pico.

### 3.1. Selecting SPI Mode

To select the SPI mode for the slave to operate in, modify the `SELECTED_SPI_MODE` variable at the beginning of the `if __name__ == "__main__":` block in the script:

```python
# Example: To select SPI Mode 0
SELECTED_SPI_MODE = 0 

# Example: To select SPI Mode 1
# SELECTED_SPI_MODE = 1 

# ... and so on for modes 2 and 3.
```

### 3.2. Configuring Expected and Reply Data

The script allows you to define what data the Pico slave expects to receive from the master and what data it should send back. This is configured using `EXPECTED_FROM_MASTER` and `DATA_FOR_MASTER` variables within the `if __name__ == "__main__":` block, typically within the conditional blocks for `SELECTED_SPI_MODE`.

*   **`EXPECTED_FROM_MASTER`**:
    *   Set to a list of bytes (e.g., `[0xAA, 0xBB, 0xCC]`) that the slave expects to receive from the master. The slave will verify the received data against this list.
    *   Set to `None` or an empty list `[]` if you want the slave to operate in "capture mode," where it receives any data from the master until CS is deactivated (or a timeout occurs) without specific verification against a predefined pattern.

*   **`DATA_FOR_MASTER`**:
    *   Set to a list of bytes (e.g., `[0x11, 0x22, 0x33]`) that the slave should send back to the master.
    *   If `DATA_FOR_MASTER` is an empty list `[]` (and `EXPECTED_FROM_MASTER` was not empty), the slave will by default echo back the data it received (or an incremented version, depending on the exact logic in `PioSpiSlave.process()`). Check the `process()` method for the precise default behavior when `reply_data` is empty.
    *   If `EXPECTED_FROM_MASTER` is `None` (capture mode) and `DATA_FOR_MASTER` is `[]`, the slave will echo the captured data.

**Examples:**

```python
# Example for Mode 0: Expect specific data, send specific data
if SELECTED_SPI_MODE == 0:
    EXPECTED_FROM_MASTER = [0xA1, 0xB2, 0xC3] # Master is expected to send this
    DATA_FOR_MASTER = [0x3C, 0x2B, 0x1A]      # Slave will reply with this

# Example for Mode 1: Capture mode, echo back
elif SELECTED_SPI_MODE == 1:
    EXPECTED_FROM_MASTER = None 
    DATA_FOR_MASTER = [] # Will echo received data

# Example for Mode 2: Expect specific data, send default (echo/incremented)
elif SELECTED_SPI_MODE == 2:
    EXPECTED_FROM_MASTER = [0xDE, 0xAD]
    DATA_FOR_MASTER = [] 
```

### 3.3. Running the Script on Pico

1.  Connect the Raspberry Pi Pico to your computer via USB.
2.  Open `test_spi_pio.py` in an editor like Thonny IDE.
3.  Make your configurations for SPI mode, expected data, and reply data as described above.
4.  Run the script (e.g., click the "Run" button in Thonny, or use `mpremote run test_spi_pio.py` if using `mpremote`).
5.  The Pico will print initialization messages and then wait for the Chip Select (CS) line to be activated by the master.

## 4. Venusa Master Application Setup and Execution

The Venusa SDK provides several SPI master applications, each configured for a specific SPI mode. These applications are designed to test communication with an SPI slave like the Pico script.

### 4.1. Available Venusa Master Applications

The pre-configured master applications are located in `src/venusa/spi/`:

*   **Mode 0:** `src/venusa/spi/spi_master_pol0_pha0/`
*   **Mode 1:** `src/venusa/spi/spi_master_pol0_pha1/`
*   **Mode 2:** `src/venusa/spi/spi_master_pol1_pha0/`
*   **Mode 3:** `src/venusa/spi/spi_master_pol1_pha1/`

Each of these applications contains a `main.c` file that runs a series of tests, typically within a function like `test_spi_pio_variable_lengths_and_patterns`. These tests send various data patterns and lengths and verify the received data.

### 4.2. Aligning Test Patterns

For successful end-to-end testing, the data patterns configured in the Venusa master application must align with the Pico slave's `EXPECTED_FROM_MASTER` and `DATA_FOR_MASTER` settings.

The Venusa master tests are implemented in `run_spi_test_case` within its `main.c`. This function takes `p_send_data` (what the master sends) and `p_expected_receive_data` (what the master expects back from the slave).

**Example Alignment:**

If a Venusa master test case in `spi_master_pol0_pha1/main.c` is configured as:
```c
// In Venusa's main.c
uint8_t master_sends[] = {0x11, 0x22, 0x33};
uint8_t master_expects_from_slave[] = {0xCC, 0xBB, 0xAA};
run_spi_test_case("TestName", CSK_SPI_TXIO_PIO | CSK_SPI_RXIO_PIO, 
                  sizeof(master_sends), master_sends, master_expects_from_slave);
```

Then, the `test_spi_pio.py` script (with `SELECTED_SPI_MODE = 1`) should be configured as:
```python
# In test_spi_pio.py for Mode 1
EXPECTED_FROM_MASTER = [0x11, 0x22, 0x33]  # Must match master_sends
DATA_FOR_MASTER = [0xCC, 0xBB, 0xAA]       # Must match master_expects_from_slave
```

Loopback tests in the Venusa master (where `p_expected_receive_data` is the same as `p_send_data`, or NULL) require the Pico slave to be configured to echo the data it receives. This can usually be achieved by setting `DATA_FOR_MASTER = []` on the Pico side when `EXPECTED_FROM_MASTER` is set.

### 4.3. Building and Running Venusa Applications

The specific steps to build and run the Venusa SPI master applications depend on your Venusa development environment and build system. Generally:

1.  Navigate to the specific application directory (e.g., `cd src/venusa/spi/spi_master_pol0_pha1`).
2.  Use the provided `Makefile` and build system commands (e.g., `make`) to compile the application.
3.  Load and run the compiled binary on the Venusa target hardware.
4.  Observe the console output from the Venusa device for test results and logs.

## 5. Running a Test - Example Workflow

Here’s a step-by-step example for testing SPI Mode 1:

1.  **Choose SPI Mode:** Mode 1 (CPOL=0, CPHA=1).
2.  **Configure Pico Slave (`test_spi_pio.py`):**
    *   Set `SELECTED_SPI_MODE = 1`.
    *   Assume a test case in the Venusa `spi_master_pol0_pha1/main.c` sends `[0xAA, 0xBB]` and expects the slave to return `[0xCC, 0xDD]`.
    *   In `test_spi_pio.py`, set:
        ```python
        if SELECTED_SPI_MODE == 1:
            EXPECTED_FROM_MASTER = [0xAA, 0xBB]
            DATA_FOR_MASTER = [0xCC, 0xDD]
        ```
3.  **Run Pico Slave Script:**
    *   Start `test_spi_pio.py` on the Raspberry Pi Pico.
    *   The Pico's console will show it's initialized for Mode 1 and waiting for CS activation.
4.  **Build and Run Venusa Master:**
    *   Build the `spi_master_pol0_pha1` application from `src/venusa/spi/spi_master_pol0_pha1/`.
    *   Run the compiled application on the Venusa device.
5.  **Observe Results:**
    *   The Venusa console will show logs from `run_spi_test_case`, indicating the data sent, data received, and assertion results (e.g., "Data verification successful." or "Received data does not match expected data.").
    *   The Pico console will show logs indicating CS activation, data received (e.g., "Received: 0xaa 0xbb"), verification against its `EXPECTED_FROM_MASTER`, data loaded to its TX FIFO, and CS deactivation.

By comparing the logs and ensuring both sides report successful data exchange and verification, you can confirm the SPI communication for the chosen mode and data patterns. Repeat for other modes and data configurations as needed.
