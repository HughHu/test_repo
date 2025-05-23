
from rp2 import PIO, StateMachine, asm_pio
from machine import Pin
import time

# Pin definitions (ensure these match your hardware setup)
PIN_MOSI = 3  # Master Out, Slave In (connected to GP3 on Pico)
PIN_MISO = 4  # Master In, Slave Out (connected to GP4 on Pico)
PIN_SCK = 2   # Serial Clock (connected to GP2 on Pico) - Used by PIO for `wait`
PIN_CS = 5    # Chip Select (connected to GP5 on Pico) - Handled by Python

# Global constant for SCK pin number, accessible by the PIO program.
# Note: @asm_pio decorated functions capture globals from their definition context.
# This means PIN_SCK must be defined before pio_spi_slave_mode0.
# This is a common way to pass "constants" to PIO programs.
SCK_PIN_FOR_PIO = PIN_SCK 

@asm_pio(
    autopush=True,      # Automatically push data from ISR to RX FIFO
    push_thresh=8,      # Push to RX FIFO after 8 bits (1 byte)
    autopull=True,      # Automatically pull data from TX FIFO to OSR
    pull_thresh=8,      # Pull from TX FIFO when OSR has 8 bits (ready for 1 byte)
    in_shiftdir=PIO.SHIFT_LEFT,  # Shift data in from MSB first
    out_shiftdir=PIO.SHIFT_LEFT, # Shift data out from MSB first
    out_init=PIO.OUT_LOW # Initialize MISO pin to low (applies to all modes)
)
def pio_spi_slave_mode0(): # CPOL=0, CPHA=0
    # SCK idle low. Sample on rising edge, output on falling edge.
    wrap_target()
    label("bit_loop_mode0")
    wait(1, gpio, SCK_PIN_FOR_PIO)
    in_(pins, 1)
    wait(0, gpio, SCK_PIN_FOR_PIO)
    out(pins, 1)
    jmp("bit_loop_mode0")
    wrap()

@asm_pio(
    autopush=True, push_thresh=8, autopull=True, pull_thresh=8,
    in_shiftdir=PIO.SHIFT_LEFT, out_shiftdir=PIO.SHIFT_LEFT,
    out_init=PIO.OUT_LOW
)
def pio_spi_slave_mode1(): # CPOL=0, CPHA=1
    # SCK idle low. Output on rising edge, sample on falling edge.
    wrap_target()
    label("bit_loop_mode1")
    wait(1, gpio, SCK_PIN_FOR_PIO)
    out(pins, 1)
    wait(0, gpio, SCK_PIN_FOR_PIO)
    in_(pins, 1)
    jmp("bit_loop_mode1")
    wrap()

@asm_pio(
    autopush=True, push_thresh=8, autopull=True, pull_thresh=8,
    in_shiftdir=PIO.SHIFT_LEFT, out_shiftdir=PIO.SHIFT_LEFT,
    out_init=PIO.OUT_LOW # MISO starts low, will be driven high if SCK idle high and first bit is 1
)
def pio_spi_slave_mode2(): # CPOL=1, CPHA=0
    # SCK idle high. Sample on falling edge, output on rising edge.
    wrap_target()
    label("bit_loop_mode2")
    wait(0, gpio, SCK_PIN_FOR_PIO)
    in_(pins, 1)
    wait(1, gpio, SCK_PIN_FOR_PIO)
    out(pins, 1)
    jmp("bit_loop_mode2")
    wrap()

@asm_pio(
    autopush=True, push_thresh=8, autopull=True, pull_thresh=8,
    in_shiftdir=PIO.SHIFT_LEFT, out_shiftdir=PIO.SHIFT_LEFT,
    out_init=PIO.OUT_LOW
)
def pio_spi_slave_mode3(): # CPOL=1, CPHA=1
    # SCK idle high. Output on falling edge, sample on rising edge.
    wrap_target()
    label("bit_loop_mode3")
    wait(0, gpio, SCK_PIN_FOR_PIO)
    out(pins, 1)
    wait(1, gpio, SCK_PIN_FOR_PIO)
    in_(pins, 1)
    jmp("bit_loop_mode3")
    wrap()

class PioSpiSlave:
    def __init__(self, sm_id, spi_mode, 
                 mosi_pin_num, miso_pin_num, sck_pin_num_for_pio_wait, cs_pin_num,
                 expected_data_to_receive=None, data_to_send_on_receive=None):
        
        global SCK_PIN_FOR_PIO
        SCK_PIN_FOR_PIO = sck_pin_num_for_pio_wait 

        self.spi_mode = spi_mode
        self.mosi_pin = Pin(mosi_pin_num)
        self.miso_pin = Pin(miso_pin_num, Pin.OUT)
        self.cs_pin = Pin(cs_pin_num, Pin.IN, Pin.PULL_UP)

        self.expected_data = expected_data_to_receive if expected_data_to_receive is not None else []
        self.reply_data = data_to_send_on_receive if data_to_send_on_receive is not None else []

        pio_program = None
        if spi_mode == 0:
            pio_program = pio_spi_slave_mode0
        elif spi_mode == 1:
            pio_program = pio_spi_slave_mode1
        elif spi_mode == 2:
            pio_program = pio_spi_slave_mode2
        elif spi_mode == 3:
            pio_program = pio_spi_slave_mode3
        else:
            raise ValueError("Invalid SPI mode specified. Must be 0, 1, 2, or 3.")

        # StateMachine configuration
        # `out_init` for MISO is set in each @asm_pio decorator.
        # `sideset_init` is not used for SCK control as slave PIO reacts to external SCK.
        self.sm = StateMachine(
            sm_id,
            pio_program,
            freq=50_000_000, # PIO clock frequency
            in_base=self.mosi_pin,
            out_base=self.miso_pin,
            # No sideset_base for SCK if using `wait(gpio, pin_num)`
        )
        
        self.sm.restart()
        self.sm.active(1)

        print(f"SPI Slave (Mode {self.spi_mode}) initialized and running on SM{sm_id}.")
        print(f"  MOSI: GP{mosi_pin_num}, MISO: GP{miso_pin_num}, SCK (for PIO wait): GP{sck_pin_num_for_pio_wait}, CS: GP{cs_pin_num}")
        if self.expected_data:
            self._log_data("  Expecting to receive", self.expected_data)
        else:
            print("  Configured to capture any incoming data.")
        if self.reply_data:
            self._log_data("  Will reply with", self.reply_data)
        else:
            print("  Will reply with default pattern (echo/incremented/0x00s).")

    def _log_data(self, message, data_list):
        if not data_list: # Handle empty lists gracefully
            print(f"{message}: (empty)")
            return
        print(f"{message}: {' '.join([f'0x{b:02x}' for b in data_list])}")

    def process(self):
        print("\nWaiting for CS activation (CS low)...")
        # Wait for CS to go low (active)
        cs_activation_timeout_ms = 30000 # 30 seconds timeout for CS to activate
        start_cs_activation_wait = time.ticks_ms()
        while self.cs_pin.value() == 1: # CS is active low
            if time.ticks_diff(time.ticks_ms(), start_cs_activation_wait) > cs_activation_timeout_ms:
                print("Timeout: Waited for CS activation, but CS did not go low.")
                return # Skip this transaction
            time.sleep_ms(5)
        print("CS activated (low).")

        received_buffer = []
        # Clear any stale data from RX FIFO from previous partial transactions or noise
        while self.sm.rx_fifo() > 0:
            self.sm.get()
        # Also clear any stale data from TX FIFO that might have been put but not sent
        # This is harder without disrupting the SM. Restarting SM on each CS cycle can do this.
        # Or, ensure PIO program is robust. Current PIO always tries to send from OSR.
        # If TX FIFO had old data, and master clocked it, it would have been sent.
        # `self.sm.restart()` was done in init, which is good. Let's assume TX is clean enough.

        # Reception phase
        if self.expected_data: # Expecting a specific number of bytes
            print(f"Expecting to receive {len(self.expected_data)} byte(s).")
            for i in range(len(self.expected_data)):
                rx_byte_timeout_ms = 5000 # Timeout per byte
                start_rx_byte_time = time.ticks_ms()
                while self.sm.rx_fifo() == 0:
                    if self.cs_pin.value() == 1:
                        print("Error: CS deactivated prematurely during reception of expected data.")
                        self._log_data("Partially received", received_buffer)
                        return 
                    if time.ticks_diff(time.ticks_ms(), start_rx_byte_time) > rx_byte_timeout_ms:
                        print(f"Error: Timeout waiting for byte {i+1} from master.")
                        self._log_data("Partially received", received_buffer)
                        return
                    time.sleep_us(100) # Polling delay

                data_byte = self.sm.get() & 0xFF
                received_buffer.append(data_byte)
            
            self._log_data("Received", received_buffer)
            if received_buffer == self.expected_data:
                print("Success: Received data matches expected data.")
            else:
                print("Error: Received data does not match expected data.")
                self._log_data("Expected", self.expected_data)
                # Behavior on mismatch: continue to send reply, or abort?
                # Current: continue and send configured/default reply.
        else: # Capture mode: receive until CS deactivates or timeout
            print("No specific expected data. Capturing incoming data until CS deactivates...")
            # Overall timeout for capture to prevent locking up if CS never deactivates
            capture_timeout_ms = 10000 # 10 seconds max capture time
            start_capture_time = time.ticks_ms()
            last_data_received_time = time.ticks_ms()
            inter_byte_timeout_ms = 1000 # If no data for 1s during capture, assume master done sending this segment

            while self.cs_pin.value() == 0: # While CS is active (low)
                if time.ticks_diff(time.ticks_ms(), start_capture_time) > capture_timeout_ms:
                    print("Error: Capture mode timed out (overall transaction).")
                    break
                if self.sm.rx_fifo() > 0:
                    data_byte = self.sm.get() & 0xFF
                    received_buffer.append(data_byte)
                    last_data_received_time = time.ticks_ms()
                elif time.ticks_diff(time.ticks_ms(), last_data_received_time) > inter_byte_timeout_ms:
                    print("Capture mode: No data received for a while, assuming master finished sending this segment.")
                    break 
                time.sleep_us(50) # Polling delay
            self._log_data("Captured", received_buffer)

        # Transmission phase
        data_to_send = []
        if self.reply_data:
            data_to_send = self.reply_data
            print("Using pre-configured data for reply.")
        elif received_buffer: # If nothing pre-configured, echo what was received (if anything)
            data_to_send = received_buffer
            print("Echoing received data as reply.")
        elif self.expected_data: # Expected data, nothing received (error), but still need a reply length
             data_to_send = [0x00] * len(self.expected_data) # Send zeros of expected length
             print("Nothing received to echo, expected data was set, sending zeros for reply length.")
        else: # No reply data, nothing received, not expecting data - send a single 0 byte?
            data_to_send = [0x00] # Default to one byte of 0x00
            print("No reply data, nothing received/expected, sending a single 0x00 byte.")

        if data_to_send:
            self._log_data("Preparing to send", data_to_send)
            # Ensure TX FIFO has space. PIO pulls 8 bits at a time.
            # sm.put(value, 8) is used. TX FIFO depth is 8 words (32 bytes).
            for byte_val in data_to_send:
                put_timeout_ms = 1000 # Timeout for sm.put()
                start_put_time = time.ticks_ms()
                # Wait for space in TX FIFO. tx_fifo() returns number of words.
                # If it's not 0, there's some space. If it's 8, it's full.
                # The PIO `pull_thresh=8` means it tries to keep OSR full.
                # TX FIFO is 8 words deep (not 4 as sometimes misremembered).
                # We put 8 bits. It goes into a 32-bit word buffer.
                while self.sm.tx_fifo() >= 8 : # Approximates "FIFO is full"
                    if self.cs_pin.value() == 1:
                        print("Error: CS deactivated while trying to load data into TX FIFO.")
                        return
                    if time.ticks_diff(time.ticks_ms(), start_put_time) > put_timeout_ms:
                        print("Error: Timeout waiting for space in TX FIFO.")
                        return
                    time.sleep_us(10)
                self.sm.put(byte_val, 8) # Put 8 bits into TX FIFO
            print(f"Data ({len(data_to_send)} byte(s)) loaded into TX FIFO.")
        else:
            print("No data to send.")

        # Wait for CS to be de-asserted by the master
        print("Waiting for CS deactivation (CS high)...")
        cs_deactivation_timeout_ms = 10000 # Max time for master to finish reading and raise CS
        start_cs_deactivation_wait = time.ticks_ms()
        while self.cs_pin.value() == 0: # While CS is still active low
            if time.ticks_diff(time.ticks_ms(), start_cs_deactivation_wait) > cs_deactivation_timeout_ms:
                print("Error: Timeout waiting for CS deactivation. Master may not have raised CS.")
                break 
            time.sleep_ms(1)

        if self.cs_pin.value() == 1:
            print("CS deactivated. Transaction finished.")
        else:
            print("CS still active after timeout. Transaction may be incomplete or master error.")
        
        # Check if TX FIFO was emptied by master clocking out the data
        # This is an approximation. If sm.tx_fifo() > 0, some data was not shifted out.
        # Note: PIO pulls 8 bits to OSR, then shifts bit by bit.
        # If TX FIFO is not empty, means OSR wasn't even fully reloaded for the last few bytes.
        tx_remaining_words = self.sm.tx_fifo()
        if tx_remaining_words > 0:
            # Each word is 4 bytes. `pull_thresh=8` means it pulls one byte at a time to OSR.
            # So if tx_fifo() > 0, it means at least one full byte (or more) didn't make it to OSR.
            print(f"Warning: {tx_remaining_words * 4} byte(s) worth of data may remain in TX FIFO (not clocked out by master).")

        print("-" * 30)

# Main script execution
if __name__ == "__main__":
    # --- Test Configurations ---
    # You can set these directly or add a simple input mechanism
    SELECTED_SPI_MODE = 0  # Change this to 0, 1, 2, or 3 to test different modes
    
    # Example data for testing (can be adjusted per mode if needed)
    # For simplicity, using the same data pattern for all modes initially.
    # Specific tests might require different expected/reply data based on how a master for that mode behaves.
    if SELECTED_SPI_MODE == 0:
        EXPECTED_FROM_MASTER = [0xA1, 0xB2, 0xC3] # Master sends this
        DATA_FOR_MASTER = [0x3C, 0x2B, 0x1A]      # Slave replies with this
    elif SELECTED_SPI_MODE == 1:
        EXPECTED_FROM_MASTER = [0x11, 0x22, 0x33]
        DATA_FOR_MASTER = [0xCC, 0xBB, 0xAA]
    elif SELECTED_SPI_MODE == 2:
        EXPECTED_FROM_MASTER = [0xDE, 0xAD]
        DATA_FOR_MASTER = [0xBE, 0xEF]
    elif SELECTED_SPI_MODE == 3:
        EXPECTED_FROM_MASTER = [0xCA, 0xFE]
        DATA_FOR_MASTER = [0xFE, 0xCA]
    else: # Default fallback
        EXPECTED_FROM_MASTER = [0x55, 0x55]
        DATA_FOR_MASTER = [0xAA, 0xAA]


    # --- End Test Configurations ---

    # Select the configuration to use for the test:
    current_expected = EXPECTED_FROM_MASTER
    current_reply = DATA_FOR_MASTER
    
    print(f"Attempting to start SPI Slave in Mode {SELECTED_SPI_MODE}.")
    print(f"  SCK for PIO wait is GP{PIN_SCK}.")
    print(f"  MOSI=GP{PIN_MOSI}, MISO=GP{PIN_MISO}, CS=GP{PIN_CS}")
    print(f"  Selected SCK_PIN_FOR_PIO: GP{SCK_PIN_FOR_PIO}")


    slave = PioSpiSlave(
        sm_id=0, 
        spi_mode=SELECTED_SPI_MODE,
        mosi_pin_num=PIN_MOSI,
        miso_pin_num=PIN_MISO,
        sck_pin_num_for_pio_wait=SCK_PIN_FOR_PIO,
        cs_pin_num=PIN_CS,
        expected_data_to_receive=current_expected,
        data_to_send_on_receive=current_reply
    )

    try:
        while True:
            slave.process() # Each call to process handles one full CS cycle
    except KeyboardInterrupt:
        print("\nSlave stopped by user (KeyboardInterrupt).")
    finally:
        if 'slave' in locals() and hasattr(slave, 'sm'):
            slave.sm.active(0) # Deactivate the state machine
            print("PIO State Machine deactivated.")
        print("Script finished.")

# Commented out old code:
# # 初始化状态机
# sm = StateMachine(
#     0,                  # 使用状态机0
#     spi_slave,
#     freq=10_000_000,     # 状态机时钟频率
#     in_pin=Pin(3),      # MOSI引脚（GP0）
#     out_pin=Pin(4),     # MISO引脚（GP1）
#     sideset_pins=Pin(2) # SCK引脚（GP2）
# )
# # 启动状态机
# sm.active(1)
# pin_cs = Pin(5, Pin.IN)
# print("SPI slave started...")
# # 等待片选信号拉低，表示主设备开始通信
# while not pin_cs.value():
#     pass  # 等待片选信号拉低
# print("CS activated, ready to receive data...")
# # 数据接收示例
# while True:
#     if sm.rx_fifo() > 0:
#         received_data = sm.get()
#         print(f"Received: 0x{received_data:02x}")
#         # 发送响应数据（示例：接收值+1）
#         sm.put((received_data + 1) & 0xFF)

# Remove other commented out code as well
# # import rp2
# # from machine import Pin
# # @rp2.asm_pio(out_shiftdir=0, autopull=True, pull_thresh=8, autopush=True, push_thresh=8, sideset_init=(rp2.PIO.OUT_LOW, rp2.PIO.OUT_HIGH), out_init=rp2.PIO.OUT_LOW)
# # def spi_cpha0():
# # ... (rest of spi_cpha0)
# # class PIOSPI:
# # ... (rest of PIOSPI class)
