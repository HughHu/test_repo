import rp2
from machine import Pin
import time
import machine
import array

@rp2.asm_pio(in_shiftdir=rp2.PIO.SHIFT_LEFT, out_shiftdir=rp2.PIO.SHIFT_LEFT, autopull=True, pull_thresh=8, autopush=True, push_thresh=8, out_init=rp2.PIO.OUT_LOW, sideset_init=rp2.PIO.OUT_LOW)
def spi_master_cpol0_cpha0():
    wrap_target()
    set(x, 7)  
    pull(ifempty)            .side(0x0)   [1]
    label("bitloop")
    out(pins, 1)             .side(0x0)   [3]
    in_(pins, 1)             .side(0x1)   [4]
    jmp(x_dec, "bitloop")    .side(0x0)     
    wrap()

@rp2.asm_pio(in_shiftdir=rp2.PIO.SHIFT_LEFT, out_shiftdir=rp2.PIO.SHIFT_LEFT, autopull=True, pull_thresh=8, autopush=True, push_thresh=8, out_init=rp2.PIO.OUT_LOW, sideset_init=rp2.PIO.OUT_LOW)
def spi_master_cpol0_cpha1():
    wrap_target()
    set(x, 7)  
    pull(ifempty)            .side(0x0)   [1]
    label("bitloop")
    out(pins, 1)             .side(0x1)   [4]
    in_(pins, 1)             .side(0x0)   [3]
    jmp(x_dec, "bitloop")    .side(0x0)
    wrap()

@rp2.asm_pio(in_shiftdir=rp2.PIO.SHIFT_LEFT, out_shiftdir=rp2.PIO.SHIFT_LEFT, autopull=True, pull_thresh=8, autopush=True, push_thresh=8, out_init=rp2.PIO.OUT_LOW, sideset_init=rp2.PIO.OUT_HIGH)
def spi_master_cpol1_cpha0():
    wrap_target()
    set(x, 7)  
    pull(ifempty)            .side(0x1)   [1]
    label("bitloop")
    out(pins, 1)             .side(0x1)   [4]
    in_(pins, 1)             .side(0x0)   [3]
    jmp(x_dec, "bitloop")    .side(0x0)
    wrap()

@rp2.asm_pio(in_shiftdir=rp2.PIO.SHIFT_LEFT, out_shiftdir=rp2.PIO.SHIFT_LEFT, autopull=True, pull_thresh=8, autopush=True, push_thresh=8, out_init=rp2.PIO.OUT_LOW, sideset_init=rp2.PIO.OUT_HIGH)
def spi_master_cpol1_cpha1():
    wrap_target()
    set(x, 7)  
    pull(ifempty)            .side(0x1)   [1]
    label("bitloop")
    out(pins, 1)             .side(0x0)   [4]
    in_(pins, 1)             .side(0x1)   [3]
    jmp(x_dec, "bitloop")    .side(0x1)
    wrap()

      



class PIOSPI:

    def __init__(self, sm_id, pin_mosi, pin_miso, pin_sck, cpol=False, cpha=False, freq=100000):
        if cpol and cpha:
            spi_master_func = spi_master_cpol1_cpha1
        elif cpol and not cpha:
            spi_master_func = spi_master_cpol1_cpha0
        elif not cpol and cpha:
            spi_master_func = spi_master_cpol0_cpha1
        else:
            spi_master_func = spi_master_cpol0_cpha0


        self._sm = rp2.StateMachine(sm_id, spi_master_func, freq=10 * freq, sideset_base=Pin(pin_sck), out_base=Pin(pin_mosi, Pin.OUT), in_base=Pin(pin_miso, Pin.IN))
        # self._sm.irq(lambda pio: print(pio.irq().flags()))
        self.tx_buffer = bytearray(100)  # TX buffer for outgoing data
        self.rx_buffer = bytearray(100)  # RX buffer for incoming data
        self.tx_len = 0
        self.tx_pos = 0
        self.rx_len = 0
        # self._sm.irq(self.handler)
        self._sm.active(1)


    def read_buffer_clear(self):
        while self._sm.rx_fifo() > 0:
            b = self._sm.get() & 0xff
            self.rx_len = 0


    def write_buffer_clear(self):
        self.tx_pos = 0
        self.tx_len = 0




    # def write_nonblocking(self, wdata):
    #     print("write_nonblocking")
    #     # Add data to the TX buffer
    #     # self.tx_buffer.extend(wdata)
    #     # self.tx_buffer = wdata.copy()
    #     self.tx_buffer[:len(wdata)] = wdata  # Copy data to the tx_buffer
    #     self.tx_len = len(wdata)
    #     while self._sm.tx_fifo() < 4:
    #         b = self.tx_buffer[self.tx_pos]
    #         self.tx_pos += 1
    #         self._sm.put(b << 24)


    # def write_blocking(self, wdata):
    #     for b in wdata:
    #         self._sm.put(b << 24)

    # def read_blocking(self, n):
    #     data = []
    #     for i in range(n):
    #         data.append(self._sm.get() & 0xff)
    #     return data

    def write_read_blocking(self, wdata):
        rdata = []
        for b in wdata:
            self._sm.put(b << 24)
            rdata.append(self._sm.get() & 0xff)
        return rdata


if __name__ == "__main__":

# spi1
    # PIN_MOSI = 15
    # PIN_SCK = 17
    # PIN_CS = 19
    # PIN_MISO = 14

# spi0
    PIN_MOSI = 19
    PIN_SCK = 16
    PIN_CS = 17
    PIN_MISO = 18

    cpol = False
    cpha = False
    freq = 100000

    # PIN_MOSI = 16  # VenusA - 19
    # PIN_SCK = 17   # VenusA - 16
    # PIN_CS = 18    # VenusA - 17
    # PIN_MISO = 19  # VenusA - 18

    pinmosi = Pin(PIN_MOSI, Pin.OUT)
    pinsck = Pin(PIN_SCK, Pin.OUT)
    pincs = Pin(PIN_CS, Pin.OUT)
    pinmiso = Pin(PIN_MISO, Pin.IN)

    pio_spi = PIOSPI(0, PIN_MOSI, PIN_MISO, PIN_SCK, cpol=cpol, cpha=cpha, freq=freq)

    data_send = bytearray([0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08])
    # data_send = [1]
    pio_spi.write_buffer_clear()
    pio_spi.read_buffer_clear()
    time.sleep(0.01)
    # pio_spi.write_nonblocking(data_send)

    # time_stamp = []
    time_stamp = array.array('I', [0] * 100)  # Preallocate an array of 100 unsigned integers
    time_stamp_pos = 0
    while True:

        time_stamp_pos = 0
        print("CS activated")
        pincs.value(0)
        time.sleep(0.001)
        data_recv = pio_spi.write_read_blocking(data_send)
        time.sleep(0.001)
        pincs.value(1)
        print("CS deactivated")

          



        # data_send = data_recv.copy()
        data_send = data_recv

        print(f"Send data: {[hex(b) for b in data_send]}")


        pio_spi.write_buffer_clear()
        pio_spi.read_buffer_clear()
        time.sleep(0.001)
        
        pio_spi._sm.active(0)  # Stop the state machine to clear the TX FIFO
        time.sleep(0.001) 
        pio_spi = PIOSPI(0, PIN_MOSI, PIN_MISO, PIN_SCK, cpol=cpol, cpha=cpha, freq=freq)
        time.sleep(0.001) 


        time.sleep(0.1) 

        print("CS activated")
        pincs.value(0)
        time.sleep(0.001)
        data_recv = pio_spi.write_read_blocking(data_send)
        time.sleep(0.001)
        pincs.value(1)
        print("CS deactivated")

        while True:
            pass



