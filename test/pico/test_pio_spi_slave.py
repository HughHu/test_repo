import rp2
from machine import Pin
import time
import machine
import array

@rp2.asm_pio(
    in_shiftdir=rp2.PIO.SHIFT_LEFT,
    out_shiftdir=rp2.PIO.SHIFT_LEFT,
    autopull=True,
    pull_thresh=8,
    autopush=True,
    push_thresh=8,
    out_init=rp2.PIO.OUT_LOW)
def spi_slave_cpol0_cpha0():
    wrap_target()
    # wait(1, irq, 0)             # Wait for IRQ 0 - CS asserted, clear irq automatically
    set(x, 7)                   # 8 bits per byte (x counts down)
    label("full_duplex")
    out(pins, 1)
    wait(1, irq, 2)             # Wait for IRQ 2 - SCK rising edge, clear irq automatically
    in_(pins, 1)                # Sample MOSI
    wait(1, irq,3)              # Wait for IRQ 3 - SCK falling edge, clear irq automatically
    jmp(x_dec, "full_duplex")   # Loop for 8 bits
    wrap()


@rp2.asm_pio(
    in_shiftdir=rp2.PIO.SHIFT_LEFT,
    out_shiftdir=rp2.PIO.SHIFT_LEFT,
    autopull=True,
    pull_thresh=8,
    autopush=True,
    push_thresh=8,
    out_init=rp2.PIO.OUT_LOW)
def spi_slave_cpol0_cpha1():
    wrap_target()
    # wait(1, irq, 0)             # Wait for IRQ 0 - CS asserted, clear irq automatically
    set(x, 7)                   # 8 bits per byte (x counts down)
    label("full_duplex")
    wait(1, irq, 2)             # Wait for IRQ 2 - SCK rising edge, clear irq automatically
    out(pins, 1)
    wait(1, irq,3)              # Wait for IRQ 3 - SCK falling edge, clear irq automatically
    in_(pins, 1)                # Sample MOSI
    jmp(x_dec, "full_duplex")   # Loop for 8 bits
    wrap()


@rp2.asm_pio(
    in_shiftdir=rp2.PIO.SHIFT_LEFT,
    out_shiftdir=rp2.PIO.SHIFT_LEFT,
    autopull=True,
    pull_thresh=8,
    autopush=True,
    push_thresh=8,
    out_init=rp2.PIO.OUT_LOW)
def spi_slave_cpol1_cpha0():
    wrap_target()
    # wait(1, irq, 0)             # Wait for IRQ 0 - CS asserted, clear irq automatically
    set(x, 7)                   # 8 bits per byte (x counts down)
    label("full_duplex")
    out(pins, 1)
    wait(1, irq, 2)             # Wait for IRQ 2 - SCK falling edge, clear irq automatically
    in_(pins, 1)                # Sample MOSI
    wait(1, irq,3)              # Wait for IRQ 3 - SCK rising edge, clear irq automatically
    jmp(x_dec, "full_duplex")   # Loop for 8 bits
    wrap()


@rp2.asm_pio(
    in_shiftdir=rp2.PIO.SHIFT_LEFT,
    out_shiftdir=rp2.PIO.SHIFT_LEFT,
    autopull=True,
    pull_thresh=8,
    autopush=True,
    push_thresh=8,
    out_init=rp2.PIO.OUT_LOW)
def spi_slave_cpol1_cpha1():
    wrap_target()
    # wait(1, irq, 0)             # Wait for IRQ 0 - CS asserted, clear irq automatically
    set(x, 7)                   # 8 bits per byte (x counts down)
    label("full_duplex")
    wait(1, irq, 2)             # Wait for IRQ 2 - SCK falling edge, clear irq automatically
    out(pins, 1)
    wait(1, irq,3)              # Wait for IRQ 3 - SCK rising edge, clear irq automatically
    in_(pins, 1)                # Sample MOSI
    jmp(x_dec, "full_duplex")   # Loop for 8 bits
    wrap()


@rp2.asm_pio()
def spi_slave_sck_cpol0():
    wrap_target()
    wait(1, pin, 0)     # Wait for SCK to go high
    irq(2)
    wait(0, pin, 0)     # Wait for SCK to go low
    irq(3)
    wrap()

@rp2.asm_pio()
def spi_slave_sck_cpol1():
    wrap_target()
    wait(0, pin, 0)     # Wait for SCK to go low
    irq(2)
    wait(1, pin, 0)     # Wait for SCK to go high
    irq(3)
    wrap()


# @rp2.asm_pio()
# def spi_slave_cs():

#     wrap_target()
#     wait(0, pin, 0)     # Wait for CS to go low
#     irq(0)
#     wait(1, pin, 0)     # Wait for CS to go high
#     irq(1)
#     wrap()
class PIOSPI:

    def __init__(self, sm_id, pin_mosi, pin_miso, pin_sck, pin_cs, cpha=False, cpol=False, freq=1000000):
        if cpol and cpha:
            spi_slave_func = spi_slave_cpol1_cpha1
            spi_slave_sck = spi_slave_sck_cpol1
        elif cpol and not cpha:
            spi_slave_func = spi_slave_cpol1_cpha0
            spi_slave_sck = spi_slave_sck_cpol1
        elif not cpol and cpha:
            spi_slave_func = spi_slave_cpol0_cpha1
            spi_slave_sck = spi_slave_sck_cpol0
        else:
            spi_slave_func = spi_slave_cpol0_cpha0
            spi_slave_sck = spi_slave_sck_cpol0

        self.tx_buffer = bytearray(1024)  # TX buffer for outgoing data
        self.rx_buffer = bytearray(1024)  # RX buffer for incoming data
        self.tx_len = 0
        self.tx_pos = 0
        self.rx_len = 0
        # self._sm.irq(self.handler)


        rp2.PIO(0).remove_program()
        rp2.PIO(1).remove_program()
        # rp2.PIO(2).remove_program()
        self._sm = rp2.StateMachine(0, spi_slave_func, freq=freq*10, out_base=Pin(pin_miso, Pin.OUT), in_base=Pin(pin_mosi, Pin.IN))
        self.sm_sck = rp2.StateMachine(1, spi_slave_sck, freq=freq*10, in_base=Pin(pin_sck, Pin.IN))
        # self.sm_cs = rp2.StateMachine(2, spi_slave_cs, freq=freq*10, in_base=Pin(pin_cs, Pin.IN))
        

    def read_buffer_clear(self):
        while self._sm.rx_fifo() > 0:
            # Read data from RX FIFO
            b = self._sm.get() & 0xff
            # print(f"RX FIFO popped = {b}")
            self.rx_len = 0

    def write_buffer_clear(self):
        self.tx_pos = 0
        self.tx_len = 0

        while self._sm.tx_fifo() > 0:
            # print(f"TX FIFO = {self._sm.tx_fifo()}")
            self._sm.exec("out(osr, 8)")
            self._sm.exec("pull()")
            self._sm.exec("nop()")

        self._sm.exec("out(osr, 8)")
        self._sm.exec("nop()")



    def write_nonblocking(self, wdata):
        print("write_nonblocking")
        # Add data to the TX buffer
        # self.tx_buffer.extend(wdata)
        # self.tx_buffer = wdata.copy()
        self.tx_buffer[:len(wdata)] = wdata  # Copy data to the tx_buffer
        self.tx_len = len(wdata)
        while self._sm.tx_fifo() < 4:
            b = self.tx_buffer[self.tx_pos]
            self.tx_pos += 1
            self._sm.put(b << 24)



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

    # PIN_MOSI = 16  # VenusA - 19
    # PIN_SCK = 17   # VenusA - 16
    # PIN_CS = 18    # VenusA - 17
    # PIN_MISO = 19  # VenusA - 18

    pincs = Pin(PIN_CS, Pin.IN)
    pinmosi = Pin(PIN_MOSI, Pin.IN)
    pinsck = Pin(PIN_SCK, Pin.IN)
    pinmiso = Pin(PIN_MISO, Pin.OUT)

    pio_spi = PIOSPI(1, PIN_MOSI, PIN_MISO, PIN_SCK, PIN_CS, cpol=cpol, cpha=cpha)

    data_send = bytearray(1024)

    pio_spi._sm.active(0)
    pio_spi.sm_sck.active(0)  # Stop the state machine to clear the TX FIFO 
    pio_spi.write_buffer_clear()
    pio_spi.read_buffer_clear()
    time.sleep(0.001)
    pio_spi.write_nonblocking(data_send)

    pio_spi._sm.active(1)
    pio_spi.sm_sck.active(1) 

    while True:
        print("wait for CS activated")
        # wait for CS to be low
        while pincs.value():
            pass


        while pincs.value() == 0:
            if pio_spi._sm.tx_fifo() < 4:
                # print(f"TX FIFO = {pio_spi._sm.tx_fifo()}")
                # b = pio_spi.tx_buffer.pop(0)
                # print(f"TX FIFO popped = {b}")

                b = pio_spi.tx_buffer[pio_spi.tx_pos]
                pio_spi.tx_pos += 1
                pio_spi._sm.put(b << 24)

            if pio_spi._sm.rx_fifo() > 0:
                # print(f"RX FIFO = {pio_spi._sm.rx_fifo()}")
                # Read data from RX FIFO and store in rx_buffer
                b = pio_spi._sm.get() & 0xff
                # print(f"RX FIFO popped = {b}")
                # pio_spi.rx_buffer.append(b)

                if pio_spi.rx_len < len(pio_spi.rx_buffer):
                    pio_spi.rx_buffer[pio_spi.rx_len] = b
                    pio_spi.rx_len += 1

            pass

        print("CS deactivated")

        while pio_spi._sm.rx_fifo() > 0:
            # Read data from RX FIFO and store in rx_buffer
            b = pio_spi._sm.get() & 0xff
            # print(f"RX FIFO popped = {b}")
            # pio_spi.rx_buffer.append(b)

            if pio_spi.rx_len < len(pio_spi.rx_buffer):
                pio_spi.rx_buffer[pio_spi.rx_len] = b
                pio_spi.rx_len += 1

        data_recv = pio_spi.rx_buffer[:pio_spi.rx_len]  # Copy only the received data

        print(f"Received data: {[hex(b) for b in data_recv]}")
        

        data_send = data_recv[:pio_spi.rx_len]  # Copy only the received data

        print(f"Send data: {[hex(b) for b in data_send]}")

        pio_spi._sm.active(0)
        pio_spi.sm_sck.active(0) 
        pio_spi.write_buffer_clear()
        pio_spi.read_buffer_clear()

        pio_spi = PIOSPI(1, PIN_MOSI, PIN_MISO, PIN_SCK, PIN_CS, cpol=cpol, cpha=cpha)

        pio_spi.write_nonblocking(data_send)

        pio_spi._sm.active(1)
        pio_spi.sm_sck.active(1)


