import rp2
from machine import Pin
import time
import machine

@rp2.asm_pio(in_shiftdir=rp2.PIO.SHIFT_LEFT, out_shiftdir=0, autopull=True, pull_thresh=8, autopush=True, push_thresh=8, out_init=rp2.PIO.OUT_LOW)
def spi_slave():
    wrap_target()
    # pull(ifempty) 
    # get tx fifo ready for the first byte
    # pull()
    irq(rel(0))
     
    wait(0, pin, 2)         # Wait for CS to go low (optional, if you use CS)
    set(x, 7)               # 8 bits per byte (x counts down)
    
    # jmp(not_osre, "full_duplex")    
    
    
    # label("rx_only")    
    # wait(1, pin, 1)     # Wait for SCK rising edge 
    # in_(pins, 1)            # Sample MOSI
    # wait(0, pin, 1)     # Wait for SCK falling edge    
    # jmp(x_dec, "rx_only")   # Loop for 8 bits
    # wrap()

    label("full_duplex")
    out(pins, 1) 
    wait(1, pin, 1)     # Wait for SCK rising edge 
    in_(pins, 1)            # Sample MOSI
    wait(0, pin, 1)     # Wait for SCK falling edge
    jmp(x_dec, "full_duplex")   # Loop for 8 bits
    # irq(rel(0))
    # push()
    wrap()

# @rp2.asm_pio(in_shiftdir=rp2.PIO.SHIFT_LEFT, autopull=False, pull_thresh=8, autopush=True, push_thresh=8, out_init=rp2.PIO.OUT_LOW)
# def spi_slave_receive_only():
#     wrap_target()
#     # get tx fifo ready for the first byte
#     # irq(rel(0))
#     # wait(0, pin, 2)         # Wait for CS to go low (optional, if you use CS)
#     set(x, 7)               # 8 bits per byte (x counts down)
#     label("bitloop")
#     # out(pins, 1)  
#     wait(1, pin, 1)     # Wait for SCK rising edge 
#     in_(pins, 1)            # Sample MOSI
#     wait(0, pin, 1)     # Wait for SCK falling edge
#     # out(pins, 1)
#     jmp(x_dec, "bitloop")   # Loop for 8 bits
#     # print("Received 1byte") # Print received byte (optional)
#     # irq(rel(0))
#     wrap()



class PIOSPI:

    def __init__(self, sm_id, pin_mosi, pin_miso, pin_sck, cpha=False, cpol=False, freq=1000000):
        # assert(not(cpol or cpha))
        self._sm = rp2.StateMachine(sm_id, spi_slave, freq=10*freq, out_base=Pin(pin_miso, Pin.OUT), in_base=Pin(pin_mosi, Pin.IN))
        # self._sm.irq(lambda pio: print(pio.irq().flags()))
        self.tx_buffer = []  # TX buffer for outgoing data
        self.rx_buffer = []  # RX buffer for incoming data
        self._sm.irq(self.handler)
        self._sm.active(1)

    def handler(self, pio):
        # Fill TX FIFO from tx_buffer if not full
        # while self._sm.tx_fifo() < 4 and self.tx_buffer:
        print("interrupt handler")
        if self._sm.tx_fifo() < 4:
            
            while len(self.tx_buffer) > 0 and self._sm.tx_fifo() < 4:
                b = self.tx_buffer.pop(0)
                # print(f"TX FIFO popped = {b}")
                self._sm.put(b << 24)

        while self._sm.rx_fifo() > 0:
            # Read data from RX FIFO and store in rx_buffer
            b = self._sm.get() & 0xff
            # print(f"RX FIFO popped = {b}")
            self.rx_buffer.append(b)
        

    def read_buffer_clear(self):
        while self._sm.rx_fifo() > 0:
            # Read data from RX FIFO and store in rx_buffer
            b = self._sm.get() & 0xff
            # print(f"RX FIFO popped = {b}")
            self.rx_buffer.append(b)
        
        self.rx_buffer.clear()  # Clear the RX buffer


    def write_buffer_clear(self):
        print("write_buffer_clear")
        # Clear the TX buffer
        self.tx_buffer.clear()
        print(f"tx_buffer length = {len(self.tx_buffer)}")
        print(f"TX FIFO = {self._sm.tx_fifo()}")

        self._sm.active(0)
        while self._sm.tx_fifo() > 0:
            print(f"TX FIFO = {self._sm.tx_fifo()}")
            self._sm.exec("out(osr, 8)")
            self._sm.exec("pull()")
            self._sm.exec("nop()")
            self._sm.exec("nop()")
            self._sm.exec("nop()")
            self._sm.exec("nop()")
            self._sm.exec("nop()")
            self._sm.exec("nop()")
            self._sm.exec("nop()")
            self._sm.exec("nop()")
            self._sm.exec("nop()")
            self._sm.exec("nop()")
            self._sm.exec("nop()")
            self._sm.exec("nop()")
            self._sm.exec("nop()")
            self._sm.exec("nop()")
            self._sm.exec("nop()")
            self._sm.exec("nop()")
            self._sm.exec("nop()")
            self._sm.exec("nop()")
            self._sm.exec("nop()")
            self._sm.exec("nop()")
            self._sm.exec("nop()")
            self._sm.exec("nop()")
            self._sm.exec("nop()")
            self._sm.exec("nop()")
            self._sm.exec("nop()")
            # self._sm.exec("out(osr, 8)")
            # self._sm.exec("mov(x, osr)")
            # self._sm.exec("mov(osr, x)")

        self._sm.exec("out(osr, 8)")
        self._sm.exec("nop()")
        self._sm.exec("nop()")
        self._sm.exec("nop()")
        self._sm.exec("nop()")
        self._sm.exec("nop()")
        self._sm.exec("nop()")
        self._sm.exec("nop()")
        self._sm.exec("nop()")
        self._sm.exec("nop()")
        self._sm.exec("nop()")
        self._sm.exec("nop()")
        self._sm.exec("nop()")
        self._sm.exec("nop()")
        self._sm.exec("nop()")
        self._sm.exec("nop()")
        self._sm.exec("nop()")
        self._sm.exec("nop()")
        self._sm.exec("nop()")
        self._sm.exec("nop()")
        self._sm.exec("nop()")        
        self._sm.exec("nop()")
        self._sm.exec("nop()")
        self._sm.exec("nop()")
        self._sm.exec("nop()")
        self._sm.exec("nop()")
        self._sm.exec("nop()")
        self._sm.exec("nop()")
        self._sm.exec("nop()")
        self._sm.exec("nop()")
        self._sm.exec("nop()")
        # self._sm.exec("pull(ifempty)")
        # self._sm.exec("mov(osr, x)")


        # self._sm.exec("push()")

        self._sm.active(1)

        # self._sm.active(0)
        # while self._sm.tx_fifo() > 0:
        #     print(f"TX FIFO = {self._sm.tx_fifo()}")
            
        #     self._sm.exec("out(pins, 8)") 

        # self._sm.active(1)

        # self._sm.clear_tx_fifo()  # Clear the TX FIFO
        # self._sm.reset()  # Reset the state machine to clear the TX FIFO


    def write_nonblocking(self, wdata):
        print("write_nonblocking")
        # Add data to the TX buffer
        self.tx_buffer.extend(wdata)
        while self._sm.tx_fifo() < 4 and len(self.tx_buffer) > 0:
            b = self.tx_buffer.pop(0)
            self._sm.put(b << 24)


    def write_blocking(self, wdata):
        for b in wdata:
            self._sm.put(b << 24)

    def read_blocking(self, n):
        data = []
        for i in range(n):
            data.append(self._sm.get() & 0xff)
        return data

    def write_read_blocking(self, wdata):
        rdata = []
        for b in wdata:
            self._sm.put(b << 24)
            rdata.append(self._sm.get() & 0xff)
        return rdata
    

PIN_MOSI = 2
PIN_SCK = 3
PIN_CS = 4
PIN_MISO = 5

# PIN_MOSI = 16  # VenusA - 19
# PIN_SCK = 17   # VenusA - 16
# PIN_CS = 18    # VenusA - 17
# PIN_MISO = 19  # VenusA - 18

pincs = Pin(PIN_CS, Pin.IN)
pinmosi = Pin(PIN_MOSI, Pin.IN)
pinsck = Pin(PIN_SCK, Pin.IN)

pio_spi = PIOSPI(1, PIN_MOSI, PIN_MISO, PIN_SCK)

data_send = [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15]
# data_send = [1]
pio_spi.write_buffer_clear()
pio_spi.read_buffer_clear()
time.sleep(0.01)
pio_spi.write_nonblocking(data_send)


while True:
    # wait for CS to be low
    while pincs.value():
        pass

    
    print("CS activated")

    # data_recv = []
    
    # # receive data until CS goes high
    # while pincs.value() == 0:
    #     if pio_spi._sm.rx_fifo() > 0:
    #         data_recv.append(pio_spi._sm.get() & 0xff)

    # # fetch remaining data in RX FIFO
    # while pio_spi._sm.rx_fifo() > 0:
    #     data_recv.append(pio_spi._sm.get() & 0xff)

    # reset the state machine
    # pio_spi._sm.active(0)
    # time.sleep(0.01)
    # pio_spi._sm.active(1)

    while pincs.value() == 0:
        pass

    data_recv = pio_spi.rx_buffer.copy()

    print(f"Received data: {[hex(b) for b in data_recv]}")
    print("CS deactivated")


    # add i to each received byte to compose the response
    # data_send = [(b + 1) & 0xff for b in data_recv]
    # for i in range(len(data_send)):
    #     data_send[i] = (data_send[i] + i) & 0xff

    data_send = data_recv.copy()

    print(f"Send data: {[hex(b) for b in data_send]}")

    pio_spi.write_buffer_clear()
    pio_spi.read_buffer_clear()
    time.sleep(0.01)
    

    # while pio_spi._sm.tx_fifo() > 0:
    #     print(f"DBG: TX FIFO = {pio_spi._sm.tx_fifo()}")
    #     pio_spi._sm.exec("out(pins, 8)")


        
    # pio_spi._sm.active(0)
    # time.sleep(0.001)
    # pio_spi._sm.active(1)
    # pio_spi.

    # pio_spi._sm.reset(machine.HARD_RESET)  # 等同于按下复位按钮

    pio_spi.write_nonblocking(data_send)





