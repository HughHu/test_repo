
# from rp2 import PIO, StateMachine, asm_pio
# from machine import Pin

# @asm_pio(
#     autopush=True,  # 自动将接收数据推送到RX FIFO
#     push_thresh=8,  # 每8位组成一个字节
#     in_shiftdir=rp2.PIO.SHIFT_LEFT,  # 数据左移（MSB first）
#     out_shiftdir=rp2.PIO.SHIFT_LEFT
# )

# def spi_slave():
#     # 引脚绑定：in_pins(MOSI), out_pins(MISO), sideset_pins(SCK, CS)
#     wrap_target()
#     # 等待CS拉低（片选激活）
#     wait(0, pin, 0)
    
#     # 接收/发送循环
#     label("loop")
#     # 在SCK上升沿读取MOSI（SPI Mode 0）
#     in_(pins, 1)        .side(0)    [1]  # SCK低电平时准备
#     wait(1, pin, 1)     .side(1)        # 等待SCK上升沿
#     # 在SCK下降沿输出MISO
#     out(pins, 1)        .side(0)    [1]
#     jmp("loop")         .side(0)
    
#     wrap()


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



import rp2
from machine import Pin
import time


@rp2.asm_pio(out_shiftdir=0, autopull=False, pull_thresh=8, autopush=True, push_thresh=8, sideset_init=rp2.PIO.OUT_LOW, out_init=rp2.PIO.OUT_LOW)
def spi_cpha0():
    # Note X must be preinitialised by setup code before first byte, we reload after sending each byte
    # Would normally do this via exec() but in this case it's in the instruction memory and is only run once
    set(x, 6)
    # Actual program body follows
    wrap_target()
    pull(ifempty)            .side(0x2)   [1]
    label("bitloop")
    out(pins, 1)             .side(0x0)   [1]
    in_(pins, 1)             .side(0x1)
    jmp(x_dec, "bitloop")    .side(0x1)

    out(pins, 1)             .side(0x0)
    set(x, 6)                .side(0x0) # Note this could be replaced with mov x, y for programmable frame size
    in_(pins, 1)             .side(0x1)
    jmp(not_osre, "bitloop") .side(0x1) # Fallthru if TXF empties

    nop()                    .side(0x0)   [1] # CSn back porch
    wrap()


class PIOSPI:

    def __init__(self, sm_id, pin_mosi, pin_miso, pin_sck, cpha=False, cpol=False, freq=1000000):
        assert(not(cpol or cpha))
        self._sm = rp2.StateMachine(sm_id, spi_cpha0, freq=4*freq, sideset_base=Pin(pin_sck), out_base=Pin(pin_mosi), in_base=Pin(pin_miso))
        self._sm.active(1)

    # Note this code will die spectacularly cause we're not draining the RX FIFO
    def write_blocking(self, wdata):
        for b in wdata:
            self._sm.put(b << 24)

    def read_blocking(n):
        data = []
        for i in range(n):
            data.append(self._sm.get() & 0xff)
        return data

    def write_read_blocking(wdata):
        rdata = []
        for b in wdata:
            self._sm.put(b << 24)
            rdata.append(self._sm.get() & 0xff)
        return rdata
    

PIN_CS = 1
PIN_SCK = 2
PIN_MOSI = 3
PIN_MISO = 0

pin_cs = Pin(PIN_CS, Pin.OUT)
pin_cs.value(1)




pio_spi = PIOSPI(0, PIN_MOSI, PIN_MISO, PIN_SCK)


pin_cs.value(0)

# pio_spi.write_blocking(b'1234567890123456789012345678901234567890123456789012345678901234')
pio_spi.write_blocking(b'12')
time.sleep(0.001)
pin_cs.value(1)
cnt = 0
while True:
    # 假设设备会返回一些数据

    print(f'time: {cnt}')
    cnt += 1
    time.sleep(1)   

