from machine import Pin, SPI
import time

PIN_CS = 1
PIN_SCK = 2
PIN_MOSI = 3
PIN_MISO = 0

pin_cs = Pin(PIN_CS, Pin.OUT)
pin_cs.value(1)
spi = SPI(0, baudrate=3000000, polarity=1, phase=1, sck=Pin(PIN_SCK), mosi=Pin(PIN_MOSI), miso=Pin(PIN_MISO))

pin_cs.value(0)
# 发送数据
spi.write(b'1234567890123456789012345678901234567890123456789012345678901234')
# spi.write(b'1234')
# time.sleep(0.1)
pin_cs.value(1)

cnt = 0
while True:
    # 假设设备会返回一些数据
    # response = spi.read(10)
    print(f'time: {cnt}')
    cnt += 1
    time.sleep(1)