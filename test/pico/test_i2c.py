from machine import I2C, Pin
import time

i2c = I2C(0, scl=Pin(21), sda=Pin(20), freq=100000)

# 发送一些数据
i2c.writeto(0x3C, b'Hello I2C')

while True:
    data = i2c.readfrom(0x3C, 10)
    print('Received:', data)
    time.sleep(1)