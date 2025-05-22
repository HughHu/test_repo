from machine import Pin, SPI
import time


# 初始化 SPI 从模式（以 SPI0 为例）
spi_slave = SPI(
    0,                          # 使用 SPI0 控制器
    baudrate=1000000,           # 波特率需与主设备匹配
    polarity=0,                 # 时钟极性（CPOL）
    phase=0,                    # 时钟相位（CPHA）
    bits=8,                     # 数据位宽
    firstbit=SPI.MSB,           # 高位优先
    sck=Pin(2),                 # SCK 引脚
    mosi=Pin(3),                # MOSI 引脚
    miso=Pin(4),                # MISO 引脚
    cs=Pin(5),               # CS 引脚
    # slave=True                  # 关键：启用从模式
)
# pin_cs = Pin(5, Pin.OUT)

print('SPI slave started...')
# 向主设备发送响应数据
response = bytearray([0x01, 0x02, 0x03])
spi_slave.write(response)
cnt = 0

print('SPI slave receive...')
while True:
    # 等待主设备发送数据
    received_data = spi_slave.read(4)  # 读取 4 字节数据
    # 假设设备会返回一些数据
    # response = spi.read(10)
    if received_data:
        print(f'Received data: {received_data}')
    
    print(f'time: {cnt}')
    cnt += 1
    time.sleep(1)