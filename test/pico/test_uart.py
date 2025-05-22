from machine import UART, Pin

uart = UART(0, baudrate=115200, tx=Pin(0), rx=Pin(1))
# uart = UART(1, baudrate=115200, tx=Pin(8), rx=Pin(9))

print("start UART")
# wait for the uart ready on device
while True:
    if uart.any():
        data = uart.read()
        print(data)
        if "Please input" in data:
            break

uart.write('Agree')

reic_data = b''
while True:
    if uart.any():
        reic_data += uart.read()
        if "Uart receive success" in reic_data:
            break
        if "Agree" in reic_data:
            break

# print the received data
print(reic_data)
# print the end of the test
print('test done')
