from machine import Pin, Timer

led = Pin("LED", Pin.OUT)
led.value(0)
tim = Timer()
def tick(timer):
    global led
    led.toggle()

tim.init(freq=10, mode=Timer.PERIODIC, callback=tick)