from machine import Pin, Timer
import time
pin_in = Pin(1, Pin.IN)
pin_out = Pin(0, Pin.OUT)

val_prev = pin_in.value()
pin_out.value(val_prev)
print("start")

start_time = time.time()
end_time = start_time
while True:
    end_time = time.time()
    if end_time > start_time + 10:
        print("timeout")
        break
    
    val = pin_in.value()

    if val != val_prev:
        start_time = time.time()

        val_prev = val
        pin_out.value(val)
        print(val)


print("test done")
