import subprocess
import time

def run_micropython_script(script_name, port):
    """run micropython script and return the output"""
    process = subprocess.Popen(['mpremote', 'connect', port, 'run', script_name], stdout=subprocess.PIPE)
    output, _ = process.communicate()
    print(output)
    return output

# def test_gpio(port):
#     output = run_micropython_script('test_gpio.py', port)
#     assert "test done" in output

def test_uart(port):
    output = run_micropython_script('test_uart.py', port)
    assert "Uart receive success" in output and "test done" in output

# def test_i2c():
#     output = run_micropython_script('i2c_test.py')
#     assert b'Hello I2C' in output

# def test_spi():
#     output = run_micropython_script('spi_test.py')
#     assert b'Hello SPI' in output



    