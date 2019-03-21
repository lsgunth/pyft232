# pyft232

This module provides a simple Serial-Like interface to FT232 chips while still
allowing access to the CBUS pins for controlling simple signals.

Ctypes is used to interface with either FTDI's d2xx library (when available)
or libftdi. In this way this package works on both Windows and Linux (and other
libftdi supported OS's) without needing to mess with the FTDI drivers on windows.

In contrast, pyftdi is more feature rich and also cross platform but requires
special drivers on windows so you can't also use the FTDI chip as a pure
COM port.


## Usage

```python
import ft232

serial_number = "FT1234"

try:
    sp = ft232.Ft232(serial_number, baudrate=115200)
except ft232.Ft232Exception:
    print("Unable to open the ftdi device: %s" % serial_number)
    sys.exit(1)

#You may use sp as you would a Serial object
sp.write(b"Hello World!\n")
resp = sp.read(100)

#If you want to use the CBUS pins, you enable them with cbus_setup
# 'mask' is a bitmask which specifies which pins to enable
# 'init' is a bitmask for the initial value for each pin
sp.cbus_setup(mask=3, init=3)

#Change the current value of all setup pins
sp.cbus_write(2)

#Print the current value of all setup pins
print("CBUS: %s" % sp.cbus_read())

```

### GPIO

Simple Blinker example:

```python
# Import a specific chip (FT232H, FT2232H, FT2232D, FT4232H)
from ft232 import FT232H as Board
# Import GPIO for the GPIO Commands
from ft232 import GPIO
from time import sleep

# Open connection to the chosen board, config it as GPIO board and get a Pin at Pin C3 (Pin 12).
pin = Board("").gpio().pin(Board.C3)

# Set Pin as output
pin.direction(GPIO.OUTPUT)
# Read and print current value
print(pin.value())
# Pull/Set Pin HIGH (1)
pin.pull(GPIO.HIGH)
print(pin.value())
# Start blinking
while True:
    pin.pull(GPIO.HIGH)
    sleep(1)
    pin.pull(GPIO.LOW)
    sleep(1)
```

### SPI

Simple SPI example:

```python
# Import a specific chip (FT232H, FT2232H, FT2232D, FT4232H)
from ft232 import FT232H as Board
# Import GPIO for the SPI Commands
from ft232 import SPI
from time import sleep

# Open connection to the chosen board, config it as SPI board and get a Slave at Pin C3 (Pin 12).
spiSlave = Board("").spi(Clock=3e6, Mode=0, BitOrder=SPI.BIT_ORDER_MSB).slave(Board.C3)
# Write data to the spiSlave connected to Pin C3
spiSlave.write([0xff, 0x0f, 0b00001111])
# Read 3 bytes from spiSlave connected to Pin C3 and print data
print(spiSlave.read(3))
```