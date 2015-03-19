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

try:
    sp = ft232.Ft232(serial, baudrate=115200)
except ft232.Ft232Exception:
    print("Unable to open the ftdi device: %s" % serial)
    sys.exit(1)

#You may use sp as you would a Serial object
sp.writeline("Hello World!\n")
resp = sp.read(100)

#If you want to use the CBUS pins, you enable them with cbus_setup
# 'mask' is a bitmask which specifies which pins to enable
# 'init' is a bitmask for the initial value for each pin
sp.cbus_setup(mask=3, init=3)

#Change the current value of all setup pins
sp.cbus_write(2)

```
