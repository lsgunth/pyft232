#!/usr/bin/python
#
# Author: Logan Gunthorpe <logang@deltatee.com>
# Copyright (c) Deltatee Enterprises Ltd. 2015, All rights reserved.
#
# This library is free software; you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation; either
# version 3.0 of the License, or (at your option) any later version.

# This library is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public
# License along with this library.

import io
import ctypes as c
import serial
import time
from serial import (FIVEBITS, SIXBITS, SEVENBITS, EIGHTBITS, PARITY_NONE,
                    PARITY_EVEN, PARITY_ODD, STOPBITS_ONE, STOPBITS_TWO)

for lib in ("libftdi.so", "libftdi.so.1", "libftdi.dylib", "libftdi.dylib.1"):
    try:
        ftdi = c.CDLL(lib)
        break
    except OSError:
        continue

VENDOR = 0x0403
PRODUCT = 0x6001

ftdi.ftdi_new.restype = c.c_void_p
ftdi.ftdi_usb_open_desc.argtypes = [c.c_void_p, c.c_int, c.c_int,
                                    c.c_char_p, c.c_char_p]
ftdi.ftdi_get_error_string.restype = c.c_char_p

SIO_DISABLE_FLOW_CTRL = 0x0
SIO_RTS_CTS_HS = (0x1 << 8)
SIO_DTR_DSR_HS = (0x2 << 8)
SIO_XON_XOFF_HS = (0x4 << 8)

class usb_dev_handle(c.Structure):
        pass

c_ubyte_p = c.POINTER(c.c_ubyte)
usb_dev_handle_p = c.POINTER(usb_dev_handle)
class FtdiContext(c.Structure):
    _fields_ = [# USB specific
                ('usb_dev', usb_dev_handle_p), # struct usb_dev_handle *usb_dev;
                ('usb_read_timeout', c.c_int),
                ('usb_write_timeout', c.c_int),
                # FTDI specific
                ('type', c.c_int),               # enum ftdi_chip_type type;
                ('baudrate', c.c_int),
                ('bitbang_enabled', c.c_ubyte),
                ('readbuffer', c_ubyte_p),
                ('readbuffer_offset', c.c_uint),
                ('readbuffer_remaining', c.c_uint),
                ('readbuffer_chunksize', c.c_uint),
                ('writebuffer_chunksize', c.c_uint),
                # FTDI FT2232C requirements
                ('interface', c.c_int),
                ('index', c.c_int),
                ('in_ep', c.c_int),
                ('out_ep', c.c_int),
                # 1: (default) Normal bitbang mode, 2: FT2232C SPI bitbang mode
                ('bitbang_mode', c.c_ubyte),
                ('eeprom_size', c.c_int),
                ('error_str', c.c_char_p),
                ('async_usb_buffer', c.c_char_p),
                ('async_usb_buffer_size', c.c_uint),
                ('module_detact_mode', c.c_int)]

class LibUsbDevice(c.Structure):
    pass

class FtdiDeviceList(c.Structure):
    pass
FtdiDeviceList._fields_ = [('next', c.POINTER(FtdiDeviceList)),
                           ('dev', c.POINTER(LibUsbDevice))]

ftdi.ftdi_usb_find_all.argtypes = [c.c_void_p, c.POINTER(c.POINTER(FtdiDeviceList)),
                                   c.c_int, c.c_int]
ftdi.ftdi_usb_get_strings.argtypes = [c.c_void_p, c.POINTER(LibUsbDevice),
                                      c.c_char_p, c.c_int, c.c_char_p, c.c_int,
                                      c.c_char_p, c.c_int]

class LibFtdiException(Exception):
    def __init__(self, context):
        self.msg = ftdi.ftdi_get_error_string(context)

    def __str__(self):
        return self.msg

def list_devices():
    devices = c.POINTER(FtdiDeviceList)()

    result = []

    ctx = ftdi.ftdi_new()
    if ctx == 0:
        raise LibFtdiException(self._context)

    try:
        ndevs = ftdi.ftdi_usb_find_all(ctx, c.byref(devices), VENDOR, PRODUCT)
        if ndevs < 0:
            raise LibFtdiException(self._context)

        d = devices
        while d:
            serial = c.create_string_buffer(20)
            desc = c.create_string_buffer(200)
            ret = ftdi.ftdi_usb_get_strings(ctx, d[0].dev, None, 0, desc,
                                            c.sizeof(desc), serial,
                                            c.sizeof(serial))
            if ret < 0:
                raise LibFtdiException(self._context)

            result.append((serial.value.decode(), desc.value.decode()))

            d = d[0].next

    finally:
        if ndevs >= 0:
            ftdi.ftdi_list_free(c.byref(devices))
        ftdi.ftdi_free(ctx)

    return result

class LibFtdi(io.RawIOBase):
    BAUDRATES = (50,75,110,134,150,200,300,600,1200,1800,2400,4800,9600,
                 19200,38400,57600,115200,230400,460800,500000,576000,921600,
                 1000000,1152000,1500000,2000000,2500000,3000000,3500000,4000000)
    BYTESIZES = (FIVEBITS, SIXBITS, SEVENBITS, EIGHTBITS)
    PARITIES  = (PARITY_NONE, PARITY_EVEN, PARITY_ODD)
    STOPBITS  = (STOPBITS_ONE, STOPBITS_TWO)

    _parity_map = {PARITY_NONE : 0,
                   PARITY_ODD  : 1,
                   PARITY_EVEN : 2}

    def __init__(self, port=None, serial_number=None, description=None, baudrate=9600, bytesize=8, parity='N',
                 stopbits=1, timeout=0, xonxoff=0, rtscts=0,
                 writeTimeout=0):
        self._isopen = False
        self.snstr = port or serial_number
        self.descstr = description

        self._context = ftdi.ftdi_new()
        self._struct = FtdiContext.from_address(self._context)
        self._context = c.c_void_p(self._context)

        if self._context == 0:
            raise LibFtdiException(self._context)

        if self.snstr:
            serial = c.create_string_buffer(self.snstr.encode())
            ret = ftdi.ftdi_usb_open_desc(self._context, VENDOR,
                                          PRODUCT, None, serial)
            if ret != 0: raise LibFtdiException(self._context)
        elif description:
            desc = c.create_string_buffer(description.encode())
            ret = ftdi.ftdi_usb_open_desc(self._context, VENDOR,
                                          PRODUCT, desc, None)
            if ret != 0: raise LibFtdiException(self._context)
        else:
            raise LibFtdiException(self._context)

        self._cbus_mask = 0
        self._cbus_outputs = 0

        self.baudrate = baudrate
        self._bytesize = bytesize
        self._stopbits = stopbits
        self._parity = parity
        self._setDataCharacteristics()

        self.timeout = timeout
        self._writeTimeout = writeTimeout

        self._flow = SIO_DISABLE_FLOW_CTRL
        if xonxoff:
            self._flow = SIO_XON_XOFF_HS
        elif rtscts:
            self._flow = SIO_RTS_CTS_HS

        self._setFlowControl()

        self._isopen = True

    def __del__(self):
        if ftdi and self._context:
            ftdi.ftdi_free(self._context)
            self._context = None

    def close(self):
        self.__del__()

    def setBaudrate(self, baudrate):
        """Change the current baudrate."""

        #Need to reset the bitmode to work around a bug in recent libftdi
        # versions.
        if ftdi.ftdi_set_bitmode(self._context, 0, 0):
            raise LibFtdiException(self._context)

        try:
            self._baudrate = int(baudrate)
        except TypeError:
            raise ValueError("Not a valid baudrate: %r" % baudrate)
        ret = ftdi.ftdi_set_baudrate(self._context, self._baudrate)
        if ret != 0:
            raise LibFtdiException(self._context)

        self.cbus_write(None)


    def getBaudrate(self):
        """Get the current baudrate setting."""
        return self._baudrate

    baudrate = property(getBaudrate, setBaudrate, doc="Baudrate setting")

    def _setDataCharacteristics(self):
        if self._stopbits == STOPBITS_ONE:
            sb = 0
        else:
            sb = 2

        ret = ftdi.ftdi_set_line_property(self._context,
                                          self._bytesize, sb,
                                          self._parity_map[self._parity])
        if ret != 0:
            raise LibFtdiException(self._context)

    def setByteSize(self, bytesize):
        """Change byte size."""
        if bytesize not in self.BYTESIZES: raise ValueError("Not a valid byte size: %r" % bytesize)
        self._bytesize = bytesize
        self._setDataCharacteristics()

    def getByteSize(self):
        """Get the current byte size setting."""
        return self._bytesize

    bytesize = property(getByteSize, setByteSize, doc="Byte size setting")


    def setParity(self, parity):
        """Change parity setting."""
        if parity not in self.PARITIES: raise ValueError("Not a valid parity: %r" % parity)
        self._parity = parity
        self._setDataCharacteristics()

    def getParity(self):
        """Get the current parity setting."""
        return self._parity

    parity = property(getParity, setParity, doc="Parity setting")


    def setStopbits(self, stopbits):
        """Change stopbits size."""
        if stopbits not in self.STOPBITS: raise ValueError("Not a valid stopbit size: %r" % stopbits)
        self._stopbits = stopbits
        self._setDataCharacteristics()

    def getStopbits(self):
        """Get the current stopbits setting."""
        return self._stopbits

    stopbits = property(getStopbits, setStopbits, doc="Stopbits setting")



    def setTimeout(self, timeout):
        """Change timeout setting."""
        if timeout is None:
            raise ValueError("Cannot disable the timeout.")
        if timeout < 0: raise ValueError("Not a valid timeout: %r" % timeout)

        try:
            self._timeout = float(timeout)
        except TypeError:
            raise ValueError("Not a valid timeout: %r" % timeout)

        self._struct.usb_read_timeout = int(timeout*1000)

    def getTimeout(self):
        """Get the current timeout setting."""
        return self._timeout

    timeout = property(getTimeout, setTimeout, doc="Timeout setting for read()")

    def setWriteTimeout(self, timeout):
        """Change timeout setting."""
        if timeout is None:
            raise ValueError("Cannot disable the timeout.")
        if timeout < 0: raise ValueError("Not a valid timeout: %r" % timeout)

        try:
            self._writeTimeout = float(timeout)
        except TypeError:
            raise ValueError("Not a valid timeout: %r" % timeout)

        self._struct.usb_write_timeout = int(timeout*1000)

    def getWriteTimeout(self):
        """Get the current timeout setting."""
        return self._writeTimeout

    writeTimeout = property(getWriteTimeout, setWriteTimeout, doc="Timeout setting for write()")

    def _setFlowControl(self):
        if ftdi.ftdi_setflowctrl(self._context, self._flow) != 0:
            raise LibFtdiException(self._context)

    def setXonXoff(self, xonxoff):
        """Change XonXoff setting."""
        if not xonxoff and self._flow == SIO_XON_XOFF_HS:
            self._flow = SIO_DISABLE_FLOW_CTRL
        elif not xonxoff:
            return
        else:
            self._flow = SIO_XON_XOFF_HS
        self._setFlowControl()

    def getXonXoff(self):
        """Get the current XonXoff setting."""
        return self._flow == SIO_XON_XOFF_HS

    xonxoff = property(getXonXoff, setXonXoff, doc="Xon/Xoff setting")

    def setRtsCts(self, rtscts):
        """Change RtsCts flow control setting."""
        if not xonxoff and self._flow == SIO_RTS_CTS_HS:
            self._flow = SIO_DISABLE_FLOW_CTRL
        elif not xonxoff:
            return
        else:
            self._flow = SIO_RTS_CTS_HS
        self._setFlowControl()

    def getRtsCts(self):
        """Get the current RtsCts flow control setting."""
        return self._flow == SIO_RTS_CTS_HS

    rtscts = property(getRtsCts, setRtsCts, doc="RTS/CTS flow control setting")

    def setDsrDtr(self, dsrdtr=None):
        """Change DsrDtr flow control setting."""
        if not xonxoff and self._flow == SIO_DTR_DSR_HS:
            self._flow = SIO_DISABLE_FLOW_CTRL
        elif not xonxoff:
            return
        else:
            self._flow = SIO_DTR_DSR_HS
        self._setFlowControl()

    def getDsrDtr(self):
        """Get the current DsrDtr flow control setting."""
        return self._flow == SIO_DTR_DSR_HS

    dsrdtr = property(getDsrDtr, setDsrDtr, "DSR/DTR flow control setting")

    def __repr__(self):
        """String representation of the current port settings and its state."""
        return ("%s<id=0x%x>(serial_number=%r, description=%r, baudrate=%r, bytesize=%r, "
                             "parity=%r, stopbits=%r, timeout=%r, "
                             "xonxoff=%r, rtscts=%r, dsrdtr=%r)" %
                (self.__class__.__name__,
                 id(self),
                 self.snstr,
                 self.descstr,
                 self.baudrate,
                 self.bytesize,
                 self.parity,
                 self.stopbits,
                 self.timeout,
                 self.xonxoff,
                 self.rtscts,
                 self.dsrdtr,
                ))


    def write(self, s):
        buf = c.create_string_buffer(s)
        written = ftdi.ftdi_write_data(self._context, buf, len(s))

        if written < 0:
            raise LibFtdiException(self._context)

        return written

    def read(self, size=1):
        s = c.create_string_buffer(size)
        ret = b""
        starttime = time.time()
        while self._timeout == 0 or time.time() - starttime < self._timeout:
            read = ftdi.ftdi_read_data(self._context, s,
                                       size-len(ret))

            if read < 0:
                return None

            ret += s.raw[0:read]
            if len(ret) == size:
                break
            time.sleep(0.01)
        return ret

    def cbus_setup(self, mask, init=0):
        self._cbus_mask = int(mask) & 0xf
        self._cbus_outputs = int(init) & 0xf

        mask = (self._cbus_mask << 4) | (self._cbus_outputs & self._cbus_mask)

        if ftdi.ftdi_set_bitmode(self._context, mask, 0x20):
            raise LibFtdiException(self._context)

    def cbus_write(self, output):
        if output is not None:
            self._cbus_outputs = int(output) & 0xf

        mask = (self._cbus_mask << 4) | (self._cbus_outputs & self._cbus_mask)

        if ftdi.ftdi_set_bitmode(self._context, mask, 0x20) != 0:
            raise LibFtdiException(self._context)

    def cbus_read(self):
        inputs = c.c_char()
        if ftdi.ftdi_read_pins(self._context, c.byref(inputs)) != 0:
            raise LibFtdiException(self._context)

        return ord(inputs.value) & 0xf

    def flushInput(self):
        if ftdi.ftdi_usb_purge_rx_buffer(self._context) != 0:
            raise LibFtdiException(self._context)

    def flushOutput(self):
        if ftdi.ftdi_usb_purge_tx_buffer(self._context) != 0:
            raise LibFtdiException(self._context)


    def flush(self):
        pass
