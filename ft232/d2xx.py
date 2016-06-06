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
#

import io
import ctypes as c
import serial
from serial import (FIVEBITS, SIXBITS, SEVENBITS, EIGHTBITS, PARITY_NONE,
                    PARITY_EVEN, PARITY_ODD, STOPBITS_ONE, STOPBITS_TWO)

try:
    d2xx = c.windll.ftd2xx
except AttributeError:
    d2xx = c.cdll.ftd2xx


FT_OK = 0
FT_OPEN_BY_SERIAL_NUMBER = 1
FT_OPEN_BY_DESCRIPTION = 2
FT_OPEN_BY_LOCATION = 4

FT_FLOW_NONE = 0x0000
FT_FLOW_RTS_CTS = 0x0100

FT_FLOW_DTR_DSR = 0x0200
FT_FLOW_XON_XOFF = 0x0400

FT_PURGE_RX = 1
FT_PURGE_TX = 2

class D2XXException(Exception):
    errors = ["No Error",
              "Invalid Handle",
              "Device Not Found",
              "Device Not Opened",
              "I/O Error",
              "Insufficient Resources",
              "Invalid Parameter",
              "Invalid Baud Rate",
              "Device not opened for erase",
              "Device not opened for write",
              "Failed to write device",
              "EEPROM Read Failed",
              "EEPROM Write Failed",
              "EEPROM Not Present",
              "EEPROM Not Programmed",
              "Invalid Args",
              "Not Supported",
              "Other Error",
              "Device list not ready"]
    def __init__(self, status_code):
        self.msg = self.errors[status_code]

    def __str__(self):
        return self.msg

def list_devices():
    numdevs = c.c_ulong()
    d2xx.FT_CreateDeviceInfoList(c.byref(numdevs))

    flags = c.c_ulong()
    typ = c.c_ulong()
    ID = c.c_ulong()
    LocID = c.c_ulong()
    serial = c.create_string_buffer(20)
    desc = c.create_string_buffer(64)
    handle = c.c_void_p()

    ret = []

    for i in range(numdevs.value):
        status = d2xx.FT_GetDeviceInfoDetail(i, c.byref(flags), c.byref(typ),
                                             c.byref(ID), c.byref(LocID), serial,
                                             desc, c.byref(handle))
        if status != FT_OK: raise D2XXException(status)

        ret.append((serial.value, desc.value))

    return ret

class D2xx(io.RawIOBase):
    BAUDRATES = (50,75,110,134,150,200,300,600,1200,1800,2400,4800,9600,
                 19200,38400,57600,115200,230400,460800,500000,576000,921600,
                 1000000,1152000,1500000,2000000,2500000,3000000,3500000,4000000)
    BYTESIZES = (FIVEBITS, SIXBITS, SEVENBITS, EIGHTBITS)
    PARITIES  = (PARITY_NONE, PARITY_EVEN, PARITY_ODD)
    STOPBITS  = (STOPBITS_ONE, STOPBITS_TWO)

    _parity_map = {PARITY_NONE : 0,
                   PARITY_ODD  : 1,
                   PARITY_EVEN : 2}

    def __init__(self, port=None, baudrate=9600, bytesize=8, parity='N',
                 stopbits=1, timeout=10, xonxoff=0, rtscts=0,
                 writeTimeout=10):
        self._isopen = False
        self.portstr = port
        serial = c.create_string_buffer(port.encode())
        self.handle = c.c_void_p()
        status = d2xx.FT_OpenEx(serial, FT_OPEN_BY_SERIAL_NUMBER,
                                c.byref(self.handle))
        if status != FT_OK: raise D2XXException(status)

        self.baudrate = baudrate
        self._bytesize = bytesize
        self._stopbits = stopbits
        self._parity = parity
        self._setDataCharacteristics()

        self._timeout = timeout
        self._writetimeout = writeTimeout
        self._setTimeouts()

        self._flow = FT_FLOW_NONE
        if xonxoff:
            self._flow = FT_FLOW_XON_XOFF
        elif rtscts:
            self._flow = FT_FLOW_RTS_CTS

        self._setFlowControl()

        self._cbus_mask = 0
        self._cbus_outputs = 0
        self._isopen = True


    def __del__(self):
        if self._isopen:
            d2xx.FT_Close(self.handle)

    def close(self):
        d2xx.FT_ResetDevice(self.handle)
        d2xx.FT_Close(self.handle)
        self._isopen = False

    def setBaudrate(self, baudrate):
        """Change the current baudrate."""

        try:
            self._baudrate = int(baudrate)
        except TypeError:
            raise ValueError("Not a valid baudrate: %r" % baudrate)

        b = c.c_int(self._baudrate)
        status = d2xx.FT_SetBaudRate(self.handle, b)
        if status != FT_OK: raise D2XXException(status)

    def getBaudrate(self):
        """Get the current baudrate setting."""
        return self._baudrate

    baudrate = property(getBaudrate, setBaudrate, doc="Baudrate setting")

    def _setDataCharacteristics(self):
        b = c.c_int(self._bytesize)
        if self._stopbits == STOPBITS_ONE:
            sb = 0
        else:
            sb = 2
        s = c.c_int(sb)
        p = c.c_int(self._parity_map[self._parity])
        status = d2xx.FT_SetDataCharacteristics(self.handle, b, s, p)
        if status != FT_OK: raise D2XXException(status)

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


    def _setTimeouts(self):
        t = c.c_int(int(self._timeout*1000))
        w = c.c_int(int(self._writetimeout*1000))
        status = d2xx.FT_SetTimeouts(self.handle, t, w)
        if status != FT_OK: raise D2XXException(status)


    def setTimeout(self, timeout):
        """Change timeout setting."""
        if timeout is None:
            raise ValueError("Cannot disable the timeout.")
        if timeout < 0: raise ValueError("Not a valid timeout: %r" % timeout)

        try:
            self._timeout = float(timeout)
        except TypeError:
            raise ValueError("Not a valid timeout: %r" % timeout)

        self._setTimeouts()

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
            self._writetimeout = int(timeout)
        except TypeError:
            raise ValueError("Not a valid timeout: %r" % timeout)

        self._setTimeouts()

    def getWriteTimeout(self):
        """Get the current timeout setting."""
        return self._writeTimeout

    writeTimeout = property(getWriteTimeout, setWriteTimeout, doc="Timeout setting for write()")

    def _setFlowControl(self):
        f = c.c_int(self._flow)
        status = d2xx.FT_SetFlowControl(self.handle, f, c.c_int(0x11),
                                        c.c_int(0x13))
        if status != FT_OK: raise D2XXException(status)


    def setXonXoff(self, xonxoff):
        """Change XonXoff setting."""
        if not xonxoff and self._flow == FT_FLOW_XON_XOFF:
            self._flow = FT_FLOW_NONE
        elif not xonxoff:
            return
        else:
            self._flow = FT_FLOW_XON_XOFF
        self._setFlowControl()

    def getXonXoff(self):
        """Get the current XonXoff setting."""
        return self._flow == FT_FLOW_XON_XOFF

    xonxoff = property(getXonXoff, setXonXoff, doc="Xon/Xoff setting")

    def setRtsCts(self, rtscts):
        """Change RtsCts flow control setting."""
        if not xonxoff and self._flow == FT_FLOW_RTS_CTS:
            self._flow = FT_FLOW_NONE
        elif not xonxoff:
            return
        else:
            self._flow = FT_FLOW_RTS_CTS
        self._setFlowControl()

    def getRtsCts(self):
        """Get the current RtsCts flow control setting."""
        return self._flow == FT_FLOW_RTS_CTS

    rtscts = property(getRtsCts, setRtsCts, doc="RTS/CTS flow control setting")

    def setDsrDtr(self, dsrdtr=None):
        """Change DsrDtr flow control setting."""
        if not xonxoff and self._flow == FT_FLOW_DTR_DSR:
            self._flow = FT_FLOW_NONE
        elif not xonxoff:
            return
        else:
            self._flow = FT_FLOW_DTR_DSR
        self._setFlowControl()

    def getDsrDtr(self):
        """Get the current DsrDtr flow control setting."""
        return self._flow == FT_FLOW_DTR_DSR

    dsrdtr = property(getDsrDtr, setDsrDtr, "DSR/DTR flow control setting")

    def __repr__(self):
        """String representation of the current port settings and its state."""
        return ("%s<id=0x%x>(port=%r, baudrate=%r, bytesize=%r, "
                             "parity=%r, stopbits=%r, timeout=%r, "
                             "xonxoff=%r, rtscts=%r, dsrdtr=%r)" %
                (self.__class__.__name__,
                 id(self),
                 self.portstr,
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
        written = c.c_ulong()
        status = d2xx.FT_Write(self.handle, buf, len(s), c.byref(written))
        if status != FT_OK: raise D2XXException(status)
        return written.value

    def read(self, size=1):
        buf = c.create_string_buffer(size)
        read = c.c_ulong()
        status = d2xx.FT_Read(self.handle, buf, size, c.byref(read))
        if status != FT_OK: raise D2XXException(status)
        return buf.raw[0:read.value]

    def cbus_setup(self, mask, init=0):
        self._cbus_mask = int(mask) & 0xf
        self._cbus_outputs = int(init) & 0xf

        mask = (self._cbus_mask << 4) | (self._cbus_outputs & self._cbus_mask)

        status = d2xx.FT_SetBitMode(self.handle, mask, 0x20)
        if status != FT_OK: raise D2XXException(status)

    def cbus_write(self, output):
        self._cbus_outputs = int(output) & 0xf

        mask = (self._cbus_mask << 4) | (self._cbus_outputs & self._cbus_mask)

        status = d2xx.FT_SetBitMode(self.handle, mask, 0x20)
        if status != FT_OK: raise D2XXException(status)

    def cbus_read(self):
        inputs = c.c_char()
        status = d2xx.FT_GetBitMode(self.handle, c.byref(inputs))
        if status != FT_OK: raise D2XXException(status)

        return inputs & 0xf

    def flushInput(self):
        status = d2xx.FT_Purge(self.handle, FT_PURGE_RX)
        if status != FT_OK: raise D2XXException(status)

    def flushOutput(self):
        status = d2xx.FT_Purge(self.handle, FT_PURGE_TX)
        if status != FT_OK: raise D2XXException(status)


    def flush(self):
        pass
