import math
from ft232 import Ft232

class MPSSE():
    """ This class implements all the functions and commands to setup a MPSSE device.
    All commands are described in this reference sheet 
    https://www.ftdichip.com/Support/Documents/AppNotes/AN_108_Command_Processor_for_MPSSE_and_MCU_Host_Bus_Emulation_Modes.pdf
    """
    # Read/Write command
    BIT_ORDER_MSB = 0
    BIT_ORDER_LSB = 1
    READ_DATA = 0b10
    WRITE_DATA = 0b01
    READ_WRITE_DATA = 0b11
    AS_BITS = 1
    AS_BYTES = 0
    WRITE_CLK_EDGE_NEGATIVE = 0b001
    WRITE_CLK_EDGE_POSITIVE = 0b100
    TMS_ON = 1
    TMS_OFF = 0

    # Pin config command
    CONFIG_HIGH_PINS = 1
    CONFIG_LOW_PINS = 0
    SET_PIN_CONFIG = 0
    GET_PIN_CONFIG = 1

    # Pin setup (direction, value)
    INPUT_PIN = 0
    OUTPUT_PIN = 1
    HIGH_PIN = 1
    LOW_PIN = 0

    def __init__(self, Ft2xx):
        self._Ft232 = Ft2xx
        self._setBitModeMPSSE()
        self._PinDirections = 0b0000000000001011
        self._PinValues = 0x0000000000000000

    def _setBitModeMPSSE(self):
        self._Ft232.setBitMode(mask=0, mode=Ft232.BM_RESET)
        self._Ft232.setBitMode(mask=0, mode=Ft232.BM_MPSSE)

    def _dataCommand(self, ReadWriteData, AsBitsBytes, WriteClkEdge, BitOrder, TmsOnOff):
        """ Before writing or reading data a Commandbyte has to be sent in advance which defines how the data is sent or received and how it should be interpreted. After sending the Commandbyte the Databytes are expected. 
        This function implements chapter 3.2 - Data Shifting Commands.
        The return is one of the OPCODEs in the tables 3.3 - MSB FIRST, 3.4 - LSB First and 3.5 - TMS Commands.
        """
        return (TmsOnOff << 6) | (ReadWriteData << 4) | (BitOrder << 3) | (AsBitsBytes << 1) | WriteClkEdge

    def _pinConfigCommand(self, ConfigHighLowPins, SetGetPinConfig):
        """ To configure the pins or read, wheather they are inputs or outputs and if their initial state is HIGH or LOW a Commandbyte has to be sent in advance which defines if the High Pins (pins ADBUS 7-0) or the Low Pins (pins ACBUS 7-0) are set up or read from. After sending the Commandbyte a Valuebyte is expected, which defines the initial state of the pins (HIGH or LOW) and a Directionbyte, which defines the direction of the pins, weather they are INPUTS or OUTPUTS.
        This function implements chapter 3.6 - Set / Read Data Bits High / Low Bytes.
        The return is one of the OPCODEs in the tables 3.6 - Set / Read Data Bits High / Low Bytes.
        """
        return 0x80 | (ConfigHighLowPins << 1) | SetGetPinConfig

    def _enableLoopback(self):
        """ When Loopback is enabled TDI/DO and TDO/DI pins are connected with eachother internally to transfer data without an external device. This is for testing.
        This function implements chapter 3.7 - Loopback Commands.
        It does not return anything but directly writes the Commandbyte to the device.
        """
        self._Ft232.write(bytes([0x84]))

    def _disableLoobback(self):
        """ This disables Loopback. This function _enableLoopback for mor details
        This function implements chapter 3.7 - Loopback Commands.
        It does not return anything but directly writes the Commandbyte to the device.
        """
        self._Ft232.write(bytes([0x85]))

    def _setClockDivisor(self, Divisor):
        """ The Clock of the MPSSE device can be changed by setting a divisor, which lowers the Baseclock of the device by following equation:
        Clock = Baseclock / (( 1 + Divisor ) * 2)
        This function implements chapter 3.8 - Clock Divisor.
        It does not return anything but directly writes the Commandbyte and new Divisor to the device.
        """
        DivisorLowByte = Divisor & 0xff
        DivisorHighByte = (Divisor >> 8) & 0xff
        print("clock divisor set to " + str(Divisor))
        self._Ft232.write(bytes([0x86, DivisorLowByte, DivisorHighByte]))

    def _cpuMode(self): #feel free to change the name
        """ To be implemented.
        See chapter 4 - Instructions for CPU mode for more details
        """
        pass

    def _mcuHost(self): #feel free to change the name
        """ To be implemented.
        See chapter 5 - Instructions for use in both MPSSE and MCU Host Emulation Modes for more details
        """

    def _enableClockDivide(self):
        """ This will turn on the Baseclock devide of the MPSSE unit by 5.
        This function implements chapter 6.2. Enable Clk Divide by 5.
        It does not return anything but directly writes the Commandbyte and new Divisor to the device.
        """
        self._Ft232.write(bytes([0x8b]))

    def _disableClockDivide(self):
        """ This turn off the Baseclock devide of the MPSSE unit by 5.
        This function implements chapter 6.1. Disable Clk Divide by 5.
        It does not return anything but directly writes the Commandbyte and new Divisor to the device.
        """
        self._Ft232.write(bytes([0x8a]))

    def _enableThreePhaseClocking(self):
        """ Missing description.
        This function implements chapter 6.3 Enable 3 Phase Data Clocking.
        It does not return anything but directly writes the Commandbyte and new Divisor to the device.
        """
        self._Ft232.write(bytes([0x8c]))

    def _disableThreePhaseClocking(self):
        """ Missing description.
        This function implements chapter 6.4 Disable 3 Phase Data Clocking
        It does not return anything but directly writes the Commandbyte and new Divisor to the device.
        """
        self._Ft232.write(bytes([0x8d]))

    def _clockWithoutDataUntilBits(self, NumberBits):
        """ Missing description.
        This function implements chapter 6.5 Clock For n bits with no data transfer.
        It does not return anything but directly writes the Commandbyte and new Divisor to the device.
        """
        self._Ft232.write(bytes([0x8e, NumberBits & 8]))

    def _clockWithoutDataUntilBytes(self, NumberBytes):
        """ Missing description.
        This function implements chapter 6.6 Clock For n x 8 bits with no data transfer.
        It does not return anything but directly writes the Commandbyte and new Divisor to the device.
        """
        NumberLowByte = NumberBytes & 0xff
        NumberHighByte = (NumberBytes >> 8) & 0xff
        self._Ft232.write(bytes([0x8f, NumberLowByte, NumberHighByte]))

    def _clockWithoutDataUntilHigh(self):
        """ Missing description.
        This function implements chapter 6.7 Clk continuously and Wait On I/O High.
        It does not return anything but directly writes the Commandbyte and new Divisor to the device.
        """
        self._Ft232.write(bytes([0x94]))

    def _clockWithoutDataUntilLow(self):
        """ Missing description.
        This function implements chapter 6.8 Clk continuously and Wait On I/O Low.
        It does not return anything but directly writes the Commandbyte and new Divisor to the device.
        """
        self._Ft232.write(bytes([0x95]))

    def _enableAdaptiveClock(self):
        """ Missing description.
        This function implements chapter 6.9 Turn On Adaptive clocking.
        It does not return anything but directly writes the Commandbyte and new Divisor to the device.
        """
        self._Ft232.write(bytes([0x96]))

    def _disableAdaptiveClock(self):
        """ Missing description.
        This function implements chapter 6.10 Turn Off Adaptive clocking.
        It does not return anything but directly writes the Commandbyte and new Divisor to the device.
        """
        self._Ft232.write(bytes([0x97]))

    def _clockWithoutDataUntilHighOrBytes(self, NumberBytes):
        """ Missing description.
        This function implements chapter 6.11 Clock For n x 8 bits with no data transfer or Until GPIOL1 is High.
        It does not return anything but directly writes the Commandbyte and new Divisor to the device.
        """
        NumberLowByte = NumberBytes & 0xff
        NumberHighByte = (NumberBytes >> 8) & 0xff
        self._Ft232.write(bytes([0x9c, NumberLowByte, NumberHighByte]))

    def _clockWithoutDataUntilLowOrBytes(self, NumberBytes):
        """ Missing description.
        This function implements chapter 6.12 Clock For n x 8 bits with no data transfer or Until GPIOL1 is Low.
        It does not return anything but directly writes the Commandbyte and new Divisor to the device.
        """
        NumberLowByte = NumberBytes & 0xff
        NumberHighByte = (NumberBytes >> 8) & 0xff
        self._Ft232.write(bytes([0x9d, NumberLowByte, NumberHighByte]))

    def _setIoDriveTristate(self): #feel free to change the name
        """ Missing description.
        This function implements chapter 7.1 Set I/O to only drive on a ‘0’ and tristate on a ‘1’.
        It does not return anything but directly writes the Commandbyte and new Divisor to the device.
        """
        self._Ft232.write(bytes([0x9e]))

##########################################################################################################
##########################################################################################################
##########################################################################################################

    def _sync(self):
        pass

    def _setPinConfig(self):
        Data = []
        Data.append(self._pinConfigCommand(MPSSE.CONFIG_HIGH_PINS, MPSSE.SET_PIN_CONFIG))
        Data.append((self._PinValues >> 8) & 0xff)
        Data.append((self._PinDirections >> 8) & 0xff)
        Data.append(self._pinConfigCommand(MPSSE.CONFIG_LOW_PINS, MPSSE.SET_PIN_CONFIG))
        Data.append((self._PinValues >> 0) & 0xff)
        Data.append((self._PinDirections >> 0) & 0xff)
        self._Ft232.write(bytes(Data))

    def _getPinConfig(self):
        pass

    def _setPinDirection(self, Pin, InputOutputPin):
        """ Make a Pin an input or output. This does not actually set the pin physically. Call _setPinConfig() after using this function to set the direction physically.

        Parameters
        ----------
        Pin : int, 4 - 15
            The Pin whose direction should be set.
        
        InputOutputPin : mpsse.INPUT_PIN / mpsse.OUTPUT_PIN
            The direction the Pin is set to.
        """       
        if (Pin >= 4) and (Pin <= 15):
            self._PinDirections = (self._PinDirections & ~(1 << Pin)) | (InputOutputPin << Pin)
        else:
            print("error, Pin is not allowed")
            return
        
    def setPinDirection(self, Pin, InputOutputPin):
        """ Make a Pin an input or output.

        Parameters
        ----------
        Pin : int, 4 - 15
            The Pin whose direction should be set.
        
        InputOutputPin : mpsse.INPUT_PIN / mpsse.OUTPUT_PIN
            The direction the Pin is set to.
        """       
        self._setPinDirection(Pin, InputOutputPin)
        self._setPinConfig()

    def setPinsDirection(self, Pin, InputOutputPin):
        pass

    def _setPinValue(self, Pin, HighLowPin):
        """ Set Pin HIGH or LOW. This does not actually set the pin physically. Call _setPinConfig() after using this function to set the value physically.

        Parameters
        ----------
        Pin : int, 0 - 15
            The Pin whose value should be set.
        
        HighLowPin : mpsse.HIGH_PIN / mpsse.LOW_PIN
            The value the Pin is set to.
        """
        if (Pin >= 0) and (Pin <= 15):
            self._PinValues = (self._PinValues & ~(1 << Pin)) | (HighLowPin << Pin)
        else:
            print("error, Pin is not allowed")
            return    
        
    def setPinValue(self, Pin, HighLowPin):
        """ Set Pin HIGH or LOW.

        Parameters
        ----------
        Pin : int, 0 - 15
            The Pin whose value should be set.
        
        HighLowPin : mpsse.HIGH_PIN / mpsse.LOW_PIN
            The value the Pin is set to.
        """
        self._setPinValue(Pin, HighLowPin)
        self._setPinConfig()

    def setPinsValue(self, Pin, HighLowPin):
        pass

    def _getPinValue(self, Pin, HighLowPin):
        pass   
        
    def getPinValue(self, Pin, HighLowPin):
        pass

    def getPinsValue(self, Pin, HighLowPin):
        pass

    def _setClock(self, Hz, ClockDevide=False, ThreePhase=False, AdaptiveClock=False):
        # This function is inspired and partially copied from Adafruit_Python_GPIO/Adafruit_GPIO/FT232H.py on GitHub.
        # for more information see: https://github.com/adafruit/Adafruit_Python_GPIO/blob/master/Adafruit_GPIO/FT232H.py

        """Set the clock speed of the MPSSE engine. Can be any value from 450hz to 30mhz and will pick that speed or the closest speed below it.
        """
        # Disable clock divisor by 5 to enable faster speeds on FT232H.
        if ClockDevide:
            self._enableClockDivide()
        else:
            self._disableClockDivide()
        # Turn on/off adaptive clocking.
        if AdaptiveClock:
            self._enableAdaptiveClock()
        else:
            self._disableAdaptiveClock()
        # Turn on/off three phase clock (needed for I2C).
        # Also adjust the frequency for three-phase clocking as specified in section 2.2.4
        # of this document:
        #   http://www.ftdichip.com/Support/Documents/AppNotes/AN_255_USB%20to%20I2C%20Example%20using%20the%20FT232H%20and%20FT201X%20devices.pdf
        if ThreePhase:
            self._enableThreePhaseClocking()
        else:
            self._disableThreePhaseClocking()
        # Compute divisor for requested clock.
        # Use equation from section 3.8.1 of:
        #  http://www.ftdichip.com/Support/Documents/AppNotes/AN_108_Command_Processor_for_MPSSE_and_MCU_Host_Bus_Emulation_Modes.pdf
        # Note equation is using 60mhz master clock instead of 12mhz.
        MasterClock = 60 * 1000000
        if ClockDevide:
            MasterClock /= 5 
        Divisor = math.ceil(MasterClock / (2 * Hz) - 1) & 0xffff
        if ThreePhase:
            Divisor = int(Divisor * (2.0 / 3.0))
        # Send command to set divisor from low and high byte values.
        self._setClockDivisor(Divisor)

    def _write(self, Data, AsBitsBytes, WriteClkEdge, BitOrder):
        """ Generic write command. 

        Parameters
        ----------
        Data : list, bytes
            The data which should be send. Can be 1 to 65538 bytes long.

        AsBitsBytes : mpsse.AS_BITS / mpsse.AS_BYTES
            Weather a single byte of data is send or a list of bytes.

        WriteClkEdge : mpsse.WRITE_CLK_EDGE_POSITIVE / mpsse.WRITE_CLK_EDGE_NEGATIVE
            Weather the data should be send on positive or negative clock edge.
        
        BitOrder : mpsse.BIT_ORDER_MSB / mpsse.BIT_ORDER_LSB
            Weather the data should be send MSB first or LSB first.
        """
        Command = self._dataCommand(MPSSE.WRITE_DATA, AsBitsBytes, WriteClkEdge, BitOrder, MPSSE.TMS_OFF)
        if AsBitsBytes == MPSSE.AS_BITS:
            Length = Data[0].bit_length() - 1
            self._Ft232.write(bytes([Command, Length]))
        elif AsBitsBytes == MPSSE.AS_BYTES:
            Length = len(Data) - 1
            LengthLowByte = Length & 0xff
            LengthHighByte = (Length >> 8) & 0xff
            self._Ft232.write(bytes([Command, LengthLowByte, LengthHighByte]))
        else:
            print("an error in _write()")
            return
        self._Ft232.write(bytes(Data))

    def _read(self, DataLength, AsBitsBytes, WriteClkEdge, BitOrder):
        """ Generic read command. 

        Parameters
        ----------
        DataLength : int
            The number of bytes or bits which should be read. Can be 1 to 65538 in bytes. Or 1 - 8 if data is read AS_BITS

        AsBitsBytes : mpsse.AS_BITS / mpsse.AS_BYTES
            Weather a number of bytes (AS_BYTES) is read or a number of bits (AS_BITS).

        WriteClkEdge : mpsse.WRITE_CLK_EDGE_POSITIVE / mpsse.WRITE_CLK_EDGE_NEGATIVE
            Weather the data should be read on positive (WRITE_CLK_EDGE_NEGATIVE) or negative (WRITE_CLK_EDGE_POSITIVE) clock edge. 
        
        BitOrder : mpsse.BIT_ORDER_MSB / mpsse.BIT_ORDER_LSB
            Weather the data should be read MSB first or LSB first.
        """
        Command = self._dataCommand(MPSSE.READ_DATA, AsBitsBytes, WriteClkEdge, BitOrder, MPSSE.TMS_OFF)
        if AsBitsBytes == MPSSE.AS_BITS:
            DataLength = DataLength - 1
            self._Ft232.write(bytes([Command, DataLength]))
        elif AsBitsBytes == MPSSE.AS_BYTES:
            DataLength = DataLength - 1
            LenghtLowByte = DataLength & 0xff
            LenghtHighByte = (DataLength >> 8) & 0xff
            self._Ft232.write(bytes([Command, LenghtLowByte, LenghtHighByte]))
        else:
            print("an error in _read()")
            return
        return self._Ft232.read(DataLength)

class SPI(MPSSE):
    CHIP_ACTIVE = MPSSE.LOW_PIN
    CHIP_INACTIVE = MPSSE.HIGH_PIN
    
    def __init__(self, Ft2xx, Cs=[], Clock=1000000, Mode=0, BitOrder=MPSSE.BIT_ORDER_MSB):        
        super().__init__(Ft2xx=Ft2xx)
        self._setup(Cs, Mode, Clock, BitOrder)

    def _setup(self, CS, Mode, Clock, BitOrder):
        self.setMode(Mode)
        self.setClock(Clock)
        self.BitOrder = BitOrder
        self.initCs(CS)

    def initCs(self, Cs):        
        self.Cs = []
        for i in Cs:
            self.Cs.append(i)
            self.setPinDirection(i, MPSSE.OUTPUT_PIN) 
            self.setPinValue(i, SPI.CHIP_INACTIVE)

    def chipSelect(self, Cs, ChipActiveInactive):
        if Cs in self.Cs:
            self.setPinValue(Cs, ChipActiveInactive)

    def setMode(self, Mode):
        """Set SPI mode which controls clock polarity and phase. Should be a numeric value 0, 1, 2, or 3. See wikipedia page for details on meaning: http://en.wikipedia.org/wiki/Serial_Peripheral_Interface_Bus
        """
        if Mode < 0 or Mode > 3:
            raise ValueError('Mode must be a value 0, 1, 2, or 3.')
        if Mode == 0:
            # Mode 0 captures on rising clock, propagates on falling clock
            self.WriteClkEdge = MPSSE.WRITE_CLK_EDGE_NEGATIVE
            # Clock base is low.
            ClockIdle = MPSSE.LOW_PIN
        elif Mode == 1:
            # Mode 1 capture of falling edge, propagate on rising clock
            self.WriteClkEdge = MPSSE.WRITE_CLK_EDGE_POSITIVE
            # Clock base is low.
            ClockIdle = MPSSE.LOW_PIN
        elif Mode == 2:
            # Mode 2 capture on rising clock, propagate on falling clock
            self.WriteClkEdge = MPSSE.WRITE_CLK_EDGE_NEGATIVE
            # Clock base is high.
            ClockIdle = MPSSE.HIGH_PIN
        elif Mode == 3:
            # Mode 3 capture on falling edge, propagage on rising clock
            self.WriteClkEdge = MPSSE.WRITE_CLK_EDGE_POSITIVE
            # Clock base is high.
            ClockIdle = MPSSE.HIGH_PIN
        # Set clock and DO as output, DI as input.  Also start clock at its base value.
        self.setPinValue(0, ClockIdle)

    def setClock(self, Hz):
        if (Hz >= 91.553) and (Hz <= 30 * 1000000):
            if Hz < (457763):
                self._setClock(Hz, True, False, False)
            elif Hz >= (457763):
                self._setClock(Hz, False, False, False)
        else:
            print("Hz is out of range")      

    def write(self, Data, Cs=None):
        if Cs != None:
            self.chipSelect(Cs, SPI.CHIP_ACTIVE)
        self._write(Data, MPSSE.AS_BYTES, self.WriteClkEdge, self.BitOrder)
        if Cs != None:
            self.chipSelect(Cs, SPI.CHIP_INACTIVE)

    def read(self, DataLength, Cs=None):
        if Cs != None:
            self.chipSelect(Cs, SPI.CHIP_ACTIVE)
        Data = self._read(DataLength, MPSSE.AS_BYTES, self.WriteClkEdge, self.BitOrder)
        if Cs != None:
            self.chipSelect(Cs, SPI.CHIP_INACTIVE)
        return Data

### Some temp debug stuff ###

# sp = Ft232("")
# spi = SPI(sp, [5])

# # sp.setBitMode(0)
# # sp.setBitMode(0, sp.BM_MPSSE)
# spi._enableLoopback()
# spi.write([0x4f, 0xfd], 5)

# sp.write(bytes([0x20, 0x02, 0x00]))
# read = sp.read(10)
# print(read.hex())