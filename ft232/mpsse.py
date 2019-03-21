import math

class MPSSE():
    """ This class implements all the functions and commands to setup a MPSSE device. This class shoudl not be created directly. Its meant to be subclassed to create custom bus protocols. 
    All commands are described in this reference sheet 
    https://www.ftdichip.com/Support/Documents/AppNotes/AN_108_Command_Processor_for_MPSSE_and_MCU_Host_Bus_Emulation_Modes.pdf
    """
    ## Read/Write command
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

    ## Pin config command
    CONFIG_HIGH_PINS = 1
    CONFIG_LOW_PINS = 0
    SET_PIN_CONFIG = 0
    GET_PIN_CONFIG = 1

    def __init__(self, Ftx232):
        self.__Ft232 = Ftx232
        ## Sets LowByte and HighByte inital directions and values
        # The Direction of the first 4 pins are fixed by hardware
        self.__setupMPSSE(PinDirections=bytearray([0b00001011, 0]), PinValues=bytearray([0, 0]))
    
    def __setupMPSSE(self, PinDirections, PinValues):
        """ Sets the Ftx232 to MPSSE Mode and initalizes the PinDirections and PinValues. This should only be called on __init__().
        """
        self.__setBitModeMPSSE()
        self.__PinDirections = PinDirections
        self.__PinValues = PinValues
        self.__setPinConfig()

    def __dataCommand(self, ReadWriteData, AsBitsBytes, WriteClkEdge, BitOrder, TmsOnOff):
        """ Before writing or reading data a Commandbyte has to be sent in advance which defines how the data is sent or received and how it should be interpreted. After sending the Commandbyte the Databytes are expected. 
        This function implements chapter 3.2 - Data Shifting Commands.
        The return is one of the OPCODEs in the tables 3.3 - MSB FIRST, 3.4 - LSB First and 3.5 - TMS Commands.
        """
        return (TmsOnOff << 6) | (ReadWriteData << 4) | (BitOrder << 3) | (AsBitsBytes << 1) | WriteClkEdge

    def __pinConfigCommand(self, ConfigHighLowPins, SetGetPinConfig):
        """ To configure the pins or read, wheather they are inputs or outputs and if their initial state is HIGH or LOW a Commandbyte has to be sent in advance which defines if the High Pins (pins ADBUS 7-0) or the Low Pins (pins ACBUS 7-0) are set up or read from. After sending the Commandbyte a Valuebyte is expected, which defines the initial state of the pins (HIGH or LOW) and a Directionbyte, which defines the direction of the pins, weather they are INPUTS or OUTPUTS.
        This function implements chapter 3.6 - Set / Read Data Bits High / Low Bytes.
        The return is one of the OPCODEs in the tables 3.6 - Set / Read Data Bits High / Low Bytes.
        """
        return 0x80 | (ConfigHighLowPins << 1) | SetGetPinConfig

    def enableLoopback(self):
        """ When Loopback is enabled TDI/DO and TDO/DI pins are connected with eachother internally to transfer data without an external device. This is for testing.
        This function implements chapter 3.7 - Loopback Commands.
        It does not return anything but directly writes the Commandbyte to the device.
        """
        self.__Ft232.write(bytes([0x84]))

    def disableLoopback(self):
        """ This disables Loopback. See enableLoopback() for mor details
        This function implements chapter 3.7 - Loopback Commands.
        It does not return anything but directly writes the Commandbyte to the device.
        """
        self.__Ft232.write(bytes([0x85]))

    def __setClockDivisor(self, Divisor):
        """ The Clock of the MPSSE device can be changed by setting a divisor, which lowers the Baseclock of the device by following equation:
        Clock = Baseclock / (( 1 + Divisor ) * 2)
        This should not be called directly. Instead use _setClock().
        This function implements chapter 3.8 - Clock Divisor.
        It does not return anything but directly writes the Commandbyte and new Divisor to the device.
        """
        DivisorLowByte = Divisor & 0xff
        DivisorHighByte = (Divisor >> 8) & 0xff
        print("clock divisor set to " + str(Divisor))
        self.__Ft232.write(bytes([0x86, DivisorLowByte, DivisorHighByte]))

    def _cpuMode(self): #feel free to change the name
        """ To be implemented.
        See chapter 4 - Instructions for CPU mode for more details
        """
        pass

    def _mcuHost(self): #feel free to change the name
        """ To be implemented.
        See chapter 5 - Instructions for use in both MPSSE and MCU Host Emulation Modes for more details
        """
        pass

    def __enableClockDivide(self):
        """ This will turn on the Baseclock devide of the MPSSE unit by 5.
        This function implements chapter 6.2. Enable Clk Divide by 5.
        It does not return anything but directly writes the Commandbyte and new Divisor to the device.
        """
        if self.__isModernDevice():
            self.__Ft232.write(bytes([0x8b]))

    def __disableClockDivide(self):
        """ This turn off the Baseclock devide of the MPSSE unit by 5.
        This function implements chapter 6.1. Disable Clk Divide by 5.
        It does not return anything but directly writes the Commandbyte and new Divisor to the device.
        """
        if self.__isModernDevice():
            self.__Ft232.write(bytes([0x8a]))

    def __enableThreePhaseClocking(self):
        """ Missing description.
        This function implements chapter 6.3 Enable 3 Phase Data Clocking.
        It does not return anything but directly writes the Commandbyte and new Divisor to the device.
        """
        if self.__isModernDevice():
            self.__Ft232.write(bytes([0x8c]))

    def __disableThreePhaseClocking(self):
        """ Missing description.
        This function implements chapter 6.4 Disable 3 Phase Data Clocking
        It does not return anything but directly writes the Commandbyte and new Divisor to the device.
        """
        if self.__isModernDevice():
            self.__Ft232.write(bytes([0x8d]))

    def clockWithoutDataUntilBits(self, NumberBits):
        """ Missing description.
        This function implements chapter 6.5 Clock For n bits with no data transfer.
        It does not return anything but directly writes the Commandbyte and new Divisor to the device.
        """
        if self.__isModernDevice():
            self.__Ft232.write(bytes([0x8e, NumberBits & 8]))

    def clockWithoutDataUntilBytes(self, NumberBytes):
        """ Missing description.
        This function implements chapter 6.6 Clock For n x 8 bits with no data transfer.
        It does not return anything but directly writes the Commandbyte and new Divisor to the device.
        """
        if self.__isModernDevice():
            NumberLowByte = NumberBytes & 0xff
            NumberHighByte = (NumberBytes >> 8) & 0xff
            self.__Ft232.write(bytes([0x8f, NumberLowByte, NumberHighByte]))

    def clockWithoutDataUntilHigh(self):
        """ Missing description.
        This function implements chapter 6.7 Clk continuously and Wait On I/O High.
        It does not return anything but directly writes the Commandbyte and new Divisor to the device.
        """
        if self.__isModernDevice():
            self.__Ft232.write(bytes([0x94]))

    def clockWithoutDataUntilLow(self):
        """ Missing description.
        This function implements chapter 6.8 Clk continuously and Wait On I/O Low.
        It does not return anything but directly writes the Commandbyte and new Divisor to the device.
        """
        if self.__isModernDevice():
            self.__Ft232.write(bytes([0x95]))

    def __enableAdaptiveClock(self):
        """ Missing description.
        This function implements chapter 6.9 Turn On Adaptive clocking.
        It does not return anything but directly writes the Commandbyte and new Divisor to the device.
        """
        if self.__isModernDevice():
            self.__Ft232.write(bytes([0x96]))

    def __disableAdaptiveClock(self):
        """ Missing description.
        This function implements chapter 6.10 Turn Off Adaptive clocking.
        It does not return anything but directly writes the Commandbyte and new Divisor to the device.
        """
        if self.__isModernDevice():
            self.__Ft232.write(bytes([0x97]))

    def clockWithoutDataUntilHighOrBytes(self, NumberBytes):
        """ Missing description.
        This function implements chapter 6.11 Clock For n x 8 bits with no data transfer or Until GPIOL1 is High.
        It does not return anything but directly writes the Commandbyte and new Divisor to the device.
        """
        if self.__isModernDevice():
            NumberLowByte = NumberBytes & 0xff
            NumberHighByte = (NumberBytes >> 8) & 0xff
            self.__Ft232.write(bytes([0x9c, NumberLowByte, NumberHighByte]))

    def clockWithoutDataUntilLowOrBytes(self, NumberBytes):
        """ Missing description.
        This function implements chapter 6.12 Clock For n x 8 bits with no data transfer or Until GPIOL1 is Low.
        It does not return anything but directly writes the Commandbyte and new Divisor to the device.
        """
        if self.__isModernDevice():
            NumberLowByte = NumberBytes & 0xff
            NumberHighByte = (NumberBytes >> 8) & 0xff
            self.__Ft232.write(bytes([0x9d, NumberLowByte, NumberHighByte]))

    def setIoDriveTristate(self): #feel free to change the name
        """ Missing description.
        This function implements chapter 7.1 Set I/O to only drive on a ‘0’ and tristate on a ‘1’.
        It does not return anything but directly writes the Commandbyte and new Divisor to the device.
        """
        if self.__isModernDevice():
            self.__Ft232.write(bytes([0x9e]))

    def _sync(self):
        pass

    def __setBitModeMPSSE(self):
        """ This sets the Ftx232 to MPSSE Mode.  
        """
        self.__Ft232.setBitMode(mask=0, mode=self.__Ft232.BM_MPSSE)

    def __isModernDevice(self):
        """ This checks if the board is a modern device. 
        
        Returns
        -------
        bool
        """
        return self.__Ft232.MPSSE_CLOCK == 60e6

    def __highByteExists(self):
        """ This checks if the board has more than 8 pins. 
        
        Returns
        -------
        bool
        """
        return (len(self.__Ft232.PortC) >= 1)

    def __isPinLowByte(self, Pin):
        """ This checks if the Pin is one of the first 8 pins (LowBytePin).
        
        Returns
        -------
        bool
        """
        return (0 <= Pin <= 7)

    def __isPinHighByte(self, Pin):
        """ This checks if the Pin is one of the pins 9 ++ (HighBytePin).
        
        Returns
        -------
        bool
        """
        return self.__highByteExists() and (8 <= Pin <= (len(self.__Ft232.PortC) + 7))

    def __setPinConfig(self):
        """ Sets the __PinValues and __PinDirections to the device. 
        """
        Data = []
        Data.append(self.__pinConfigCommand(MPSSE.CONFIG_LOW_PINS, MPSSE.SET_PIN_CONFIG))
        Data.append(self.__PinValues[0])
        Data.append(self.__PinDirections[0])
        ## Checks if the board has more than 8 Pins
        if self.__highByteExists():
            Data.append(self.__pinConfigCommand(MPSSE.CONFIG_HIGH_PINS, MPSSE.SET_PIN_CONFIG))        
            Data.append(self.__PinValues[1])
            Data.append(self.__PinDirections[1])
        self.__Ft232.write(bytes(Data))

    def __getPinConfig(self):
        """ Gets the actual PinValues from the device as a list.
        
        Returns
        -------
        list
            The PinValues as a list of bytes
        """
        Command = []
        Command.append(self.__pinConfigCommand(MPSSE.CONFIG_LOW_PINS, MPSSE.GET_PIN_CONFIG))
        ## Checks if the board has more than 8 Pins
        if self.__highByteExists():
            Command.append(self.__pinConfigCommand(MPSSE.CONFIG_HIGH_PINS, MPSSE.GET_PIN_CONFIG))
        self.__Ft232.write(bytes(Command))
        return self.__Ft232.read(len(Command))

    def __setPinDirection(self, Pin, InputOutputPin):
        """ Make a Pin an input or output. This does not actually set the pin physically. Use _setPinDirection() or _setPinsDirection() instead. Otherwise call _setPinConfig() after using this function to set the direction physically.
        Parameters
        ----------
        Pin : int, 4 - 15
            The Pin whose direction should be set.
        
        InputOutputPin : GPIO.INPUT / GPIO.OUTPUT
            The direction the Pin is set to.
        """
        if (Pin >= self.__Ft232.D3) and self.__isPinLowByte(Pin):
            self.__PinDirections[0] = (self.__PinDirections[0] & ~(1 << Pin)) | (InputOutputPin << Pin)
        elif self.__isPinHighByte(Pin):
            self.__PinDirections[1] = (self.__PinDirections[1] & ~(1 << (Pin - 8))) | (InputOutputPin << (Pin - 8))
        else:
            print("Pin is out of range")
        
    def setPinDirection(self, Pin, InputOutputPin):
        """ Make a Pin an input or output.
        Parameters
        ----------
        Pin : int, 4 - 15
            The Pin whose direction should be set.
        
        InputOutputPin : GPIO.INPUT / GPIO.OUTPUT
            The direction the Pin is set to.
        """       
        self.__setPinDirection(Pin, InputOutputPin)
        self.__setPinConfig()

    def setPinsDirection(self, PinsDirection):
        """ Make a dictionary of Pins an input or output.
        Parameters
        ----------
        PinDirection : {int : int} {4 - 15 : GPIO.INPUT / GPIO.OUTPUT}
            The Pins whose directions should be set.
        """ 
        for Pin, InputOutputPin in iter(PinsDirection.items()):
            self.__setPinDirection(Pin, InputOutputPin)
        self.__setPinConfig()

    def __setPinValue(self, Pin, HighLowPin):
        """ Set Pin HIGH or LOW. This does not actually set the pin physically. Use _setPinValue() or _setPinsValue() instead. Otherwise call _setPinConfig() after using this function to set the value physically.
        Parameters
        ----------
        Pin : int, 0 - 15
            The Pin whose value should be set.
        
        HighLowPin : GPIO.HIGH / GPIO.LOW
            The value the Pin is set to.
        """
        if self.__isPinLowByte(Pin):
            self.__PinValues[0] = (self.__PinValues[0] & ~(1 << Pin)) | (HighLowPin << Pin)
        elif self.__isPinHighByte(Pin):
            self.__PinValues[1] = (self.__PinValues[1] & ~(1 << (Pin - 8))) | (HighLowPin << (Pin - 8))
        else:
            print("Pin is out of range")  
        
    def setPinValue(self, Pin, HighLowPin):
        """ Set Pin HIGH or LOW.
        Parameters
        ----------
        Pin : int, 0 - 15
            The Pin whose value should be set.
        
        HighLowPin : GPIO.HIGH / GPIO.LOW
            The value the Pin is set to.
        """
        self.__setPinValue(Pin, HighLowPin)
        self.__setPinConfig()

    def setPinsValue(self, PinsValue):
        """ Sets the PinValue to a dictionary of Pins.
        Parameters
        ----------
        PinValue : {int : int}, {0 - 15 : GPIO.HIGH / GPIO.LOW}
            The Pins whose values should be set.
        """ 
        for Pin, HighLowPin in iter(PinsValue.items()):
            self.__setPinValue(Pin, HighLowPin)
        self.__setPinConfig()

    def __getPinValue(self, Pin, PinsConfig):
        """ Get the PinValue of a Pin out of a PinsConfig. This does not actually read the pin physically values. Use _getPinValue() or _getPinsValue() instead. Otherwise call _getPinConfig() before using this function to get the values physically and use them for the param PinsConfig.
        Parameters
        ----------
        Pin : int, 0 - 15
            The Pin whose value should be read.
        PinConfig : list(byte)
            The PinConfig got from __getPinConfig()
        return
        ------
        int
            The Value of the Pin
        """
        if self.__isPinLowByte(Pin):
            return (PinsConfig[0] >> Pin) & 1
        elif self.__isPinHighByte(Pin):
            return (PinsConfig[1] >> (Pin-8)) & 1
        
    def getPinValue(self, Pin):
        """ Get the PinValue of a Pin.
        Parameters
        ----------
        Pin : int, 0 - 15
            The Pin whose value should be read.
        return
        ------
        int
            The Value of the Pin
        """
        PinsConfig = self.__getPinConfig()
        return self.__getPinValue(Pin, PinsConfig)

    def getPinsValue(self, Pins):
        """ Get the PinValues of a list of Pins.
        Parameters
        ----------
        Pins : list(int), 0 - 15
            The Pins whose values should be read.
        Returns
        -------
        list(int)
            The Values of the Pins
        """
        PinsConfig = self.__getPinConfig()
        PinsValue = {}
        for Pin in Pins:
            PinsValue[Pin] = self.__getPinValue(Pin, PinsConfig)
        return PinsValue

    def setClock(self, Hz, ThreePhase=False, AdaptiveClock=False):
        # This function is inspired and partially copied from Adafruit_Python_GPIO/Adafruit_GPIO/FT232H.py on GitHub.
        # for more information see: https://github.com/adafruit/Adafruit_Python_GPIO/blob/master/Adafruit_GPIO/FT232H.py

        """Set the clock speed of the MPSSE engine. Can be any value from 92hz to 30mhz (FT232H, FT2232H, FT4232H) or 92hz to 6mhz (FT2232D) and will pick that speed or the closest speed below it. When ThreePhaseClocking is enabled clock speed drops down by 1.5 (62hz - MasterClock/3).
        Parameter
        ---------
        Hz: int, 92hz to 30mhz (FT232H, FT2232H, FT4232H), 92hz to 6mhz (FT2232D)
            Frequency in hz
        ThreePhase: bool
            Enables/Disables ThreePhaseClocking reduces Hz range by factor 1.5
        AdaptiveClock: bool
            Enables/Disables AdaptiveClocking
        Raises
        ------
        ValueError:
            Hz must be within 62 - MasterClock/3  when ThreePhaseClocking is enabled
        ValueError:
            Hz must be within 92 - MasterClock/2    
        """
        # Turn on/off adaptive clocking.
        if AdaptiveClock:
            self.__enableAdaptiveClock()
        else:
            self.__disableAdaptiveClock()

        # Turn on/off three phase clock (needed for I2C).
        # Also adjust the frequency for three-phase clocking as specified in section 2.2.4
        # of this document:
        #   http://www.ftdichip.com/Support/Documents/AppNotes/AN_255_USB%20to%20I2C%20Example%20using%20the%20FT232H%20and%20FT201X%20devices.pdf
        if ThreePhase:
            self.__enableThreePhaseClocking()
            Hz *= 1.5
        else:
            self.__disableThreePhaseClocking()

        # Compute divisor for requested clock.
        # Use equation from section 3.8.1 of:
        # http://www.ftdichip.com/Support/Documents/AppNotes/AN_108_Command_Processor_for_MPSSE_and_MCU_Host_Bus_Emulation_Modes.pdf
        MasterClock = self.__Ft232.MPSSE_CLOCK
        
        if 92 <= Hz <= (MasterClock / 2):
            if Hz < (457763) and self.__isModernDevice():
                self.__enableClockDivide()
                MasterClock /= 5
            elif Hz >= (457763) and self.__isModernDevice():
                self.__disableClockDivide()
        elif ThreePhase:
            raise ValueError("Hz must be within 62 - MasterClock/3  when ThreePhaseClocking is enabled")
        else:
            raise ValueError("Hz must be within 92 - MasterClock/2")        

        Divisor = math.ceil(MasterClock / (2 * Hz) - 1) & 0xffff
        
        # Send command to set divisor from low and high byte values.
        self.__setClockDivisor(Divisor)

    def _write(self, Data, AsBitsBytes, WriteClkEdge, BitOrder, Tms=TMS_OFF):
        """ Generic write command. 
        Parameters
        ----------
        Data : list, bytes, bytearray
            The data which should be send. Can be 1 to 65538 bytes long. Or 1 - 8 if data is read AS_BITS.
        AsBitsBytes : MPSSE.AS_BITS / MPSSE.AS_BYTES
            Weather a single byte of data is send or a list of bytes.
        WriteClkEdge : MPSSE.WRITE_CLK_EDGE_POSITIVE / MPSSE.WRITE_CLK_EDGE_NEGATIVE
            Weather the data should be send on positive or negative clock edge.
        
        BitOrder : MPSSE.BIT_ORDER_MSB / MPSSE.BIT_ORDER_LSB
            Weather the data should be send MSB first or LSB first.
        Tms : MPSSE.TMS_ON / MPSSE.TMS_OFF
            Weather TMS is enabled/disabled.
        """
        Command = self.__dataCommand(MPSSE.WRITE_DATA, AsBitsBytes, WriteClkEdge, BitOrder, Tms)
        if AsBitsBytes == MPSSE.AS_BITS:
            Length = Data[0].bit_length() - 1
            self.__Ft232.write(bytes([Command, Length]))
        elif AsBitsBytes == MPSSE.AS_BYTES:
            Length = len(Data) - 1
            LengthLowByte = Length & 0xff
            LengthHighByte = (Length >> 8) & 0xff
            self.__Ft232.write(bytes([Command, LengthLowByte, LengthHighByte]))
        self.__Ft232.write(bytes(Data))

    def _read(self, ReadLength, AsBitsBytes, WriteClkEdge, BitOrder, Tms=TMS_OFF):
        """ Generic read command. 
        Parameters
        ----------
        ReadLength : int
            The number of bytes or bits which should be read. Can be 1 to 65538 in bytes. Or 1 - 8 if data is read AS_BITS
        AsBitsBytes : MPSSE.AS_BITS / MPSSE.AS_BYTES
            Weather a number of bytes (AS_BYTES) is read or a number of bits (AS_BITS).
        WriteClkEdge : MPSSE.WRITE_CLK_EDGE_POSITIVE / MPSSE.WRITE_CLK_EDGE_NEGATIVE
            Weather the data should be read on positive (WRITE_CLK_EDGE_NEGATIVE) or negative (WRITE_CLK_EDGE_POSITIVE) clock edge. 
        
        BitOrder : MPSSE.BIT_ORDER_MSB / MPSSE.BIT_ORDER_LSB
            Weather the data should be read MSB first or LSB first.
        Tms : MPSSE.TMS_ON / MPSSE.TMS_OFF
            Weather TMS is enabled/disabled.
        Returns
        -------
        list(int)
            A list with the read bytes.
        """
        Command = self.__dataCommand(MPSSE.READ_DATA, AsBitsBytes, WriteClkEdge, BitOrder, Tms)
        if AsBitsBytes == MPSSE.AS_BITS:
            ReadLength = ReadLength - 1
            self.__Ft232.write(bytes([Command, ReadLength]))
        elif AsBitsBytes == MPSSE.AS_BYTES:
            ReadLength = ReadLength - 1
            LenghtLowByte = ReadLength & 0xff
            LenghtHighByte = (ReadLength >> 8) & 0xff
            self.__Ft232.write(bytes([Command, LenghtLowByte, LenghtHighByte]))
        return self.__Ft232.read(ReadLength)
    
    def _transfer(self, Data, ReadLength, AsBitsBytes, WriteClkEdge, BitOrder, Tms=TMS_OFF):
        """ Generic transfer command. 
        Parameters
        ----------
        Data : list, bytes, bytearray
            The data which should be send. Can be 1 to 65538 bytes long. Or 1 - 8 if data is read AS_BITS.
        ReadLength : int
            The number of bytes or bits which should be read. Can be 1 to 65538 in bytes. Or 1 - 8 if data is read AS_BITS.
        AsBitsBytes : MPSSE.AS_BITS / MPSSE.AS_BYTES
            Weather a number of bytes (AS_BYTES) is write/read or a number of bits (AS_BITS).
        WriteClkEdge : MPSSE.WRITE_CLK_EDGE_POSITIVE / MPSSE.WRITE_CLK_EDGE_NEGATIVE
            Weather the data should be write/read on positive (WRITE_CLK_EDGE_NEGATIVE) or negative (WRITE_CLK_EDGE_POSITIVE) clock edge. 
        
        BitOrder : MPSSE.BIT_ORDER_MSB / MPSSE.BIT_ORDER_LSB
            Weather the data should be write/read MSB first or LSB first.
        Tms : MPSSE.TMS_ON / MPSSE.TMS_OFF
            Weather TMS is enabled/disabled.
        Returns
        -------
        list(int)
            A list with the read bytes.
        """
        Command = self.__dataCommand(MPSSE.READ_WRITE_DATA, AsBitsBytes, WriteClkEdge, BitOrder, Tms)
        if AsBitsBytes == MPSSE.AS_BITS:
            Length = Data[0].bit_length() - 1
            self.__Ft232.write(bytes([Command, Length]))
        elif AsBitsBytes == MPSSE.AS_BYTES:
            Length = len(Data) - 1
            LengthLowByte = Length & 0xff
            LengthHighByte = (Length >> 8) & 0xff
            self.__Ft232.write(bytes([Command, LengthLowByte, LengthHighByte]))
        self.__Ft232.write(bytes(Data))
        return self.__Ft232.read(ReadLength)