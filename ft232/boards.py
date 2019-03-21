from ft232 import Ft232

class FT232H(Ft232):
    D0 = 0
    D1 = 1
    D2 = 2
    D3 = 3
    D4 = 4
    D5 = 5
    D6 = 6
    D7 = 7
    PortD = [D0, D1, D2, D3, D4, D5, D6, D7]
    
    C0 = 8
    C1 = 9
    C2 = 10
    C3 = 11
    C4 = 12
    C5 = 13
    C6 = 14
    C7 = 15
    PortC = [C0, C1, C2, C3, C4, C5, C6, C7]

    MPSSE_CLOCK = 60e6

class FT2232H(Ft232):
    D0 = 0
    D1 = 1
    D2 = 2
    D3 = 3
    D4 = 4
    D5 = 5
    D6 = 6
    D7 = 7
    PortD = [D0, D1, D2, D3, D4, D5, D6, D7]
    
    C0 = 8
    C1 = 9
    C2 = 10
    C3 = 11
    C4 = 12
    C5 = 13
    C6 = 14
    C7 = 15
    PortC = [C0, C1, C2, C3, C4, C5, C6, C7]

    MPSSE_CLOCK = 60e6

class FT2232D(Ft232):
    D0 = 0
    D1 = 1
    D2 = 2
    D3 = 3
    D4 = 4
    D5 = 5
    D6 = 6
    D7 = 7
    PortD = [D0, D1, D2, D3, D4, D5, D6, D7]
    
    C0 = 8
    C1 = 9
    C2 = 10
    C3 = 11
    PortC = [C0, C1, C2, C3]

    MPSSE_CLOCK = 12e6

class FT4232H(Ft232):
    D0 = 0
    D1 = 1
    D2 = 2
    D3 = 3
    D4 = 4
    D5 = 5
    D6 = 6
    D7 = 7
    PortD = [D0, D1, D2, D3, D4, D5, D6, D7]
    PortC = []

    MPSSE_CLOCK = 60e6