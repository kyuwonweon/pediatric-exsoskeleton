import pyb                                              # type: ignore
import stm                                              # type: ignore
from stm import mem16                                   # type: ignore
import micropython                                      # type: ignore
import sys


# -----------------------------------------------------------------------------
#
#  CLASS UPYSPI()
#
# -----------------------------------------------------------------------------
class upySPI(pyb.SPI):
    '''
    upySPI extends machine.SPI on pyboards and is intended to work as a drop-in
    replacement. The over-ridden methods can be called from ISRs with interrupt
    priorities that would result in crashes using the base class.
    '''

    # -------------------------------------------------------------------------
    #   __INIT__()
    # -------------------------------------------------------------------------
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        if sys.implementation._machine == 'Arduino Portenta H7 with STM32H747':
            self.portenta_h7 = True
        else:
            self.portenta_h7 = False

        self.BUS = args[0]

        if self.BUS == 1:
            self.STM_SPI = stm.SPI1
        elif self.BUS == 2:
            self.STM_SPI = stm.SPI2
        elif self.BUS == 3:
            self.STM_SPI = stm.SPI3
        elif self.BUS == 4:
            self.STM_SPI = stm.SPI4
        else:
            print('INVALID BUS SPECIFICATION')

        self.SPI_CR1 = self.STM_SPI + stm.SPI_CR1
        self.SPI_CR2 = self.STM_SPI + stm.SPI_CR2
        self.SPI_SR = self.STM_SPI + stm.SPI_SR

        # ----------------------------------------------------------------------
        #   STM32H747 (PORTENTA H7) SPECIFIC
        # ----------------------------------------------------------------------
        if self.portenta_h7:
            self.SPI_TXDR = self.STM_SPI + stm.SPI_TXDR
            self.SPI_RXDR = self.STM_SPI + stm.SPI_RXDR
            self.SPI_CFG1 = self.STM_SPI + stm.SPI_CFG1
            self.SPI_CFG2 = self.STM_SPI + stm.SPI_CFG2
            self.SPI_IFCR = self.STM_SPI + stm.SPI_IFCR

            self.readinto = self.readinto_portenta
            self.write = self.write_portenta
            self.write_readinto = self.write_readinto_portenta
            # self.soft_reset = self.soft_reset_portenta

            # ------------------------------------------------------------------
            #   Recover gracefully from soft reset
            # ------------------------------------------------------------------

            #   If SR[MODF]=1, then reset it using IFCR[MODFC]
            if stm.mem32[self.SPI_SR] & 0b1000000000:
                stm.mem32[self.SPI_IFCR] |= 0b1000000000

                #   Clear the CFG2[SSM] bit
                stm.mem32[self.SPI_CFG2] &= 0b11111011111111111111111111111111

        # ----------------------------------------------------------------------
        #   STM32F767 (PYBOARD D) SPECIFIC
        # ----------------------------------------------------------------------
        else:
            self.SPI_DR = self.STM_SPI + stm.SPI_DR

            self.readinto = self.readinto_pyboard
            self.write = self.write_pyboard
            self.write_readinto = self.write_readinto_pyboard
            self.soft_reset = self.soft_reset_pyboard

        # self.register_info_all(True)

        # create reference for debugging within ISR
        # self.register_info_all_reference = self.register_info_all

    # -------------------------------------------------------------------------
    #   READINTO_PORTENTA()
    # -------------------------------------------------------------------------
    @micropython.viper
    def readinto_portenta(self, read_buffer):
        '''
        readinto(write_buffer, read_buffer): full duplex transmission, sending
        write_buffer and receiving into read_buffer
        '''

        SPI_SR = self.SPI_SR
        SPI_CR1 = self.SPI_CR1

        ptr8_SPI_RXDR = ptr8(self.SPI_RXDR)                 # type: ignore
        ptr16_SPI_RXDR = ptr16(self.SPI_RXDR)               # type: ignore

        ptr8_SPI_TXDR = ptr8(self.SPI_TXDR)                 # type: ignore

        ptr16_SPI_SR = ptr16(SPI_SR)                        # type: ignore

        ptr16_SPI_CR1 = ptr16(SPI_CR1)                      # type: ignore

        ptr8_read_data = ptr8(read_buffer)                  # type: ignore

        ptr16_SPI_CR1[0] |= 0b1                     # enable SPE
        ptr16_SPI_CR1[0] |= 0b1000000000            # enable CSTART

        #
        #   EXCHANGE DATA THROUGH SPI BUS
        #
        for i in range(int(len(read_buffer))):
            while not (ptr16_SPI_SR[0] & 0b10):     # wait until SR[TXP]==1
                pass
            ptr8_SPI_TXDR[0] = 0

            while not (ptr16_SPI_SR[0] & 0b1):      # wait until SR[RXP]==1
                pass
            ptr8_read_data[i] = ptr8_SPI_RXDR[0]

        #
        #   SPI SHUTDOWN PROCEDURE
        #   (necessary for proper operation)
        #

        # wait until SR[TXE]=1
        while not(ptr16_SPI_SR[0] & 0b1000000000000):   # wait until SR[TXE]=1
            pass

        # read Rx buffer until RXWNE=0 and RXPLVL=00
        while (ptr16_SPI_SR[0] & 0b1110000000000000):
            ptr16_SPI_RXDR[0]

        # disable SPE by unsetting CR1[0]
        ptr16_SPI_CR1[0] &= 0b1111111111111110

        return

    # -------------------------------------------------------------------------
    #   WRITE_PORTENTA()
    # -------------------------------------------------------------------------
    @micropython.viper
    def write_portenta(self, write_buffer):
        '''
        readinto(write_buffer, read_buffer): full duplex transmission, sending
        write_buffer and receiving into read_buffer
        '''

        SPI_SR = self.SPI_SR
        SPI_CR1 = self.SPI_CR1

        dev_null: int = 0

        ptr8_SPI_RXDR = ptr8(self.SPI_RXDR)                 # type: ignore
        ptr16_SPI_RXDR = ptr16(self.SPI_RXDR)               # type: ignore

        ptr8_SPI_TXDR = ptr8(self.SPI_TXDR)                 # type: ignore

        ptr16_SPI_SR = ptr16(SPI_SR)                        # type: ignore

        ptr16_SPI_CR1 = ptr16(SPI_CR1)                      # type: ignore

        ptr8_write_data = ptr8(write_buffer)                # type: ignore

        ptr16_SPI_CR1[0] |= 0b1                 # enable SPE
        ptr16_SPI_CR1[0] |= 0b1000000000        # enable CSTART

        #
        #   EXCHANGE DATA THROUGH SPI BUS
        #
        # self.register_info_all(True)
        for i in range(int(len(write_buffer))):
            while not (ptr16_SPI_SR[0] & 0b10):     # wait until SR[TXP]==1
                pass
            ptr8_SPI_TXDR[0] = ptr8_write_data[i]

            while not (ptr16_SPI_SR[0] & 0b1):      # wait until SR[RXP]==1
                pass
            dev_null = ptr8_SPI_RXDR[0]

        #
        #   SPI SHUTDOWN PROCEDURE
        #   (necessary for proper operation)
        #

        while not(ptr16_SPI_SR[0] & 0b1000000000000):   # wait until SR[TXE]=1
            pass

        # read Rx buffer until RXWNE=0 and RXPLVL=00
        while (ptr16_SPI_SR[0] & 0b1110000000000000):
            ptr16_SPI_RXDR[0]

        # disable SPE by unsetting CR1[0]
        ptr16_SPI_CR1[0] &= 0b1111111111111110

        return

    # -------------------------------------------------------------------------
    #   WRITE_READINTO_PORTENTA()
    # -------------------------------------------------------------------------
    @micropython.viper
    def write_readinto_portenta(self, write_buffer, read_buffer):
        '''
        readinto(write_buffer, read_buffer): full duplex transmission, sending
        write_buffer and receiving into read_buffer
        '''

        SPI_RXDR = self.SPI_RXDR
        SPI_TXDR = self.SPI_TXDR
        SPI_SR = self.SPI_SR
        SPI_CR1 = self.SPI_CR1

        ptr8_SPI_RXDR = ptr8(SPI_RXDR)                      # type: ignore
        ptr8_SPI_TXDR = ptr8(SPI_TXDR)                      # type: ignore
        ptr16_SPI_RXDR = ptr16(SPI_RXDR)                    # type: ignore

        ptr16_SPI_SR = ptr16(SPI_SR)                        # type: ignore

        ptr16_SPI_CR1 = ptr16(SPI_CR1)                      # type: ignore

        ptr8_write_data = ptr8(write_buffer)                # type: ignore
        ptr8_read_data = ptr8(read_buffer)                  # type: ignore

        # enable SPI by setting SPE
        ptr16_SPI_CR1[0] |= 0b1
        ptr16_SPI_CR1[0] |= 0b1000000000            # enable CSTART

        #
        #   EXCHANGE DATA THROUGH SPI BUS
        #
        for i in range(int(len(write_buffer))):
            while not (ptr16_SPI_SR[0] & 0b10):     # wait until SR[TXP]==1
                pass
            ptr8_SPI_TXDR[0] = ptr8_write_data[i]

            while not (ptr16_SPI_SR[0] & 0b1):      # wait until SR[RXP]==1
                pass
            ptr8_read_data[i] = ptr8_SPI_RXDR[0]

        #
        #   SPI SHUTDOWN PROCEDURE
        #   (necessary for proper operation)
        #

        while not(ptr16_SPI_SR[0] & 0b1000000000000):   # wait until SR[TXE]=1
            pass

        # read Rx buffer until RXWNE=0 and RXPLVL=00
        while (ptr16_SPI_SR[0] & 0b1110000000000000):
            ptr16_SPI_RXDR[0]

        # disable SPE by unsetting CR1[0]
        ptr16_SPI_CR1[0] &= 0b1111111111111110

        return

    # -------------------------------------------------------------------------
    #   SOFT_RESET_PORTENTA()
    # -------------------------------------------------------------------------
    @micropython.viper
    def soft_reset_portenta(self):
        '''
        Soft reset SPI peripheral on Portenta H7
        '''
        SPI_RXDR = self.SPI_RXDR
        SPI_SR = self.SPI_SR
        SPI_CR1 = self.SPI_CR1
        SPI_CFG2 = self.SPI_CFG2
        SPI_IFCR = self.SPI_IFCR

        ptr16_SPI_RXDR = ptr16(SPI_RXDR)                    # type: ignore
        ptr16_SPI_SR = ptr16(SPI_SR)                        # type: ignore
        ptr16_SPI_CR1 = ptr16(SPI_CR1)                      # type: ignore
        ptr16_SPI_IFCR = ptr16(SPI_IFCR)               # type: ignore

        while not(ptr16_SPI_SR[0] & 0b1000000000000):   # wait until SR[TXE]=1
            pass

        # read Rx buffer until RXWNE=0 and RXPLVL=00
        while (ptr16_SPI_SR[0] & 0b1110000000000000):
            ptr16_SPI_RXDR[0]

        # # disable SPE by unsetting CR1[0]
        # ptr16_SPI_CR1[0] &= 0b1111111111111110

        # #   If SR[MODF]=1, then reset it using IFCR[MODFC]
        # if ptr16_SPI_SR[0] & 0b1000000000:
        #     ptr16_SPI_IFCR[0] |= 0b1000000000

        #     #   Clear the CFG2[SSM] bit
        #     ptr16_SPI_IFCR[0] &= 0b11111011111111111111111111111111
        return

    # -------------------------------------------------------------------------
    #   READINTO_PYBOARD()
    # -------------------------------------------------------------------------
    @micropython.viper
    def readinto_pyboard(self, read_buffer):
        '''
        readinto(write_buffer, read_buffer): full duplex transmission, sending
        write_buffer and receiving into read_buffer
        '''

        SPI_DR = self.SPI_DR
        SPI_SR = self.SPI_SR
        SPI_CR1 = self.SPI_CR1

        ptr8_SPI_DR = ptr8(SPI_DR)                          # type: ignore
        ptr16_SPI_DR = ptr16(SPI_DR)                        # type: ignore

        ptr16_SPI_SR = ptr16(SPI_SR)                        # type: ignore

        ptr16_SPI_CR1 = ptr16(SPI_CR1)                      # type: ignore

        ptr8_read_data = ptr8(read_buffer)                  # type: ignore

        # enable SPI by setting SPE
        ptr16_SPI_CR1[0] |= 0b1000000

        #
        #   EXCHANGE DATA THROUGH SPI BUS
        #
        for i in range(int(len(read_buffer))):
            while not (ptr16_SPI_SR[0] & 0b10):     # wait until SR[TXE]==1
                pass
            ptr8_SPI_DR[0] = 0

            while not (ptr16_SPI_SR[0] & 0b1):      # wait until SR[RXNE]==1
                pass
            ptr8_read_data[i] = ptr8_SPI_DR[0]

        #
        #   SPI SHUTDOWN PROCEDURE
        #   (necessary for proper operation)
        #

        # wait until FTLVL=0
        while True:
            if not (ptr16_SPI_SR[0] & 0b1100000000000):
                break

        # wait until BSY=0
        while True:
            if not (ptr16_SPI_SR[0] & 0b10000000):
                break

        # disable SPE by unsetting CR1[6]
        ptr16_SPI_CR1[0] &= 0b1111111110111111

        # read Rx buffer until FRLVL=0
        while (ptr16_SPI_SR[0] & 0b11000000000):
            ptr16_SPI_DR[0]

        return

    # -------------------------------------------------------------------------
    #   WRITE_PYBOARD()
    # -------------------------------------------------------------------------
    @micropython.viper
    def write_pyboard(self, write_buffer):
        '''
        readinto(write_buffer, read_buffer): full duplex transmission, sending
        write_buffer and receiving into read_buffer
        '''

        SPI_DR = self.SPI_DR
        SPI_SR = self.SPI_SR
        SPI_CR1 = self.SPI_CR1

        dev_null: int = 0

        ptr8_SPI_DR = ptr8(SPI_DR)                          # type: ignore
        ptr16_SPI_DR = ptr16(SPI_DR)                        # type: ignore

        ptr16_SPI_SR = ptr16(SPI_SR)                        # type: ignore

        ptr16_SPI_CR1 = ptr16(SPI_CR1)                      # type: ignore

        ptr8_write_data = ptr8(write_buffer)                # type: ignore

        # enable SPI by setting SPE
        ptr16_SPI_CR1[0] |= 0b1000000

        #
        #   EXCHANGE DATA THROUGH SPI BUS
        #
        for i in range(int(len(write_buffer))):
            while not (ptr16_SPI_SR[0] & 0b10):     # wait until SR[TXE]==1
                pass
            ptr8_SPI_DR[0] = ptr8_write_data[i]

            while not (ptr16_SPI_SR[0] & 0b1):      # wait until SR[RXNE]==1
                pass
            dev_null = ptr8_SPI_DR[0]

        #
        #   SPI SHUTDOWN PROCEDURE
        #   (necessary for proper operation)
        #

        # wait until FTLVL=0
        while True:
            if not (ptr16_SPI_SR[0] & 0b1100000000000):
                break

        # wait until BSY=0
        while True:
            if not (ptr16_SPI_SR[0] & 0b10000000):
                break

        # disable SPE by unsetting CR1[6]
        ptr16_SPI_CR1[0] &= 0b1111111110111111

        # read Rx buffer until FRLVL=0
        while (ptr16_SPI_SR[0] & 0b11000000000):
            ptr16_SPI_DR[0]

        return

    # -------------------------------------------------------------------------
    #   WRITE_READINTO_PYBOARD()
    # -------------------------------------------------------------------------
    @micropython.viper
    def write_readinto_pyboard(self, write_buffer, read_buffer):
        '''
        readinto(write_buffer, read_buffer): full duplex transmission, sending
        write_buffer and receiving into read_buffer
        '''

        SPI_DR = self.SPI_DR
        SPI_SR = self.SPI_SR
        SPI_CR1 = self.SPI_CR1

        ptr8_SPI_DR = ptr8(SPI_DR)                          # type: ignore
        ptr16_SPI_DR = ptr16(SPI_DR)                        # type: ignore

        ptr16_SPI_SR = ptr16(SPI_SR)                        # type: ignore

        ptr16_SPI_CR1 = ptr16(SPI_CR1)                      # type: ignore

        ptr8_write_data = ptr8(write_buffer)                # type: ignore
        ptr8_read_data = ptr8(read_buffer)                  # type: ignore

        # enable SPI by setting SPE
        ptr16_SPI_CR1[0] |= 0b1000000

        #
        #   EXCHANGE DATA THROUGH SPI BUS
        #
        for i in range(int(len(write_buffer))):
            while not (ptr16_SPI_SR[0] & 0b10):     # wait until SR[TXE]==1
                pass
            ptr8_SPI_DR[0] = ptr8_write_data[i]

            while not (ptr16_SPI_SR[0] & 0b1):      # wait until SR[RXNE]==1
                pass
            ptr8_read_data[i] = ptr8_SPI_DR[0]

        #
        #   SPI SHUTDOWN PROCEDURE
        #   (necessary for proper operation)
        #

        # wait until FTLVL=0
        while True:
            if not (ptr16_SPI_SR[0] & 0b1100000000000):
                break

        # wait until BSY=0
        while True:
            if not (ptr16_SPI_SR[0] & 0b10000000):
                break

        # disable SPE by unsetting CR1[6]
        ptr16_SPI_CR1[0] &= 0b1111111110111111

        # read Rx buffer until FRLVL=0
        while (ptr16_SPI_SR[0] & 0b11000000000):
            ptr16_SPI_DR[0]

        return

    # -------------------------------------------------------------------------
    #   SOFT_RESET_PYBOARD()
    # -------------------------------------------------------------------------
    def soft_reset_pyboard(self):
        '''
        Soft reset SPI peripheral on Pyboard D
        '''
        ptr16_SPI_DR = ptr16(SPI_DR)                        # type: ignore
        ptr16_SPI_SR = ptr16(SPI_SR)                        # type: ignore
        ptr16_SPI_CR1 = ptr16(SPI_CR1)                      # type: ignore

        # wait until FTLVL=0
        while True:
            if not (ptr16_SPI_SR[0] & 0b1100000000000):
                break

        # wait until BSY=0
        while True:
            if not (ptr16_SPI_SR[0] & 0b10000000):
                break

        # disable SPE by unsetting CR1[6]
        ptr16_SPI_CR1[0] &= 0b1111111110111111

        # read Rx buffer until FRLVL=0
        while (ptr16_SPI_SR[0] & 0b11000000000):
            ptr16_SPI_DR[0]

        return

    # -------------------------------------------------------------------------
    #   REGISTER_INFO_ALL()
    # -------------------------------------------------------------------------
    def register_info_all(self, _):
        if self.portenta_h7:
            self.register_info('CR1', True)
            self.register_info('CR2', True)
            self.register_info('SR', True)
            self.register_info('CFG1', True)
            self.register_info('CFG2', True)
        else:
            self.register_info('CR1', True)
            self.register_info('CR2', True)
            self.register_info('SR', True)

    # -------------------------------------------------------------------------
    #   REGISTER_INFO()
    # -------------------------------------------------------------------------
    def register_info(self, ID, verbose=False):
        if self.portenta_h7:
            if ID == 'CR1':
                value = stm.mem32[self.SPI_CR1]
                output = {
                    'SPE': value & 0b1,
                    'MASRX': (value >> 8) & 0b1,
                    'CSTART': (value >> 9) & 0b1,
                    'HDDIR': (value >> 11) & 0b1,
                    'SSI': (value >> 12) & 0b1,
                    'CRC33_17': (value >> 13) & 0b1,
                    'RCRCINI': (value >> 14) & 0b1,
                    'TCRCINI': (value >> 15) & 0b1,
                    'IOLOCK': (value >> 16) & 0b1
                }
            elif ID == 'CR2':
                value = stm.mem32[self.SPI_CR2]
                output = {
                    'TSIZE': value & 0xFFFF,
                    'TSER': (value >> 16) & 0xFFFF
                }
            elif ID == 'SR':
                value = stm.mem32[self.SPI_SR]
                output = {
                    'RXP': value & 0b1,
                    'TXP': (value >> 1) & 0b1,
                    'DXP': (value >> 2) & 0b1,
                    'EOT': (value >> 3) & 0b1,
                    'TXTF': (value >> 4) & 0b1,
                    'UDR': (value >> 5) & 0b1,
                    'OVR': (value >> 6) & 0b1,
                    'CRCE': (value >> 7) & 0b1,
                    'TIFRE': (value >> 8) & 0b1,
                    'MODF': (value >> 9) & 0b1,
                    'TSERF': (value >> 10) & 0b1,
                    'SUSP': (value >> 11) & 0b1,
                    'TXC': (value >> 12) & 0b1,
                    'RXPLVL': (value >> 13) & 0b11,
                    'RXWNE': (value >> 15) & 0b1,
                    'CTSIZE': (value >> 16) & 0xFFFF
                }
            elif ID == 'CFG1':
                value = stm.mem32[self.SPI_CFG1]
                output = {
                    'DSIZE': value & 0b11111,
                    'FTHLV': (value >> 5) & 0b1111,
                    'UDRCFG': (value >> 9) & 0b11,
                    'UDRDET': (value >> 11) & 0b11,
                    'RXDMAEN': (value >> 14) & 0b1,
                    'TXDMAEN': (value >> 15) & 0b1,
                    'CRCSIZE': (value >> 16) & 0b11111,
                    'CRCEN': (value >> 22) & 0b1,
                    'MBR': (value >> 28) & 0b111
                }
            elif ID == 'CFG2':
                value = stm.mem32[self.SPI_CFG2]
                output = {
                    'MSSI': value & 0b1111,
                    'MIDI': (value >> 4) & 0b1111,
                    'IOSWP': (value >> 15) & 0b1,
                    'COMM': (value >> 17) & 0b11,
                    'SP': (value >> 19) & 0b111,
                    'MASTER': (value >> 22) & 0b1,
                    'LSBFRST': (value >> 23) & 0b1,
                    'CPHA': (value >> 24) & 0b1,
                    'CPOL': (value >> 25) & 0b1,
                    'SSM': (value >> 26) & 0b1,
                    'SSIOP': (value >> 28) & 0b1,
                    'SSOE': (value >> 29) & 0b1,
                    'SSOM': (value >> 30) & 0b1,
                    'AFCNTR': (value >> 31) & 0b1
                }
            else:
                print('INVALID REGISTER ID')
                return

        else:
            if ID == 'CR1':
                value = stm.mem16[self.SPI_CR1]
                output = {
                    'CPHA': value & 0b1,
                    'CPOL': (value >> 1) & 0b1,
                    'MSTR': (value >> 2) & 0b1,
                    'BR': (value >> 3) & 0b111,
                    'SPE': (value >> 6) & 0b1,
                    'LSBFIRST': (value >> 7) & 0b1,
                    'SSI': (value >> 8) & 0b1,
                    'SSM': (value >> 9) & 0b1,
                    'RXONLY': (value >> 10) & 0b1,
                    'CRCL': (value >> 11) & 0b1,
                    'CRCNEXT': (value >> 12) & 0b1,
                    'CRCEN': (value >> 13) & 0b1,
                    'BIDIOE': (value >> 14) & 0b1,
                    'BIDIMODE': (value >> 15) & 0b1
                }
            elif ID == 'CR2':
                value = stm.mem16[self.SPI_CR2]
                output = {
                    'RXDMAEN': value & 0b1,
                    'TXDMAEN': (value >> 1) & 0b1,
                    'SSOE': (value >> 2) & 0b1,
                    'NSSP': (value >> 3) & 0b1,
                    'FRF': (value >> 4) & 0b1,
                    'ERRIE': (value >> 5) & 0b1,
                    'RXNEIE': (value >> 6) & 0b1,
                    'TXEIE': (value >> 7) & 0b1,
                    'DS': (value >> 8) & 0b1111,
                    'FRXTH': (value >> 12) & 0b1,
                    'LDMA_RX': (value >> 13) & 0b1,
                    'LDMA_TX': (value >> 14) & 0b1
                }
            elif ID == 'SR':
                value = stm.mem16[self.SPI_SR]
                output = {
                    'RXNE': value & 0b1,
                    'TXE': (value >> 1) & 0b1,
                    'CHSIDE': (value >> 2) & 0b1,
                    'UDR': (value >> 3) & 0b1,
                    'CRCERROR': (value >> 4) & 0b1,
                    'MODF': (value >> 5) & 0b1,
                    'OVR': (value >> 6) & 0b1,
                    'BSY': (value >> 7) & 0b1,
                    'FRE': (value >> 8) & 0b1,
                    'FRLVL': (value >> 9) & 0b11,
                    'FTLVL': (value >> 11) & 0b11
                }
            else:
                print('INVALID REGISTER ID')
                return

        if verbose:
            print('-------------')
            print('REGISTER:', ID)
            print(hex(value))
            print('-------------')

            for key, value in output.items():
                print(('{:<10}:{}').format(key, value))
        return output
