import pyb                                  # type: ignore

print('IMPORTING LPS25HB')


class LPS25HB:
    def __init__(self, i2c, address=0x5C):
        self._i2c = i2c
        self._address = address
        self._buf = bytearray(1)
        self._buf[0] = 0xC0                 # Set to active, with internal sample rate of 25 Hz (max)
        self._i2c.writeto_mem(self._address, 0x20, self._buf)
        pyb.delay(5)
        self._buf[0] = 0x40                 # Enable FIFO
        self._i2c.writeto_mem(self._address, 0x21, self._buf)
        pyb.delay(5)
        self._buf[0] = 0xDF                 # Moving average of of 32
        self._i2c.writeto_mem(self._address, 0x2E, self._buf)
        pyb.delay(5)
        self._rbuff = bytearray(3)
        self._convert_press = 0.0

    def readPress(self):
        self._i2c.readfrom_mem_into(self._address, 0x28 | 0x80, self._rbuff)
        # [print('0x28=',self._rbuff[2], self._rbuff[1],self._rbuff[0])]
        self._convert_press = ((self._rbuff[2] << 16) | (self._rbuff[1] << 8) | (self._rbuff[0]))
        # print('pressure=', pressure)
        # tmp = ((self._rbuff[2] * 65536 + self._rbuff[1] * 256 + self._rbuff[0]) & 0x01ffe0) >> 5
        return self._convert_press
