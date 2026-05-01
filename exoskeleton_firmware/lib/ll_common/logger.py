###############################
# This class helps to save data from a CAN bus onto a Pyboard.
#
#
# Author: Levi J Hargrove
# Date: Jan 18, 2020
###############################

import struct

print('IMPORTING LOGGER')


class Logger:
    # Initialization function. Set up variables, register callbacks, etc.
    def __init__(self, write_buffer, ftype):
        self._f_handle = None
        self.write_buffer = write_buffer
        self.len_write = len(write_buffer)
        self.write_bytearray = bytearray(self.len_write * 4)
        self.ftype = ftype
        self.FileOpen = False

    def openFile(self, f_name):
        print(f_name)
        self._f_handle = open(f_name, 'wb')
        self.FileOpen = True

    def writeData(self):
        if self.ftype == 1:
            for a in range(self.len_write):
                self._f_handle.write(struct.pack('i', self.write_buffer[a]))

        elif self.ftype == 2:
            for a in range(self.len_write):
                self._f_handle.write(struct.pack('f', self.write_buffer[a]))

    def checkWriting(self):
        return self._writing_data

    def closeFile(self):
        print(self._f_handle)
        if self.FileOpen:
            self._f_handle.close()
            self._f_handle = None
            self.FileOpen = False
