import micropython                                          # type: ignore
import uctypes                                              # type: ignore
import stm                                                  # type: ignore
import array


# -------------------------------------------------------------------------
#
#   upyFloat CLASS
#
# -------------------------------------------------------------------------
class upyFloat():
    '''
    upyFloat(value=0.0) encapsulates a single float. Its value can be read or
    updated via the following attributes for which getters and setters are
    provided, as below (note that changing the value of the float being
    represented always modifies the same four bytes in memory):

    attribute name      [type] description
    --------------------------------------------------------------------------
    val                 [float]

    adr                 [int] memory address of the raw byte representation
                        as stored in single element float array. Use should
                        probably be limited to getting and not setting

    raw_bytearray       [bytearray(4)] raw bytes representation

    raw_bytes_as_int    [int] raw bytes representation as int


    Examples:

    >> foo = upyFloat(1.2)
    >> foo.val
    1.2

    >> bar = foo.val * 2.4
    >> bar
    2.88

    >> assembler_functions.f_mult(foo.adr, foo.adr, foo.adr)
    >> foo.val
    1.44

    >> foo.raw_bytearray
    bytearray(b'\xe8Q\xb8?')
    >> spi.write(foo.raw_bytearray)         # send value over SPI

    >> baz = array.array('f', [0.0])
    >> stm.mem32[uctypes.addressof(baz)] = foo.raw_bytes_as_int
    >> baz
    array('f', [1.44])

    '''

    # -------------------------------------------------------------------------
    #
    #   upyFloat METHODS
    #
    # -------------------------------------------------------------------------

    # ------------
    #   __init__()
    # ------------
    @micropython.native
    def __init__(self, value=0.0) -> None:

        self.__float_array = array.array('f', [value])
        self._float_array_address: int = uctypes.addressof(self.__float_array)

        self._raw_bytearray: bytearray = bytearray(4)
        self._raw_bytearray_address: int = (
            uctypes.addressof(self._raw_bytearray)
        )

        stm.mem32[self._raw_bytearray_address] = (
            stm.mem32[self._float_array_address]
        )

    # -------------------------------------------------------------------------
    #
    #   upyFloat PROPERTIES
    #
    # -------------------------------------------------------------------------

    # --------------
    #   upyFloat.val
    # --------------

    @micropython.native
    @property
    def val(self) -> float:
        return self.__float_array[0]

    @micropython.native
    @val.setter
    def val(self, input: float) -> None:
        self.__float_array[0] = input

    # --------------
    #   upyFloat.adr
    # --------------

    @micropython.native
    @property
    def adr(self) -> int:
        return self._float_array_address

    @micropython.native
    @adr.setter
    def adr(self, input: int) -> None:
        self._float_array_address = input

    # ------------------------
    #   upyFloat.raw_bytearray
    # ------------------------

    @micropython.native
    @property
    def raw_bytearray(self) -> bytearray:
        # stm.mem32[self._raw_bytearray_address] = (
        #     stm.mem32[self._float_array_address]
        # )
        # return self._raw_bytearray

        int = stm.mem32[self._float_array_address]
        arr = self._raw_bytearray

        arr[0] = int & 0xFF
        arr[1] = (int >> 8) & 0xFF
        arr[2] = (int >> 16) & 0xFF
        arr[3] = (int >> 24) & 0xFF

        return arr

    @micropython.native
    @raw_bytearray.setter
    def raw_bytearray(self, input: bytearray) -> None:
        address = self._float_array_address
        stm.mem8[address] = input[0]
        stm.mem8[address + 1] = input[1]
        stm.mem8[address + 2] = input[2]
        stm.mem8[address + 3] = input[3]

    # ---------------------------
    #   upyFloat.raw_bytes_as_int
    # ---------------------------
    '''
    Commented out because this allocates heap for ints that use more than 30
    bits (which includes all negative float values)
    '''
    # @micropython.native
    # @property
    # def raw_bytes_as_int(self) -> int:
    #     return stm.mem32[self._float_array_address]

    # @micropython.native
    # @raw_bytes_as_int.setter
    # def raw_bytes_as_int(self, input: int) -> None:
    #     # stm.mem32[self._float_array_address] = input
    #     address = self._float_array_address
    #     stm.mem8[address] = input & 0xFF
    #     stm.mem8[address + 1] = (input >> 8) & 0xFF
    #     stm.mem8[address + 2] = (input >> 16) & 0xFF
    #     stm.mem8[address + 3] = (input >> 24) & 0xFF
