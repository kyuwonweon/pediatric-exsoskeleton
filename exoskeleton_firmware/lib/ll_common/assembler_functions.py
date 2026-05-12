####################################################################################################
# @file             assembler_functions.py
#
# @details          The is a collection of assembler functions mostly created by Levi Hargrove
#                   that can be used with a pyboard
#
# @author           Frank Ursetta and Levi J Hargrove
#
# @date             02/08/2021
####################################################################################################

# Ignore type-checking this file, as nearly every line has an undefined reference (see pep8.org, and PEP-484)
# type: ignore

print('IMPORTING ASSEMBLER FUNCTIONS')


# ==================================================================================================
#   MATH UTILITY FUNCTIONS
# --------------------------------------------------------------------------------------------------
# --------------------------------------------------------------------------------------------------


####################################################################################################
# @brief            Multiplies two floating point numbers
#
# @details          Equivalent form: r0 = r1 * r2
#                   Example:
#                           res = array('f', [0.0])
#                           x = array('f', [2.5])
#                           y = array('f', [0.5])
#                           f_mult(res, x, y)
#                           print(res)
#                               >>> array('f', [1.25])
#
# @param[out]       r0                  Product of [r1] and [r2]; Floating point array of size 1
# @param[in]        r1                  Multiplicand; Floating point array of size 1
# @param[in]        r2                  Multiplier; Floating point array of size 1
#
# @warning          Parameters must be passed in as arrays of size 1 for this function to work
####################################################################################################


@micropython.asm_thumb
def f_mult(r0, r1, r2):
    vldr(s0, [r1, 0])
    vldr(s1, [r2, 0])
    vmul(s1, s1, s0)
    vstr(s1, [r0, 0])

####################################################################################################
# @brief            Calculates log(x) using Taylor Series approximation
#
# @param[in,out]    r0                  Floating point array consisting of Taylor series expansion
#                                       coefficients for log calculation.
#                                       Output is returned at r0[0].
# @param[in]        r1                  Integer array holding the initial barometer reading
#                                       and the current reading to convert
####################################################################################################


@micropython.asm_thumb
def ass_log(r0, r1):
    vldr(s0, [r1, 0])                   # barom_init
    vcvt_f32_s32(s0, s0)                # float(barom_init)
    vldr(s1, [r1, 4])                   # barom_last
    vcvt_f32_s32(s1, s1)                # float(barom_last)
    vdiv(s0, s1, s0)                    # x = barom_last/barom_init
    vldr(s1, [r0, 20])                  # load -1
    vsub(s0, s0, s1)                    # s0 = x-1
    vmul(s1, s0, s0)                    # (x-1)^2
    vmul(s2, s1, s0)                    # (x-1)^3
    vmul(s3, s2, s0)                    # (x-1)^4
    vldr(s4, [r0, 4])                   # .5
    vldr(s5, [r0, 8])                   # 1/3
    vldr(s6, [r0, 12])                  # .25
    vmul(s7, s4, s1)
    vmul(s8, s5, s2)
    vmul(s9, s6, s3)
    vsub(s0, s0, s7)
    vadd(s0, s0, s8)
    vsub(s0, s0, s9)
    vldr(s10, [r0, 16])
    vdiv(s0, s0, s10)
    vstr(s0, [r0, 0])


####################################################################################################
# @brief            Multiplies two floating point numbers and converts the product to an integer
#
# @details          Equivalent form: r0[0] = int(r1[0] * r2[0])
#
# @param[out]       r0                  Product of [r1] and [r2]; Integer array of size 1
# @param[in]        r1                  Multiplicand; Floating point array of size 1
# @param[in]        r2                  Multiplier; Floating point array of size 1
#
# @warning          Parameters must be passed in as arrays of size 1 for this function to work
####################################################################################################


@micropython.asm_thumb
def f_mult_and_round(r0, r1, r2):
    vldr(s0, [r1, 0])
    vldr(s1, [r2, 0])
    vmul(s1, s1, s0)
    vcvt_s32_f32(s1, s1)
    vstr(s1, [r0, 0])


####################################################################################################
# @brief            Shifts and scales the input value to the appropriate zero centerpoint and range
#
# @details          Equivalent form: r0 = (r3 - r1) / r2
#
# @param[out]       r0                  Shifted and scaled output; Floating point array of size 1
# @param[in]        r1                  Zero offset of the value; Floating point array of size 1
# @param[in]        r2                  Gain of the value; Floating point array of size 1
# @param[in]        r3                  Value to be scaled; Passed as an integer
#
# @warning          Parameters [r0, r1, r2] must be passed in as arrays of size 1
####################################################################################################


@micropython.asm_thumb
def zero_and_scale(r0, r1, r2, r3):
    vmov(s0, r3)                        # s0 = r3
    vcvt_f32_s32(s0, s0)                # s0 = float(s0)
    vldr(s1, [r1, 0])                   # s1 = r1[0], which is a float
    vldr(s2, [r2, 0])                   # s2 = r2[0], which is a float
    vsub(s0, s0, s1)                    # s0 = s0 - s1
    vdiv(s0, s0, s2)                    # s0 = s0 / s2
    vstr(s0, [r0, 0])                   # r0[0] = s0


# ==================================================================================================
# ARRAY FUNCTIONS
# --------------------------------------------------------------------------------------------------
# --------------------------------------------------------------------------------------------------


####################################################################################################
# @brief            Returns the 1st element of an array as an integer
#
# @details          Equivalent form: r0 = int(r1[0]); return r0
#
# @param[out]       r0                  Placeholder input and should be placed as 0
#                                       Value placed in this register is returned by the function
# @param[in]        r1                  Floating point array; only the 1st element is accessed
#
# @returns          Assembler functions return the output placed in the r0 register.
#                   This function returns the converted float.
####################################################################################################


@micropython.asm_thumb
def cast_integer(r0, r1):
    vldr(s0, [r1, 0])                   # s0 = r1[0]
    vcvt_s32_f32(s1, s0)                # s1 = int(s0)
    vmov(r0, s1)                        # r0 = s1


####################################################################################################
# @brief            Copies the data stored in r0[r1] and store it in r2[r3]
#
# @details          Equivalent form: r2[r3] = r0[r1]
#
# @param[in]        r0                  Floating point array to copy from
# @param[in]        r1                  Index in bytes / memory address of the target element
#                                       inside the array
# @param[out]       r2                  Floating point array to paste the value to
# @param[out]       r3                  Index in bytes / memory address of where in the array the
#                                       copied value should be pasted to
####################################################################################################


@micropython.asm_thumb
def mov_from_to(r0, r1, r2, r3):
    add(r0, r0, r1)                     # r0 = r0 + r1 where r1 is an index
    add(r2, r2, r3)                     # r2 = r2 + r3 where r3 is an index
    vldr(s0, [r0, 0])                   # s0 = r0 the data stored at the r0 + r1 location
    vstr(s0, [r2, 0])                   # at the r2 + r3 location store the data from the r0 + r1


####################################################################################################
# @brief            Converts an integer to a floating point number
#
# @details          Equivalent form: r1[0] = float(r0)
#
# @param[in]        r0                  Integer value to be converted
# @param[out]       r1                  Contains the converted float value; only the
#                                       first element is accessed in this floating point array
#
# @note             If r1 is an array of size > 1, the converted value will still be placed in its
#                   1st element (i.e. r1[0])
####################################################################################################


@micropython.asm_thumb
def int_to_float_arr(r0, r1):
    vmov(s0, r0)
    vcvt_f32_s32(s0, s0)
    vstr(s0, [r1, 0])


####################################################################################################
# @brief            Converts an integer inside an array to a floating point number
#
# @details          Equivalent form: r1[0] = float(r0[0])
#
# @param[in]        r0                  Contains the integer value to be converted; only the
#                                       first element is accessed in this integer array
# @param[out]       r1                  Contains the converted float value; only the
#                                       first element is accessed in this floating point array
#
# @note             This function accesses the 1st element of both arrays even when array size > 1
####################################################################################################


@micropython.asm_thumb
def buff_to_float_arr(r0, r1):
    vldr(s0, [r0, 0])
    vcvt_f32_s32(s0, s0)
    vstr(s0, [r1, 0])


# ==================================================================================================
# SPECIFIC TASK FUNCTIONS
# --------------------------------------------------------------------------------------------------
# --------------------------------------------------------------------------------------------------


####################################################################################################
# @brief            Calculates the angular velocity of the knee joint
#
# @details          Equivalent form: r2[0] = r1[0] / r0[0]. Knee joint speed is calculated based
#                   on the current motor velocity and current transmission ratio
#
# @param[in]        r0                  Transmission ratio of the device; Float array
# @param[in]        r1                  Motor velocity array; Only 1st element is accessed r1[0]
# @param[out]       r2                  Calculated joint velocity of the knee; Float array
####################################################################################################


@micropython.asm_thumb
def compute_joint_velocity(r0, r1, r2):
    vldr(s0, [r0, 0])                   # Move the value of r0[0] (the current transmission ratio) into s0
    vldr(s1, [r1, 0])                   # Move the value in r1 offset by 0 into s1 (this value is the motor velocity before the transmission ratio)
    vdiv(s1, s1, s0)                    # Divide, s1 / s0, so divide by current motor velocity by the transmission ratio
    vstr(s1, [r2, 0])                   # Make the value in r2 offset by 0 equal to s1. Set r2[0]=velocity/transmission ratio


####################################################################################################
# @brief            Loads an 8-byte message from the buffer and stores it to its proper location
#
# @param[in]        r0                  Not used in this function; Placed in as 0
# @param[out]       r1                  Location to store the message; Bytearray of size 8
# @param[in]        r2                  Buffer that holds the data; Bytearray of size 8
####################################################################################################


@micropython.asm_thumb
def cpy_can(r0, r1, r2):
    ldr(r5, [r2, 0])                    # Loads the 1st 4 bytes of r2
    str(r5, [r1, 0])                    # Stores it to 1st half of r1
    ldr(r5, [r2, 4])                    # Loads the 2nd 4 bytes of r2
    str(r5, [r1, 4])                    # Stores it to 2nd half of r1


####################################################################################################
# @brief            Checks and caps the value if it exceeds the set lower or upper limit
#
# @details          Checks r1[0] against both r2 and r3, if in either case r1[0] exceeds the
#                   determined boundary value, r1[0] == r2 or r1[0] == r3.
#
# @param[in]        r0                  Not used as an input; Anything placed here will simply be
#                                       overwritten. Just place in 0.
# @param[in,out]    r1                  Contains the value to be checked; If the value exceeds the
#                                       limits in [r2,r3], it will be capped to the limit.
#                                       Floating point array size 1
# @param[in]        r2                  Lower limit; integer
# @param[in]        r3                  Upper limit; integer
####################################################################################################


@micropython.asm_thumb
def value_limit(r0, r1, r2, r3):
    vldr(s0, [r1, 0])                   # Grab the value to be checked: s0 = r1[0]
    vcvt_s32_f32(s1, s0)                # s1 = int(r1[0])
    vmov(r0, s1)                        # r0 = int(r1[0])
    cmp(r0, r3)                         # Compare the value (now in r0) to the upper limit (in r3)
    ittt(pl)                            # If value minus upper limit is positive, then follow next 3 lines:
    mov(r0, r3)                         # ### then1: cap the value to be the upper limit, r0 = r3
    vmov(s0, r3)                        # ### then2: s0 = r3
    vcvt_f32_s32(s0, s0)                # ### then3: s0 = float(r3)
    vcvt_s32_f32(s1, s0)                # s1 = int(s0)
    vmov(r0, s1)                        # r0 = int(s0)
    cmp(r0, r2)                         # Compare the value (now in r0) to the lower limit (in r2)
    ittt(mi)                            # If value minues lower limit is negative, then follow next 3 lines:
    mov(r0, r2)                         # ### then1: cap the value to be the lower limit, r0 = r2
    vmov(s0, r2)                        # ### then2: s0 = r2
    vcvt_f32_s32(s0, s0)                # ### then3: s0 = float(r2)
    vstr(s0, [r1, 0])                   # Move s0 to r1[0]; Value is now capped if it exceeded the limit


@micropython.asm_thumb
def and_fun_0xffffffff(r0):
    movwt(r1, 0xffffffff)
    and_(r0, r1)


@micropython.asm_thumb
def fast_caps_read_16bit(r0, r1, r2, r3):
    ldr(r5, [r3, 0])
    str(r5, [r0, 0])
    ldr(r5, [r3, 4])
    str(r5, [r0, 4])
    mov(r4, 8)
    add(r0, r0, 1)
    ldrb(r5, [r0, 0])
    lsl(r5, r5, 8)
    ldrb(r6, [r0, 1])
    add(r5, r5, r6)
    vmov(s0, r5)
    vcvt_f32_s32(s0, s0)
    vldr(s1, [r1, 0])
    vldr(s2, [r1, 4])
    vsub(s0, s0, s1)
    vdiv(s0, s0, s2)
    ldr(r7, [r2, 0])
    vstr(s0, [r7, 0])
    add(r2, r2, 4)
    add(r0, r0, 2)
    add(r1, r1, r4)
    ldrb(r5, [r0, 0])
    lsl(r5, r5, 8)
    ldrb(r6, [r0, 1])
    add(r5, r5, r6)
    vmov(s0, r5)
    vcvt_f32_s32(s0, s0)
    vldr(s1, [r1, 0])
    vldr(s2, [r1, 4])
    vsub(s0, s0, s1)
    vdiv(s0, s0, s2)
    ldr(r7, [r2, 0])
    vstr(s0, [r7, 0])
    add(r2, r2, 4)
    add(r0, r0, 2)
    add(r1, r1, r4)
    ldrb(r5, [r0, 0])
    lsl(r5, r5, 8)
    ldrb(r6, [r0, 1])
    add(r5, r5, r6)
    vmov(s0, r5)
    vcvt_f32_s32(s0, s0)
    vldr(s1, [r1, 0])
    vldr(s2, [r1, 4])
    vsub(s0, s0, s1)
    vdiv(s0, s0, s2)
    ldr(r7, [r2, 0])
    vstr(s0, [r7, 0])

# This function takes in a CAN message that has a sequence number, and 4 12 bit unsigned values
# It modifies them by the zero and gain for each channel, then loads them into the addresses
# specified in an array of ints in r2
@micropython.asm_thumb
def fast_caps_read_12bit_4chans(r0, r1, r2, r3):
    b(START0)                           # This branch puts a constant value in the impedance output
    label(READ2CHANS)
    ldrb(r5, [r0, 0])                   # load byte into r4
    lsl(r5, r5, 4)                      # shift it by 4 bits
    ldrb(r6, [r0, 1])                   # load the next byte
    lsr(r6, r6, 4)                      # get the nibble.
    add(r5, r5, r6)
    vmov(s0, r5)                        # Move to a floating point register
    vcvt_f32_s32(s0, s0)                # Convert to a floating point number
    vldr(s1, [r1, 0])                   # load the zero,
    vldr(s2, [r1, 4])                   # load the scale
    add(r1, 8)                          # Increment the storage register
    vsub(s0, s0, s1)                    # subtract the scale
    vdiv(s0, s0, s2)                    # divide by the gain
    ldr(r7, [r2, 0])                    # load the memory location to store the result
    vstr(s0, [r7, 0])                   # store the result
    add(r2, 4)                          # increment the storage register
    add(r0, 1)                          # increment to the byte
    ldrb(r5, [r0, 0])
    mov(r4, 0xF)
    and_(r5, r4)
    lsl(r5, r5, 8)
    add(r0, 1)
    ldrb(r6, [r0, 0])
    add(r5, r5, r6)
    vmov(s0, r5)
    vcvt_f32_s32(s0, s0)
    vldr(s1, [r1, 0])
    vldr(s2, [r1, 4])
    add(r1, 8)
    vsub(s0, s0, s1)
    vdiv(s0, s0, s2)
    ldr(r7, [r2, 0])
    vstr(s0, [r7, 0])
    add(r2, 4)
    add(r0, 1)
    bx(lr)
    label(START0)

    ldr(r5, [r3, 0])                    # Copy fbuf to the CAN message data
    str(r5, [r0, 0])
    ldr(r5, [r3, 4])
    str(r5, [r0, 4])
    add(r0, 1)                          # increment r0 byt 1 byte ignore sequence number
    bl(READ2CHANS)
    bl(READ2CHANS)

@micropython.asm_thumb
def fw_CAPS_u12bit(r0, r1, r2):
    ldr(r7, [r1, 0])
    vldr(s0, [r7, 0])
    vldr(s1, [r2, 0])
    vmul(s1, s1, s0)                    # Scaled value
    vcvt_s32_f32(s1, s1)                # Convert it to an int
    vmov(r4, s1)                        # Move it back to the fixed point registers in r4
    movw(r5, 0x7fff)                    # Load the value 0x7fff into r5
    add(r4, r4, r5)                     # Add 0x7fff to the scaled value
    movw(r5, 0xffff)                    # Load the value 0xffff into r5
    cmp(r4, r5)                         # See if the scaled int is larger than 0xffff
    it(gt)                              # if then
    movw(r4, 0xffff)                    # Overwrite r4 with 0xffff to cap it
    lsr(r4, r4, 4)                      # Shift the value to the right by 4 bits r4 = r4>>4
    mov(r3, r4)                         # Move a copy of the shifted value into r3. This is the 12 bit value
    lsr(r4, r4, 4)                      # Shift it a further 4 bits so we now have the MSB in r4
    strb(r4, [r0, 1])                   # Store this byte in the can.msg.data[1]
    ldrb(r5, [r0, 2])                   # load the 3rd byte of the can message into r5
    lsl(r3, r3, 4)                      # shift it left by 4 bits  r6 = r6 << 4
    mov(r7, 0xf0)                       # move 11110000 into r7
    and_(r3, r7)                        # and the value in r6 with r7 and store it in r6.
    mov(r7, 0x0f)                       # mov 00001111 into r7
    and_(r5, r7)                        # and the value in r5 with r7 and store it in r5
    orr(r5, r3)                         # or the values in r5 and r6
    strb(r5, [r0, 2])                   # store that byte in can message [2]

    ldr(r7, [r1, 4])
    vldr(s0, [r7, 0])
    vldr(s1, [r2, 4])
    vmul(s1, s1, s0)
    vcvt_s32_f32(s1, s1)
    vmov(r4, s1)
    movw(r5, 0x7fff)
    add(r4, r4, r5)
    movw(r5, 0xffff)
    cmp(r4, r5)
    it(gt)
    movw(r4, 0xffff)
    lsr(r4, r4, 4)
    mov(r3, r4)
    mov(r7, 0xff)
    and_(r3, r7)
    strb(r3, [r0, 3])
    ldrb(r5, [r0, 2])
    mov(r7, 0xf0)
    and_(r5, r7)
    lsr(r4, r4, 8)
    mov(r7, 0x0f)
    and_(r4, r7)
    orr(r5, r4)
    strb(r5, [r0, 2])

    add(r0, r0, 3)                      # make the pointer to the byte array increment by 3
    ldr(r7, [r1, 8])
    vldr(s0, [r7, 0])
    vldr(s1, [r2, 8])
    vmul(s1, s1, s0)                    # Scaled value
    vcvt_s32_f32(s1, s1)                # Convert it to an int
    vmov(r4, s1)                        # Move it back to the fixed point registers in r4
    movw(r5, 0x7fff)                    # Load the value 0x7fff into r5
    add(r4, r4, r5)                     # Add 0x7fff to the scaled value
    movw(r5, 0xffff)                    # Load the value 0xffff into r5
    cmp(r4, r5)                         # See if the scaled int is larger than 0xffff
    it(gt)                              # if then
    movw(r4, 0xffff)                    # Overwrite r4 with 0xffff to cap it
    lsr(r4, r4, 4)                      # Shift the value to the right by 4 bits r4 = r4>>4
    mov(r3, r4)                         # Move a copy of the shifted value into r3. This is the 12 bit value
    lsr(r4, r4, 4)                      # Shift it a further 4 bits so we now have the MSB in r4
    strb(r4, [r0, 1])                   # Store this byte in the can.msg.data[1]
    ldrb(r5, [r0, 2])                   # load the 3rd byte of the can message into r5
    lsl(r3, r3, 4)                      # shift it left by 4 bits  r6 = r6 << 4
    mov(r7, 0xf0)                       # move 11110000 into r7
    and_(r3, r7)                        # and the value in r6 with r7 and store it in r6.
    mov(r7, 0x0f)                       # mov 00001111 into r7
    and_(r5, r7)                        # and the value in r5 with r7 and store it in r5
    orr(r5, r3)                         # or the values in r5 and r6
    strb(r5, [r0, 2])                   # store that byte in can message [2]

    ldr(r7, [r1, 12])
    vldr(s0, [r7, 0])
    vldr(s1, [r2, 12])
    vmul(s1, s1, s0)
    vcvt_s32_f32(s1, s1)
    vmov(r4, s1)
    movw(r5, 0x7fff)
    add(r4, r4, r5)
    movw(r5, 0xffff)
    cmp(r4, r5)
    it(gt)
    movw(r4, 0xffff)
    lsr(r4, r4, 4)
    mov(r3, r4)
    mov(r7, 0xff)
    and_(r3, r7)
    strb(r3, [r0, 3])
    ldrb(r5, [r0, 2])
    mov(r7, 0xf0)
    and_(r5, r7)
    lsr(r4, r4, 8)
    mov(r7, 0x0f)
    and_(r4, r7)
    orr(r5, r4)
    strb(r5, [r0, 2])

# Encode 3 float channels to 16-bit big-endian CAN format (inverse of fast_caps_read_16bit).
# r0 = data (bytearray, 8 bytes: byte 0 = seq num, bytes 1-6 = 3 x 16-bit channels)
# r1 = outmap (array of 3 int addresses pointing to source floats)
# r2 = gain_map (array of 3 float gains)
# Per channel: raw = clamp(int(value * gain) + 0x7FFF, 0, 0xFFFF)
@micropython.asm_thumb
def fw_CAPS_u16bit(r0, r1, r2):
    # Channel 0 → data[1] (MSB), data[2] (LSB)
    ldr(r7, [r1, 0])
    vldr(s0, [r7, 0])
    vldr(s1, [r2, 0])
    vmul(s0, s0, s1)
    vcvt_s32_f32(s0, s0)
    vmov(r4, s0)
    movw(r5, 0x7fff)
    add(r4, r4, r5)
    cmp(r4, 0)
    it(lt)
    mov(r4, 0)
    movw(r5, 0xffff)
    cmp(r4, r5)
    it(gt)
    movw(r4, 0xffff)
    lsr(r3, r4, 8)
    strb(r3, [r0, 1])
    mov(r3, 0xff)
    and_(r4, r3)
    strb(r4, [r0, 2])
    # Channel 1 → data[3] (MSB), data[4] (LSB)
    ldr(r7, [r1, 4])
    vldr(s0, [r7, 0])
    vldr(s1, [r2, 4])
    vmul(s0, s0, s1)
    vcvt_s32_f32(s0, s0)
    vmov(r4, s0)
    movw(r5, 0x7fff)
    add(r4, r4, r5)
    cmp(r4, 0)
    it(lt)
    mov(r4, 0)
    movw(r5, 0xffff)
    cmp(r4, r5)
    it(gt)
    movw(r4, 0xffff)
    lsr(r3, r4, 8)
    strb(r3, [r0, 3])
    mov(r3, 0xff)
    and_(r4, r3)
    strb(r4, [r0, 4])
    # Channel 2 → data[5] (MSB), data[6] (LSB)
    ldr(r7, [r1, 8])
    vldr(s0, [r7, 0])
    vldr(s1, [r2, 8])
    vmul(s0, s0, s1)
    vcvt_s32_f32(s0, s0)
    vmov(r4, s0)
    movw(r5, 0x7fff)
    add(r4, r4, r5)
    cmp(r4, 0)
    it(lt)
    mov(r4, 0)
    movw(r5, 0xffff)
    cmp(r4, r5)
    it(gt)
    movw(r4, 0xffff)
    lsr(r3, r4, 8)
    strb(r3, [r0, 5])
    mov(r3, 0xff)
    and_(r4, r3)
    strb(r4, [r0, 6])


@micropython.asm_thumb
def fob_value_calc(r0, r1, r2, r3):
    vmov(s0, r1)                        # Move r1 to s0
    vcvt_f32_s32(s0, s0)                # Convert s0 to a float
    vmov(s1, r2)                        # Move r2 to s1
    vcvt_f32_s32(s1, s1)                # Convert s1 to float
    vldr(s2, [r3, 0])                   # s2 = 100
    vldr(s3, [r3, 4])                   # s3 = 5
    vmul(s0, s0, s2)                    # s0 = first FOB value * 100
    vadd(s0, s0, s1)                    # s0 = first FOB value * 100 + seond FOB value
    vdiv(s0, s0, s2)                    # s0 = (first FOB value * 100 + seond FOB value) / 100
    vsub(s0, s0, s3)                    # s0 = (first FOB value * 100 + seond FOB value) / 100 - 5
    vstr(s0, [r0, 0])                   # r0 = s0


@micropython.asm_thumb
def imu_isr(r0):                        # Save the output into r0[0]
    # First make things signed or unsigned
    ldr(r1, [r0, 4])                    # Load the address of imu.raw_quaternion_array and save it in r1
    ldr(r2, [r0, 8])                    # Load the address of imu.raw_gyroscope_array and save it in r2
    ldr(r3, [r0, 12])                   # Load the address of imu.raw_accelleration_array and save it in r3
    mov(r4, 3)                          # Make a loop counter
    movw(r5, 0x8000)                    # Comparison value 32768
    movwt(r6, 0x10000)                  # Subtraction value 65536
    label(LOOP)                         # Make a loop
    ldr(r7, [r1, 0])
    cmp(r7, r5)
    itt(ge)
    sub(r7, r7, r6)
    str(r7, [r1, 0])
    ldr(r7, [r2, 0])
    cmp(r7, r5)
    itt(ge)
    sub(r7, r7, r6)
    str(r7, [r2, 0])
    ldr(r7, [r3, 0])
    cmp(r7, r5)
    itt(ge)
    sub(r7, r7, r6)
    str(r7, [r3, 0])
    add(r1, 4)
    add(r2, 4)
    add(r3, 4)
    sub(r4, 1)                          # Will loop t3 times (value of r4)
    bgt(LOOP)
    ldr(r7, [r1, 0])                    # Need to do one more the quaternion because it has 4 elements in the array
    cmp(r7, r5)
    itt(ge)
    sub(r7, r7, r6)
    str(r7, [r1, 0])
    # Done with checking the sign

    ldr(r1, [r0, 4])                    # Load the address of imu.raw_quaternion_array and save it in r1
    ldr(r2, [r0, 0])                    # Load the address of arc_tan2_taylor_coefs and save it in r2
    vldr(s0, [r1, 0])                   # s0 is w
    vcvt_f32_s32(s0, s0)                # Converting w value to a float
    vldr(s1, [r1, 4])                   # s1 is x
    vcvt_f32_s32(s1, s1)                # Converting x value to a float
    vldr(s2, [r1, 8])                   # s2 is y
    vcvt_f32_s32(s2, s2)                # Converting y value to a float
    vldr(s3, [r1, 12])                  # s3 is z
    vcvt_f32_s32(s3, s3)                # Converting z value to a float

    vldr(s4, [r2, 24])                  # Take the 6th item in r2 and save it to s4 (1/(1<<14)) formatting value
    vldr(s5, [r2, 28])                  # Take the 7th item in r2 and save it to s5 (2)
    vldr(s15, [r2, 36])                 # Take the 9th item in r2 and save it to s15 (1)
    vldr(s17, [r2, 52])                 # Take the 13th item in r2 and save it to s17 (value is either 1 or -1)
    vmul(s0, s0, s4)                    # w*formatting value
    vmul(s1, s1, s4)                    # x*formatting value
    vmul(s2, s2, s4)                    # y*formatting value
    vmul(s3, s3, s4)                    # z*formatting value
    vmul(s12, s1, s1)                   # x * x
    vmul(s13, s2, s2)                   # y * y
    vadd(s14, s12, s13)                 # (x*x) + (y*y)
    vmul(s14, s14, s5)                  # 2 * ((x*x) + (y*y))
    vsub(s14, s15, s14)                 # 1-(2 * ((x*x) + (y*y)))
    vstr(s14, [r2, 48])                 # Take s14 and put it into the r2[48], denominator of value
    vmul(s0, s0, s1)                    # s0 = w * x
    vmul(s1, s2, s3)                    # s1 = y * z
    vadd(s0, s0, s1)                    # (w * x) + (y * z)
    vmul(s0, s0, s5)                    # ((w * x) + (y * z)) * 2 = s0
    vstr(s0, [r2, 44])                  # Take s0 and put it into the r2[44], numerator of value
    vdiv(s0, s0, s14)                   # (((w * x) + (y * z)) * 2)/(1-(2 * ((x*x) + (y*y))))
    vmul(s0, s0, s17)                   # (((w * x) + (y * z)) * 2)/(1-(2 * ((x*x) + (y*y)))) * sign
    vstr(s0, [r2, 32])                  # Take s0 and put it into the r2[8], arc_tan2_taylor_coefs[8]

    b(START1)                           # Identifying the start of a branch
    label(BETWEEN)                      # Name of the branch function
    vldr(s0, [r2, 32])                  # Move the value for calculating arctan into s0
    vmul(s1, s0, s0)                    # s0^2
    vmul(s1, s1, s0)                    # s1 = s0^3
    vmul(s2, s1, s0)                    # s0^4
    vmul(s2, s2, s0)                    # s2 = s0^5
    vmul(s3, s1, s1)                    # s0^6
    vmul(s3, s3, s0)                    # s3 = s0^7
    vmul(s4, s1, s2)                    # s0^8
    vmul(s4, s4, s0)                    # s4 = s0^9
    vldr(s5, [r2, 4])                   # r0[1] saved to s4 - 2nd coefficient
    vldr(s6, [r2, 8])                   # r0[2] saved to s5 - 3rd coefficient
    vldr(s7, [r2, 12])                  # r0[3] saved to s6 - 4th coefficient
    vldr(s8, [r2, 16])                  # r0[4] saved to s8 - 5th coefficient, 0.111111
    vmul(s9, s5, s1)                    # s7 = 0.3333333 * s0^3
    vmul(s10, s6, s2)                   # s8 = 0.2 * s0^5
    vmul(s11, s7, s3)                   # s9 = 0.142857 * s0^7
    vmul(s12, s8, s4)                   # s9 = 0.111111 * s0^9
    vsub(s13, s0, s9)                   # s10 = s0 - 0.333333 * s0^3
    vadd(s13, s13, s10)                 # s10 = s0 - 0.333333 * s0^3 + 0.2 * s0^5
    vsub(s13, s13, s11)                 # s10 = s0 - 0.333333 * s0^3 + 0.2 * s0^5 - 0.142857 * s0^7
    vsub(s13, s13, s12)                 # s10 = s0 - 0.333333 * s0^3 + 0.2 * s0^5 - 0.142857 * s0^7 + 0.11111 * S0^9
    vldr(s14, [r2, 20])                 # r[5] saved to s14, which is 180/pi
    vmul(s13, s13, s14)                 # s13 = (180/pi) * (s0 - 0.333333 * s0^3 + 0.2 * s0^5 - 0.142857 * s0^7)
    vstr(s13, [r2, 0])                  # save the value from s10 into r2[0], arc_tan2_taylor_coefs[0]
    bx(lr)                              # Pass data from this branch
    label(START1)

    b(START2)                           # Identifying the start of a branch
    label(GREATER)                      # Name of the branch function
    vldr(s0, [r2, 32])                  # Move the value for calculating arctan into s0
    vldr(s15, [r2, 36])                 # ro[9] saved to s15, value is 1
    vdiv(s0, s15, s0)                   # 1 / s0
    vmul(s1, s0, s0)                    # 1 / s0^2
    vmul(s1, s1, s0)                    # s1 = 1 / s0^3
    vmul(s2, s1, s0)                    # 1 / s0^4
    vmul(s2, s2, s0)                    # s2 = 1 / s0^5
    vmul(s3, s1, s1)                    # 1 / s0^6
    vmul(s3, s3, s0)                    # s3 = 1 / s0^7
    vmul(s4, s1, s2)                    # 1 / s0^8
    vmul(s4, s4, s0)                    # s4 = 1 / s0^9
    vldr(s5, [r2, 4])                   # r0[1] saved to s5 - second coefficient, 0.333333
    vldr(s6, [r2, 8])                   # r0[2] saved to s6 - third coefficient, 0.2
    vldr(s7, [r2, 12])                  # r0[3] saved to s7 - forth coefficient, 0.142857
    vldr(s8, [r2, 16])                  # r0[4] saved to s8 - fifth coefficient, 0.111111
    vldr(s16, [r2, 40])                 # ro[10] saved to s15 - This is pi()/2, 1.570796
    vldr(s23, [r2, 44])                 # r0[11] saved to s24 - 12th coeffeicient, the numerator of value

    vsub(s24, s5, s5)                   # Create a zero value for comparision
    vcmp(s23, s24)                      # Compare s24 and s25
    vmrs(APSR_nzcv, FPSCR)              # Line needed to evaluate comparison
    it(lt)                              # If s24 < s25, then next line:
    vneg(s16, s16)

    vmul(s9, s5, s1)                    # s9 = 0.3333333 * 1 / s0^3
    vmul(s10, s6, s2)                   # s10 = 0.2 * 1 / s0^5
    vmul(s11, s7, s3)                   # s11 = 0.142857 * 1 / s0^7
    vmul(s12, s8, s4)                   # s12 = 0.111111 * 1 / s0^9
    vsub(s13, s16, s0)                  # s13 = pi()/2 - 1 / s0
    vadd(s13, s13, s9)                  # s13 = pi()/2 - 1 / s0 + 0.333333 / s0^3
    vsub(s13, s13, s10)                 # s13 = pi()/2 - 1 / s0 + 0.333333 / s0^3 - 0.2 / s0^5
    vadd(s13, s13, s11)                 # s13 = pi()/2 - 1 / s0 + 0.333333 / s0^3 - 0.2 / s0^5 + 0.142857 / s0^7
    vsub(s13, s13, s12)                 # s13 = pi()/2 - 1 / s0 + 0.333333 / s0^3 - 0.2 / s0^5 + 0.142857 / s0^7 - 0.111111 / s0^9
    vldr(s14, [r2, 20])                 # ro[5] saved to s14, which is 180/pi, 57.2958
    vmul(s13, s13, s14)                 # s13 = (180/pi) * (pi()/2 - 1 / s0 + 0.333333 / s0^3 - 0.2 / s0^5 + 0.142857 / s0^7 - 0.111111 / s0^9)
    vstr(s13, [r2, 0])                  # save the value from s10 into r2[0], arc_tan2_taylor_coefs[0]
    bx(lr)                              # Pass data from this branch
    label(START2)

    b(START3)                           # Identifying the start of a branch
    label(LESS)                         # Name of the branch function
    vldr(s0, [r2, 32])                  # Move the value for calculating arctan into s0
    vldr(s15, [r2, 36])                 # ro[9] saved to s15, value is 1
    vdiv(s0, s15, s0)                   # 1 / s0
    vmul(s1, s0, s0)                    # 1 / s0^2
    vmul(s1, s1, s0)                    # s1 = 1 / s0^3
    vmul(s2, s1, s0)                    # 1 / s0^4
    vmul(s2, s2, s0)                    # s2 = 1 / s0^5
    vmul(s3, s1, s1)                    # 1 / s0^6
    vmul(s3, s3, s0)                    # s3 = 1 / s0^7
    vmul(s4, s1, s2)                    # 1 / s0^8
    vmul(s4, s4, s0)                    # s4 = 1 / s0^9
    vldr(s5, [r2, 4])                   # r0[1] saved to s5 - second coefficient, 0.333333
    vldr(s6, [r2, 8])                   # r0[2] saved to s6 - third coefficient, 0.2
    vldr(s7, [r2, 12])                  # r0[3] saved to s7 - forth coefficient, 0.142857
    vldr(s8, [r2, 16])                  # r0[4] saved to s8 - fifth coefficient, 0.111111
    vldr(s16, [r2, 40])                 # ro[10] saved to s16 - This is pi()/2, 1.570796
    vldr(s23, [r2, 44])                 # r0[11] saved to s24 - 12th coeffeicient, the numerator of value

    vsub(s24, s5, s5)                   # Create a zero value for comparision
    vcmp(s23, s24)                      # Compare s24 and s25
    vmrs(APSR_nzcv, FPSCR)              # Line needed to evaluate comparison
    it(gt)                              # If s24 < s25, then next line:
    vneg(s16, s16)

    vmul(s9, s5, s1)                    # s9 = 0.3333333 * 1 / s0^3
    vmul(s10, s6, s2)                   # s10 = 0.2 * 1 / s0^5
    vmul(s11, s7, s3)                   # s11 = 0.142857 * 1 / s0^7
    vmul(s12, s8, s4)                   # s12 = 0.111111 * 1 / s0^9
    vneg(s16, s16)                      # Makes s16 negative, so -pi()/2
    vsub(s13, s16, s0)                  # s10 = -pi()/2 - 1 / s0
    vadd(s13, s13, s9)                  # s10 = -pi()/2 - 1 / s0 + 0.333333 / s0^3
    vsub(s13, s13, s10)                 # s10 = -pi()/2 - 1 / s0 + 0.333333 / s0^3 - 0.2 / s0^5
    vadd(s13, s13, s11)                 # s10 = -pi()/2 - 1 / s0 + 0.333333 / s0^3 - 0.2 / s0^5 + 0.142857 / s0^7
    vsub(s13, s13, s12)                 # s10 = -pi()/2 - 1 / s0 + 0.333333 / s0^3 - 0.2 / s0^5 + 0.142857 / s0^7 - 0.11111 / s0^9
    vldr(s14, [r2, 20])                 # ro[5] saved to s14, which is 180/pi
    vmul(s13, s13, s14)                 # s13= (180/pi) * (-pi()/2 - 1 / s0 - 0.333333 / s0^3 + 0.2 / s0^5 - 0.142857 / s0^7)
    vstr(s13, [r2, 0])                  # save the value from s10 into r2[0], arc_tan2_taylor_coefs[0]
    bx(lr)                              # Pass data from this branch
    label(START3)

    vldr(s20, [r2, 32])                 # Move the value for calculating arctan into s20
    mov(r1, 1)                          # Sets value of r1 = 1
    vmov(s21, r1)                       # Move value of 1 into s21
    vcvt_f32_s32(s21, s21)              # Convert s21 value of 1 into a float
    vmul(s22, s20, s20)                 # Square the value for calculating arctan and save that into s22
    vsqrt(s22, s22)                     # Take square root of value of squared value - essentially taking absolute value

    vcmp(s22, s21)                      # Comparing absolute value of value for calculating arctan to 1
    vmrs(APSR_nzcv, FPSCR)              # Line needed to evaluate comparison
    it(mi)                              # If abs(value for calculating arctan) < 1 execute "BETWEEN" function
    bl(BETWEEN)                         # Call to "between" function

    vcmp(s20, s21)                      # Comparing value for calculating arctan to 1
    vmrs(APSR_nzcv, FPSCR)              # Line needed to evaluate comparison
    it(pl)                              # If value for calculating arctan > 1 execute "GREATER" function
    bl(GREATER)                         # Call to "GREATER" function

    vsub(s22, s21, s21)                 # s22 = s21 - s21, s22 = 1 - 1 = 0
    vsub(s22, s22, s21)                 # s22 = 0 - 1 = -1
    vcmp(s20, s22)                      # Comparing absolute value of value for calculating arctan to -1
    vmrs(APSR_nzcv, FPSCR)              # Line needed to evaluate comparison
    it(mi)                              # If value for calculating arctan < -1 execute "LESS" function
    bl(LESS)
    # Done the arctan section

    # Start the imu_acc_and_gyro_int_to_float section
    ldr(r1, [r0, 8])
    ldr(r2, [r0, 12])
    ldr(r3, [r0, 16])
    ldr(r4, [r0, 20])
    vldr(s0, [r1, 0])
    vldr(s1, [r1, 4])
    vldr(s2, [r1, 8])
    vldr(s3, [r2, 0])
    vldr(s4, [r2, 4])
    vldr(s5, [r2, 8])
    vldr(s6, [r3, 4])
    vldr(s7, [r3, 0])
    vcvt_f32_s32(s0, s0)
    vcvt_f32_s32(s1, s1)
    vcvt_f32_s32(s2, s2)
    vcvt_f32_s32(s3, s3)
    vcvt_f32_s32(s4, s4)
    vcvt_f32_s32(s5, s5)
    vdiv(s0, s0, s6)
    vdiv(s1, s1, s6)
    vdiv(s2, s2, s6)
    vdiv(s3, s3, s7)
    vdiv(s4, s4, s7)
    vdiv(s5, s5, s7)
    vstr(s0, [r4, 20])                      # Store the   into imu_data[5]
    vstr(s1, [r4, 24])                      # Store the   into imu_data[6]
    vstr(s2, [r4, 28])                      # Store the   into imu_data[7]
    vstr(s3, [r4, 8])                       # Store the   into imu_data[2]
    vstr(s4, [r4, 12])                      # Store the   into imu_data[3]
    vstr(s5, [r4, 16])                      # Store the   into imu_data[4]
    # Done imu_acc section, move to pack_shank_and_thigh
    # pack_shank_and_thigh
    ldr(r1, [r0, 0])
    ldr(r2, [r0, 24])
    ldr(r3, [r0, 28])
    ldr(r4, [r0, 20])
    vldr(s0, [r1, 0])                       # Load values into r1 - this is raw shank (if upright) / thigh (if inverted)
    vneg(s0, s0)                            # Flip raw shank/thigh angle sign
    vldr(s1, [r2, 0])                       # Load PARAM_DICT['JOINT_ANGLE'] into s1

    vsub(s18, s17, s17)                     # Create a zero value for comparison: s18 = s17- s17 = 0
    vcmp(s17, s18)                          # Comparing s17 and s18: X = s17 - s18, this is needed to check leg orientation
    vmrs(APSR_nzcv, FPSCR)                  # Line needed to evaluate comparison
    ite(pl)                                 # IF:   X is positive, then s0 is shank, else s0 is thigh
    vsub(s2, s0, s1)                        # THEN: Knee is upright, Thigh angle (s2) = shank angle (s0) - knee angle (s1)
    vadd(s2, s0, s1)                        # ELSE: Knee is inverted, Shank angle (s2) = thigh angle (s0) + knee angle (s1)

    vcmp(s17, s18)                          # Check leg orientation
    vmrs(APSR_nzcv, FPSCR)                  # Line needed to evaluate comparison
    ite(pl)                                 # IF:   X is positive, Knee is upright
    vstr(s0, [r4, 4])                       # THEN: Knee is upright, Store shank angle (s0) into imu_data[1]
    vstr(s2, [r4, 4])                       # ELSE: Knee is inverted, Store shank angle (s2) into imu_data[1]

    vcmp(s17, s18)                          # Check leg orientation
    vmrs(APSR_nzcv, FPSCR)                  # Line needed to evaluate comparison
    ite(pl)                                 # IF:   X is positive, Knee is upright
    vstr(s2, [r4, 0])                       # THEN: Knee is upright, Store thigh angle (s2) into imu_data[0]
    vstr(s0, [r4, 0])                       # ELSE: Knee is inverted, Store thigh angle (s0) into imu_data[0]

    vcmp(s17, s18)                          # Check leg orientation
    vmrs(APSR_nzcv, FPSCR)                  # Line needed to evaluate comparison
    ite(pl)                                 # IF:   X is positive, Knee is upright
    vstr(s0, [r3, 17 * 4])                  # THEN: Knee is upright, Store shank angle (s0) into sm_channels_current[16]
    vstr(s2, [r3, 17 * 4])                  # ELSE: Knee is inverted, Store shank angle (s2) into sm_channels_current[16]

    vcmp(s17, s18)                          # Check leg orientation
    vmrs(APSR_nzcv, FPSCR)                  # Line needed to evaluate comparison
    ite(pl)                                 # IF:   X is positive, Knee is upright
    vstr(s2, [r3, 16 * 4])                  # THEN: Knee is upright, Store thigh angle (s2) into sm_channels_current[17]
    vstr(s0, [r3, 16 * 4])                  # ELSE: Knee is inverted, Store thigh angle (s0) into sm_channels_current[17]

    vldr(s0, [r4, 8])                       # Load imu_data[2] into s0
    vstr(s0, [r3, 18 * 4])                  # Store the flipped shank angle into joint_encoder_mapping_array[17]
    vldr(s0, [r4, 12])
    vstr(s0, [r3, 19 * 4])
    vldr(s0, [r4, 16])
    vstr(s0, [r3, 20 * 4])
    vldr(s0, [r4, 20])
    vstr(s0, [r3, 21 * 4])
    vldr(s0, [r4, 24])
    vstr(s0, [r3, 22 * 4])
    vldr(s0, [r4, 28])
    vstr(s0, [r3, 23 * 4])
    # Done pack shank_and_thigh, move to sm_scale_imu_channels
    ldr(r1, [r0, 28])
    ldr(r0, [r0, 32])
    vldr(s0, [r0, 0 * 4])
    vldr(s1, [r0, 1 * 4])
    vldr(s2, [r1, 16 * 4])
    vldr(s3, [r1, 17 * 4])
    vldr(s4, [r1, 18 * 4])
    vldr(s5, [r1, 19 * 4])
    vldr(s6, [r1, 20 * 4])
    vldr(s7, [r1, 21 * 4])
    vldr(s8, [r1, 22 * 4])
    vldr(s9, [r1, 23 * 4])
    vdiv(s2, s2, s0)
    vdiv(s3, s3, s0)
    vdiv(s4, s4, s1)
    vdiv(s5, s5, s1)
    vdiv(s6, s6, s1)
    vdiv(s7, s7, s1)
    vdiv(s8, s8, s1)
    vdiv(s9, s9, s1)
    vstr(s2, [r1, 16 * 4])
    vstr(s3, [r1, 17 * 4])
    vstr(s4, [r1, 18 * 4])
    vstr(s5, [r1, 19 * 4])
    vstr(s6, [r1, 20 * 4])
    vstr(s7, [r1, 21 * 4])
    vstr(s8, [r1, 22 * 4])
    vstr(s9, [r1, 23 * 4])


@micropython.asm_thumb
def calculate_polynomial(r0, r1, r2, r3):
    '''
    Calculate y(x) for a polynomial of form y = a_0 + a_1 * x + a_2 * x^2 + ... + a_n * x^n

    r0[0]:  location for return value of y(x). r0 is an array of float(s) and y is returned at first element location
    r1:     the location for x, where x is a floating point number.
    r2[]:   float array of the polynomial coefficients in the form [a_0, a_1, ..., a_n] (as above)
    r3:     integer value of the number of coefficients (n+1)
    '''

    vldr(s1, [r1, 0])                       # Save value at r1 into s1
    mov(r4, 0)                              # r4 = 0
    vmov(s0, r4)                            # Move the value in r4 into s0, so s0 = 0
    vcvt_f32_s32(s0, s0)                    # Convert r4 to a float, so s0 = 0.0
    vadd(s7, s1, s0)                        # s7 = r1[0] + 0.0
    vldr(s2, [r2, 0])                       # s2 = coefficient 0 (a_0 in equation above)
    add(r2, 4)                              # Increment the storage register r2[1]
    vadd(s0, s2, s0)                        # s0 = coef_0 + 0.0
    sub(r3, 1)                              # Decrement the counter storage register
    mov(r4, 1)                              # r4 = 1
    vmov(s1, r4)                            # Move the r4 value into s1, so s1 = 1
    vcvt_f32_s32(s1, s1)                    # Convert s1 to a float, so s1 = 1.0
    label(LOOP)
    vmul(s1, s1, s7)                        # s1 = s1 * s7
    vldr(s2, [r2, 0])                       # coefficient 2
    add(r2, 4)
    vmul(s5, s2, s1)                        # s5 = s2 * s1
    vadd(s0, s5, s0)                        # s0 = s5 + s0
    sub(r3, 1)
    bgt(LOOP)
    vstr(s0, [r0, 0])


@micropython.asm_thumb
def fw0(r0, r1, r2):
    vldr(s0, [r1, 0])                   # Load in the value
    vldr(s1, [r2, 0])                   # load in the gain
    vmul(s1, s1, s0)                    # Scaled value
    vcvt_s32_f32(s1, s1)                # Convert it to an int
    vmov(r4, s1)                        # Move it back to the fixed point registers in r4
    movw(r5, 0x7fff)                    # Load the value 0x7fff into r5
    add(r4, r4, r5)                     # Add 0x7fff to the scaled value
    movw(r5, 0xffff)                    # Load the value 0xffff into r5
    cmp(r4, r5)                         # See if the scaled int is larger than 0xffff
    it(pl)                              # if then
    movw(r4, 0xffff)                    # Overwrite r4 with 0xffff to cap it
    lsr(r4, r4, 4)                      # Shift the value to the right by 4 bits r4 = r4>>4
    mov(r3, r4)                         # Move a copy of the shifted value into r3. This is the 12 bit value
    lsr(r4, r4, 4)                      # Shift it a further 4 bits so we now have the MSB in r4
    strb(r4, [r0, 1])                   # Store this byte in the can.msg.data[1]
    ldrb(r5, [r0, 2])                   # load the 3rd byte of the can message into r5
    lsl(r3, r3, 4)                      # shift it left by 4 bits  r6 = r6 << 4
    mov(r7, 0xf0)                       # move 11110000 into r7
    and_(r3, r7)                        # and the value in r6 with r7 and store it in r6.
    mov(r7, 0x0f)                       # mov 00001111 into r7
    and_(r5, r7)                        # and the value in r5 with r7 and store it in r5
    orr(r5, r3)                         # or the values in r5 and r6
    strb(r5, [r0, 2])                   # store that byte in can message [2]


@micropython.asm_thumb
def fw2(r0, r1, r2):
    add(r0, r0, 3)                      # make the pointer to the byte array increment by 3
    vldr(s0, [r1, 0])                   # Load in the value
    vldr(s1, [r2, 0])                   # load in the gain
    vmul(s1, s1, s0)                    # Scaled value
    vcvt_s32_f32(s1, s1)                # Convert it to an int
    vmov(r4, s1)                        # Move it back to the fixed point registers in r4
    movw(r5, 0x7fff)                    # Load the value 0x7fff into r5
    add(r4, r4, r5)                     # Add 0x7fff to the scaled value
    movw(r5, 0xffff)                    # Load the value 0xffff into r5
    cmp(r4, r5)                         # See if the scaled int is larger than 0xffff
    it(pl)                              # if then
    movw(r4, 0xffff)                    # Overwrite r4 with 0xffff to cap it
    lsr(r4, r4, 4)                      # Shift the value to the right by 4 bits r4 = r4>>4
    mov(r3, r4)                         # Move a copy of the shifted value into r3. This is the 12 bit value
    lsr(r4, r4, 4)                      # Shift it a further 4 bits so we now have the MSB in r4
    strb(r4, [r0, 1])                   # Store this byte in the can.msg.data[1]
    ldrb(r5, [r0, 2])                   # load the 3rd byte of the can message into r5
    lsl(r3, r3, 4)                      # shift it left by 4 bits  r6 = r6 << 4
    mov(r7, 0xf0)                       # move 11110000 into r7
    and_(r3, r7)                        # and the value in r6 with r7 and store it in r6.
    mov(r7, 0x0f)                       # mov 00001111 into r7
    and_(r5, r7)                        # and the value in r5 with r7 and store it in r5
    orr(r5, r3)                         # or the values in r5 and r6
    strb(r5, [r0, 2])                   # store that byte in can message [2]


@micropython.asm_thumb
def fw1(r0, r1, r2):
    vldr(s0, [r1, 0])
    vldr(s1, [r2, 0])
    vmul(s1, s1, s0)
    vcvt_s32_f32(s1, s1)
    vmov(r4, s1)
    movw(r5, 0x7fff)
    add(r4, r4, r5)
    movw(r5, 0xffff)
    cmp(r4, r5)
    it(pl)
    movw(r4, 0xffff)
    lsr(r4, r4, 4)
    mov(r3, r4)
    mov(r7, 0xff)
    and_(r3, r7)
    strb(r3, [r0, 3])
    ldrb(r5, [r0, 2])
    mov(r7, 0xf0)
    and_(r5, r7)
    lsr(r4, r4, 8)
    mov(r7, 0x0f)
    and_(r4, r7)
    orr(r5, r4)
    strb(r5, [r0, 2])


@micropython.asm_thumb
def fw3(r0, r1, r2):
    add(r0, r0, 3)
    vldr(s0, [r1, 0])
    vldr(s1, [r2, 0])
    vmul(s1, s1, s0)
    vcvt_s32_f32(s1, s1)
    vmov(r4, s1)
    movw(r5, 0x7fff)
    add(r4, r4, r5)
    movw(r5, 0xffff)
    cmp(r4, r5)
    it(pl)
    movw(r4, 0xffff)
    lsr(r4, r4, 4)
    mov(r3, r4)
    mov(r7, 0xff)
    and_(r3, r7)
    strb(r3, [r0, 3])
    ldrb(r5, [r0, 2])
    mov(r7, 0xf0)
    and_(r5, r7)
    lsr(r4, r4, 8)
    mov(r7, 0x0f)
    and_(r4, r7)
    orr(r5, r4)
    strb(r5, [r0, 2])