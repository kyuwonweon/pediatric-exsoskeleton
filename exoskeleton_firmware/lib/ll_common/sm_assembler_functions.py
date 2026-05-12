####################################################################################################
# @file             sm_assembler_functions.py
#
# @details          The is a collection of state machine related assembler functions mostly created by Levi Hargrove
#                   that can be used with a pyboard
#
# @author           Frank Ursetta and Levi J Hargrove
#
# @date             09/22/2022
####################################################################################################

# Ignore type-checking this file, as nearly every line has an undefined reference (see pep8.org, and PEP-484)
# type: ignore

print('IMPORTING STATE MACHINE ASSEMBLER FUNCTIONS')


@micropython.asm_thumb
def state_change_cleanup(r0, r1):
    ldr(r2, [r0, 0])
    ldr(r3, [r0, 4])
    ldr(r4, [r0, 8])
    ldr(r5, [r0, 12])

    label(LOOP)
    vldr(s0, [r2, 0])
    vstr(s0, [r3, 0])
    vstr(s0, [r4, 0])
    vstr(s0, [r5, 0])
    add(r2, 4)
    add(r3, 4)
    add(r4, 4)
    add(r5, 4)
    sub(r1, 1)
    bgt(LOOP)


@micropython.asm_thumb
def check_sub_events(r0, r1, r2, r3):

    # All branching LOGIC HERE!
    b(CHECK_ADDRESS)
    label(CHECK_ADD)

    mov(r4, 4)                          # Holder for the number of bytes in a float or in an int.
    vldr(s10, [r0, 0])                  # Load in the parameters
    vldr(s23, [r0, 4])
    vldr(s24, [r0, 8])
    vldr(s25, [r0, 12])
    add(r0, 16)

    vcvt_s32_f32(s26, s23)
    vcvt_s32_f32(s10, s10)              # Check and see what type of parameter it is
    vmov(r6, s10)

    cmp(r6, 1)                          # The value is what is passed in.
    it(eq)
    nop()

    cmp(r6, 3)
    ldr(r7, [r3, 0])                    # We load the address of the chan_mav array
    itttt(eq)
    vmov(r5, s26)                       # We then find the channel number
    mul(r5, r4)                         # Multiply it by 4bytes
    add(r7, r7, r5)                     # Add that to the address
    vldr(s23, [r7, 0])                  # Then load in the value at that address

    cmp(r6, 7)                          # Just make the parameter == 0 for now.
    ittt(eq)
    mov(r7, 0)
    vmov(s30, r7)
    vcvt_f32_s32(s23, s30)

    cmp(r6, 10)
    ldr(r7, [r3, 8])                    # We load the address of the chan_mv_prev array
    itttt(eq)
    vmov(r5, s26)                       # We then find the channel number
    mul(r5, r4)                         # Multiply it by 4bytes
    add(r7, r7, r5)                     # Add that to the address
    vldr(s23, [r7, 0])                  # Then load in the value at that address

    cmp(r6, 11)
    ldr(r7, [r3, 20])                   # We load the address of st_entry_array
    itttt(eq)
    vmov(r5, s26)                       # We then find the channel number
    mul(r5, r4)                         # Multiply it by 4bytes
    add(r7, r7, r5)                     # Add that to the address
    vldr(s23, [r7, 0])                  # Then load in the value at that address

    cmp(r6, 12)
    itt(eq)
    ldr(r7, [r3, 24])                   # We load the state_tag time in ms
    vldr(s23, [r7, 0])

    cmp(r6, 14)                         # Takes the input and converts it to a percentage by dividing by 100
    itttt(eq)
    mov(r7, 100)
    vmov(s29, r7)
    vcvt_f32_s32(s29, s29)
    vdiv(s23, s23, s29)

    vmul(s10, s23, s24)
    vadd(s10, s10, s25)
    vmov(r7, s10)
    bx(lr)
    label(CHECK_ADDRESS)

    b(C_STATE)
    label(CHECK_STATE)

    vmov(r6, s30)                       # Don't consider subevents, which all have a label greater than 250
    cmp(r6, 251)
    it(ge)
    bx(lr)

    vmov(s6, r0)
    vmov(r0, s5)
    mov(r4, 4)                          # Load from the event_enabled array
    mul(r6, r4)

    ldr(r5, [r0, 4])
    add(r5, r5, r6)
    ldr(r5, [r5, 0])
    cmp(r5, 0)
    itt(eq)                             # If the event isn't enabled, break from the loop
    vmov(r0, s6)
    bx(lr)

    ldr(r5, [r0, 8])                    # Get the state from address
    add(r5, r5, r6)
    ldr(r5, [r5, 0])

    ldr(r4, [r0, 20])                   # Get the active_state address
    ldr(r4, [r4, 0])                    # Load the value into r4 and move it to s11
    cmp(r4, r5)

    itt(ne)
    vmov(r0, s6)
    bx(lr)                              # If the current state is not associated with the state_from, then break out

    # If we get to here, then we should be changing states!
    ldr(r5, [r0, 16])                   # Get the priority
    add(r5, r5, r6)
    ldr(r5, [r5, 0])
    vmov(r4, s19)
    cmp(r5, r4)
    itt(le)
    vmov(r0, s6)
    bx(lr)                              # If the current state priority is less than the existing priority, break out

    ldr(r5, [r0, 12])                   # Get load in the next state
    add(r5, r5, r6)
    ldr(r5, [r5, 0])
    vmov(s18, r5)
    vmov(r0, s6)
    bx(lr)
    label(C_STATE)

    b(START1)
    label(EQUAL_TO)
    push({lr})
    bl(CHECK_ADD)
    pop({lr})
    vmov(s11, r7)
    push({lr})
    bl(CHECK_ADD)
    pop({lr})
    vmov(s12, r7)
    vcmp(s11, s12)
    vmrs(APSR_nzcv, FPSCR)
    ite(eq)
    mov(r7, 0x1)
    mov(r7, 0x0)
    vcvt_s32_f32(s30, s30)
    vmov(r6, s30)
    mov(r4, 0x4)
    mul(r6, r4)
    add(r2, r2, r6)
    str(r7, [r2, 0])
    sub(r2, r2, r6)
    bx(lr)
    label(START1)

    b(START2)
    label(GREATER_THAN)
    push({lr})
    bl(CHECK_ADD)
    pop({lr})
    vmov(s11, r7)
    push({lr})
    bl(CHECK_ADD)
    pop({lr})
    vmov(s12, r7)
    vcmp(s11, s12)
    vmrs(APSR_nzcv, FPSCR)
    ite(gt)
    mov(r7, 0x1)
    mov(r7, 0x0)
    vcvt_s32_f32(s30, s30)
    vmov(r6, s30)
    mov(r4, 0x4)
    mul(r6, r4)
    add(r2, r2, r6)
    str(r7, [r2, 0])
    sub(r2, r2, r6)
    bx(lr)
    label(START2)

    b(START3)
    label(GREATER_EQUAL_TO)
    push({lr})
    bl(CHECK_ADD)
    pop({lr})
    vmov(s11, r7)
    push({lr})
    bl(CHECK_ADD)
    pop({lr})
    vmov(s12, r7)
    vcmp(s11, s12)
    vmrs(APSR_nzcv, FPSCR)
    ite(ge)
    mov(r7, 0x1)
    mov(r7, 0x0)
    vcvt_s32_f32(s30, s30)
    vmov(r6, s30)
    mov(r4, 0x4)
    mul(r6, r4)
    add(r2, r2, r6)
    str(r7, [r2, 0])
    sub(r2, r2, r6)
    bx(lr)
    label(START3)

    b(START4)
    label(AND_FUNCTION)
    vcvt_s32_f32(s2, s2)
    vmov(r5, s2)                        # Get the number of parameters
    mov(r4, 0x4)
    push({r1, r3})
    mov(r7, 0x1)
    mov(r3, 0x0)
    label(INNER_LOOP)
    vldr(s23, [r0, 4])
    vcvt_s32_f32(s23, s23)
    vmov(r6, s23)
    mul(r6, r4)
    add(r2, r2, r6)
    ldr(r1, [r2, 0])
    sub(r2, r2, r6)
    cmp(r1, r3)
    it(eq)
    mov(r7, 0x0)
    add(r0, 16)
    sub(r5, 1)
    bgt(INNER_LOOP)
    pop({r1, r3})
    vcvt_s32_f32(s30, s30)
    vmov(r6, s30)
    mov(r4, 0x4)
    mul(r6, r4)
    add(r2, r2, r6)
    str(r7, [r2, 0])
    sub(r2, r2, r6)
    bx(lr)
    label(START4)

    b(START6)
    label(LESS_THAN)
    push({lr})
    bl(CHECK_ADD)
    pop({lr})
    vmov(s11, r7)
    push({lr})
    bl(CHECK_ADD)
    pop({lr})
    vmov(s12, r7)
    vcmp(s11, s12)
    vmrs(APSR_nzcv, FPSCR)
    ite(lt)
    mov(r7, 0x1)
    mov(r7, 0x0)
    vcvt_s32_f32(s30, s30)
    vmov(r6, s30)
    mov(r4, 0x4)
    mul(r6, r4)
    add(r2, r2, r6)
    str(r7, [r2, 0])
    sub(r2, r2, r6)
    bx(lr)
    label(START6)

    b(START7)
    label(LESS_EQUAL_TO)
    push({lr})
    bl(CHECK_ADD)
    pop({lr})
    vmov(s11, r7)
    push({lr})
    bl(CHECK_ADD)
    pop({lr})
    vmov(s12, r7)
    vcmp(s11, s12)
    vmrs(APSR_nzcv, FPSCR)
    ite(le)
    mov(r7, 0x1)
    mov(r7, 0x0)
    vcvt_s32_f32(s30, s30)
    vmov(r6, s30)
    mov(r4, 0x4)
    mul(r6, r4)
    add(r2, r2, r6)
    str(r7, [r2, 0])
    sub(r2, r2, r6)
    bx(lr)
    label(START7)

    b(START8)
    label(FIND_PEAK)

    vldr(s16, [r0, 4])                  # See which channel we are finding the peak from
    mov(r4, 0x0)
    vcvt_s32_f32(s16, s16)
    vmov(r5, s16)
    mul(r5, r4)                         # Offset
    ldr(r6, [r3, 12])                   # Load in the max of the channel in the state
    add(r6, r6, r5)
    vldr(s15, [r6, 0])                  # Assume that the channel has gain of 1 and no offset???

    push({lr})
    bl(CHECK_ADD)
    pop({lr})
    vmov(s11, r7)
    push({lr})
    bl(CHECK_ADD)
    pop({lr})
    vmov(s12, r7)
    push({lr})
    bl(CHECK_ADD)
    pop({lr})
    vmov(s13, r7)
    push({lr})
    bl(CHECK_ADD)
    pop({lr})
    vmov(s14, r7)

    vmul(s6, s15, s12)                  # Get the percent from peak
    vcmp(s11, s6)                       # See if the current value is less than that % from peak
    vmrs(APSR_nzcv, FPSCR)
    ite(lt)
    mov(r7, 0x1)
    mov(r7, 0x0)
    vcmp(s11, s13)                      # See if the current value is less than that % from peak
    vmrs(APSR_nzcv, FPSCR)
    ite(gt)
    mov(r7, 0x1)
    mov(r7, 0x0)
    vcvt_s32_f32(s30, s30)
    vmov(r6, s30)
    mov(r4, 0x4)
    mul(r6, r4)
    add(r2, r2, r6)
    str(r7, [r2, 0])
    sub(r2, r2, r6)
    bx(lr)
    label(START8)

    b(START9)
    label(ST_CHANGE)

    # TO DO: Add ST Change Logic For now, just evaluate it to false
    mov(r7, 0x0)
    vcvt_s32_f32(s30, s30)
    vmov(r6, s30)
    mov(r4, 0x4)
    mul(r6, r4)
    add(r2, r2, r6)
    str(r7, [r2, 0])
    sub(r2, r2, r6)
    bx(lr)
    label(START9)

    b(START10)
    label(MAN_ST_OVERRIDE)

    # TO DO: Add ST Change Logic, For now, just evaluate it to false    # Get the manual state override variable
    mov(r7, 0x0)
    str(r7, [r2, 0])
    vcvt_s32_f32(s30, s30)
    vmov(r6, s30)
    mov(r4, 0x4)
    mul(r6, r4)
    add(r2, r2, r6)
    str(r7, [r2, 0])
    sub(r2, r2, r6)
    bx(lr)
    label(START10)

    b(START11)
    label(MIN_INC)

    vldr(s16, [r0, 4])                  # See which channel we are finding the peak from
    mov(r4, 0x0)
    vcvt_s32_f32(s16, s16)
    vmov(r5, s16)
    mul(r5, r4)                         # Offset
    ldr(r6, [r3, 16])                   # Load in the min of the channel in the state
    add(r6, r6, r5)
    vldr(s15, [r6, 0])                  # Assume that the channel has gain of 1 and no offset???

    push({lr})
    bl(CHECK_ADD)
    pop({lr})
    vmov(s11, r7)
    push({lr})
    bl(CHECK_ADD)
    pop({lr})
    vmov(s12, r7)
    push({lr})
    bl(CHECK_ADD)
    pop({lr})
    vmov(s13, r7)

    mov(r7, 0x01)

    vcmp(s15, s13)
    vmrs(APSR_nzcv, FPSCR)
    it(ge)
    mov(r7, 0x00)

    vadd(s14, s15, s12)
    vcmp(s11, s15)
    vmrs(APSR_nzcv, FPSCR)
    it(le)
    mov(r7, 0x00)

    vcvt_s32_f32(s30, s30)
    vmov(r6, s30)
    mov(r4, 0x4)
    mul(r6, r4)
    add(r2, r2, r6)
    str(r7, [r2, 0])
    sub(r2, r2, r6)
    bx(lr)
    label(START11)

    b(START13)
    label(CLOSE_TO)
    push({lr})
    bl(CHECK_ADD)
    pop({lr})
    vmov(s11, r7)
    push({lr})
    bl(CHECK_ADD)
    pop({lr})
    vmov(s12, r7)
    push({lr})
    bl(CHECK_ADD)
    pop({lr})
    vmov(s13, r7)
    vmul(s13, s12, s13)
    vsub(s11, s11, s12)
    vmul(s11, s11, s11)
    vsqrt(s11, s11)
    vcmp(s11, s13)
    vmrs(APSR_nzcv, FPSCR)
    ite(le)
    mov(r7, 0x1)
    mov(r7, 0x0)
    vcvt_s32_f32(s30, s30)
    vmov(r6, s30)
    mov(r4, 0x4)
    mul(r6, r4)
    add(r2, r2, r6)
    str(r7, [r2, 0])
    sub(r2, r2, r6)
    bx(lr)
    label(START13)

    b(START200)
    label(CUSTOM1)                      # Note this function does not use the  variable that is noramlly used in caps.
    mov(r4, 4)                          # Holder for the number of bytes in a float or in an int.
    vldr(s10, [r0, 0])                  # Load in the parameters
    vldr(s23, [r0, 4])
    vldr(s24, [r0, 8])
    vldr(s25, [r0, 12])
    add(r0, 16)
    vcvt_s32_f32(s26, s23)
    ldr(r7, [r3, 0])                    # We load the address of the chan_mav array
    vmov(r5, s26)                       # We then find the channel number
    mul(r5, r4)                         # Multiply it by 4bytes
    add(r7, r7, r5)                     # Add that to the address
    vldr(s23, [r7, 0])                  # Then load in the value at that address
    vmul(s10, s23, s24)
    vadd(s10, s10, s25)
    vmov(r7, s10)
    vmov(s11, r7)
    push({lr})
    bl(CHECK_ADD)
    pop({lr})
    vmov(s12, r7)
    push({lr})
    bl(CHECK_ADD)
    pop({lr})
    vmov(s13, r7)
    push({lr})
    bl(CHECK_ADD)
    pop({lr})
    vmov(s14, r7)

    vcmp(s11, s12)
    vmrs(APSR_nzcv, FPSCR)
    ite(eq)
    mov(r7, 0x1)
    mov(r7, 0x0)
    vcvt_s32_f32(s30, s30)
    vmov(r6, s30)
    mov(r4, 0x4)
    mul(r6, r4)
    add(r2, r2, r6)
    str(r7, [r2, 0])
    sub(r2, r2, r6)
    # TO DO: Put code for CUSTOM function here in here!!
    bx(lr)
    label(START200)

    # Done with the branching LOGIC!

    vmov(s5, r0)
    mov(r4, 0)
    vmov(s19, r4)
    vmov(s18, r4)
    ldr(r0, [r0, 0])
    label(LOOP)
    vldr(s30, [r0, 0])                  # Get the event_id
    vldr(s0, [r0, 4])                   # Get the function code
    vldr(s1, [r0, 8])                   # Get the minFrameCount
    vldr(s2, [r0, 12])                  # Get the numParams
    add(r0, 16)                         # Reset the event_array point to point at the first parmeter

    mov(r4, 1)                          # Each time in the loop, we reset the function code checker to 0
    vmov(s20, r4)
    vcvt_f32_s32(s20, s20)
    vcmp(s0, s20)                       # see if function == 1
    vmrs(APSR_nzcv, FPSCR)
    it(eq)
    bl(EQUAL_TO)

    mov(r4, 2)                          # Each time in the loop, we reset the function code checker to 0
    vmov(s20, r4)
    vcvt_f32_s32(s20, s20)
    vcmp(s0, s20)                       # See if function == 2
    vmrs(APSR_nzcv, FPSCR)
    it(eq)
    bl(GREATER_THAN)

    mov(r4, 3)                          # Each time in the loop, we reset the function code checker to 0
    vmov(s20, r4)
    vcvt_f32_s32(s20, s20)
    vcmp(s0, s20)                       # see if function == 3
    vmrs(APSR_nzcv, FPSCR)
    it(eq)
    bl(GREATER_EQUAL_TO)

    mov(r4, 4)                          # Each time in the loop, we reset the function code checker to 0
    vmov(s20, r4)
    vcvt_f32_s32(s20, s20)
    vcmp(s0, s20)                       # see if function == 4
    vmrs(APSR_nzcv, FPSCR)
    it(eq)
    bl(AND_FUNCTION)

    mov(r4, 6)                          # Each time in the loop, we reset the function code checker to 0
    vmov(s20, r4)
    vcvt_f32_s32(s20, s20)
    vcmp(s0, s20)                       # see if function == 6
    vmrs(APSR_nzcv, FPSCR)
    it(eq)
    bl(LESS_THAN)

    mov(r4, 7)                          # Each time in the loop, we reset the function code checker to 0
    vmov(s20, r4)
    vcvt_f32_s32(s20, s20)
    vcmp(s0, s20)                       # see if function == 7
    vmrs(APSR_nzcv, FPSCR)
    it(eq)
    bl(LESS_EQUAL_TO)

    mov(r4, 8)                          # Each time in the loop, we reset the function code checker to 0
    vmov(s20, r4)
    vcvt_f32_s32(s20, s20)
    vcmp(s0, s20)                       # see if function == 8
    vmrs(APSR_nzcv, FPSCR)
    it(eq)
    bl(FIND_PEAK)

    mov(r4, 9)                          # Each time in the loop, we reset the function code checker to 0
    vmov(s20, r4)
    vcvt_f32_s32(s20, s20)
    vcmp(s0, s20)                       # see if function ==9
    vmrs(APSR_nzcv, FPSCR)
    it(eq)
    bl(ST_CHANGE)

    mov(r4, 10)                         # Each time in the loop, we reset the function code checker to 0
    vmov(s20, r4)
    vcvt_f32_s32(s20, s20)
    vcmp(s0, s20)                       # see if function == 10
    vmrs(APSR_nzcv, FPSCR)
    it(eq)
    bl(MAN_ST_OVERRIDE)

    mov(r4, 11)                         # Each time in the loop, we reset the function code checker to 0
    vmov(s20, r4)
    vcvt_f32_s32(s20, s20)
    vcmp(s0, s20)                       # see if function == 11
    vmrs(APSR_nzcv, FPSCR)
    it(eq)
    bl(MIN_INC)

    mov(r4, 13)                         # Each time in the loop, we reset the function code checker to 0
    vmov(s20, r4)
    vcvt_f32_s32(s20, s20)
    vcmp(s0, s20)                       # see if function == 13
    vmrs(APSR_nzcv, FPSCR)
    it(eq)
    bl(CLOSE_TO)

    mov(r4, 200)                        # Each time in the loop, we reset the function code checker to 0
    vmov(s20, r4)
    vcvt_f32_s32(s20, s20)
    vcmp(s0, s20)                       # see if function == 0xC8 == decimal 200
    vmrs(APSR_nzcv, FPSCR)
    it(eq)
    bl(CUSTOM1)

    cmp(r7, 1)
    it(eq)
    bl(CHECK_STATE)

    cmp(r1, r0)
    bgt_w(LOOP)
    vmov(r4, s18)
    cmp(r4, 0)
    itttt(gt)
    vmov(s6, r0)
    vmov(r0, s5)
    ldr(r5, [r0, 20])
    str(r4, [r5, 0])


# Assume state_array is in r0
# Assume sensor address_array is in r1
# Assume start index of the state is in length is in r2,
# Assume r3 is an array of address. The address points to the number of dofs (int) and the second is a floating point array of the DOF outputs
# NOTE: this function defined twice
@micropython.asm_thumb
def check_states(r0, r1, r2, r3):

    # All branching LOGIC HERE!
    b(CHECK_ADDRESS)
    label(CHECK_ADD)

    mov(r4, 0x4)                        # Holder for the number of bytes in a float or in an int.
    vldr(s10, [r0, 0])                  # Load in the parameters
    vldr(s23, [r0, 4])
    vldr(s24, [r0, 8])
    vldr(s25, [r0, 12])
    add(r0, 16)

    vcvt_s32_f32(s26, s23)
    vcvt_s32_f32(s10, s10)              # Check and see what type of parameter it is
    vmov(r6, s10)

    cmp(r6, 1)                          # The value is what is passed in.
    it(eq)
    nop()

    cmp(r6, 3)
    ldr(r7, [r1, 0])                    # We load the address of the chan_mav array
    itttt(eq)
    vmov(r5, s26)                       # We then find the channel number
    mul(r5, r4)                         # Multiply it by 4bytes
    add(r7, r7, r5)                     # Add that to the address
    vldr(s23, [r7, 0])                  # Then load in the value at that address

    cmp(r6, 6)                          # This is the previous SM_DOF
    it(eq)
    vldr(s23, [r3, 0])

    cmp(r6, 7)                          # Just make the parameter == 0 for now. This is a classifier output
    ittt(eq)
    mov(r7, 0x0)
    vmov(s30, r7)
    vcvt_f32_s32(s23, s30)

    cmp(r6, 10)
    ldr(r7, [r1, 8])                    # We load the address of the chan_mv_prev array
    itttt(eq)
    vmov(r5, s26)                       # We then find the channel number
    mul(r5, r4)                         # Multiply it by 4bytes
    add(r7, r7, r5)                     # Add that to the address
    vldr(s23, [r7, 0])                  # Then load in the value at that address

    cmp(r6, 11)
    ldr(r7, [r1, 20])                   # We load the address of st_entry_array
    itttt(eq)
    vmov(r5, s26)                       # We then find the channel number
    mul(r5, r4)                         # Multiply it by 4bytes
    add(r7, r7, r5)                     # Add that to the address
    vldr(s23, [r7, 0])                  # Then load in the value at that address

    cmp(r6, 12)
    itt(eq)
    ldr(r7, [r1, 24])                   # We load the state_tag time in ms
    vldr(s23, [r7, 0])

    cmp(r6, 14)                         # Takes the input and converts it to a percentage by dividing by 100
    itttt(eq)
    mov(r7, 100)
    vmov(s29, r7)
    vcvt_f32_s32(s29, s29)
    vdiv(s23, s23, s29)

    vmul(s10, s23, s24)
    vadd(s10, s10, s25)
    vmov(r7, s10)
    bx(lr)
    label(CHECK_ADDRESS)

    b(TIME)                             # This branch puts a constant value in the impedance output. It has 1 parameter in the function
    label(GET_TIME)
    ldr(r7, [r1, 24])                   # We load the address of the timing variables
    vldr(s23, [r7, 0])                  # Time since entering state is first entry in the timing vars array
    vmov(r7, s23)                       # Put the value in r7
    bx(lr)
    label(TIME)

    b(FRAME)                            # This branch puts a constant value in the impedance output. It has 1 parameter in the function
    label(GET_FRAME)
    ldr(r7, [r1, 24])                   # We load the address of the timing variables
    vldr(s23, [r7, 4])                  # Time since entering state is first entry in the timing vars array
    vmov(r7, s23)                       # Put the value in r7
    bx(lr)
    label(FRAME)

    b(START0)                           # This branch puts a constant value in the impedance output. It has 1 parameter in the function
    label(CONSTANT)                     # This branch is funtional and has been preliminarily tested
    push({lr})
    bl(CHECK_ADD)
    pop({lr})
    vmov(s11, r7)                       # Put the parameter in s11, this value needs to be stored in the correct location of the output array
    vmul(s11, s11, s4)
    vadd(s11, s11, s5)
    vstr(s11, [r3, 0])
    bx(lr)
    label(START0)

    b(START1)
    label(LINEAR_RAMP)                  # This branch executes the linear ramp. It has 3 parameters in the function
    push({lr})                          # This branch is functional and has been preliminarily tested
    bl(CHECK_ADD)
    pop({lr})
    vmov(s11, r7)
    push({lr})
    bl(CHECK_ADD)
    pop({lr})
    vmov(s12, r7)
    push({lr})
    bl(CHECK_ADD)
    pop({lr})
    vmov(s13, r7)
    push({lr})
    bl(GET_TIME)
    pop({lr})
    vmov(s14, r7)
    vdiv(s15, s14, s13)                 # Divide the current time by the specified time
    vsub(s16, s12, s11)                 # Calculate difference between start value and ending value
    vmul(s17, s16, s15)
    vadd(s17, s11, s17)
    mov(r7, 1)                          # Move the value 1 into a register and convert it to floating point
    vmov(s18, r7)
    vcvt_f32_s32(s18, s18)
    vmul(s12, s12, s4)
    vadd(s12, s12, s5)
    vmul(s17, s17, s4)
    vadd(s17, s17, s5)
    vcmp(s15, s18)                      # See if the ramp portion is complete (cur/ramptime >=0)
    vmrs(APSR_nzcv, FPSCR)
    ite(ge)
    vstr(s12, [r3, 0])                  # if >= , then just save the target value
    vstr(s17, [r3, 0])                  # else save the ramp value
    bx(lr)
    label(START1)

    b(START3)
    label(DIFFERENTIAL)                 # This branch executes the differential. It has 4 parameters in the function
    push({lr})                          # Implemented and preliminarily tested
    bl(CHECK_ADD)
    pop({lr})
    vmov(s11, r7)
    push({lr})
    bl(CHECK_ADD)
    pop({lr})
    vmov(s12, r7)
    push({lr})
    bl(CHECK_ADD)
    pop({lr})
    vmov(s13, r7)
    push({lr})
    bl(CHECK_ADD)
    pop({lr})
    vmov(s14, r7)
    vsub(s11, s11, s12)
    vstr(s11, [r3, 0])
    vcmp(s11, s13)
    vmrs(APSR_nzcv, FPSCR)
    it(le)
    vstr(s13, [r3, 0])
    vcmp(s11, s14)
    vmrs(APSR_nzcv, FPSCR)
    it(ge)
    vstr(s14, [r3, 0])
    bx(lr)
    label(START3)

    b(START4)
    label(DIFFERENTIAL_INC)              # This branch executes the differential_inc. It has 4 parameters in the function
    push({lr})                           # Currently not implemented
    bl(CHECK_ADD)
    pop({lr})
    vmov(s11, r7)
    push({lr})
    bl(CHECK_ADD)
    pop({lr})
    vmov(s12, r7)
    push({lr})
    bl(CHECK_ADD)
    pop({lr})
    vmov(s13, r7)
    push({lr})
    bl(CHECK_ADD)
    pop({lr})
    vmov(s14, r7)
    bx(lr)
    label(START4)

    b(START5)
    label(STATE_CHANGE_LATCH)           # This branch executes the state_change_latch. It has 3 parameters in the function
    push({lr})                          # Implemented, not tested
    bl(CHECK_ADD)
    pop({lr})
    vmov(s11, r7)
    push({lr})
    bl(CHECK_ADD)
    pop({lr})
    vmov(s12, r7)
    push({lr})
    bl(CHECK_ADD)
    pop({lr})
    vmov(s13, r7)
    vcmp(s11, s12)
    vmrs(APSR_nzcv, FPSCR)
    itt(le)
    vmov(r7, s12)
    vmov(s11, r7)
    vcmp(s11, s13)
    vmrs(APSR_nzcv, FPSCR)
    itt(ge)
    vmov(r7, s12)
    vmov(s11, r7)
    push({lr})
    bl(GET_FRAME)
    pop({lr})
    vmov(s12, r7)
    vcvt_s32_f32(s12, s12)
    vmov(r7, s12)
    cmp(r7, 1)
    it(le)
    vstr(s11, [r3, 0])
    bx(lr)
    label(START5)

    b(START6)
    label(SWITCH_VALUE)                 # This branch executes the switch value. It has 8 parameters in the function
    push({lr})                          # This branch is functional and has been preliminaryly tested
    bl(CHECK_ADD)
    pop({lr})
    vmov(s11, r7)
    push({lr})
    bl(CHECK_ADD)
    pop({lr})
    vmov(s12, r7)
    push({lr})
    bl(CHECK_ADD)
    pop({lr})
    vmov(s13, r7)
    push({lr})
    bl(CHECK_ADD)
    pop({lr})
    vmov(s14, r7)
    push({lr})
    bl(CHECK_ADD)
    pop({lr})
    vmov(s15, r7)
    push({lr})
    bl(CHECK_ADD)
    pop({lr})
    vmov(s16, r7)
    push({lr})
    bl(CHECK_ADD)
    pop({lr})
    vmov(s17, r7)
    push({lr})
    bl(CHECK_ADD)
    pop({lr})
    vmov(s18, r7)
    mov(r7, 1)
    mov(r6, 1)
    vcmp(s13, s14)
    vmrs(APSR_nzcv, FPSCR)
    it(lt)
    mov(r7, 0)
    vcmp(s15, s16)
    vmrs(APSR_nzcv, FPSCR)
    it(lt)
    mov(r6, 0)
    add(r7, r6, r7)
    mov(r6, 0)
    cmp(r6, r7)
    itt(eq)
    vmov(r7, s12)
    vmov(s11, r7)
    vstr(s11, [r3, 0])
    vcmp(s11, s17)
    vmrs(APSR_nzcv, FPSCR)
    it(le)
    vstr(s17, [r3, 0])
    vcmp(s11, s18)
    vmrs(APSR_nzcv, FPSCR)
    it(ge)
    vstr(s18, [r3, 0])
    bx(lr)
    label(START6)

    b(START7)
    label(EXP_RAMP)                     # This branch executes the exponential ramp, not implemented. It has 8 parameters in the function
    push({lr})
    bl(CHECK_ADD)
    pop({lr})
    vmov(s11, r7)
    push({lr})
    bl(CHECK_ADD)
    pop({lr})
    vmov(s12, r7)
    push({lr})
    bl(CHECK_ADD)
    pop({lr})
    vmov(s13, r7)
    push({lr})
    bl(CHECK_ADD)
    pop({lr})
    vmov(s14, r7)
    push({lr})
    bl(CHECK_ADD)
    pop({lr})
    vmov(s15, r7)
    bx(lr)
    label(START7)

    b(START8)
    label(MULTIPLY_SUM)                 # This branch executes the switch value. It has 8 parameters in the function
    push({lr})                          # This function has been implemented and preliminarily tested
    bl(CHECK_ADD)
    pop({lr})
    vmov(s11, r7)
    push({lr})
    bl(CHECK_ADD)
    pop({lr})
    vmov(s12, r7)
    push({lr})
    bl(CHECK_ADD)
    pop({lr})
    vmov(s13, r7)
    push({lr})
    bl(CHECK_ADD)
    pop({lr})
    vmov(s14, r7)
    push({lr})
    bl(CHECK_ADD)
    pop({lr})
    vmov(s15, r7)
    vadd(s12, s12, s13)                 # s11 * (s12 + 13)
    vmul(s11, s11, s12)
    vstr(s11, [r3, 0])
    vcmp(s11, s14)
    vmrs(APSR_nzcv, FPSCR)
    it(le)
    vstr(s14, [r3, 0])
    vcmp(s11, s15)
    vmrs(APSR_nzcv, FPSCR)
    it(ge)
    vstr(s15, [r3, 0])
    bx(lr)
    label(START8)

    b(START10)
    label(SWITCH_HOLD)                  # This branch executes the switch hold. It has 8 parameters in the function
    push({lr})
    bl(CHECK_ADD)
    pop({lr})
    vmov(s11, r7)
    push({lr})
    bl(CHECK_ADD)
    pop({lr})
    vmov(s12, r7)
    push({lr})
    bl(CHECK_ADD)
    pop({lr})
    vmov(s13, r7)
    push({lr})
    bl(CHECK_ADD)
    pop({lr})
    vmov(s14, r7)
    push({lr})
    bl(CHECK_ADD)
    pop({lr})
    vmov(s15, r7)
    push({lr})
    bl(CHECK_ADD)
    pop({lr})
    vmov(s16, r7)
    push({lr})
    bl(CHECK_ADD)
    pop({lr})
    vmov(s17, r7)
    push({lr})
    bl(CHECK_ADD)
    pop({lr})
    vmov(s18, r7)
    bx(lr)
    label(START10)

    b(START11)
    label(RATE_BASED)                   # This branch executes the rate_based. It has 8 parameters in the function
    push({lr})                          # Implemented but not tested
    bl(CHECK_ADD)
    pop({lr})
    vmov(s11, r7)
    push({lr})
    bl(CHECK_ADD)
    pop({lr})
    vmov(s12, r7)
    push({lr})
    bl(CHECK_ADD)
    pop({lr})
    vmov(s13, r7)
    push({lr})
    bl(CHECK_ADD)
    pop({lr})
    vmov(s14, r7)
    push({lr})
    bl(CHECK_ADD)
    pop({lr})
    vmov(s15, r7)
    push({lr})
    bl(CHECK_ADD)
    pop({lr})
    vmov(s16, r7)
    push({lr})
    bl(CHECK_ADD)
    pop({lr})
    vmov(s17, r7)
    push({lr})
    bl(CHECK_ADD)
    pop({lr})
    vmov(s18, r7)
    vsub(s23, s11, s15)
    vsub(s24, s15, s16)
    vdiv(s23, s23, s24)
    vsub(s24, s13, s14)
    vmul(s11, s12, s23)
    vmul(s11, s11, s24)
    vadd(s11, s11, s13)
    vstr(s11, [r3, 0])
    vcmp(s11, s17)
    vmrs(APSR_nzcv, FPSCR)
    it(le)
    vstr(s17, [r3, 0])
    vcmp(s11, s18)
    vmrs(APSR_nzcv, FPSCR)
    it(ge)
    vstr(s18, [r3, 0])
    bx(lr)
    label(START11)

    add(r0, r0, r2)                     # Point the correct location in the state array list, we are done now with the input r2
    # r0 should now by pointing at the address of the current active state. We will need to loop
    # through each DOF output and compute its values based on the state parameters.

    ldr(r2, [r3, 0])                    # See how many state outputs we need to construct
    ldr(r2, [r2, 0])                    # We shouldn't overwrite r2 anymore unless we save it. This is the loop counter and gets
    # subtracted by 1 at the bottom of the loop each time through.

    ldr(r3, [r3, 4])                    # We need to get the correct register to store the impedencae output.

    vldr(s0, [r0, 0])                   # Node_id
    vldr(s1, [r0, 4])                   # Min hold frame count of the node.
    add(r0, 0x8)                        # Move the pointer

    label(LOOP)
    vldr(s2, [r0, 0])                   # Function code
    vldr(s4, [r0, 4])                   # Output gain
    vldr(s5, [r0, 8])                   # Output offset
    vldr(s6, [r0, 12])                  # Number of parameters in the function
    add(r0, 16)                         # Move the pointer to the start of the parameters

    # From here we need the branching logic

    mov(r4, 1)
    vmov(s7, r4)
    vcvt_f32_s32(s7, s7)
    vcmp(s2, s7)                        # see if function code stored in s2 == 1
    vmrs(APSR_nzcv, FPSCR)
    it(eq)
    bl(CONSTANT)

    mov(r4, 2)
    vmov(s7, r4)
    vcvt_f32_s32(s7, s7)
    vcmp(s2, s7)                        # see if function code stored in s2 == 2
    vmrs(APSR_nzcv, FPSCR)
    it(eq)
    bl(LINEAR_RAMP)

    mov(r4, 3)
    vmov(s7, r4)
    vcvt_f32_s32(s7, s7)
    vcmp(s2, s7)                        # see if function code stored in s2 == 3
    vmrs(APSR_nzcv, FPSCR)
    it(eq)
    bl(DIFFERENTIAL)

    mov(r4, 4)
    vmov(s7, r4)
    vcvt_f32_s32(s7, s7)
    vcmp(s2, s7)                        # see if function code stored in s2 == 3
    vmrs(APSR_nzcv, FPSCR)
    it(eq)
    bl(DIFFERENTIAL_INC)

    mov(r4, 5)
    vmov(s7, r4)
    vcvt_f32_s32(s7, s7)
    vcmp(s2, s7)                        # see if function code stored in s2 == 5
    vmrs(APSR_nzcv, FPSCR)
    it(eq)
    bl(STATE_CHANGE_LATCH)

    mov(r4, 6)
    vmov(s7, r4)
    vcvt_f32_s32(s7, s7)
    vcmp(s2, s7)                        # see if function code stored in s2 == 5
    vmrs(APSR_nzcv, FPSCR)
    it(eq)
    bl(SWITCH_VALUE)

    mov(r4, 7)
    vmov(s7, r4)
    vcvt_f32_s32(s7, s7)
    vcmp(s2, s7)                        # see if function code stored in s2 == 10
    vmrs(APSR_nzcv, FPSCR)
    it(eq)
    bl(EXP_RAMP)

    mov(r4, 8)
    vmov(s7, r4)
    vcvt_f32_s32(s7, s7)
    vcmp(s2, s7)                        # see if function code stored in s2 == 10
    vmrs(APSR_nzcv, FPSCR)
    it(eq)
    bl(MULTIPLY_SUM)

    mov(r4, 10)
    vmov(s7, r4)
    vcvt_f32_s32(s7, s7)
    vcmp(s2, s7)                        # see if function code stored in s2 == 10
    vmrs(APSR_nzcv, FPSCR)
    it(eq)
    bl(SWITCH_HOLD)

    mov(r4, 11)
    vmov(s7, r4)
    vcvt_f32_s32(s7, s7)
    vcmp(s2, s7)                        # see if function code stored in s2 == 11
    vmrs(APSR_nzcv, FPSCR)
    it(eq)
    bl(RATE_BASED)

    add(r3, 4)                          # Move to the next point in the output array
    sub(r2, 1)                          # Decrement the loop index variable.
    bgt_w(LOOP)


@micropython.asm_thumb
def update_sm_input_vars(r0, r1, r2, r3):
    vmov(s0, r2)                            # Get time in state in ms
    vmov(s1, r3)                            # Get number of frames in state
    vcvt_f32_s32(s0, s0)                    # Convert to float
    vcvt_f32_s32(s1, s1)                    # Convert to float
    movw(r7, 1000)                          # Convert from ms to s (need to sort out if CAPS requires values in second or ms)
    vmov(s3, r7)                            # Move to floating point processor
    vcvt_f32_s32(s3, s3)                    # Conver to float
    vdiv(s0, s0, s3)                        # Divide time in ms by 1000 to conver to seconcs
    ldr(r7, [r0, 0])                        # Load the address of the timing variables into R7
    vstr(s0, [r7, 0])                       # Store the time in state (in seconds)
    vstr(s0, [r7, 4])                       # Store the number of frames.
    # Done with r2 and r3 now, so we can reuse them them
    # This seciont updates the max and min of the variables within the state
    ldr(r2, [r0, 4])                        # Put sm_current_channels in r2
    ldr(r3, [r0, 8])                        # Put max in r3
    ldr(r4, [r0, 12])                       # Put min in r4
    ldr(r5, [r0, 16])                       # put prev in r5
    ldr(r6, [r0, 20])                       # put slope in r6
    label(LOOP)
    vldr(s0, [r2, 0])                       # Load sm_current_chans
    vldr(s1, [r3, 0])                       # Load max
    vldr(s2, [r4, 0])                       # Load min
    vcmp(s0, s1)
    vmrs(APSR_nzcv, FPSCR)
    it(gt)
    vstr(s0, [r3, 0])
    vcmp(s0, s2)
    vmrs(APSR_nzcv, FPSCR)
    it(lt)
    vstr(s0, [r4, 0])
    vldr(s1, [r5, 0])
    vsub(s2, s0, s1)
    vstr(s2, [r6, 0])
    vstr(s0, [r5, 0])
    add(r2, 4)                              # Go to the next array element
    add(r3, 4)                              # Go to the next array element
    add(r4, 4)                              # Go to the next array element
    add(r5, 4)                              # Go to the next array element
    add(r6, 4)                              # Go to the next array element
    sub(r1, 1)                              # Decrement the loop counter
    bgt(LOOP)
