    ;;
    ;; GameCube to N64 converter: use your GameCube controllers with an N64 console
    ;; Copyright (C) 2004 Micah Dowty <micah@navi.cx>
    ;;               2011 Jacques Gagnon <darthcloud@gmail.com>
    ;;
    ;;   This program is free software; you can redistribute it and/or modify
    ;;   it under the terms of the GNU General Public License as published by
    ;;   the Free Software Foundation; either version 2 of the License, or
    ;;   (at your option) any later version.
    ;;
    ;; This firmware is designed to run on a PIC18F14K22 microcontroller
    ;; clocked at 64 MHz using the internal 16 MHz clock multiplied by
    ;; the 4x PLL.
    ;;
    ;; See n64gc_comm.inc for code and documentation related to the protocol
    ;; used between here, the N64, and the GameCube.
    ;;
    ;; This file doesn't implement all of the N64 controller protocol, but
    ;; it should correctly emulate an official N64 controller with rumble pak.
    ;; It might not respond to unusual circumstances the same as a real N64
    ;; controller would, both due to potential gaps in the reverse engineering,
    ;; and due to corners cut in the algorithm implementation to fit it on
    ;; this microcontroller.
    ;;

    ;; Definitions for the PIC18F14K22 version
    ifdef  __18F14K22
        #include p18f14k22.inc

        CONFIG FOSC = IRC, PLLEN = ON, PCLKEN = OFF, FCMEN = OFF, IESO = OFF
        CONFIG PWRTEN = OFF, BOREN = OFF
        CONFIG WDTEN = ON, WDTPS = 1
        CONFIG HFOFST = ON, MCLRE = OFF
        CONFIG STVREN = OFF, LVP = OFF, BBSIZ = OFF, XINST = ON, DEBUG = OFF
        CONFIG CP0 = OFF, CP1 = OFF
        CONFIG CPB = OFF, CPD = OFF
        CONFIG WRT0 = OFF, WRT1 = OFF
        CONFIG WRTC = OFF, WRTB = OFF, WRTD = OFF
        CONFIG EBTR0 = OFF, EBTR1 = OFF
        CONFIG EBTRB = OFF

        #define N64_PIN         PORTB, 4
        #define N64_TRIS        TRISB, 4
        #define N64_PIN2        PORTB, 6
        #define N64_TRIS2       TRISB, 6
        #define GAMECUBE_PIN    PORTA, 4
        #define GAMECUBE_TRIS   TRISA, 4
        #define N64C_PIN        PORTA, 2
        #define N64C_TRIS       TRISA, 2
        #define LED_PIN         LATA, 5

io_init macro
        clrf    PORTA, a
        clrf    PORTB, a
        clrf    WPUA, a                          ; Disable pull-ups.
        clrf    WPUB, a
        movlw   0xDF
        movwf   TRISA, a
        setf    TRISB, a
        clrf    PORTC, a                         ; Debug port
        clrf    TRISC, a                         ; Debug port
        clrf    ANSEL, a                         ; Set IOs to digital.
        clrf    ANSELH, a
        bsf     IOCB, IOCB4, a                   ; Enable interrupt on N64_PIN.
        btfsc   N64_PIN2, a                      ; If 2nd port connected,
        bsf     IOCB, IOCB6, a                   ; enable interrupt on N64_PIN2.
        endm

    else
        messg    "Unsupported processor"
    endif

    ;; Delay of about ~2 ms that allow the PLL to shift the frequency to 64 MHz.
pll_startup_delay macro
    bcf     INTCON, TMR0IF, a                    ; Clear overflow bit.
    movlw   0x44                                 ; Set 8-bit mode and 1:32 prescaler.
    movwf   T0CON, a
    clrf    TMR0L, a                             ; Clear timer0.
    bsf     T0CON, TMR0ON, a                     ; Enable timer0 and
    btfss   INTCON, TMR0IF, a                    ; wait for timer0 overflow.
    goto    $-2
    bcf     INTCON, TMR0IF, a                    ; Clear overflow bit and
    bcf     T0CON, TMR0ON, a                     ; disable timer0.
    endm

    #include cube64.inc
    #include n64gc_comm.inc

    ;; Reset and interrupt vectors
    org 0x00
    goto    startup
    org 0x08
    clrf    STKPTR, a
    call    n64_rx_command
    call    n64_detect_command
    call    n64_reinit
    goto    int_reentry

    ;; Variables.
    cblock 0x00
        n64_status_buffer:0
        n64_status_buffer0:4
        n64_status_buffer1:4
        flags
        flags2
        option_flags
        atomic_flags
        uncommit_flags
        nv_flags
        nv_config_js
        nv_config_cs
        temp
        temp2
        temp3
        byte_count
        bit_count
        bus_byte_count
        virtual_map
        calibration_count
        level2_button
        level3_button
        target_slot_status
        temp_key_map
        crc_work

        ;; Stored calibration for each GameCube axis.
        joystick_x_calibration
        joystick_y_calibration
        cstick_x_calibration
        cstick_y_calibration
        left_calibration
        right_calibration

        ;; These four items must be contiguous.
        ;; Note that n64_command and n64_bus_address point to the same memory.
        ;; This is explained in the bus write receive code near n64_detect_command.
        n64_command:0
        n64_bus_address:2
        n64_bus_packet:.32
        n64_crc

        n64_id_buffer:3
        n64_slot_status                          ; Can't store directly in id buffer since transmission modify the buffers.
        gamecube_buffer:6                        ; Last 2 bytes for gc_buffer are within the first 2
        gamecube_curve:6                         ; bytes of gc_curve which in turn last 2 bytes for gc_curve are within the first 2
        gamecube_scale:8                         ; bytes of gc_scale as it allows using the same macro for all buffers.
    endc


    ;; *******************************************************************************
    ;; ******************************************************  Initialization  *******
    ;; *******************************************************************************

startup
    movlb   0x00                                 ; Set bank 0 active for non-mirrored data.
    clrf    FSR2H, a
    clrf    FSR2L, a
    bcf     INTCON, GIE, a                       ; Init interrupts.
    bsf     INTCON, RABIE, a
    bsf     INTCON2, RABPU, a
    bsf     INTCON2, RABIP, a
    bsf     RCON, IPEN, a
    movlw   0x70                                 ; Set internal clock to 16 MHz.
    movwf   OSCCON, a
    pll_startup_delay                            ; Wait for PLL to shift frequency to 64 MHz.
    io_init

    n64gc_init
    clrf    FSR0H, a
    call    clear_n64_status_buffer
    incf    FSR0H, f, a
    call    clear_n64_status_buffer
    clrf    flags, b
    clrf    flags2, b
    clrf    option_flags, b
    clrf    uncommit_flags, b
    clrf    atomic_flags, b
    clrf    nv_flags, b
    clrf    nv_config_js, b
    clrf    nv_config_cs, b
    clrf    calibration_count, b
    clrf    FSR0H, a
    clrf    FSR1H, a                             ; We only need to access first bank, so we set it in FSR high byte right away.
    clrf    TBLPTRU, a                           ; Preload table upper byte. Same for all tables.
    movlw   high crc_large_table                 ; Preload table high byte for CRC table initially.
    movwf   TBLPTRH, a
    bsf     LED_PIN, a

    movlw   0x01                                 ; Set controller id to occupied slot.
    movwf   n64_slot_status, b

    movlw   .34                                  ; Reset bus_byte_count to 34. Keeping this set beforehand,
    movwf   bus_byte_count, b                    ; saves a few precious cycles in receiving bus writes.


    ;; We use the watchdog to implicitly implement controller probing.
    ;; If we have no GameCube controller attached, gamecube_poll_status
    ;; will never return. We therefore never get around to answering the
    ;; N64's requests. This breaks down if we start up with the GameCube
    ;; controller plugged in but it's unplugged while the N64 is idle, but
    ;; the N64 should be polling us pretty constantly.
    ;;
    ;; The watchdog timeout therefore sets how often we retry in our search
    ;; for a responding GameCube controller. The WDTPS configuration is set to
    ;; a postscaler of 1:128, giving us a nominal watchdog timeout of 512 ms.
    clrwdt

    ;; Check our EEPROM for validity, and reset it if it's blank or corrupted.
    call    validate_eeprom

int_reentry
    movf    PORTA, w, a
    movf    PORTB, w, a
    nop
    bcf     INTCON, RABIF, a
    bsf     INTCON, GIEH, a
    call    gamecube_wait_for_idle
    call    gamecube_init
main_loop
    clrwdt
    call    update_led
    call    update_rumble_feedback               ; Give feedback for remapping operations using the rumble motor.
    call    update_slot_empty_timer              ; Report slot empty for 1 s following adaptor mode change.
    call    gamecube_poll_status                 ; The GameCube poll takes place during the dead period
    call    n64_translate_status                 ; between incoming N64 commands, hopefully.
    btg     FSR2L, 2, a                          ; Set older regs copy as working regs.
    call    gamecube_bus_wait
    goto    main_loop

    ;; n64_status_buffer init.
clear_n64_status_buffer
    movlw   n64_status_buffer
    movwf   FSR0L, a
    movlw   .8
    movwf   temp, b
clear_loop
    clrf    POSTINC0, a
    decfsz  temp, f, b
    bra     clear_loop
    return


    ;; *******************************************************************************
    ;; ******************************************************  Axis Calibration  *****
    ;; *******************************************************************************

    ;; Store calibration values for one GameCube axis. This takes its
    ;; actual neutral position from the gamecube_buffer, and is given
    ;; its ideal neutral position as a parameter. The resulting calibration
    ;; is added to the axis later, with clamping to prevent rollover.
store_calibration macro axis_byte, calibration, ideal_neutral
    movf    gamecube_buffer + axis_byte, w, b
    sublw   ideal_neutral
    movwf   calibration, b
    endm

    ;; Add the stored neutral values to an axis to calibrate it, clamping
    ;; it in the event of an overflow.
apply_calibration macro axis_byte, calibration
    local   negative
    local   done

    movf    calibration, w, b                    ; Add the calibration
    addwf   gamecube_buffer + axis_byte, f, b
    btfsc   calibration, 7, b                    ; Test whether the value we just added was negative
    goto    negative

    movlw   0xFF                                 ; It was positive, clamp to 0xFF if we carried
    btfsc   STATUS, C, a
    movwf   gamecube_buffer + axis_byte, b
    goto    done

negative
    btfss   STATUS, C, a                         ; It was negative, clamp to 0 if we borrowed (C=0)
    clrf    gamecube_buffer + axis_byte, b

done
    endm

    ;; Store calibration values for each axis. The controller's joysticks should
    ;; be centered and the L and R buttons should be released when this is called.
    ;; We assume this is true at startup, and it should be true when the user invokes
    ;; it by holding down X, Y, and Start.
gamecube_reset_calibration
    store_calibration   GC_JOYSTICK_X,  joystick_x_calibration, 0x80
    store_calibration   GC_JOYSTICK_Y,  joystick_y_calibration, 0x80
    store_calibration   GC_CSTICK_X,    cstick_x_calibration,   0x80
    store_calibration   GC_CSTICK_Y,    cstick_y_calibration,   0x80
    store_calibration   GC_L_ANALOG,    left_calibration,       0x00
    store_calibration   GC_R_ANALOG,    right_calibration,      0x00
    return

    ;; This runs at the beginning of each key remap to check for the X+Y+Start
    ;; calibration sequence. Since we don't have a good way to measure out exactly
    ;; 3 seconds to emulate the GameCube's behavior, we just count the number of
    ;; status polls the keys are held down for. We don't know the exact polling rate,
    ;; but on most games 30 polls should be around a second, which is long enough
    ;; to avoid accidentally recalibrating.
check_calibration_combo
    btfss   gamecube_buffer + GC_X, b
    goto    no_calibration_combo
    btfss   gamecube_buffer + GC_Y, b
    goto    no_calibration_combo
    btfss   gamecube_buffer + GC_START, b
    goto    no_calibration_combo

    incf    calibration_count, f, b
    movf    calibration_count, w, b
    xorlw   .30
    btfsc   STATUS, Z, a
    goto    gamecube_reset_calibration
    return

no_calibration_combo
    clrf    calibration_count, b
    return


    ;; *******************************************************************************
    ;; **********************************************  Static Button/Axis Mappings  **
    ;; *******************************************************************************

    ;; This static mapping layer translates axes directly from N64 to GameCube, and
    ;; it translates buttons via an intermetdiate virtual button ID that's used by
    ;; the dynamic mapping layer. This layer defines our default mappings.

    ;; Compare absolute value between 2 bytes and assign
    ;; the greater value to the destination.
assign_greater_abs_value macro prospect_byte, dest_byte
    local   next
    movf    dest_byte, w, a
    movwf   temp, b
    btfsc   dest_byte, 7, a                      ; If dest_byte negative then
    negf    temp, b                              ; two's complement it.
    movf    temp, w, b
    movff   prospect_byte, temp
    btfsc   prospect_byte, 7, b                  ; Same for prospect.
    negf    temp, b
    cpfsgt  temp, b                              ; If prospect abs value greater
    bra     next                                 ; than dest, overwrite it.
    movf    prospect_byte, w, b
    movwf   dest_byte, a
next
    endm

    ;; Map a GameCube button to a virtual button, and eventually to an N64 button.
map_button_from macro src_byte, src_bit, virtual
    movlw   virtual
    btfsc   gamecube_buffer + src_byte, src_bit, b
    call    remap_virtual_button
    endm

    ;; Map a virtual button to an N64 button. If the indicated button is the one
    ;; currently in virtual_button, sets the corresponding N64 bit and returns.
map_button_to macro virtual, dest_byte, dest_bit
    local   next
    movf    virtual_map, w, b
    xorlw   virtual
    btfss   STATUS, Z, a
    goto    next
    bsf     n64_status_buffer + dest_byte, dest_bit, a
    if dest_byte == 1 && dest_bit == 4
        incf    FSR2H, f, a
        bsf     n64_status_buffer + N64_Z, a
        clrf    FSR2H, a
    endif
    return
next
    endm

    ;; Map a virtual button to two N64 buttons. If the indicated button is the one
    ;; currently in virtual_button, sets the corresponding N64 bit and returns.
map_two_buttons_to macro virtual, dest_byte, dest_bit, dest_byte2, dest_bit2
    local   next
    movf    virtual_map, w, b
    xorlw   virtual
    btfss   STATUS, Z, a
    goto    next
    bsf     n64_status_buffer + dest_byte, dest_bit, a
    bsf     n64_status_buffer + dest_byte2, dest_bit2, a
    return
next
    endm

    ;; Sign an 8-bit 0x80-centered axis if sign=1.
    ;; Also apply a dead zone.
apply_sign_deadzone macro src_byte, sign
    local   negative_axis_value
    local   next

    if sign
        movlw   0x80                             ; Sign GC axis value.
        subwf   gamecube_buffer + src_byte, f, b
    endif
    movlw   AXIS_DEAD_ZONE
    btfsc   gamecube_buffer + src_byte, 7, b     ; Check value sign.
    goto    negative_axis_value

    ;; Current value is positive.
    subwf   gamecube_buffer + src_byte, f, b
    btfsc   gamecube_buffer + src_byte, 7, b
    clrf    gamecube_buffer + src_byte, b
    goto    next

    ;; Current value is negative.
negative_axis_value
    addwf   gamecube_buffer + src_byte, f, b
    btfss   gamecube_buffer + src_byte, 7, b
    clrf    gamecube_buffer + src_byte, b

next
    endm

    ;; This function apply a response curve function on the GameCube joysticks.
apply_js_curve macro table, axis_byte
    local   set_curve_buffer

    movf    gamecube_buffer + axis_byte, w, b
    btfss_config_js axis_byte, CURVE_BIT
    bra     set_curve_buffer

    movlw   high table
    movwf   TBLPTRH, a
    movf_config_js axis_byte
    andlw   LAYOUT_MASK
    addwf   TBLPTRH, f, a
    movff   gamecube_buffer + axis_byte, TBLPTRL
    TBLRD*
    movf    TABLAT, w, a
set_curve_buffer
    movwf   gamecube_curve + axis_byte, b
    endm

    ;; GameCube joysticks range differs from the N64. N64 has a maximum of ~84 along the axes origin
    ;; and ~71 in the diagonals. GameCube main joystick has a maximum (once dead zone & sign applied)
    ;; of ~90 and ~65 for the same. C joystick maximums are a bit lower at ~82 and ~57. N64 max value
    ;; once plot will give us a square-ish equilateral hexagon while GameCube is an equiangular &
    ;; equilateral hexagon.
    ;; This function uses the opposite axis as a reference to determine dynamically the scaling value
    ;; required. The scaling values are stored in a table using the fixed point format of 1.7. Once
    ;; multiplied by the joystick value (in 8.0 format) this gives us in our hardware multiplier a
    ;; value in the fixed point 9.7 format.
apply_js_scale macro table, axis_byte, ref_byte
    local   set_scale_buffer

    movf    gamecube_curve + axis_byte, w, b
    btfsc_config_js axis_byte, SCALE_BIT
    bra     set_scale_buffer

    movlw   high table
    movwf   TBLPTRH, a                           ; Load the right scaling table.
    movff   gamecube_buffer + ref_byte, TBLPTRL
    TBLRD*
    movf    TABLAT, w, a
    mulwf   gamecube_curve + axis_byte, b

    btfsc   gamecube_curve + axis_byte, 7, b
    subwf   PRODH, f, a                          ; If negative stick value, fixup high byte.

    rlcf    PRODL, w, a
    rlcf    PRODH, w, a                          ; We need an 8.0 value, so shift left our 9.7 value and copy the high byte.
set_scale_buffer
    movwf   gamecube_scale + axis_byte, b
    endm

    ;; Map a GameCube axis to a virtual button, and eventually to an N64 axis or button.
map_axis_from macro src_byte, virtual
    movff   gamecube_scale + src_byte, temp2
    movlw   virtual
    if virtual & 0x01                            ; Check direction (sign) of the virtual button.
        btfsc   temp2, 7, b                      ; Virtual button is negative, skip if buffer positive.
    else
        btfss   temp2, 7, b                      ; Virtual button is positive, skip if buffer negative.
    endif
    call    remap_virtual_axis                   ; Call if buffer sign match virtual button sign.
    endm

    ;; Map a virtual button to an N64 axis. If the indicated axis is the one
    ;; currently in virtual_button, sets the corresponding N64 byte if its
    ;; absolute value is greater than the current one.
map_axis_to macro virtual, dest_byte
    local   next
    movf    virtual_map, w, b
    xorlw   virtual
    btfss   STATUS, Z, a
    goto    next
    if virtual & 0x01                            ; Check direction (sign) of the virtual button.
        btfss   temp2, 7, b                      ; Virtual button is negative, skip if buffer negative.
    else
        btfsc   temp2, 7, b                      ; Virtual button is positive, skip if buffer positive.
    endif
    negf    temp2, b                             ; Two's complement buffer if sign mismatch.
    assign_greater_abs_value temp2, n64_status_buffer + dest_byte
    clrf    FSR2H, a
    return
next
    endm

    ;; Map an unsigned axis to a virtual button given a threshold.
map_button_axis macro axis_byte, virtual, thresh
    movlw   thresh + 1
    subwf   gamecube_buffer + axis_byte, w, b    ; Axis - (upper_thresh+1)
    movlw   virtual
    btfsc   STATUS, C, a
    call    remap_virtual_button                 ; C=1, B=0, (upper_thresh+1) <= axis
    endm

    ;; Map an 8-bit 0x80-centered axis to two virtual buttons,
    ;; given a threshold.
map_button_axis_sign macro axis_byte, lower_virtual, upper_virtual, thresh
    movlw   -thresh
    subwf   gamecube_buffer + axis_byte, w, b    ; Axis - lower_thresh
    movlw   lower_virtual
    btfsc   STATUS, OV, a
    goto    $+8
    btfsc   STATUS, N, a
    call    remap_virtual_button                 ; N=1, OV=0, lower_thresh > axis
    movlw   thresh + 1
    subwf   gamecube_buffer + axis_byte, w, b    ; Axis - (upper_thresh+1)
    movlw   upper_virtual
    btfsc   STATUS, OV, a
    goto    $+8
    btfss   STATUS, N, a
    call    remap_virtual_button                 ; N=0, OV=0, (upper_thresh+1) <= axis
    endm

    ;; Map a button to one axis direction.
map_axis_button macro virtual, dest_byte
    local   next
    movf    virtual_map, w, b
    xorlw   virtual
    btfss   STATUS, Z, a
    goto    next
    if virtual & 0x01                            ; Set value sign base on virtual button sign.
        movlw   -AXIS_BTN_VALUE
    else
        movlw   AXIS_BTN_VALUE
    endif
    movwf   n64_status_buffer + dest_byte, a     ; Could be overwritten by a real axis.
next
    endm

    ;; Copy status from the GameCube buffer to the N64 buffer. This first
    ;; stage maps all axes, and maps GameCube buttons to virtual buttons.
n64_translate_status
    movf    nv_flags, w, b
    andlw   LAYOUT_MASK
    movwf   temp_key_map, b
    call    check_calibration_combo

    apply_calibration   GC_JOYSTICK_X,  joystick_x_calibration
    apply_calibration   GC_JOYSTICK_Y,  joystick_y_calibration
    apply_calibration   GC_CSTICK_X,    cstick_x_calibration
    apply_calibration   GC_CSTICK_Y,    cstick_y_calibration
    apply_calibration   GC_L_ANALOG,    left_calibration
    apply_calibration   GC_R_ANALOG,    right_calibration

    apply_sign_deadzone GC_JOYSTICK_X, 1
    apply_sign_deadzone GC_JOYSTICK_Y, 1
    apply_sign_deadzone GC_CSTICK_X, 1
    apply_sign_deadzone GC_CSTICK_Y, 1
    apply_sign_deadzone GC_L_ANALOG, 0
    apply_sign_deadzone GC_R_ANALOG, 0

    bcf    STATUS, C, a                          ; Divide by 2 to fit N64 positive axis values. Close enough to the proper range.
    rrcf   gamecube_buffer + GC_L_ANALOG, w, b
    movwf  gamecube_scale + GC_L_ANALOG, b
    bcf    STATUS, C, a
    rrcf   gamecube_buffer + GC_R_ANALOG, w, b
    movwf  gamecube_scale + GC_R_ANALOG, b

    apply_js_curve      gamecube_js_curves, GC_JOYSTICK_X
    apply_js_curve      gamecube_js_curves, GC_JOYSTICK_Y
    apply_js_curve      gamecube_cs_curves, GC_CSTICK_X
    apply_js_curve      gamecube_cs_curves, GC_CSTICK_Y

    apply_js_scale      gamecube_js_scale, GC_JOYSTICK_X, GC_JOYSTICK_Y
    apply_js_scale      gamecube_js_scale, GC_JOYSTICK_Y, GC_JOYSTICK_X
    apply_js_scale      gamecube_cs_scale, GC_CSTICK_X, GC_CSTICK_Y
    apply_js_scale      gamecube_cs_scale, GC_CSTICK_Y, GC_CSTICK_X

    call    check_remap_combo                    ; Must be after calibration, since it uses analog L and R values

    ;; Restart here if layout modifier button is pressed.
n64_translate_restart
    clrf    n64_status_buffer + 0, a             ; Start out with everything zeroed...
    clrf    n64_status_buffer + 1, a
    clrf    n64_status_buffer + 2, a
    clrf    n64_status_buffer + 3, a
    incf    FSR2H, f, a
    clrf    n64_status_buffer + 0, a             ; Start out with everything zeroed...
    clrf    n64_status_buffer + 1, a
    clrf    n64_status_buffer + 2, a
    clrf    n64_status_buffer + 3, a
    clrf    FSR2H, a
    bsf     FLAG_NO_VIRTUAL_BTNS

    map_button_from     GC_A,       BTN_A
    map_button_from     GC_B,       BTN_B
    map_button_from     GC_Z,       BTN_RZ
    map_button_from     GC_R,       BTN_R
    map_button_from     GC_L,       BTN_L
    map_button_from     GC_START,   BTN_START

    map_button_from     GC_X,       BTN_X
    map_button_from     GC_Y,       BTN_Y

    map_button_from     GC_D_LEFT,  BTN_D_LEFT
    map_button_from     GC_D_RIGHT, BTN_D_RIGHT
    map_button_from     GC_D_UP,    BTN_D_UP
    map_button_from     GC_D_DOWN,  BTN_D_DOWN

    map_axis_from       GC_JOYSTICK_X,  BTN_LJ_LEFT
    map_axis_from       GC_JOYSTICK_X,  BTN_LJ_RIGHT
    map_axis_from       GC_JOYSTICK_Y,  BTN_LJ_DOWN
    map_axis_from       GC_JOYSTICK_Y,  BTN_LJ_UP
    map_axis_from       GC_CSTICK_X,  BTN_RJ_LEFT
    map_axis_from       GC_CSTICK_X,  BTN_RJ_RIGHT
    map_axis_from       GC_CSTICK_Y,  BTN_RJ_DOWN
    map_axis_from       GC_CSTICK_Y,  BTN_RJ_UP
    map_axis_from       GC_R_ANALOG,  BTN_RA
    map_axis_from       GC_L_ANALOG,  BTN_LA

    bsf     FLAG_AXIS
    map_button_axis_sign     GC_JOYSTICK_X, BTN_LJ_LEFT, BTN_LJ_RIGHT, AXIS_BTN_THRS
    map_button_axis_sign     GC_JOYSTICK_Y, BTN_LJ_DOWN, BTN_LJ_UP, AXIS_BTN_THRS
    map_button_axis_sign     GC_CSTICK_X, BTN_RJ_LEFT, BTN_RJ_RIGHT, AXIS_BTN_THRS
    map_button_axis_sign     GC_CSTICK_Y, BTN_RJ_DOWN, BTN_RJ_UP, AXIS_BTN_THRS
    map_button_axis          GC_R_ANALOG, BTN_RA, TRIGGER_BTN_THRS
    map_button_axis          GC_L_ANALOG, BTN_LA, TRIGGER_BTN_THRS
    bcf     FLAG_AXIS

    btfsc   FLAG_NO_VIRTUAL_BTNS
    bcf     AFLAG_WAITING_FOR_RELEASE
    bcf     FLAG_LAYOUT_MODIFIER
    return

    ;; This is called by remap_virtual_button to convert a virtual button code,
    ;; in virtual_button, to a set bit in the N64 status packet.
set_virtual_button
    map_button_to   BTN_D_RIGHT,    N64_D_RIGHT
    map_button_to   BTN_D_LEFT,     N64_D_LEFT
    map_button_to   BTN_D_DOWN,     N64_D_DOWN
    map_button_to   BTN_D_UP,       N64_D_UP

    map_button_to   BTN_START,      N64_START
    map_button_to   BTN_RZ,         N64_Z
    map_button_to   BTN_B,          N64_B
    map_button_to   BTN_A,          N64_A
    map_button_to   BTN_R,          N64_R
    map_button_to   BTN_L,          N64_L
    map_button_to   BTN_RA,         N64_R
    map_button_to   BTN_LA,         N64_L

    map_button_to   BTN_RJ_RIGHT,   N64_C_RIGHT
    map_button_to   BTN_RJ_LEFT,    N64_C_LEFT
    map_button_to   BTN_RJ_DOWN,    N64_C_DOWN
    map_button_to   BTN_RJ_UP,      N64_C_UP

    btfsc   FLAG_AXIS
    return

    map_axis_button BTN_LJ_RIGHT,   N64_JOYSTICK_X
    map_axis_button BTN_LJ_LEFT,    N64_JOYSTICK_X
    map_axis_button BTN_LJ_DOWN,    N64_JOYSTICK_Y
    map_axis_button BTN_LJ_UP,      N64_JOYSTICK_Y
    return

set_virtual_axis
    map_axis_to     BTN_LJ_LEFT,    N64_JOYSTICK_X
    map_axis_to     BTN_LJ_RIGHT,   N64_JOYSTICK_X
    map_axis_to     BTN_LJ_DOWN,    N64_JOYSTICK_Y
    map_axis_to     BTN_LJ_UP,      N64_JOYSTICK_Y

    incf    FSR2H, f, a
    map_axis_to     BTN_RJ_LEFT,    N64_JOYSTICK_X
    map_axis_to     BTN_RJ_RIGHT,   N64_JOYSTICK_X
    map_axis_to     BTN_RJ_DOWN,    N64_JOYSTICK_Y
    map_axis_to     BTN_RJ_UP,      N64_JOYSTICK_Y
    clrf    FSR2H, a
    return

    ;; This is called by remap_virtual_button to convert a virtual button code,
    ;; in virtual button, to a special function of the adapter. This set of virtual
    ;; buttons do not result in any key press on the host system.
set_special_button
    map_two_buttons_to   BTN_A_C_DOWN, N64_A, N64_C_DOWN
    map_two_buttons_to   BTN_B_C_LEFT, N64_B, N64_C_LEFT

    btfsc   FLAG_LAYOUT_MODIFIER                 ; Allow only one level of layout modifier.
    bra     skip_layout_modifier

    ;; Check for layout modifier special function.
    movf    virtual_map, w, b
    andlw   ~LAYOUT_MASK
    xorlw   BTN_MODIFIER
    bz      special_layout_modifier

skip_layout_modifier
    return

    ;; We got a layout modifier button and we need to start over
    ;; the mapping in n64_translate_status.
special_layout_modifier
    bsf     FLAG_LAYOUT_MODIFIER
    movlw   LAYOUT_MASK
    andwf   virtual_map, w, b
    movwf   temp_key_map, b
    pop                                          ; Pop the stack since we abort this call.
    goto    n64_translate_restart


    ;; *******************************************************************************
    ;; *************************************************  Dynamic Button Remapping  **
    ;; *******************************************************************************

    ;; This is called each time we poll status, for each virtual button that's pressed.
    ;; Here we get a virtual button code in 'w'. Normally we remap this via the EEPROM
    ;; and pass the code on to set_virtual_button.
    ;;
    ;; If we're awaiting a keypress for remapping purposes, this doesn't give any virtual
    ;; button presses to the N64.
    ;;
    ;; Our EEPROM starts with one byte per virtual button, containing the virtual button
    ;; code its mapped to. By default, each byte just contains its address, mapping all
    ;; virtual buttons to themselves.

remap_virtual_button
    ;; Remember that a button is pressed, so we can detect when all have been released
    bcf     FLAG_NO_VIRTUAL_BTNS

    ;; Leave now if we're waiting for buttons to be released
    btfsc   AFLAG_WAITING_FOR_RELEASE
    return

    ;; Accept buttons presses if we're waiting for one
    btfsc   AFLAG_MENU_LEVEL1
    goto    menu_level1
    btfsc   AFLAG_MENU_LEVEL2
    goto    menu_level2
    btfsc   AFLAG_MENU_LEVEL3
    goto    menu_level3

    ;; Pass anything else on to the N64, mapped through the EEPROM first
    eeprom_btn_addr temp_key_map, 0
    call    eeread
    andlw   BTN_MASK | SPECIAL_MASK
    movwf   virtual_map, b

    btfss   virtual_map, SPECIAL_BIT, b
    goto    set_virtual_button
    goto    set_special_button

remap_virtual_axis
    eeprom_btn_addr temp_key_map, 0
    call    eeread
    andlw   BTN_MASK | SPECIAL_MASK
    movwf   virtual_map, b
    goto    set_virtual_axis

    ;; Looks for the key combinations we use to change button mapping
check_remap_combo
    ;; Leave now if we're waiting for buttons to be released
    btfsc   AFLAG_WAITING_FOR_RELEASE
    return

    ;; Key combinations require that the L and R buttons be mostly pressed.
    ;; but that the end stop buttons aren't pressed.
    ;; Ensure the high bit of each axis is set and that the buttons are cleared.
    btfss   gamecube_buffer + GC_L_ANALOG, 7, b
    return
    btfss   gamecube_buffer + GC_R_ANALOG, 7, b
    return
    btfsc   gamecube_buffer + GC_L, b
    return
    btfsc   gamecube_buffer + GC_R, b
    return

    ;; Enter config menu if third key is Start.
    btfss   gamecube_buffer + GC_START, b
    return

    ;; The config menu button combo was pressed. Give feedback via the rumble motor,
    ;; and await button presses from the user indicating which menu option they want
    ;; access to. Selection is handled into remap_virtual_button since we need virtual
    ;; button codes.
    bcf     LED_PIN, a
    movff   atomic_flags, uncommit_flags
    bsf     UFLAG_WAITING_FOR_RELEASE
    bsf     UFLAG_MENU_LEVEL1
    goto    start_rumble_feedback

    ;; Set the right flags for next user input 
menu_level1
    clrf    option_flags, b
    movff   atomic_flags, uncommit_flags
    bcf     UFLAG_MENU_LEVEL1
    bsf     UFLAG_WAITING_FOR_RELEASE

    ;; The remap button combo was pressed. Give feedback via the rumble motor,
    ;; and await button presses from the user indicating what they want to remap.
    ;; We actually read the source and destination keys in remap_virtual_button,
    ;; since we need virtual button codes.
    btfsc   gamecube_buffer + GC_START, b
    bra     menu_level1_remap

    ;; The special function combo was pressed. Same process as remap combo.
    btfsc   gamecube_buffer + GC_Y, b
    bra     menu_level1_special

    bcf     FLAG_TRIGGER                         ; Flag not used in following commands.

    ;; The analog trigger mapping combo was pressed.
    ;; This modify both remap and special combo to allow mapping to analog trigger.
    btfsc   gamecube_buffer + GC_X, b
    bra     menu_level1_trigger_flag_set

    ;; The reset combo was pressed. Reset the EEPROM contents of the current active button
    ;; layout, and use the rumble motor for feedback if possible.
    btfsc   gamecube_buffer + GC_Z, b
    bra     menu_level1_reset_active_layout

    ;; The adapter mode submenu was selected in the config menu.
    btfsc   gamecube_buffer + GC_D_UP, b
    bra     menu_level1_mode

    ;; The button layout submenu was selected in the config menu.
    btfsc   gamecube_buffer + GC_D_LEFT, b
    bra     menu_level1_layout

    ;; Joysticks config menu.
    btfsc   gamecube_buffer + GC_D_RIGHT, b
    bra     menu_level1_joystick

    ;; Layout preset menu.
    btfsc   gamecube_buffer + GC_D_DOWN, b
    bra     menu_level1_preset

    movff   uncommit_flags, atomic_flags         ; Set flags as atomic operations. Avoid disabling interrupt.
    return

menu_level1_mode
    bsf     OFLAG_MODE
    bsf     UFLAG_MENU_LEVEL2
    goto    start_rumble_feedback

menu_level1_layout
    bsf     OFLAG_LAYOUT
    bsf     UFLAG_MENU_LEVEL2
    goto    start_rumble_feedback

menu_level1_joystick
    bsf     OFLAG_JOYSTICK
    bsf     UFLAG_MENU_LEVEL2
    goto    start_rumble_feedback

menu_level1_preset
    bsf     OFLAG_PRESET
    bsf     UFLAG_MENU_LEVEL2
    goto    start_rumble_feedback

menu_level1_remap
    bsf     OFLAG_REMAP
    bsf     UFLAG_MENU_LEVEL2
    goto    start_rumble_feedback

menu_level1_special
    bsf     OFLAG_SPECIAL
    bsf     UFLAG_MENU_LEVEL2
    goto    start_rumble_feedback

menu_level1_reset_active_layout
    movlw   0x00
    call    reset_active_eeprom_layout
    goto    start_rumble_feedback

menu_level1_trigger_flag_set
    bsf     FLAG_TRIGGER
    bsf     UFLAG_MENU_LEVEL1
    goto    start_rumble_feedback

    ;; Accept the virtual button code in 'w'. Jump to proper submenu handler.
menu_level2
    movwf   level2_button, b
    movff   atomic_flags, uncommit_flags
    bsf     UFLAG_WAITING_FOR_RELEASE
    bcf     UFLAG_MENU_LEVEL2

    bcf     FLAG_CS
    bcf     FLAG_AXIS_Y

    btfsc   OFLAG_MODE
    bra     menu_level2_mode
    btfsc   OFLAG_LAYOUT
    bra     menu_level2_layout
    btfsc   OFLAG_PRESET
    bra     menu_level2_preset

    bsf     UFLAG_MENU_LEVEL3

    btfsc   OFLAG_JOYSTICK
    bra     menu_level2_joystick

    goto    start_rumble_feedback

    ;; Accept the adapter mode selection.
menu_level2_mode
    movwf   target_slot_status, b
    xorlw   BTN_D_DOWN
    bnz     menu_level2_mode_valid
    movff   uncommit_flags, atomic_flags         ; Set flags as atomic operations. Avoid disabling interrupt.
    return

menu_level2_mode_valid
    movlw   0x02
    movwf   n64_slot_status, b

    bsf     FLAG_FORCE_EMPTIED
    bcf     FLAG_BYPASS_MODE
    goto    start_rumble_feedback

    ;; Accept the button layout selection.
menu_level2_layout
    andlw   ~LAYOUT_MASK                         ; Check for any D-pad direction.
    bnz     menu_level2_layout_invalid

    movlw   ~LAYOUT_MASK
    andwf   nv_flags, f, b
    movf    level2_button, w, b
    iorwf   nv_flags, f, b

    movff   nv_flags, EEDATA
    movlw   EEPROM_NV_FLAGS
    movwf   EEADR, a
    call    eewrite

    movf    nv_flags, w, b
    andlw   LAYOUT_MASK
    movwf   temp_key_map, b

    movlw   CONFIG_JS
    eeprom_btn_addr temp_key_map, 0
    call    eeread
    movwf   nv_config_js, b

    movlw   CONFIG_CS
    eeprom_btn_addr temp_key_map, 0
    call    eeread
    movwf   nv_config_cs, b
    goto    start_rumble_feedback

menu_level2_layout_invalid
    movff   uncommit_flags, atomic_flags         ; Set flags as atomic operations. Avoid disabling interrupt.
    return

menu_level2_joystick
    andlw   ~LAYOUT_MASK
    xorlw   BTN_LJ_UP
    bz      menu_level2_joystick_check_axis

    movf    level2_button, w, b
    andlw   ~LAYOUT_MASK
    xorlw   BTN_RJ_UP
    bz      menu_level2_joystick_check_axis-2

    bcf     UFLAG_MENU_LEVEL3
    movff   uncommit_flags, atomic_flags
    return

    bsf     FLAG_CS
menu_level2_joystick_check_axis
    decf    level2_button, f, b
    btfsc   level2_button, 1, b                  ; Check if X or Y axis.
    bsf     FLAG_AXIS_Y
    incf    level2_button, f, b

    goto    start_rumble_feedback

menu_level2_preset
    andlw   ~PRESET_MASK
    bnz     menu_level2_preset_invalid

    bcf     STATUS, C, a
    rlcf    level2_button, f, b
    swapf   level2_button, w, b
    call    reset_active_eeprom_layout
    goto    start_rumble_feedback

menu_level2_preset_invalid
    movff   uncommit_flags, atomic_flags         ; Set flags as atomic operations. Avoid disabling interrupt.
    return

    ;; Accept the virtual button code in 'w'. Jump to proper submenu handler.
menu_level3
    movwf   level3_button, b
    movff   atomic_flags, uncommit_flags
    bsf     UFLAG_WAITING_FOR_RELEASE
    bcf     UFLAG_MENU_LEVEL3

    btfsc   OFLAG_JOYSTICK
    bra     menu_level3_joystick
    btfsc   OFLAG_REMAP
    bra     menu_level3_remap
    btfsc   OFLAG_SPECIAL
    bra     menu_level3_special

    movff   uncommit_flags, atomic_flags         ; Should not get here.
    return

menu_level3_joystick
    movlw   nv_config_js
    movwf   FSR0L, a
    movlw   nv_config_cs
    btfsc   FLAG_CS
    movwf   FSR0L, a

    btfsc   gamecube_buffer + GC_A, b
    bra     menu_level3_joystick_clr_curve

    btfsc   gamecube_buffer + GC_X, b
    bra     menu_level3_joystick_clr_not_scale

    btfsc   gamecube_buffer + GC_Y, b
    bra     menu_level3_joystick_set_not_scale

    movf    level3_button, w, b
    andlw   ~LAYOUT_MASK
    bz      menu_level3_joystick_set_curve

    movff   uncommit_flags, atomic_flags
    return

menu_level3_joystick_clr_curve
    movlw   ~CURVE_MASK
    btfsc   FLAG_AXIS_Y
    movlw   ~CURVE_MASK_Y
    andwf   INDF0, f, a
    bra     menu_level3_joystick_save

menu_level3_joystick_set_curve
    movlw   ~CURVE_MASK
    btfsc   FLAG_AXIS_Y
    movlw   ~CURVE_MASK_Y
    andwf   INDF0, f, a

    movf    level3_button, w, b
    btfsc   FLAG_AXIS_Y
    swapf   level3_button, w, b
    iorwf   INDF0, f, a

    btfss   FLAG_AXIS_Y
    bsf     INDF0, CURVE_BIT, a
    btfsc   FLAG_AXIS_Y
    bsf     INDF0, CURVE_BIT_Y, a
    bra     menu_level3_joystick_save

menu_level3_joystick_clr_not_scale
    btfss   FLAG_AXIS_Y
    bcf     INDF0, SCALE_BIT, a
    btfsc   FLAG_AXIS_Y
    bcf     INDF0, SCALE_BIT_Y, a
    bra     menu_level3_joystick_save

menu_level3_joystick_set_not_scale
    btfss   FLAG_AXIS_Y
    bsf     INDF0, SCALE_BIT, a
    btfsc   FLAG_AXIS_Y
    bsf     INDF0, SCALE_BIT_Y, a

menu_level3_joystick_save
    movlw   CONFIG_JS
    btfsc   FLAG_CS
    movlw   CONFIG_CS
    eeprom_btn_addr temp_key_map, 0
    movwf   EEADR, a
    movff   INDF0, EEDATA
    call    eewrite
    goto    start_rumble_feedback

    ;; Accept the virtual button code for the remap destination in 'w', and write
    ;; the button mapping to EEPROM.
menu_level3_remap
    movwf   EEDATA, a                            ; Destination button is data, source is address.
    bra     menu_level3_common

    ;; Accept the virtual button code for the special function destination in 'w'.
menu_level3_special
    movwf   EEDATA, a

    ;; Two buttons special mapping.
    movf    EEDATA, w ,a
    andlw   ~0x01                                ; Check buttons A and B.
    xorlw   BTN_A
    bz      menu_level3_common - 2

    ;; Validate if one of the D-pad directions is pressed for the layout modifier function.
    ;; Save as a special button if so, return otherwise.
    movf    EEDATA, w, a
    andlw   ~LAYOUT_MASK                         ; Check for any D-pad direction.
    bz      menu_level3_common - 2
    bcf     FLAG_TRIGGER
    movff   uncommit_flags, atomic_flags         ; Set flags as atomic operations. Avoid disabling interrupt.
    return

    bsf     EEDATA, SPECIAL_BIT, a
menu_level3_common
    movf    level2_button, w, b
    movwf   temp2, b
    btfsc   FLAG_TRIGGER                         ; If flag set, this means we want to allow analog trigger mapping.
    bra     menu_level3_common_save_mapping

    andlw   TRIGGER_TYPE_MASK
    xorlw   BTN_LA
    btfsc   STATUS, Z, a                         ; If analog trigger, overwrite source with digital trigger.
    incf    temp2, f, b

menu_level3_common_save_mapping
    movf    temp2, w, b
    mullw   EEPROM_BTN_BYTE                      ; Offset base on how many bytes per button.
    movff   PRODL, EEADR
    movf    nv_flags, w, b                       ; Add offset to EEPROM address to read the right custom buttons layout.
    andlw   LAYOUT_MASK
    mullw   EEPROM_LAYOUT_SIZE
    movf    PRODL, w, a
    addwf   EEADR, f, a
    call    eewrite
    bcf     FLAG_TRIGGER
    goto    start_rumble_feedback


    ;; Check our EEPROM for the magic word identifying it as button mapping data for
    ;; this version of our firmware. If we don't find the magic word, reset its contents.
validate_eeprom
    movlw   EEPROM_MAGIC_ADDR                    ; Check high byte
    call    eeread
    xorlw   EEPROM_MAGIC_WORD >> 8
    btfss   STATUS, Z, a
    goto    reset_eeprom
    movlw   EEPROM_MAGIC_ADDR + 1                ; Check low byte
    call    eeread
    xorlw   EEPROM_MAGIC_WORD & 0xFF
    btfss   STATUS, Z, a
    goto    reset_eeprom

    movlw   EEPROM_NV_FLAGS                      ; Load last used custom layout.
    call    eeread
    movwf   nv_flags, b
    andlw   LAYOUT_MASK
    movwf   temp_key_map, b

    movlw   CONFIG_JS
    eeprom_btn_addr temp_key_map, 0
    call    eeread
    movwf   nv_config_js, b

    movlw   CONFIG_CS
    eeprom_btn_addr temp_key_map, 0
    call    eeread
    movwf   nv_config_cs, b
    return

    ;; Write an identity mapping and a valid magic word to the EEPROM.
reset_eeprom
    movlw   high eeprom_default                  ; Load address for EEPROM layout default.
    movwf   TBLPTRH, a

    movlw   EEPROM_LAYOUT_0                      ; Loop over all virtual buttons, writing the identity mapping.
    movwf   EEADR, a
next_eeprom_bank
    clrf    TBLPTRL, a
    TBLRD*
    movff   TABLAT, EEDATA
    call    reset_next_byte
    movf    EEADR, w, a
    xorlw   EEPROM_LAYOUT_0 + EEPROM_LAYOUT_SIZE * 4
    btfss   STATUS, Z, a
    goto    next_eeprom_bank

    movlw   EEPROM_MAGIC_ADDR                    ; Write the magic word
    movwf   EEADR, a
    movlw   EEPROM_MAGIC_WORD >> 8
    movwf   EEDATA, a
    call    eewrite
    movlw   EEPROM_MAGIC_ADDR + 1
    movwf   EEADR, a
    movlw   EEPROM_MAGIC_WORD & 0xFF
    movwf   EEDATA, a
    call    eewrite

    movlw   EEPROM_NV_FLAGS                      ; Init default layout in EEPROM.
    movwf   EEADR, a
    movlw   0x00
    movwf   EEDATA, a
    goto    eewrite

    ;; Reset only data relative to the current active button mapping layout
    ;; using preset in 'w'.
reset_active_eeprom_layout
    bcf     INTCON, GIEH, a
    clrf    nv_config_js, b
    clrf    nv_config_cs, b
    movwf   TBLPTRL, a
    movlw   high eeprom_default                  ; Load address for EEPROM layout default.
    movwf   TBLPTRH, a

    movf    nv_flags, w, b
    andlw   LAYOUT_MASK
    mullw   EEPROM_LAYOUT_SIZE
    movff   PRODL, EEADR
    TBLRD*
    movff   TABLAT, EEDATA

    ;; Reset to default all button beginning at the current address set.
reset_next_byte
    call    eewrite
    incf    EEADR, f, a
    TBLRD+*
    movff   TABLAT, EEDATA
    movf    TBLPTRL, w, a
    xorlw   EEPROM_LAYOUT_SIZE
    btfss   STATUS, Z, a
    goto    reset_next_byte
    movf    PORTA, w, a
    movf    PORTB, w, a
    nop
    bcf     INTCON, RABIF, a
    bsf     INTCON, GIEH, a
    return

    ;; Read from address 'w' of the EEPROM, return in 'w'.
eeread
    movwf   EEADR, a
    bcf     EECON1, EEPGD, a                     ; Select EEPROM.
    bcf     EECON1, CFGS, a
    bsf     EECON1, RD, a
    movf    EEDATA, w, a
    return

    ;; Write to the EEPROM using the current EEADR and EEDATA values,
    ;; block until the write is complete.
eewrite
    clrwdt
    bcf     EECON1, EEPGD, a                     ; Select EEPROM.
    bcf     EECON1, CFGS, a
    bsf     EECON1, WREN, a                      ; Enable write
    movlw   0x55                                 ; Write the magic sequence to EECON2
    movwf   EECON2, a
    movlw   0xAA
    movwf   EECON2, a
    bsf     EECON1, WR, a                        ; Begin write
    btfsc   EECON1, WR, a                        ; Wait for it to finish...
    goto    $-2
    bcf     EECON1, WREN, a                      ; Write protect
    return

    ;; Briefly enable the rumble motor on our own, as feedback during remap combos.
    ;; This use TMR0 in 16-bit mode and will provide feedback for 250 ms.
start_rumble_feedback
    bsf     FLAG_RUMBLE_FEEDBACK
    bcf     INTCON, TMR0IF, a                    ; Clear overflow bit.
    movlw   0x87                                 ; Enable 16-bit mode with 1:256 prescaler.
    movwf   T0CON, a
    movlw   0xC2
    movwf   TMR0H, a
    movlw   0xF6
    movwf   TMR0L, a                             ; TMR0 now loaded with 0xC2F6.
    movff   uncommit_flags, atomic_flags         ; Set flags as atomic operations. Avoid disabling interrupt.
    return

    ;; At each status poll, turn on the rumble motor if we're in the middle of
    ;; giving feedback.
update_rumble_feedback
    btfss   FLAG_RUMBLE_FEEDBACK
    return
    bsf     FLAG_RUMBLE_MOTOR_ON

    btfss   INTCON, TMR0IF, a
    return
    bcf     T0CON, TMR0ON, a
    movf    flags, w, b
    andlw   ~BIT_RUMBLE_FEEDBACK                 ; bcf     FLAG_RUMBLE_FEEDBACK
    andlw   ~BIT_RUMBLE_MOTOR_ON                 ; bcf     FLAG_RUMBLE_MOTOR_ON, We need to turn off the motor when we're done.
    movwf   flags, b                             ; Set flags as atomic operations. Avoid disabling interrupt.
    return

    ;; When switching between adaptor mode, we need to report the controller slot as
    ;; empty for at least 1 s. Otherwise the N64 wouldn't know we might have changed
    ;; accessories. The rumble feedback already provide 250 ms. This set TMR0 for
    ;; another 750 ms.
start_slot_empty_timer
    bcf     INTCON, TMR0IF, a                    ; Clear overflow bit.
    movlw   0x87                                 ; Enable 16-bit mode with 1:256 prescaler.
    movwf   T0CON, a
    movlw   0x48
    movwf   TMR0H, a
    movlw   0xE4
    movwf   TMR0L, a                             ; TMR0 now loaded with 0x48E4.
    return

    ;; Start timer after rumble feedback is done. Then once the timer overflow
    ;; set the proper slot status and bypass mode.
update_slot_empty_timer
    btfss   FLAG_FORCE_EMPTIED
    return
    btfsc   FLAG_RUMBLE_FEEDBACK                 ; Let rumble feedback finish first.
    return
    btfss   T0CON, TMR0ON, a                     ; Init timer.
    call    start_slot_empty_timer
    btfss   INTCON, TMR0IF, a
    return

    movf    flags, w ,b
    andlw   ~BIT_FORCE_EMPTIED                   ; bcf     FLAG_FORCE_EMPTIED
    bcf     T0CON, TMR0ON, a
    movf    target_slot_status, f, b
    btfsc   STATUS, Z, a                         ; If 0x00 we need to set bypass mode.
    iorlw   BIT_BYPASS_MODE                      ; bsf     FLAG_BYPASS_MODE
    movff   target_slot_status, n64_slot_status
    movwf   flags, b                             ; Set flags as atomic operations. Avoid disabling interrupt.
    return

update_led
    movf    atomic_flags, w, b
    btfsc   STATUS, Z, a
    bsf     LED_PIN, a
    return


    ;; *******************************************************************************
    ;; ******************************************************  N64 Interface *********
    ;; *******************************************************************************

n64_reinit
    clrf    FSR1H, a
    clrf    FSR2H, a
    btg     FSR2L, 2, a
    bsf     N64C_TRIS, a

    bcf     FLAG_AXIS
    bcf     FLAG_LAYOUT_MODIFIER
    bcf     FLAG_CTRL2
    return

    ;; Service commands coming in from the N64
n64_detect_command
    btfsc   FLAG_CTRL2
    bra     n64_detect_base_cmd

ifndef DBG_TRACE
    btfsc   FLAG_BYPASS_MODE
endif
    bra     n64_bypass_mode

    ;; We need to handle controller pak writes very fast because there's no pause
    ;; between the initial command byte and the 34 bytes following. Every
    ;; extra instuction here increases the probability of missing the first bit.
    ;;
    ;; FSR is already pointing at the right buffer- n64_rx will leave FSR pointing
    ;; at the last byte it read, which is n64_command. We've overlaid n64_command
    ;; onto the same memory as our address buffer, and the data buffer is immediately
    ;; after our address buffer.
    ;;
    ;; Since n64_rx must be called as soon as possible, this skips n64_rx if we're not
    ;; doing a write_bus command, and after the fact detects that and looks for other commands.
    ;;
    movlw   N64_COMMAND_WRITE_BUS
    xorwf   n64_command, w, b
    btfsc   STATUS, Z, a
    call    n64_rx_bus                           ; Do another receive if this was a write_bus command.

    movlw   n64_command                          ; n64_command itself might be invalid now. If FSR changed,
    xorwf   FSR1L, w, a                          ; n64_command is invalid and we're doing a bus write
    btfss   STATUS, Z, a                         ; to send the CRC.
    goto    n64_bus_write

    movf    n64_command, w, b                    ; Detect other applicable commands...
    xorlw   N64_COMMAND_READ_BUS
    btfsc   STATUS, Z, a
    goto    n64_bus_read

    bsf     N64C_TRIS, a                         ; Reset bypass bus state.

n64_detect_base_cmd
    movf    n64_command, w, b                    ; Check for both identity cmd (0x00 & 0xFF) at the same time.
    btfss   STATUS, Z, a
    comf    n64_command, w, b
    btfsc   STATUS, Z, a
    goto    n64_send_id

    movf    n64_command, w, b
    xorlw   N64_COMMAND_STATUS
    btfsc   STATUS, Z, a
    goto    n64_send_status

    return

    ;; Macro for completing the last bit and sending 1 us stop bit.
stop_bit macro cycle
    wait    cycle
    bsf     N64C_TRIS, a
    wait    .15
    bcf     N64C_TRIS, a
    wait    .15
    bsf     N64C_TRIS, a
    endm

    ;; In bypass mode we only handle the status command. Identity, read and write commands
    ;; are left to the real N64 controller to answer.
n64_bypass_mode
    ;; Need to get back receiving for read and write commands.
    movf    n64_command, w, b
    xorlw   N64_COMMAND_WRITE_BUS
    btfsc   STATUS, Z, a
    goto    n64_write_copy

    movf    n64_command, w, b
    xorlw   N64_COMMAND_READ_BUS
    btfsc   STATUS, Z, a
    goto    n64_read_copy

    stop_bit .15                                 ; Send 1 us stop bit to N64 controller.

    ;; Identity is bypassed too since it provides slot information.
    movf    n64_command, w, b                    ; Check for both identity cmd (0x00 & 0xFF) at the same time.
    btfss   STATUS, Z, a
    comf    n64_command, w, b
    btfsc   STATUS, Z, a
    goto    n64_identity_copy

    ;; Adaptor only answer status command.
    movf    n64_command, w, b
    xorlw   N64_COMMAND_STATUS
    btfsc   STATUS, Z, a

    ;; In tracing mode the adaptor simply passthough everything and
    ;; copy it on the debug port.
ifdef DBG_TRACE
    goto    n64_status_copy
else
    goto    n64_send_status
endif

    return

    ;; Copy 3 bytes from controller to host.
n64_identity_copy
    movlw   3
    bra     n64_bus_copy_device

    ;; Copy 4 bytes from controller to host.
n64_status_copy
    movlw   4
    bra     n64_bus_copy_device

    ;; Copy remaining 2 address bytes then send 1 us stop bit.
    ;; Then, copy 33 bytes from controller to host.
n64_read_copy
    wait    .8
    movlw   2
    call    n64_bus_copy_host
    stop_bit .31                                 ; Send 1 us stop bit to N64 controller.
    movlw   .33
    bra     n64_bus_copy_device

    ;; Copy remaining 32 bytes then send 1 us stop bit.
    ;; Then, copy 1 bytes CRC from controller to host.
n64_write_copy
    wait    .8
    movlw   .34
    call    n64_bus_copy_host
    stop_bit .31                                 ; Send 1 us stop bit to N64 controller.
    movlw   1
    bra     n64_bus_copy_device

    ;; Receive from the host and copy to dummy controller.
    ;; Send no stop bit.
n64_bus_copy_host
    movwf   byte_count, b
    bsf     N64C_TRIS, a
    bcf     N64C_PIN, a
    n64_bus_copy N64_PIN, N64C_TRIS, byte_count, 0, 0

    ;; Receive from dummy controller and copy to host.
    ;; Send 2 us stop bit.
n64_bus_copy_device
    movwf   byte_count, b
    bsf     N64_TRIS, a
    bcf     N64_PIN, a
    n64_bus_copy N64C_PIN, N64_TRIS, byte_count, 0, 1

    ;; The N64 requested a 32-byte write to our controller pak bus.
    ;; The start address is given in the high 11 bits of n64_bus_address.
    ;; The low 5 bits are to verify the address- the algorithm for this is
    ;; known, but to save time they are currently ignored.
    ;; Addresses from 0x0000 to 0x7FFF are only used by the memory pak's RAM.
    ;; To emulate a rumble pak, we should only need to respond to configuration
    ;; writes at 0x8000 and rumble pak motor writes at 0xC000.
    ;;
    ;; Since all the packets we'll get while emulating a rumble pak are copies
    ;; of the same byte, we assume this is always true and use a table storing
    ;; the checksums of all such packets. We always negate the checksum to indicate
    ;; that a controller pak has been detected and initialized properly.
n64_bus_write
    movlw   .25                                  ; We have about 3us to kill here, we don't
    movwf   bus_byte_count, b                    ; want to begin transmitting before the stop bit is over.
time_killing
    decfsz  bus_byte_count, f, b
    goto    time_killing

    movlw   .34                                  ; Reset bus_byte_count to 34. Keeping this set beforehand
    movwf   bus_byte_count, b                    ; saves a few precious cycles in receiving bus writes.

    movf    crc_work, w, b                       ; Computed CRC already in crc_work.
    xorlw   0xFF                                 ; Negate the CRC, we emulate a rumble pak.
    movwf   n64_crc, b                           ; Send back the CRC in a 1-byte transmission.
    movlw   n64_crc
    movwf   FSR1L, a
    movlw   1
    call    n64_tx                               ; We need a 2us stop bit after all CRCs.

    movf    n64_bus_address, w, b                ; Is this a write to the rumble pak?
    xorlw   0xC0                                 ; (only check the top 8 bits. This excludes a few address bits and all check bits).
    btfss   STATUS, Z, a
    return                                       ; Nope, return. We ignore the initialization writes to 0x8000.

    btfss   n64_slot_status, 0, b                ; Do not rumble if we are supose to be an empty controller.
    return

    bcf     FLAG_RUMBLE_MOTOR_ON                 ; Set the rumble flag from the low bit of the first data byte.
    btfsc   n64_bus_packet + 0, 0, b
    bsf     FLAG_RUMBLE_MOTOR_ON
    return

    ;; The N64 requested a 32-byte read from our controller pak bus.
    ;; If all is well, this should only happen at address 0x8000, where it
    ;; tries to identify what type of controller pak we have. Always
    ;; indicate we have a rumble pak by sending all 0x80s.
n64_bus_read
    movlw   .2                                   ; Read 2 address bytes.
    movwf   byte_count, b                        ; FSR already point at n64_bus_address.
    call    n64_rx_address

    movlw   .32
    movwf   byte_count, b

    movlw   0x01                                 ; Check if address is 0x8001, answer 0x80s if so.
    xorwf   n64_bus_address + 1, w, b
    bnz     zero_packet
    movlw   0x80
    xorwf   n64_bus_address + 0, f, b
    bnz     zero_packet
    bra     bus_read_fill_loop                   ; We conveniently already got 0x80 in w.
zero_packet
    movlw   0x00                                 ; Otherwise reply 0x00s.

bus_read_fill_loop
    movwf   PREINC1, a
    decfsz  byte_count, f, b
    goto    bus_read_fill_loop

    setf    n64_crc, b                           ; Preload n64_crc for final CRC XOR.

    movlw   n64_bus_packet                       ; Send back the data and CRC.
    movwf   FSR1L, a
    movlw   .33                                  ; Send 32 bytes data and 1 byte CRC right after.
    goto    n64_tx                               ; We need a 2us stop bit after all CRCs.

    ;; The N64 asked for our button and joystick status.
n64_send_status
    movff   FSR2H, FSR1H
    movf    FSR2L, w, a
    addlw   n64_status_buffer
    movwf   FSR1L, a

    movlw   4
    goto    n64_tx

    ;; The N64 asked for our identity. Report that we're an
    ;; N64 controller with the controller pak slot occupied or empty.
n64_send_id
    movlw   0x05
    movwf   n64_id_buffer + 0, b
    clrf    n64_id_buffer + 1, b
    movff   n64_slot_status, n64_id_buffer + 2
    btfsc   FLAG_CTRL2
    clrf    n64_id_buffer + 2, b

    movlw   n64_id_buffer                        ; Transmit the ID buffer
    movwf   FSR1L, a
    movlw   3
    goto    n64_tx

    ;; Don't return until the N64 data line has been idle long enough to ensure
    ;; we aren't in the middle of a packet already.
n64_wait_for_idle
    movlw   0x33
    movwf   temp, b
keep_waiting_for_idle
    btfss   N64_PIN, a
    goto    n64_wait_for_idle
    decfsz  temp, f, b
    goto    keep_waiting_for_idle
    return

    ;; Before transmitting, we explicitly force the output latch low- it may have
    ;; been left high by a read-modify-write operation elsewhere.
    ;; For controller response we allways need an 2us stop bit.
n64_tx
    wait    .53
    btfsc   FLAG_CTRL2
    bra     n64_tx2
    bsf     N64_TRIS, a
    bcf     N64_PIN, a
    n64gc_tx_buffer N64_TRIS, 1

n64_tx2
    bsf     N64_TRIS2, a
    bcf     N64_PIN2, a
    n64gc_tx_buffer N64_TRIS2, 1

n64_rx_bus
    n64gc_rx_buffer N64_PIN, bus_byte_count, 0

n64_rx_address
    n64gc_rx_buffer N64_PIN, byte_count, 0

n64_rx_command
    btfsc   N64_PIN, a
    bra     n64_rx_command2
    n64_rx_command_start 1
    n64_bus_copy N64_PIN, N64C_TRIS, byte_count, 0, 0
n64_rx_command2
    n64_rx_command_start 0
    n64gc_rx_buffer N64_PIN2, byte_count, 0


    ;; *******************************************************************************
    ;; ******************************************************  GameCube Interface  ***
    ;; *******************************************************************************

    ;; Don't return until the GameCube data line has been idle long enough to ensure
    ;; we aren't in the middle of a packet already.
gamecube_wait_for_idle
    movlw   0x50
    movwf   temp, b
gc_keep_waiting_for_idle
    btfss   GAMECUBE_PIN, a
    bra     gamecube_wait_for_idle
    decfsz  temp, f, b
    bra     gc_keep_waiting_for_idle
    return

    ;; Need to wait at least 15 us between GameCube controller commands.
    ;; Wait 32 us to be safe.
gamecube_bus_wait
    bcf     PIR1, TMR1IF, a                      ; Clear overflow bit.
    movlw   0xB0                                 ; Set single op R/W and 1:8 prescaler.
    movwf   T1CON, a
    setf    TMR1H, a
    movlw   0xC0
    movwf   TMR1L, a                             ; Clear timer1.
    bsf     T1CON, TMR1ON, a                     ; Enable timer1 and
    btfss   PIR1,  TMR1IF, a                     ; wait for timer1 overflow.
    bra     $-2
    bcf     PIR1, TMR1IF, a                      ; Clear overflow bit and
    bcf     T1CON, TMR1ON, a                     ; disable timer1.
    return

    ;; To support the WaveBird we must poll the controller identity first.
gamecube_get_id
    movlw   0x00                                 ; Put 0x00 in the gamecube_buffer.
    movwf   gamecube_buffer + 0, b

    movlw   gamecube_buffer                      ; Transmit the gamecube_buffer.
    movwf   FSR1L, a
    movlw   1
    call    gamecube_tx

    movlw   gamecube_buffer                      ; Receive 3 status bytes.
    movwf   FSR1L, a
    movlw   3
    call    gamecube_rx

    btfss   gamecube_buffer + 0, 7, b            ; Check only the MSB of the first byte since it's enough
    bra     gamecube_no_init                     ; to tell between normal controller and WaveBird receiver.
                                                 ; No fancy init for wired controllers.

    movlw   0x02                                 ; WaveBird don't have rumble motor so we show to the N64
    movwf   n64_slot_status, b                   ; that we are a controller with empty slot.

    movf    gamecube_buffer + 0, w, b            ; We have a WaveBird receiver connected and we check if
    xorlw   0xA8                                 ; a WaveBird is associated with it.
    bz      gamecube_no_init
    bsf     FLAG_CTRL_PENDING_INIT
    return

gamecube_no_init
    bsf     FLAG_CTRL_READY
    return

    ;; If we receive something other than 0xA8xxxx as ID we must repond with the WaveBird unique ID
    ;; at the end of command 0x4Exxxx to enable the WaveBird. It will not answer the poll status otherwise.
gamecube_init_wavebird
    movlw   0x4E                                 ; Put 0x4Exxxx in the gamecube_buffer to enable WaveBird.
    movwf   gamecube_buffer + 0, b

    ;; Two other bytes containing the WB unique ID already in the buffer.
    bcf     gamecube_buffer + 1, 5, b            ; Bits 5 and 4 are always 0 & 1 respectively in a 0x4E init command.
    bsf     gamecube_buffer + 1, 4, b

    movlw   gamecube_buffer                      ; Transmit the gamecube_buffer.
    movwf   FSR1L, a
    movlw   3
    call    gamecube_tx

    movlw   gamecube_buffer                      ; Receive 3 status bytes
    movwf   FSR1L, a
    movlw   3
    call    gamecube_rx

    movf    flags, w, b
    iorlw   BIT_CTRL_READY                       ; bsf     FLAG_CTRL_READY
    andlw   ~BIT_CTRL_PENDING_INIT               ; bcf     FLAG_CTRL_PENDING_INIT
    movwf   flags, b                             ; Set flags as atomic operations. Avoid disabling interrupt.
    return

    ;; GameCube controller init routine.
gamecube_init
    clrwdt
    btfsc   FLAG_CALIBRATED
    return
    call    gamecube_get_id
    btfsc   FLAG_CTRL_READY
    bra     gamecube_ready
    call    gamecube_bus_wait
    btfss   FLAG_CTRL_PENDING_INIT
    bra     gamecube_init
    call    gamecube_init_wavebird
gamecube_ready
    call    gamecube_bus_wait
    call    gamecube_poll_status
    call    gamecube_reset_calibration

    ;; Check if the user wants to disable Rumble Pak emulation on boot.
    btfsc   gamecube_buffer + GC_D_RIGHT, b
    clrf    n64_slot_status, b

    ;; Check if the user wants to use the bypass mode on boot.
    btfsc   gamecube_buffer + GC_D_UP, b
    bsf     FLAG_BYPASS_MODE

    call    gamecube_bus_wait
    bsf     FLAG_CALIBRATED
    return

    ;; Poll the GameCube controller's state by transmitting a magical
    ;; poll command (0x400300) then receiving 8 bytes of status.
gamecube_poll_status
    movlw   0x40                                 ; Put 0x400300 in the gamecube_buffer
    movwf   gamecube_buffer + 0, b
    movlw   0x03
    movwf   gamecube_buffer + 1, b
    movlw   0x00
    movwf   gamecube_buffer + 2, b

    btfsc   FLAG_RUMBLE_MOTOR_ON                 ; Set the low bit of our GameCube command to turn on rumble.
    bsf     gamecube_buffer + 2, 0, b

    movlw   gamecube_buffer                      ; Transmit the gamecube_buffer
    movwf   FSR1L, a
    movlw   3
    call    gamecube_tx

    movlw   gamecube_buffer                      ; Receive 8 status bytes
    movwf   FSR1L, a
    movlw   8
    call    gamecube_rx
    return

gamecube_tx
    bcf     GAMECUBE_PIN, a
    n64gc_tx_buffer GAMECUBE_TRIS, 0

gamecube_rx
    movwf   byte_count, b
    n64gc_rx_buffer GAMECUBE_PIN, byte_count, 0


    ;; *******************************************************************************
    ;; ******************************************************  Lookup Tables  ********
    ;; *******************************************************************************

    ;; This contain the default configuration used if the EEPROM is empty, corrupt or
    ;; user perform reset.
    org     0x3400
eeprom_default
    #include eeprom_default.inc

    ;; 256-byte table extracted from the test vectors, that can be used to
    ;; compute any CRC. This table is the inverted CRC generated for every
    ;; possible message with exactly one bit set.
    ;;
    ;; It was generated using the reversed large-table
    ;; implementation of the CRC, in notes/gen_asm_large_table_crc.py
    org     0x3500
crc_large_table
    #include large_table_crc.inc

    ;; This contains the scaling tables used to scale our GC joysticks values.
    org     0x3600
    #include js_scale.inc

    ;; This contains the response curve tables for GC joysticks values.
    org     0x3800
    #include js_curve.inc

    end
