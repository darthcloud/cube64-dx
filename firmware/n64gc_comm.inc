    ;;
    ;; N64 and GameCube controller port communications macros
    ;; Copyright (C) 2004 Micah Dowty <micah@navi.cx>
    ;;               2011 Jacques Gagnon <darthcloud@gmail.com>
    ;;
    ;;   This program is free software; you can redistribute it and/or modify
    ;;   it under the terms of the GNU General Public License as published by
    ;;   the Free Software Foundation; either version 2 of the License, or
    ;;   (at your option) any later version.
    ;;
    ;; This firmware is designed to run on a high-end PIC microcontroller
    ;; clocked at 64 MHz. The macros herein let you transmit and receive
    ;; arbitrary-sized buffers of data using the low-level encoding used
    ;; by the N64 and GameCube.
    ;;
    ;; Additionally, this header includes #defines related to the higher-level
    ;; bits of the protocol.
    ;;
    ;; This wouldn't have been possible without all the effort already
    ;; poured into reverse engineering the N64 and GameCube by others.
    ;; The author found these resources particularly helpful:
    ;;
    ;;   http://www.int03.co.uk/crema/hardware/gamecube/gc-control.htm
    ;;   http://www.gc-linux.org/docs/yagcd/chap8.html
    ;;   http://instruct1.cit.cornell.edu/courses/ee476/FinalProjects/s2002/jew17/lld.html
    ;;   http://www.mixdown.ca/n64dev/
    ;;
    ;; Unfortunately, the above sources still don't tell you everything you
    ;; need to know to trick an N64 into thinking you're a controller. The rest
    ;; of this information was reverse engineered by the author.
    ;;
    ;; The serial format is the same as that used by the GameCube: zeroes
    ;; are 3us off 1us on, ones are 1us off 3us on. All packets are a multiple
    ;; of 8 bits long not including the single '1' bit used as a stop bit.
    ;; The 0x02 and 0x03 packets use a 2us wide stop bit after the CRC rather
    ;; than the usual 1us.
    ;;
    ;; The N64 initiates communications by sending a 1-byte command packet. The
    ;; following commands have been observed:
    ;;
    ;;   0x00 :   Asks the controller to identify itself. Returns a 24-bit word.
    ;;   0xFF :   An official N64 controller will respond with 0x050002 if it has
    ;;            nothing in the expansion slot, 0x050001 if it has a memory or
    ;;            rumble pak installed. I've seen 0x050003 from a controller with
    ;;            a memory pak, but I don't know what the extra bit signifies.
    ;;
    ;;            This command has the side-effect of causing the controller to
    ;;            re-check whether a new controller pak has been plugged in.
    ;;
    ;;            0xFF is only used with early N64 games.
    ;;
    ;;   0x01 :   Returns a 32-bit status word including all button and axis state.
    ;;            The format of this status word is documented in the sources above.
    ;;
    ;;   0x02 :   Read 32 bytes from the controller pak bus. The N64 follows the 0x02
    ;;            with a two-byte encoded address. The controller returns 32 bytes
    ;;            of data followed by a 1-byte CRC.
    ;;
    ;;   0x03 :   Write 32 bytes to the controller pak bus. The 0x03 is followed by
    ;;            a two-byte encoded address and 32 data bytes. The controller
    ;;            responds with a 1-byte CRC.
    ;;
    ;; The address encoding is such that all reads and writes are aligned on 32-byte
    ;; boundaries, and the low 5 bits are used as a sort of CRC to validate the address.
    ;; The address encoding and data CRC are both described and implemented in the
    ;; accompanying 'notes' directory. The CRC is inverted if a controller pak is
    ;; connected.
    ;;
    ;; The controller pak bus reads and writes use a 16-bit address, however the
    ;; controller pak only includes 32 bytes of SRAM. The top half of memory is
    ;; therefore reserved for memory mapped I/O used by non-memory peripherals.
    ;;
    ;; 0x8000 is the initiralization/identification word. At startup, a packet
    ;; of all 0x80 bytes is written there to initialize the controller pak. The
    ;; N64 then reads back either all 0x80, indicating a rumble pak or 0x00,
    ;; indicating a normal SRAM controller pak. Other peripherals like the
    ;; microphone probably have different identifying bytes.
    ;;
    ;; The rumble pak motor is a memory mapped latch at 0xC000. 0x01 is written there
    ;; to turn it on, 0x00 to turn it off.
    ;;
    ;; Most games work fine if only the 0x00 and 0x01 commands are implemented,
    ;; but some games will attempt to use the controller pak read/write commands
    ;; and report that no controller is found if they fail. This includes Ocarina of
    ;; Time and Majora's Mask.
    ;;

    ;; Temporary using the pad macro to waste the additional time given by
    ;; using the PIC18F14K22 @ 64 MHz. This waste the 11 additional cycles we
    ;; get each 1 us.
pad_11_cycles    macro
    goto $+4
    goto $+4
    goto $+4
    goto $+4
    goto $+4
    nop
    endm

pad_10_cycles    macro
    goto $+4
    goto $+4
    goto $+4
    goto $+4
    goto $+4
    endm

pad_9_cycles    macro
    goto $+4
    goto $+4
    goto $+4
    goto $+4
    nop
    endm

    ;; *******************************************************************************
    ;; *********************************************************** Definitions *******
    ;; *******************************************************************************

    ;; 1-byte commands sent from N64 to controller
    #define N64_COMMAND_IDENTIFY    0x00
    #define N64_COMMAND_STATUS      0x01
    #define N64_COMMAND_READ_BUS    0x02
    #define N64_COMMAND_WRITE_BUS   0x03
    #define N64_COMMAND_OLD_IDENT   0xFF

    ;; Controller pak identities
    #define N64_PAK_MEMORY          0x00
    #define N64_PAK_RUMBLE          0x80

    ;; GameCube buttons (byte and bit offset in status packet)
    #define GC_A        0, 0
    #define GC_B        0, 1
    #define GC_X        0, 2
    #define GC_Y        0, 3
    #define GC_START    0, 4
    #define GC_D_LEFT   1, 0
    #define GC_D_RIGHT  1, 1
    #define GC_D_DOWN   1, 2
    #define GC_D_UP     1, 3
    #define GC_Z        1, 4
    #define GC_R        1, 5
    #define GC_L        1, 6

    ;; GameCube axes (byte offset in status packet)
    #define GC_JOYSTICK_X   2
    #define GC_JOYSTICK_Y   3
    #define GC_CSTICK_X     4
    #define GC_CSTICK_Y     5
    #define GC_L_ANALOG     6
    #define GC_R_ANALOG     7

    ;; N64 buttons (byte and bit offset in status packet)
    #define N64_D_RIGHT     0, 0
    #define N64_D_LEFT      0, 1
    #define N64_D_DOWN      0, 2
    #define N64_D_UP        0, 3
    #define N64_START       0, 4
    #define N64_Z           0, 5
    #define N64_B           0, 6
    #define N64_A           0, 7
    #define N64_C_RIGHT     1, 0
    #define N64_C_LEFT      1, 1
    #define N64_C_DOWN      1, 2
    #define N64_C_UP        1, 3
    #define N64_R           1, 4
    #define N64_L           1, 5

    ;; N64 axes (byte offset in status packet)
    #define N64_JOYSTICK_X  2
    #define N64_JOYSTICK_Y  3

n64gc_init macro
    ;; Put us back in bank 0 with a bit_count of 8, preconditions for rx_buffer
    movlw   8
    movwf   bit_count
    endm


    ;; *******************************************************************************
    ;; *********************************************************** Transmitter *******
    ;; *******************************************************************************

    ;; Transmit 'w' bytes of data over the indicated pin, starting at IRP:FSR.
    ;; The data is modified as it is transmitted, as we don't have enough time
    ;; to copy each byte to a temporary location. Returns when done. This macro
    ;; works with the GameCube or N64, as they both use the same low-level protocol.
n64gc_tx_buffer macro port, bit, wide_stop_bit
    local   bit_loop
    local   not_last_byte
    local   not_last_bit

    movwf   byte_count      ; Stow our count and start looping over bytes

    movlw   8               ; Set the bit count for the first time. Subsequently
    movwf   bit_count       ; it gets set in the spare cycles below.

    ;; Each bit always takes 4us, but there are several possible control paths...
    ;; Every line has been numbered with the number of microseconds since the
    ;; beginning of the current bit. There isn't enough time to use traditional
    ;; nested loops for bits and bytes, so we use the time during a bit to prepare
    ;; for the next bit. This is implemented in the form of three alternate endings
    ;; for the loop.
bit_loop
    pad_11_cycles           ; 3.1875us
    nop                     ; 3.8750us
    rlcf    INDF1, f, 0     ; 3.9375us  Rotate the next bit into C
    bcf     port, bit, 0    ; 0.0000us  The falling edge begins another bit
    goto    $+4             ; 0.0625us
    nop                     ; 0.1875us
    pad_11_cycles           ; 0.2500us
    btfsc   STATUS, C, 0    ; 0.9375us  Our bit is in the C flag...
    bsf     port, bit, 0    ; 1.0000us  Rising edge for 1 bits
    decfsz  bit_count, f    ; 1.0625us  Is this the last bit?
    goto    not_last_bit    ; 1.1250us  We have an alternate ending for the last bit...
    ;; nop (goto 2nd word)  ; 1.1875us

    decfsz  byte_count,f    ; 1.2500us  Is this the last byte?
    goto    not_last_byte   ; 1.3125us  Yet another alternate ending for the last byte...
    ;; nop (goto 2nd word)  ; 1.3750us

    pad_11_cycles           ; 1.4375us
    goto    $+4             ; 2.1250us
    pad_11_cycles           ; 2.2500us
    btfss   STATUS, C, 0    ; 2.9375us
    bsf     port, bit, 0    ; 3.0000us  Rising edge for 0 bits
    goto    $+4             ; 3.0625us
    pad_11_cycles           ; 3.1875us
    goto    $+4             ; 3.8750us
    bcf     port, bit, 0    ; 0.0000us  Begin a stop bit
    pad_11_cycles           ; 0.0625us
    movlw   8               ; 0.7500us  Setup for the next receive
    movwf   bit_count       ; 0.8125us
    goto    $+4             ; 0.8750us

    if wide_stop_bit
        goto    $+4         ; Use a 2us stop bit rather than 1us, waste 16 cycles
        goto    $+4
        nop
        pad_11_cycles
    endif

    bsf     port, bit, 0    ; 1.0000us  Rising edge for the stop bit
    return                  ; 1.0625us  Done for now. The first instruction after the
                            ; macro is at 1.6us after the beginning of the
                            ; stop bit, so it ends 2.4us after we return.

not_last_byte               ; 1.3750us
    pad_10_cycles           ; 1.4375us
    pad_11_cycles           ; 2.0625us
    incf    FSR1L, f, 0     ; 2.7500us  Point at the next byte
    movlw   8               ; 2.8125us  Reinitialize the bit counter
    movwf   bit_count       ; 2.8750us
    btfss   STATUS, C, 0    ; 2.9375us
    bsf     port, bit, 0    ; 3.0000us  Rising edge for 0 bits
    goto    bit_loop        ; 3.0625us

not_last_bit                ; 1.1875us
    pad_11_cycles           ; 1.2500us
    pad_11_cycles           ; 1.9375us
    goto    $+4             ; 2.6250us
    goto    $+4             ; 2.7500us
    nop                     ; 2.8750us
    btfss   STATUS, C, 0    ; 2.9375us
    bsf     port, bit, 0    ; 3.0000us  Rising edge for 0 bits
    goto    bit_loop        ; 3.0625us  Next bit...
    endm


    ;; *******************************************************************************
    ;; *********************************************************** Receiver **********
    ;; *******************************************************************************

    ;; Receive byte_count bytes of data over the indicated pin, starting at IRP:FSR.
    ;; Again, this macro works with the GC or the N64. This could block indefinitely
    ;; waiting for data. If this happens, the watchdog will reset us.
    ;;
    ;; Since receives often have to be started extremely quickly after a transmit
    ;; ends, some of the setup work for this loop occurs inside tx_buffer above,
    ;; during otherwise wasted cycles.
    ;;
    ;; This leaves FSR pointing at the last byte received.
    ;;
n64gc_rx_buffer macro port, bit, byte_count, clear_watchdog
    local   bit_loop
    local   not_last_bit
    local   not_last_byte

    ;; We poll for the low state signalling the beginning of a bit, time out 2us
    ;; (putting us right in the middle of the bit) and then sample. This probably isn't
    ;; how the GC/N64 hardware decodes the received data, but it's close enough.
    ;;
    ;; The overhead from sampling to the beginning of the next poll needs to
    ;; be less than 2us (32 cycles)
bit_loop
    if clear_watchdog
        clrwdt
    endif

    btfsc   port, bit, 0    ; 0.0000us  Poll for the beginning of the bit, max 0.1875us jitter
    goto    bit_loop        ; 0.0625us
    ;; nop (goto 2nd word)  ; 0.1250us
    pad_10_cycles           ; 0.1875us
    rlcf    INDF1, f, 0     ; 0.8125us  Make room for the new bit
    bcf     INDF1, 0, 0     ; 0.8750us  Assume it's 0 to begin with
    decfsz  bit_count, f    ; 0.9375us  Is this the last bit?
    goto    not_last_bit    ; 1.0000us  We have an alternate ending for the last bit...
    ;; nop (goto 2nd word)  ; 1.0625us

    decfsz  byte_count,f    ; 1.1250us  Is this the last byte?
    goto    not_last_byte   ; 1.1875us  Yet another alternate ending for the last byte...
    ;; nop (goto 2nd word)  ; 1.2500us

    movlw   8               ; 1.3125us  Reset bit count
    movwf   bit_count       ; 1.3750us
    pad_9_cycles            ; 1.4375us
    btfsc   port, bit, 0    ; 2.0000us  Sample the incoming bit
    bsf     INDF1, 0, 0     ; 2.0625us
    return                  ; 2.1250us

not_last_byte               ; 1.2500us
    pad_10_cycles           ; 1.3125us
    movlw   8               ; 1.9375us
    btfsc   port, bit, 0    ; 2.0000us  Sample the incoming bit
    bsf     INDF1, 0, 0     ; 2.0625us
    incf    FSR1L, f, 0     ; 2.1250us  Next byte...
    movwf   bit_count       ; 2.1875us  Reset bit count
    btfsc   port, bit, 0    ; 2.2500us  Wait for the data line to go back high
    goto    bit_loop        ; 2.3125us
    ;; nop (goto 2nd word)  ; 2.3750us
    goto    $-6             ; 2.4375us

not_last_bit                ; 1.0625us
    pad_11_cycles           ; 1.1250us
    nop                     ; 1.8125us
    goto    $+4             ; 1.8750us
    btfsc   port, bit, 0    ; 2.0000us  Sample the incoming bit
    bsf     INDF1, 0, 0     ; 2.0625us
    btfsc   port, bit, 0    ; 2.1250us  Wait for the data line to go back high
    goto    bit_loop        ; 2.1875us
    ;; nop (goto 2nd word)  ; 2.2500us
    goto    $-6             ; 2.3125us
    endm

    ;;; The End ;;;
