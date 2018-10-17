    ;;
    ;; A reverse engineering and debugging utility that converts commands
    ;; sent over RS-232 to commands encoded in the N64/Gamecube low-level format.
    ;; Copyright (C) 2004 Micah Dowty <micah@navi.cx>
    ;;
    ;;   This program is free software; you can redistribute it and/or modify
    ;;   it under the terms of the GNU General Public License as published by
    ;;   the Free Software Foundation; either version 2 of the License, or
    ;;   (at your option) any later version.
    ;;
    ;; This firmware is designed to run on a high-end PIC microcontroller
    ;; clocked at 64 MHz. The Nintendo device's bidirectional data line
    ;; is on RC0, serial receive is on RB5, serial transmit is on RB7.
    ;;
    ;; RA4 is set up for transition detection, useful in figuring out the
    ;; address mapping. A 47K pull down resistor is required to stabilize
    ;; the input.
    ;;
    ;; This accepts commands of the following form over the serial port,
    ;; which is configured for 115200 baud, 8-N-1:
    ;;
    ;; Host to device:
    ;;
    ;;   Byte   Function
    ;; ----------------------
    ;;   0      Always 0x7E
    ;;   1      Number of bytes to transmit (up to 40)
    ;;   2      Number of bytes to receive afterwards (up to 40)
    ;;   3-n    Data to transmit
    ;;
    ;; Device to host:
    ;;
    ;;   Byte   Function
    ;; ----------------------
    ;;   0      0x01 if transitions detected on RA4, 0x00 otherwise.
    ;;   1-n    Received data
    ;;
    errorlevel -206

    ;; Definitions for the PIC18F14K22 version
    ifdef  __18F14K22
        #include p18f14k22.inc

        CONFIG FOSC = IRC, PLLEN = ON, PCLKEN = OFF, FCMEN = OFF, IESO = OFF
        CONFIG PWRTEN = OFF, BOREN = OFF
        CONFIG WDTEN = ON, WDTPS = 4
        CONFIG HFOFST = ON, MCLRE = OFF
        CONFIG STVREN = OFF, LVP = OFF, BBSIZ = OFF, XINST = ON, DEBUG = OFF
        CONFIG CP0 = OFF, CP1 = OFF
        CONFIG CPB = OFF, CPD = OFF
        CONFIG WRT0 = OFF, WRT1 = OFF
        CONFIG WRTC = OFF, WRTB = OFF, WRTD = OFF
        CONFIG EBTR0 = OFF, EBTR1 = OFF
        CONFIG EBTRB = OFF

        ;; Hardware declarations
        #define PROBE_PIN       PORTA, 4
        #define NINTENDO_PIN    PORTC, 0
        #define NINTENDO_TRIS   TRISC, 0
        #define RX232_PIN       PORTB, 5
        #define TX232_PIN       PORTB, 7

mcu_init macro
        movlw   0x70                             ; Set internal clock to 16 MHz.
        movwf   OSCCON, a
        pll_startup_delay                        ; Wait for PLL to shift frequency to 64 MHz.
        clrf    PORTA, a
        clrf    PORTB, a
        clrf    PORTC, a
        clrf    WPUA, a                          ; Disable pull-ups.
        clrf    WPUB, a
        movlw   0x10
        movwf   TRISA, a
        movlw   0x20
        movwf   TRISB, a
        movlw   0x01
        movwf   TRISC, a
        clrf    ANSEL, a                         ; Set IOs to digital.
        clrf    ANSELH, a
        movlw   0x21                             ; Set baudrate to 115200.
        movwf   SPBRG, a
        bsf     TXSTA, BRGH, a
        bsf     RCSTA, SPEN, a
        bsf     TXSTA, TXEN, a
        bsf     RCSTA, CREN, a
        bsf     IOCA, IOCA4, a                   ; Enable interrupt on RA4.
        endm
    else
    ifdef  __18F24Q10
        #include p18f24q10.inc

        CONFIG FEXTOSC = OFF, RSTOSC = HFINTOSC_64MHZ
        CONFIG CLKOUTEN = OFF, CSWEN = OFF, FCMEN = OFF
        CONFIG MCLRE = INTMCLR, PWRTE = OFF, LPBOREN = OFF, BOREN = OFF
        CONFIG BORV = VBOR_285, ZCD = OFF, PPS1WAY = ON, STVREN = OFF, DEBUG = OFF, XINST = ON
        CONFIG WDTCPS = WDTCPS_4, WDTE = ON
        CONFIG WDTCWS = WDTCWS_6, WDTCCS = LFINTOSC
        CONFIG WRT0 = OFF, WRT1 = OFF
        CONFIG WRTC = OFF, WRTB = OFF, WRTD = OFF, SCANE = OFF, LVP = OFF
        CONFIG CP = OFF, CPD = OFF
        CONFIG EBTR0 = OFF, EBTR1 = OFF
        CONFIG EBTRB = OFF

        ;; Hardware declarations
        #define PROBE_PIN       PORTB, 2
        #define NINTENDO_PIN    PORTC, 3
        #define NINTENDO_TRIS   TRISC, 3
        #define RX232_PIN       PORTC, 7
        #define TX232_PIN       PORTC, 6

mcu_init macro
        movlb   0x0E                             ; Peripheral pin selection for UART.
        movlw   0x17
        movwf   RX1PPS, b
        movlw   0x09
        movwf   RC6PPS, b

        movlb   0x0F

        clrf    PORTA, a
        clrf    PORTB, a
        clrf    PORTC, a
        movlw   0x00
        movwf   TRISA, a
        movlw   0x04
        movwf   TRISB, a
        movlw   0x88
        movwf   TRISC, a
        movlw   0x08                             ; Not really needed for SI pins
        movwf   ODCONC, b                        ; as we toggle TRIS register on TX.
        clrf    ANSELA, b                        ; Set IOs to digital.
        clrf    ANSELB, b
        clrf    ANSELC, b
        movlw   0x21                             ; Set baudrate to 115200.
        movwf   SPBRG, a
        bsf     TXSTA, BRGH, a
        bsf     RCSTA, SPEN, a
        bsf     TXSTA, TXEN, a
        bsf     RCSTA, CREN, a
        bsf     IOCBP, IOCBP0, b                 ; Enable interrupt on RB0.
        movlb   0x00
        endm
    else
        messg    "Unsupported processor"
    endif
    endif

    ;; Delay of about ~2 ms that allow the PLL to shift the frequency to 64 MHz on K22.
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

    #include n64gc_comm.inc

    ;; Reset and interrupt vectors
    org 0x00
    goto    startup
    org 0x08
    retfie
    org 0x18
    retfie

    ;; Variables
    cblock  0x00
        temp
        bit_count
        byte_count
        cmd_tx_count
        cmd_rx_count
        buffer:.40
        crc_work
        n64_crc
    endc

startup
    movlb   0x00                                 ; Set bank 0 active.
    bcf     INTCON, GIE, a                       ; Disable interrupts.
    mcu_init
    n64gc_init
    clrf    FSR1H, a

main_loop
    call    serial_rx
    xorlw   0x7E
    btfss   STATUS, Z, a
    goto    main_loop                            ; Wait for the beginning of command marker, 0x7E

    movf    PORTA, w, a                          ; Clear RA4 transition.
    nop
ifdef __18F24Q10
    banksel IOCBF
    bcf     IOCBF, IOCBF0, b
    movlb   0x00
else
    bcf     INTCON, RABIF, a
endif

    call    serial_rx                            ; Save the transmit and receive counts
    movwf   cmd_tx_count, b
    call    serial_rx
    movwf   cmd_rx_count, b

    movf    cmd_tx_count, w, b                   ; Do we have any data to transmit?
    btfsc   STATUS, Z, a
    goto    nothing_to_transmit

    movlw   buffer                               ; Read in the data to transmit
    movwf   FSR1L, a
    movf    cmd_tx_count, w, b
    movwf   byte_count, b
tx_read_loop
    call    serial_rx
    movwf   INDF1, a
    incf    FSR1L, f, a
    decfsz  byte_count, f, b
    goto    tx_read_loop

    movlw   buffer                               ; Transmit to the Nintendo bus...
    movwf   FSR1L, a
    movf    cmd_tx_count, w, b
    call    nintendo_tx
nothing_to_transmit

    movlw   buffer                               ; ..then immediately receive
    movwf   FSR1L, a
    movf    cmd_rx_count, w, b
    btfsc   STATUS, Z                            ; but not if we have nothing to receive
    goto    main_loop
    call    nintendo_rx

    movlw   0x00                                 ; Transmit the transition detection byte
ifdef __18F24Q10
    banksel IOCBF
    btfsc   IOCBF, IOCBF0, b
else
    btfsc   INTCON, RABIF, a
endif
    movlw   0x01
ifdef __18F24Q10
    movlb   0x00
endif
    call    serial_tx

    movlw   buffer                               ; Send the data we received
    movwf   FSR1L, a
    movf    cmd_rx_count, w, b
    movwf   byte_count, b
rx_read_loop
    movf    INDF1, w, a
    call    serial_tx
    incf    FSR1L, f, a
    decfsz  byte_count, f, b
    goto    rx_read_loop

    goto    main_loop


serial_rx
    clrwdt
ifdef __18F24Q10
    banksel PIR3
    btfss   PIR3, RC1IF, b
else
    btfss   PIR1, RCIF, a
endif
    bra     serial_rx
ifdef __18F24Q10
    movlb   0x00
endif
    movf    RCREG, w, a
    return

serial_tx
    movwf   TXREG, a
    btfss   TXSTA, TRMT, a
    bra     $-2
    return

nintendo_tx
    bcf NINTENDO_PIN, a
    n64gc_tx_buffer NINTENDO_TRIS, 0

nintendo_rx
    movwf   byte_count, b
    n64gc_rx_buffer NINTENDO_PIN, byte_count, 0

    end
