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
	;; This firmware is designed to run on a PIC16F84A microcontroller
	;; clocked at 20 MHz. The Nintendo device's bidirectional data line
	;; is on RA0, serial receive is on RB0, serial transmit is on RB1.
	;;
	;; RA4 is set up for transition detection, useful in figuring out the
	;; address mapping.
	;;
	;; This accepts commands of the following form over the serial port,
	;; which is configured for 38400 baud, 8-N-1:
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
	;;   0      Number of transitions detected on RA4
	;;   1-n    Received data
	;;

	list	p=16f84a
	errorlevel -302
	#include p16f84a.inc

	#include n64gc_comm.inc
	#include rs232_comm.inc

	__CONFIG   _CP_OFF & _PWRTE_OFF & _WDT_ON & _HS_OSC

	;; Hardware declarations
	#define NINTENDO_PIN	PORTA, 0
	#define NINTENDO_TRIS	TRISA, 0
	#define RX232_PIN	PORTB, 0
	#define TX232_PIN	PORTB, 1

	;; Reset and interrupt vectors
	org 0
	goto	startup
	org 4
	retfie

	;; Variables
	cblock	0x0C
		temp
		bit_count
		byte_count
		serial_byte
		cmd_tx_count
		cmd_rx_count
		buffer:.40
	endc

startup
	;; Initialize I/O ports- all unused pins output low, nintendo pin and serial receive
	;; are initially tristated.
	bcf	STATUS, RP0
	clrf	PORTA
	clrf	PORTB
	bsf	STATUS, RP0
	movlw	0x11
	movwf	TRISA
	movlw	0x01
	movwf	TRISB

	n64gc_init

main_loop
	call	serial_rx
	xorlw	0x7E
	btfss	STATUS, Z
	goto	main_loop	; Wait for the beginning of command marker, 0x7E

	call	serial_rx	; Save the transmit and receive counts
	movwf	cmd_tx_count
	call	serial_rx
	movwf	cmd_rx_count

	movf	cmd_tx_count, w	; Do we have any data to transmit?
	btfsc	STATUS, Z
	goto	nothing_to_transmit

	movlw	buffer		; Read in the data to transmit
	movwf	FSR
	movf	cmd_tx_count, w
	movwf	byte_count
tx_read_loop
	call	serial_rx
	movwf	INDF
	incf	FSR, f
	decfsz	byte_count, f
	goto	tx_read_loop

	bsf	STATUS, RP0	; Reset the counter
	movlw	0xEF
	movwf	OPTION_REG
	bcf	STATUS, RP0
	clrf	TMR0

	movlw	buffer		; Transmit to the Nintendo bus...
	movwf	FSR
	movf	cmd_tx_count, w
	call	nintendo_tx
nothing_to_transmit

	movlw	buffer		; ..then immediately receive
	movwf	FSR
	movf	cmd_rx_count, w
	btfsc	STATUS, Z	; but not if we have nothing to receive
	goto	main_loop
	call	nintendo_rx

	bcf	STATUS, RP0
	movf	TMR0, w		; Transmit the transition detection byte
	call	serial_tx

	movlw	buffer		; Send the data we received
	movwf	FSR
	movf	cmd_rx_count, w
	movwf	byte_count
rx_read_loop
	movf	INDF, w
	call	serial_tx
	incf	FSR, f
	decfsz	byte_count, f
	goto	rx_read_loop

	goto	main_loop


serial_rx
	rs232_rx_byte	RX232_PIN, B38400, POLARITY_INVERTED, serial_byte
	movf	serial_byte, w
	return

serial_tx
	movwf	serial_byte
	rs232_tx_byte	TX232_PIN, B38400, POLARITY_INVERTED, serial_byte
	return

nintendo_tx
	bcf	NINTENDO_PIN
	n64gc_tx_buffer NINTENDO_TRIS, 0

nintendo_rx
	movwf	byte_count
	n64gc_rx_buffer NINTENDO_PIN, byte_count, 0

	end
